//! Low-level SPIRIT1 SPI transport.
//!
//! Owns the blocking [`Spi`] on SPI1 plus the **software** chip-select on PA15.
//! The SPIRIT1 needs `t_su(CS) ≥ 2 µs` between CS falling and the first SCLK
//! edge (datasheet Table 35), which the peripheral's hardware NSS can't
//! guarantee — so CS is a plain GPIO output driven low for the whole
//! transaction, with a short busy-wait after asserting it.
//!
//! Every method returns the two MC_STATE status bytes the SPIRIT1 shifts out on
//! MISO during the header + address bytes of *every* transaction (§10.2). Use
//! [`regs::state_from_status`](super::regs::state_from_status) on `status[0]` to
//! get the current MC state for free — no extra read.

#![allow(dead_code)]

use embassy_stm32::Peri;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{PA15, PB3, PB4, PB5, SPI1};
use embassy_stm32::spi::mode::Master;
use embassy_stm32::spi::{Config as SpiConfig, Error as SpiError, MODE_0, Spi};
use embassy_stm32::time::Hertz;

use super::regs;

/// SPI bus clock. ≤10 MHz per the datasheet; 8 MHz is the closest prescaler step
/// below that from the 16 MHz sysclk and is comfortable over the on-board trace.
const SPI_HZ: u32 = 8_000_000;

/// CS setup busy-wait, in core cycles at 16 MHz sysclk. 48 cycles ≈ 3 µs, above
/// the datasheet's 2 µs `t_su(CS)` minimum with margin.
const CS_SETUP_CYCLES: u32 = 48;
/// CS hold after the last clock before releasing CS (a little margin for `t_h`).
const CS_HOLD_CYCLES: u32 = 32;

/// Largest single SPI transaction: header + address + the 96-byte FIFO.
const MAX_XFER: usize = 2 + 96;

/// The two status bytes returned on MISO by every SPIRIT1 transaction
/// (`[MC_STATE0, MC_STATE1]`).
pub type Status = [u8; 2];

/// SPIRIT1 SPI transport: the blocking SPI bus plus the software CS pin.
pub struct Spirit1Spi {
    spi: Spi<'static, Blocking, Master>,
    cs: Output<'static>,
}

impl Spirit1Spi {
    /// Build the transport from the board's raw SPI1 + pins. CS starts high
    /// (de-asserted). Configures the bus for SPIRIT1: mode 0, MSB-first, 8 MHz.
    pub fn new(
        spi: Peri<'static, SPI1>,
        sck: Peri<'static, PB3>,
        mosi: Peri<'static, PB5>,
        miso: Peri<'static, PB4>,
        cs: Peri<'static, PA15>,
    ) -> Self {
        let mut cfg = SpiConfig::default();
        cfg.frequency = Hertz(SPI_HZ);
        cfg.mode = MODE_0;
        let spi = Spi::new_blocking(spi, sck, mosi, miso, cfg);
        // CS idle high (the SPIRIT1 is de-selected); ≥2 µs is enforced per
        // transaction once we pull it low.
        let cs = Output::new(cs, Level::High, Speed::VeryHigh);
        Self { spi, cs }
    }

    /// Write `data` to consecutive registers starting at `addr` (burst).
    pub fn write_regs(&mut self, addr: u8, data: &[u8]) -> Result<Status, SpiError> {
        let n = data.len();
        let mut tx = [0u8; MAX_XFER];
        let mut rx = [0u8; MAX_XFER];
        tx[0] = regs::WRITE;
        tx[1] = addr;
        tx[2..2 + n].copy_from_slice(data);
        self.xfer(&mut rx[..2 + n], &tx[..2 + n])?;
        Ok([rx[0], rx[1]])
    }

    /// Read `out.len()` consecutive registers starting at `addr` (burst).
    pub fn read_regs(&mut self, addr: u8, out: &mut [u8]) -> Result<Status, SpiError> {
        let n = out.len();
        let mut tx = [0u8; MAX_XFER];
        let mut rx = [0u8; MAX_XFER];
        tx[0] = regs::READ;
        tx[1] = addr;
        self.xfer(&mut rx[..2 + n], &tx[..2 + n])?;
        out.copy_from_slice(&rx[2..2 + n]);
        Ok([rx[0], rx[1]])
    }

    /// Read a single register.
    pub fn read_reg(&mut self, addr: u8) -> Result<(u8, Status), SpiError> {
        let mut b = [0u8; 1];
        let st = self.read_regs(addr, &mut b)?;
        Ok((b[0], st))
    }

    /// Write a single register.
    pub fn write_reg(&mut self, addr: u8, value: u8) -> Result<Status, SpiError> {
        self.write_regs(addr, &[value])
    }

    /// Send a command strobe (e.g. [`regs::CMD_TX`]).
    pub fn command(&mut self, cmd: u8) -> Result<Status, SpiError> {
        let tx = [regs::COMMAND, cmd];
        let mut rx = [0u8; 2];
        self.xfer(&mut rx, &tx)?;
        Ok(rx)
    }

    /// Write `data` into the TX FIFO (linear FIFO address).
    pub fn write_fifo(&mut self, data: &[u8]) -> Result<Status, SpiError> {
        self.write_regs(regs::FIFO, data)
    }

    /// Read `out.len()` bytes from the RX FIFO (linear FIFO address).
    pub fn read_fifo(&mut self, out: &mut [u8]) -> Result<Status, SpiError> {
        self.read_regs(regs::FIFO, out)
    }

    /// One full-duplex transfer wrapped in the software-CS pulse with the
    /// datasheet setup/hold guard times.
    fn xfer(&mut self, rx: &mut [u8], tx: &[u8]) -> Result<(), SpiError> {
        self.cs.set_low();
        cortex_m::asm::delay(CS_SETUP_CYCLES);
        let r = self.spi.blocking_transfer(rx, tx);
        cortex_m::asm::delay(CS_HOLD_CYCLES);
        self.cs.set_high();
        r
    }
}
