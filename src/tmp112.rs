//! TMP112 I²C temperature sensor driver.
//!
//! HAL-independent: generic over [`embedded_hal::i2c::I2c`], so the same driver
//! works with the embassy-stm32 blocking I²C used on the Core Module, with a
//! shared-bus device (`embedded-hal-bus`), or with a mock in host tests.
//!
//! One-shot oriented: the sensor is told to perform a single conversion on
//! demand and otherwise stays in shutdown (~1 µA), matching the low-power
//! firmware. The register-level methods ([`trigger_oneshot`](Tmp112::trigger_oneshot),
//! [`conversion_ready`](Tmp112::conversion_ready), [`read_raw`](Tmp112::read_raw))
//! are delay-agnostic; the [`oneshot`](Tmp112::oneshot) convenience ties them
//! together with an async wait.

// Reusable SDK driver surface: the full address set and lifecycle methods are
// exposed even though the current app only exercises a subset.

use embedded_hal::i2c::I2c;

/// TMP112 7-bit I²C addresses, selected by the ADD0 pin strap.
pub const ADDR_GND: u8 = 0x48;
/// ADD0 → V+ — the HARDWARIO Core Module strap.
pub const ADDR_VPLUS: u8 = 0x49;
/// ADD0 → SDA.
pub const ADDR_SDA: u8 = 0x4A;
/// ADD0 → SCL.
pub const ADDR_SCL: u8 = 0x4B;

// Pointer-register addresses.
const REG_TEMP: u8 = 0x00;
const REG_CONFIG: u8 = 0x01;

// Configuration register, high byte.
const CFG_OS: u8 = 0x80; // one-shot start / conversion-ready flag
const CFG_SD: u8 = 0x01; // shutdown mode
const CFG_BASE_HI: u8 = 0x60; // R1=R0=1 (12-bit converter), all other bits 0
// Configuration register, low byte: the reset default (CR = 4 Hz, normal mode).
// Irrelevant while shut down; kept so a register read-back matches the datasheet.
const CFG_LO: u8 = 0xA0;

/// A TMP112 on an I²C bus.
pub struct Tmp112<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C: I2c> Tmp112<I2C> {
    /// Create a driver for the TMP112 at `addr` (see the `ADDR_*` constants).
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr }
    }

    /// Put the device in shutdown so it only converts on demand. Optional —
    /// [`trigger_oneshot`](Self::trigger_oneshot) already leaves it shut down
    /// between conversions.
    pub fn shutdown(&mut self) -> Result<(), I2C::Error> {
        self.write_config(CFG_BASE_HI | CFG_SD)
    }

    /// Start a single conversion (OS=1, SD=1). The device performs exactly one
    /// conversion (~26 ms typ) and returns to shutdown on its own.
    pub fn trigger_oneshot(&mut self) -> Result<(), I2C::Error> {
        self.write_config(CFG_BASE_HI | CFG_SD | CFG_OS)
    }

    /// Whether the pending one-shot conversion has finished. In shutdown the OS
    /// bit reads 0 while converting and 1 once the result is ready.
    pub fn conversion_ready(&mut self) -> Result<bool, I2C::Error> {
        let mut cfg = [0u8; 2];
        self.i2c.write_read(self.addr, &[REG_CONFIG], &mut cfg)?;
        Ok(cfg[0] & CFG_OS != 0)
    }

    /// Read the temperature register as a sign-extended 12-bit count
    /// (1 LSB = 0.0625 °C). Convert with [`raw_to_millicelsius`].
    pub fn read_raw(&mut self) -> Result<i16, I2C::Error> {
        let mut b = [0u8; 2];
        self.i2c.write_read(self.addr, &[REG_TEMP], &mut b)?;
        // 12-bit result, left-justified in 16 bits; arithmetic >>4 sign-extends.
        Ok(i16::from_be_bytes(b) >> 4)
    }

    /// Consume the driver and hand the I²C bus back to the caller.
    pub fn release(self) -> I2C {
        self.i2c
    }

    fn write_config(&mut self, hi: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[REG_CONFIG, hi, CFG_LO])
    }
}

/// Convert a raw 12-bit count from [`Tmp112::read_raw`] to milli-degrees Celsius.
///
/// Integer-only (no FPU on the Cortex-M0+): 1 LSB = 0.0625 °C = 125/2 m°C.
pub fn raw_to_millicelsius(raw: i16) -> i32 {
    raw as i32 * 125 / 2
}

// --- Async convenience (embassy-time) ---------------------------------------

use embassy_time::Timer;

/// Conversion-wait tuning for [`Tmp112::oneshot`]. Typical conversion is ~26 ms;
/// we poll the ready flag every `POLL_MS` and give up after `POLL_TRIES` (so the
/// worst-case wait is `POLL_MS * POLL_TRIES`).
const POLL_MS: u64 = 5;
const POLL_TRIES: usize = 10;

impl<I2C: I2c> Tmp112<I2C> {
    /// Trigger a one-shot conversion, wait for it (polling the ready flag), and
    /// return the raw count. I²C is only touched briefly; the wait `await`s so
    /// the executor can run other tasks (or the core can sleep) meanwhile.
    pub async fn oneshot(&mut self) -> Result<i16, I2C::Error> {
        self.trigger_oneshot()?;
        for _ in 0..POLL_TRIES {
            Timer::after_millis(POLL_MS).await;
            if self.conversion_ready()? {
                break;
            }
        }
        self.read_raw()
    }
}
