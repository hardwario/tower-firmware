//! SPIRIT1 chip handle: power/shutdown control, device-ID verification, the
//! MC-state machine (READY / STANDBY / SLEEP, with stuck-state recovery), and the
//! nIRQ-driven async [`tx`](Spirit1::tx) / [`rx`](Spirit1::rx) operations.
//!
//! Register sequencing runs over [`Spirit1Spi`]; the SDN pin (PB7) and the nIRQ
//! line (PA7 `ExtiInput`) are owned here. RF configuration is applied separately
//! (see [`config`](super::config)).

use embassy_futures::select::{Either, select};
use embassy_stm32::Peri;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{PA15, PB3, PB4, PB5, PB7, SPI1};
use embassy_stm32::spi::Error as SpiError;
use embassy_time::{Duration, Instant, Timer};

use super::config::SignalQuality;
use super::regs;
use super::spi::{Spirit1Spi, Status};

/// Largest over-the-air frame (the SPIRIT1 FIFO).
pub const MAX_FRAME: usize = 96;

/// An error from the SPIRIT1 driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadioError {
    /// SPI transport failure (effectively never in blocking master mode).
    Spi(SpiError),
    /// Device ID did not match the expected SPIRIT1 (part 304 / version 48).
    WrongDevice { partnum: u8, version: u8 },
    /// The MC state machine did not reach the requested state in time; carries
    /// the last observed `STATE[6:0]` code.
    StuckState(u8),
    /// CSMA/CCA gave up: the channel stayed busy (max-backoff IRQ).
    Busy,
    /// No completion IRQ within the window, or RX timed out.
    Timeout,
    /// A received packet failed the hardware CRC.
    CrcError,
    /// TX/RX FIFO under/overflow.
    FifoError,
    /// Frame longer than the 96-byte FIFO.
    TooLong,
    /// TX is locked because the reserve-ahead TX-counter watermark could not be
    /// durably persisted (EEPROM full/faulted). Transmitting past the last durable
    /// watermark would, after a reboot that resumes at the stale watermark, reuse a
    /// CCM nonce — so we fail **closed** rather than risk it. Recovers on the next
    /// boot once the watermark persists (free EEPROM space) or after a re-key.
    NonceLocked,
}

impl From<SpiError> for RadioError {
    fn from(e: SpiError) -> Self {
        RadioError::Spi(e)
    }
}

impl core::fmt::Display for RadioError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            RadioError::Spi(_) => f.write_str("SPI transport error"),
            RadioError::WrongDevice { partnum, version } => {
                write!(f, "wrong device id (part {partnum}/{version}, expected SPIRIT1)")
            }
            RadioError::StuckState(code) => write!(f, "radio stuck in state 0x{code:02x}"),
            RadioError::Busy => f.write_str("channel busy (CSMA)"),
            RadioError::Timeout => f.write_str("timeout"),
            RadioError::CrcError => f.write_str("CRC error"),
            RadioError::FifoError => f.write_str("FIFO under/overflow"),
            RadioError::TooLong => f.write_str("frame longer than the 96-byte FIFO"),
            RadioError::NonceLocked => {
                f.write_str("TX locked: could not persist the nonce watermark (EEPROM full/faulted)")
            }
        }
    }
}

/// SPIRIT1 device identity (DEVICE_INFO registers).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceId {
    /// PARTNUM register byte (0xF0); expected 0x01.
    pub partnum: u8,
    /// VERSION register byte (0xF1); expected 0x30 (= 48).
    pub version: u8,
}

impl DeviceId {
    /// The ST library's combined 16-bit part number `(partnum << 8) | version`,
    /// which equals 304 for a genuine SPIRIT1.
    #[must_use]
    pub fn part_number(&self) -> u16 {
        ((self.partnum as u16) << 8) | self.version as u16
    }

    /// Whether this matches the expected SPIRIT1 (304 / 48).
    #[must_use]
    pub fn is_supported(&self) -> bool {
        self.partnum == regs::EXPECT_PARTNUM && self.version == regs::EXPECT_VERSION
    }
}

/// How many 1 ms poll iterations a state transition waits before giving up
/// (mirrors the reference driver's 100 ms budget).
const STATE_POLL_TRIES: u32 = 100;

/// The SPIRIT1 transceiver: SPI transport, the SDN pin, and the nIRQ EXTI line
/// (PA7, active-low) used to await TX/RX completion without busy-polling.
pub struct Spirit1 {
    spi: Spirit1Spi,
    sdn: Output<'static>,
    irq: ExtiInput<'static, Async>,
}

impl Spirit1 {
    /// Build the handle from the board's raw radio resources. The SDN pin starts
    /// **high** (the part stays in SHUTDOWN); call [`exit_shutdown`](Self::exit_shutdown)
    /// to enable it.
    pub fn new(
        spi: Peri<'static, SPI1>,
        sck: Peri<'static, PB3>,
        mosi: Peri<'static, PB5>,
        miso: Peri<'static, PB4>,
        cs: Peri<'static, PA15>,
        sdn: Peri<'static, PB7>,
        irq: ExtiInput<'static, Async>,
    ) -> Self {
        let spi = Spirit1Spi::new(spi, sck, mosi, miso, cs);
        let sdn = Output::new(sdn, Level::High, Speed::Low);
        Self { spi, sdn, irq }
    }

    /// Borrow the SPI transport (for layers that need raw register access, e.g.
    /// [`config`](super::config)).
    pub fn spi(&mut self) -> &mut Spirit1Spi {
        &mut self.spi
    }

    /// Put the radio into SHUTDOWN (register contents lost). Drives SDN high.
    pub fn enter_shutdown(&mut self) {
        self.sdn.set_high();
    }

    /// Bring the radio out of SHUTDOWN: drive SDN low, wait out the POR
    /// (~650 µs SHUTDOWN→READY), and confirm it reaches READY. Toggling SDN
    /// guarantees a clean reset even if it was already low.
    pub async fn exit_shutdown(&mut self) -> Result<(), RadioError> {
        // A defined high→low edge so the POR fires regardless of prior state.
        self.sdn.set_high();
        Timer::after(Duration::from_micros(50)).await;
        self.sdn.set_low();
        // POR + XO settling; the part lands in READY by default.
        Timer::after(Duration::from_millis(2)).await;
        self.wait_for_state(regs::STATE_READY).await
    }

    /// Read and verify the device ID; returns it on success or
    /// [`RadioError::WrongDevice`] on mismatch.
    pub fn read_device_id(&mut self) -> Result<DeviceId, RadioError> {
        let mut b = [0u8; 2];
        self.spi.read_regs(regs::DEVICE_INFO1_PARTNUM, &mut b)?;
        let id = DeviceId {
            partnum: b[0],
            version: b[1],
        };
        if id.is_supported() {
            Ok(id)
        } else {
            Err(RadioError::WrongDevice {
                partnum: id.partnum,
                version: id.version,
            })
        }
    }

    /// Read DEVICE_INFO without verifying (for diagnostics / register dumps).
    pub fn read_device_id_raw(&mut self) -> Result<DeviceId, RadioError> {
        let mut b = [0u8; 2];
        self.spi.read_regs(regs::DEVICE_INFO1_PARTNUM, &mut b)?;
        Ok(DeviceId {
            partnum: b[0],
            version: b[1],
        })
    }

    /// Read the current MC `STATE[6:0]` code (see the `STATE_*` consts in
    /// [`regs`])).
    pub fn mc_state(&mut self) -> Result<u8, RadioError> {
        let (v, _) = self.spi.read_reg(regs::MC_STATE0)?;
        Ok(regs::state_from_status(v))
    }

    /// Send a command strobe; returns the status bytes (`status[0]` carries the
    /// MC state at transaction start).
    pub fn command(&mut self, cmd: u8) -> Result<Status, RadioError> {
        Ok(self.spi.command(cmd)?)
    }

    /// Transition to READY from any state (strobe READY from STANDBY/SLEEP,
    /// SABORT from TX/RX), then wait for it.
    pub async fn to_ready(&mut self) -> Result<(), RadioError> {
        let state = self.mc_state()?;
        if state == regs::STATE_READY {
            return Ok(());
        }
        match state {
            regs::STATE_STANDBY | regs::STATE_SLEEP => {
                self.spi.command(regs::CMD_READY)?;
            }
            regs::STATE_TX | regs::STATE_RX | regs::STATE_LOCK => {
                self.spi.command(regs::CMD_SABORT)?;
            }
            _ => {
                // Transient/unknown — a SABORT is always safe to break a deadlock.
                self.spi.command(regs::CMD_SABORT)?;
            }
        }
        self.wait_for_state(regs::STATE_READY).await
    }

    /// Transition to STANDBY (lowest-power state that retains config).
    pub async fn to_standby(&mut self) -> Result<(), RadioError> {
        if self.mc_state()? == regs::STATE_STANDBY {
            return Ok(());
        }
        self.to_ready().await?;
        self.spi.command(regs::CMD_STANDBY)?;
        self.wait_for_state(regs::STATE_STANDBY).await
    }

    /// Transition to SLEEP (wake-timer state).
    ///
    /// **Deasserts nIRQ first (mask all + clear status).** The SPIRIT1 holds its GPIO0
    /// nIRQ line asserted (low) as long as any *unmasked* IRQ event bit is set, and the
    /// bit survives into SLEEP. On a battery node the nIRQ pin is an STM32 EXTI wake
    /// source (PA7): a stuck-low nIRQ keeps the EXTI pending, so embassy's low-power
    /// executor wakes out of STOP immediately and busy-runs at mA instead of idling at
    /// µA — the radio itself is asleep, but the *MCU* never STOPs (bench-measured
    /// 2026-07-11: a node locked at ~9.6 mA after its first post-STOP uplink, radio
    /// confirmed in SLEEP the whole time). Masking every IRQ and reading the
    /// read-to-clear status word forces nIRQ high before we sleep, so the pin can't
    /// veto STOP. Re-armed by [`Self::rx`]/[`Self::tx`], which set their own masks.
    pub async fn to_sleep(&mut self) -> Result<(), RadioError> {
        self.set_irq_mask(0)?;
        let _ = self.irq_status();
        if self.mc_state()? == regs::STATE_SLEEP {
            return Ok(());
        }
        self.to_ready().await?;
        self.spi.command(regs::CMD_SLEEP)?;
        self.wait_for_state(regs::STATE_SLEEP).await
    }

    /// Program the SLEEP/LDC wake-up timer: period ≈ `prescaler × counter / f_rco`
    /// (the RC oscillator is ~34.7 kHz). The SLEEP state needs a non-zero counter
    /// to hold — at the reset value (counter = 0) the timer expires immediately and
    /// the part bounces straight back to READY.
    pub fn set_wake_timer(&mut self, prescaler: u8, counter: u8) -> Result<(), RadioError> {
        self.spi.write_reg(regs::TIMERS3_LDC_PRESCALER, prescaler)?;
        self.spi.write_reg(regs::TIMERS2_LDC_COUNTER, counter)?;
        Ok(())
    }

    /// Flush both FIFOs (e.g. on FIFO-error recovery).
    pub fn flush_fifos(&mut self) -> Result<(), RadioError> {
        self.spi.command(regs::CMD_FLUSHRXFIFO)?;
        self.spi.command(regs::CMD_FLUSHTXFIFO)?;
        Ok(())
    }

    /// Configure GPIO0 as the active-low nIRQ output (digital output, low power).
    pub fn configure_irq_gpio(&mut self) -> Result<(), RadioError> {
        self.spi.write_reg(regs::GPIO0_CONF, regs::GPIO_CONF_IRQ)?;
        Ok(())
    }

    /// Set the IRQ mask (32-bit INT_MASK; bits per [`regs`] `IRQ_*`).
    /// A 1 routes that event to nIRQ.
    pub fn set_irq_mask(&mut self, mask: u32) -> Result<(), RadioError> {
        // IRQ_MASK3..0 hold INT_MASK[31:24]..[7:0]; write MSB-first at 0x90.
        let bytes = mask.to_be_bytes();
        self.spi.write_regs(regs::IRQ_MASK3, &bytes)?;
        Ok(())
    }

    /// Read and clear the 32-bit IRQ status word (IRQ_STATUS is read-and-reset).
    pub fn irq_status(&mut self) -> Result<u32, RadioError> {
        let mut b = [0u8; 4];
        self.spi.read_regs(regs::IRQ_STATUS3, &mut b)?;
        // IRQ_STATUS3..0 = INT_EVENT[31:24]..[7:0].
        Ok(u32::from_be_bytes(b))
    }

    /// Enable/disable the unmodulated CW carrier (for bring-up / range testing).
    ///
    /// On: go to READY, set TXSOURCE=PN9 (so the TX state stays continuous instead
    /// of underflowing an empty FIFO), set MOD0.CW (constant tone), strobe TX.
    /// Off: SABORT, clear MOD0.CW, restore TXSOURCE=normal, back to READY.
    pub async fn cw_test(&mut self, on: bool) -> Result<(), RadioError> {
        if on {
            self.to_ready().await?;
            self.smps_for(false)?;
            let (p1, _) = self.spi.read_reg(regs::PCKTCTRL1)?;
            self.spi.write_reg(regs::PCKTCTRL1, p1 | 0x0C)?; // TXSOURCE = PN9
            let (mod0, _) = self.spi.read_reg(regs::MOD0)?;
            self.spi.write_reg(regs::MOD0, mod0 | 0x80)?; // CW bit
            self.spi.command(regs::CMD_TX)?;
        } else {
            self.spi.command(regs::CMD_SABORT)?;
            let (mod0, _) = self.spi.read_reg(regs::MOD0)?;
            self.spi.write_reg(regs::MOD0, mod0 & !0x80)?;
            let (p1, _) = self.spi.read_reg(regs::PCKTCTRL1)?;
            self.spi.write_reg(regs::PCKTCTRL1, p1 & !0x0C)?; // TXSOURCE = normal
            self.to_ready().await?;
        }
        Ok(())
    }

    /// Set the SMPS switching frequency for the upcoming mode (ST WaCmdStrobe):
    /// PM_CONFIG1 = 0x98 for RX, 0x20 for TX. Running RX with the TX value (the
    /// reset default) injects SMPS spurs into the RX band and breaks demodulation.
    fn smps_for(&mut self, rx: bool) -> Result<(), RadioError> {
        self.spi
            .write_reg(regs::PM_CONFIG1, if rx { 0x98 } else { 0x20 })?;
        if !rx {
            self.spi.write_reg(0xA9, 0x11)?; // enable VCO_L buffer for TX
        }
        Ok(())
    }

    /// Enter persistent RX (flush the RX FIFO first). Used for raw RSSI reads and
    /// the bring-up sniffer; the full IRQ-driven RX lives in the driver layer.
    pub async fn enter_rx(&mut self) -> Result<(), RadioError> {
        self.to_ready().await?;
        self.smps_for(true)?;
        self.spi.command(regs::CMD_FLUSHRXFIFO)?;
        self.spi.command(regs::CMD_RX)?;
        Ok(())
    }

    /// Read the raw RSSI register (0.5 dB/step). NB: RSSI_LEVEL only latches when
    /// the part leaves RX (SABORT / RX-timeout / sync-detect), not continuously —
    /// use [`rssi_sample`](Self::rssi_sample) for an on-demand channel reading.
    pub fn rssi_raw(&mut self) -> Result<u8, RadioError> {
        let (v, _) = self.spi.read_reg(regs::RSSI_LEVEL)?;
        Ok(v)
    }

    /// Sample the channel RSSI on demand: enter RX, let the measurement settle,
    /// then SABORT (which latches RSSI_LEVEL) and read it. Returns the raw value.
    pub async fn rssi_sample(&mut self) -> Result<u8, RadioError> {
        self.enter_rx().await?;
        Timer::after(Duration::from_millis(20)).await;
        self.spi.command(regs::CMD_SABORT)?;
        self.rssi_raw()
    }

    /// Whether the calibrator flagged a lock error (MC_STATE1.ERROR_LOCK bit0).
    pub fn error_lock(&mut self) -> Result<bool, RadioError> {
        let (v, _) = self.spi.read_reg(regs::MC_STATE1)?;
        Ok(v & 0x01 != 0)
    }

    /// Whether the nIRQ line is currently asserted (active-low → reads low).
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.irq.is_low()
    }

    /// Await the nIRQ line going low (an unmasked SPIRIT1 event). Returns
    /// immediately if it is already low. (Diagnostics / driver use.)
    pub async fn wait_irq(&mut self) {
        self.irq.wait_for_low().await;
    }

    /// Number of bytes currently in the linear TX FIFO (0..96).
    pub fn tx_fifo_count(&mut self) -> Result<u8, RadioError> {
        let (v, _) = self.spi.read_reg(regs::LINEAR_FIFO_STATUS1_TX)?;
        Ok(v & 0x7F)
    }

    /// Number of bytes currently in the linear RX FIFO (0..96).
    pub fn rx_fifo_count(&mut self) -> Result<u8, RadioError> {
        let (v, _) = self.spi.read_reg(regs::LINEAR_FIFO_STATUS0_RX)?;
        Ok(v & 0x7F)
    }

    /// Read PQI (packet quality) and SQI (sync quality) for the last reception.
    pub fn link_quality(&mut self) -> Result<(u8, u8), RadioError> {
        let (pqi, _) = self.spi.read_reg(regs::LINK_QUALIF2_PQI)?;
        let (sqi_raw, _) = self.spi.read_reg(regs::LINK_QUALIF1_SQI)?;
        Ok((pqi, sqi_raw & 0x7F)) // SQI is bits[6:0]; bit7 is CS
    }

    /// Read the AFC correction word of the last packet (signed). Scale to Hz with
    /// the data-rate-dependent factor in [`config`](super::config) when needed.
    pub fn afc_corr(&mut self) -> Result<i8, RadioError> {
        let (v, _) = self.spi.read_reg(regs::AFC_CORR)?;
        Ok(v as i8)
    }

    /// Length (bytes) of the most recently received packet (RX_PCKT_LEN).
    pub fn rx_packet_len(&mut self) -> Result<u16, RadioError> {
        let mut b = [0u8; 2];
        self.spi.read_regs(regs::RX_PCKT_LEN1, &mut b)?;
        Ok((b[0] as u16) << 8 | b[1] as u16)
    }

    /// Enable/disable the CSMA (CCA) engine (PROTOCOL1.CSMA_ON). The CSMA timing
    /// parameters are programmed by [`config`](super::config); here we just gate it.
    pub fn set_csma(&mut self, on: bool) -> Result<(), RadioError> {
        let (p1, _) = self.spi.read_reg(regs::PROTOCOL1)?;
        let v = if on { p1 | 0x04 } else { p1 & !0x04 };
        self.spi.write_reg(regs::PROTOCOL1, v)?;
        Ok(())
    }

    /// Read the full signal-quality set for the most recent reception.
    fn read_quality(&mut self) -> Result<SignalQuality, RadioError> {
        let rssi_raw = self.rssi_raw()?;
        let (pqi, sqi) = self.link_quality()?;
        let afc = self.afc_corr()?;
        Ok(SignalQuality {
            rssi: super::config::rssi_to_dbm(rssi_raw),
            rssi_raw,
            lqi: pqi,
            sqi,
            afc_raw: afc,
        })
    }

    /// Transmit one frame (≤96 B), optionally preceded by CSMA/CCA. Loads the
    /// FIFO, strobes TX, and waits on nIRQ for TX-done (or CCA-busy / FIFO-error /
    /// `timeout`). The variable-length packet's length field is set automatically.
    pub async fn tx(&mut self, data: &[u8], use_csma: bool, timeout: Duration) -> Result<(), RadioError> {
        if data.len() > MAX_FRAME {
            return Err(RadioError::TooLong);
        }
        self.to_ready().await?;
        self.set_irq_mask(regs::IRQ_TX_DATA_SENT | regs::IRQ_MAX_BO_CCA_REACH | regs::IRQ_TX_FIFO_ERROR)?;
        self.set_csma(use_csma)?;
        self.smps_for(false)?;
        self.spi.command(regs::CMD_FLUSHTXFIFO)?;
        self.spi.write_regs(regs::PCKTLEN1, &[0, data.len() as u8])?;
        self.spi.write_fifo(data)?;
        let _ = self.irq_status(); // clear -> nIRQ de-asserts
        self.spi.command(regs::CMD_TX)?;

        let outcome = match select(self.irq.wait_for_low(), Timer::after(timeout)).await {
            Either::First(_) => {
                let s = self.irq_status()?;
                if s & regs::IRQ_TX_DATA_SENT != 0 {
                    Ok(())
                } else if s & regs::IRQ_MAX_BO_CCA_REACH != 0 {
                    Err(RadioError::Busy)
                } else if s & regs::IRQ_TX_FIFO_ERROR != 0 {
                    let _ = self.flush_fifos();
                    Err(RadioError::FifoError)
                } else {
                    Err(RadioError::Timeout)
                }
            }
            Either::Second(_) => {
                let _ = self.command(regs::CMD_SABORT);
                Err(RadioError::Timeout)
            }
        };
        if use_csma {
            let _ = self.set_csma(false);
        }
        outcome
    }

    /// Receive one frame into `buf`, waiting up to `timeout` for a packet. Returns
    /// the length and signal quality. CRC failures, FIFO errors and oversize
    /// frames are reported; filtered/discarded packets re-arm RX transparently.
    pub async fn rx(
        &mut self,
        buf: &mut [u8],
        timeout: Duration,
    ) -> Result<(usize, SignalQuality), RadioError> {
        self.to_ready().await?;
        self.smps_for(true)?;
        self.spi.command(regs::CMD_FLUSHRXFIFO)?;
        self.set_irq_mask(
            regs::IRQ_RX_DATA_READY | regs::IRQ_RX_DATA_DISC | regs::IRQ_CRC_ERROR | regs::IRQ_RX_FIFO_ERROR,
        )?;
        let _ = self.irq_status();
        self.spi.command(regs::CMD_RX)?;

        // Absolute deadline: a stream of filtered/discarded frames re-arms RX
        // (below) but must NOT keep restarting the timeout, or a busy channel
        // would starve the caller's ACK/bulk window.
        let deadline = Instant::now() + timeout;
        loop {
            match select(self.irq.wait_for_low(), Timer::at(deadline)).await {
                Either::First(_) => {
                    let s = self.irq_status()?;
                    if s & regs::IRQ_RX_DATA_READY != 0 {
                        let len = self.rx_packet_len()? as usize;
                        if len > buf.len() || len > MAX_FRAME {
                            let _ = self.command(regs::CMD_SABORT);
                            let _ = self.flush_fifos();
                            return Err(RadioError::TooLong);
                        }
                        self.spi.read_fifo(&mut buf[..len])?;
                        let q = self.read_quality()?;
                        let _ = self.command(regs::CMD_SABORT);
                        return Ok((len, q));
                    } else if s & regs::IRQ_CRC_ERROR != 0 {
                        let _ = self.flush_fifos();
                        let _ = self.command(regs::CMD_SABORT);
                        return Err(RadioError::CrcError);
                    } else if s & regs::IRQ_RX_FIFO_ERROR != 0 {
                        let _ = self.flush_fifos();
                        let _ = self.command(regs::CMD_SABORT);
                        return Err(RadioError::FifoError);
                    } else {
                        // Discarded/filtered or spurious: re-arm RX and keep waiting.
                        self.spi.command(regs::CMD_FLUSHRXFIFO)?;
                        self.spi.command(regs::CMD_RX)?;
                    }
                }
                Either::Second(_) => {
                    let _ = self.command(regs::CMD_SABORT);
                    return Err(RadioError::Timeout);
                }
            }
        }
    }

    /// Poll MC_STATE until it equals `target` or the budget expires.
    async fn wait_for_state(&mut self, target: u8) -> Result<(), RadioError> {
        let mut last = 0xFF;
        for _ in 0..STATE_POLL_TRIES {
            last = self.mc_state()?;
            if last == target {
                return Ok(());
            }
            Timer::after(Duration::from_millis(1)).await;
        }
        Err(RadioError::StuckState(last))
    }
}
