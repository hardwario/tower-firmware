//! LIS2DH12 3-axis accelerometer driver (I²C).
//!
//! HAL-independent: the register-level methods are generic over [`embedded_hal::i2c::I2c`],
//! like [`tmp112`](crate::tmp112). (The tilt-report `min_interval` throttle uses `embassy-time`.)
//! Configured for the firmware's defaults: **normal mode (10-bit), 10 Hz ODR,
//! ±2 g** full scale — plenty for orientation/tilt and low power.
//!
//! Two features over the raw readings:
//!   * **Orientation / "dice"** — [`Accel::dice`] maps a reading to which face is
//!     up (1–6, opposite faces summing to 7), or `None` while the device is moving
//!     or held at an angle. Uses the gravity-vector method from the HARDWARIO
//!     reference firmware (a face is accepted when every axis is within 0.6 g of
//!     the corresponding axis-aligned unit vector).
//!   * **Tilt / manipulation alert** — [`enable_tilt`](Lis2dh12::enable_tilt)
//!     programs the chip's hardware interrupt generator (high-pass filtered, so
//!     static gravity is removed and only *movement* triggers) and routes it to
//!     the **INT1** pin. On the Core Module INT1 is wired to PB6, so the MCU can
//!     sleep and be woken by an EXTI edge; [`tilt_triggered`](Lis2dh12::tilt_triggered)
//!     reads and clears the latched source. Sensitivity is selectable via
//!     [`Sensitivity`].
//!
//! ```ignore
//! let mut accel = Lis2dh12::new(i2c, lis2dh12::ADDR_DEFAULT);
//! accel.init()?;                                       // 10 Hz, normal mode, ±2 g
//! accel.enable_tilt(TiltConfig::new(Sensitivity::Medium))?; // optional movement IRQ
//! let face = accel.read()?.dice();                     // Some(1..=6) or None
//! ```

// Reusable SDK driver surface: the full register/feature set is exposed even if
// an app uses a subset.

use embassy_time::{Duration, Instant};
use embedded_hal::i2c::I2c;

/// 7-bit I²C address with SA0/SDO tied high — the HARDWARIO Core Module strap.
pub const ADDR_DEFAULT: u8 = 0x19;
/// Address with SA0/SDO tied low.
pub const ADDR_SA0_LOW: u8 = 0x18;
/// Expected value of the `WHO_AM_I` register.
pub const WHO_AM_I_ID: u8 = 0x33;

// Register map (subset).
const WHO_AM_I: u8 = 0x0F;
const CTRL_REG1: u8 = 0x20;
const CTRL_REG2: u8 = 0x21;
const CTRL_REG3: u8 = 0x22;
const CTRL_REG4: u8 = 0x23;
const CTRL_REG5: u8 = 0x24;
const REFERENCE: u8 = 0x26;
const STATUS_REG: u8 = 0x27;
const OUT_X_L: u8 = 0x28;
const INT1_CFG: u8 = 0x30;
const INT1_SRC: u8 = 0x31;
const INT1_THS: u8 = 0x32;
const INT1_DURATION: u8 = 0x33;

/// Set in a sub-address to auto-increment the register pointer (burst reads).
const AUTO_INCREMENT: u8 = 0x80;

// CTRL_REG1: ODR=10 Hz (0b0010), normal/high-res select via LPen=0, X/Y/Z on.
const CTRL1_10HZ_XYZ: u8 = 0b0010_0111;
// CTRL_REG4: BDU=1 (coherent high/low byte), FS=±2 g (00), HR=0 -> normal 10-bit.
const CTRL4_BDU_2G_NORMAL: u8 = 0b1000_0000;

// Tilt (movement) interrupt wiring.
const CTRL2_HPF_INT1: u8 = 0b0000_0001; // HPIS1: high-pass the INT1 generator
const CTRL3_I1_IA1: u8 = 0b0100_0000; // route interrupt-activity 1 to INT1 pin
const CTRL5_LIR_INT1: u8 = 0b0000_1000; // latch INT1 until INT1_SRC is read
const INT1_CFG_HIGH_OR: u8 = 0b0010_1010; // OR of X/Y/Z "high" events (movement)
const INT1_SRC_IA: u8 = 0b0100_0000; // interrupt-active flag in INT1_SRC

/// Sensitivity of the [tilt](Lis2dh12::enable_tilt) (movement) interrupt: the
/// motion threshold, where 1 LSB = 16 mg at ±2 g. Lower threshold = more sensitive.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Sensitivity {
    /// ~768 mg — only a firm shake or knock.
    Low,
    /// ~384 mg — a deliberate pick-up.
    Medium,
    /// ~192 mg — a gentle nudge.
    High,
    /// ~96 mg — a light touch.
    Ultra,
}

impl Sensitivity {
    /// Threshold written to `INT1_THS` (16 mg/LSB at ±2 g).
    const fn threshold(self) -> u8 {
        match self {
            Sensitivity::Low => 48,
            Sensitivity::Medium => 24,
            Sensitivity::High => 12,
            Sensitivity::Ultra => 6,
        }
    }
}

/// Configuration for the [tilt](Lis2dh12::enable_tilt) (movement) interrupt.
#[derive(Clone, Copy)]
pub struct TiltConfig {
    /// Motion threshold — see [`Sensitivity`].
    pub sensitivity: Sensitivity,
    /// Minimum time between *reported* tilts. During continuous motion the
    /// interrupt re-fires every sample; triggers within this window are still
    /// cleared from the hardware but not reported, so a single shake yields one
    /// event instead of a flood. The default is 500 ms.
    pub min_interval: Duration,
}

impl TiltConfig {
    /// Tilt config with the given sensitivity and the default 500 ms `min_interval`.
    pub const fn new(sensitivity: Sensitivity) -> Self {
        Self {
            sensitivity,
            min_interval: Duration::from_millis(500),
        }
    }
}

impl Default for TiltConfig {
    fn default() -> Self {
        Self::new(Sensitivity::Medium)
    }
}

/// A 3-axis acceleration sample, in **milli-g** (1000 = 1 g).
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct Accel {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// 1 g, in milli-g.
const G: i32 = 1000;
/// Orientation slack (0.4 g), matching the reference firmware's `ORIENTATION_THR`.
/// A face is accepted when each axis is within `G - THR` = **0.6 g** of the
/// axis-aligned unit vector (the larger this slack, the *tighter* the window).
const ORIENT_THR: i32 = 400;

impl Accel {
    /// Which die face is up: `Some(1..=6)`, or `None` when the device is moving
    /// or tilted (no axis clearly aligned with gravity). Mapping matches the
    /// HARDWARIO `twr_dice` table (opposite faces sum to 7):
    /// 1 = Z+, 2 = X+, 3 = Y+, 4 = Y-, 5 = X-, 6 = Z-.
    pub fn dice(&self) -> Option<u8> {
        let (x, y, z) = (self.x as i32, self.y as i32, self.z as i32);
        let near = G - ORIENT_THR; // 600 mg
        let aligned = |v: i32, target: i32| (v - target).abs() < near;
        let face = |fx: i32, fy: i32, fz: i32| aligned(x, fx) && aligned(y, fy) && aligned(z, fz);

        if face(0, 0, G) {
            Some(1)
        } else if face(G, 0, 0) {
            Some(2)
        } else if face(0, G, 0) {
            Some(3)
        } else if face(0, -G, 0) {
            Some(4)
        } else if face(-G, 0, 0) {
            Some(5)
        } else if face(0, 0, -G) {
            Some(6)
        } else {
            None
        }
    }
}

/// A LIS2DH12 on an I²C bus.
pub struct Lis2dh12<I2C> {
    i2c: I2C,
    addr: u8,
    min_interval: Duration,
    last_tilt: Option<Instant>,
}

impl<I2C: I2c> Lis2dh12<I2C> {
    /// Create a driver for the device at `addr` (see [`ADDR_DEFAULT`]).
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            min_interval: TiltConfig::new(Sensitivity::Medium).min_interval,
            last_tilt: None,
        }
    }

    /// Read `WHO_AM_I`; compare against [`WHO_AM_I_ID`] to confirm the part.
    pub fn who_am_i(&mut self) -> Result<u8, I2C::Error> {
        self.read_reg(WHO_AM_I)
    }

    /// Bring the device up in the firmware's standard mode: 10 Hz ODR, normal
    /// mode (10-bit), ±2 g, block-data-update on.
    pub fn init(&mut self) -> Result<(), I2C::Error> {
        self.write_reg(CTRL_REG4, CTRL4_BDU_2G_NORMAL)?;
        self.write_reg(CTRL_REG1, CTRL1_10HZ_XYZ)
    }

    /// Read one acceleration sample (milli-g) from the output registers.
    pub fn read(&mut self) -> Result<Accel, I2C::Error> {
        let mut b = [0u8; 6];
        self.i2c
            .write_read(self.addr, &[OUT_X_L | AUTO_INCREMENT], &mut b)?;
        Ok(Accel {
            x: to_mg(b[0], b[1]),
            y: to_mg(b[2], b[3]),
            z: to_mg(b[4], b[5]),
        })
    }

    /// Whether a new sample is ready (`STATUS_REG` ZYXDA bit).
    pub fn data_ready(&mut self) -> Result<bool, I2C::Error> {
        Ok(self.read_reg(STATUS_REG)? & 0x08 != 0)
    }

    /// Enable the **tilt / manipulation** interrupt: a high-pass-filtered movement
    /// detector (gravity removed) at the config's [`Sensitivity`], latched and
    /// routed to the INT1 pin, with the config's reporting [`min_interval`](TiltConfig::min_interval).
    /// After this, an INT1 edge means "device moved"; call
    /// [`tilt_triggered`](Self::tilt_triggered) to confirm and clear it.
    pub fn enable_tilt(&mut self, config: TiltConfig) -> Result<(), I2C::Error> {
        self.min_interval = config.min_interval;
        self.last_tilt = None;
        self.write_reg(CTRL_REG2, CTRL2_HPF_INT1)?; // HPF the INT1 path
        self.write_reg(CTRL_REG3, CTRL3_I1_IA1)?; // IA1 -> INT1 pin (active high)
        self.write_reg(CTRL_REG5, CTRL5_LIR_INT1)?; // latch until INT1_SRC read
        self.write_reg(INT1_THS, config.sensitivity.threshold())?;
        self.write_reg(INT1_DURATION, 0)?; // fire on the first over-threshold sample
        let _ = self.read_reg(REFERENCE)?; // reset the high-pass filter to "now"
        self.write_reg(INT1_CFG, INT1_CFG_HIGH_OR)?; // enable, last per the datasheet
        let _ = self.read_reg(INT1_SRC)?; // clear any startup latch
        Ok(())
    }

    /// Turn the tilt interrupt off (mask INT1's generator and unroute it).
    pub fn disable_tilt(&mut self) -> Result<(), I2C::Error> {
        self.write_reg(INT1_CFG, 0)?;
        self.write_reg(CTRL_REG3, 0)
    }

    /// Read and clear the latched INT1 source, applying the configured
    /// [`min_interval`](TiltConfig::min_interval). Returns `true` for a fresh,
    /// reportable tilt; `false` if none fired or it fell within the interval. The
    /// hardware latch is *always* cleared (reading `INT1_SRC` re-arms the pin), so
    /// continuous motion is throttled to one event per interval rather than a flood.
    pub fn tilt_triggered(&mut self) -> Result<bool, I2C::Error> {
        if self.read_reg(INT1_SRC)? & INT1_SRC_IA == 0 {
            return Ok(false);
        }
        let now = Instant::now();
        if let Some(prev) = self.last_tilt
            && now.saturating_duration_since(prev) < self.min_interval
        {
            return Ok(false); // within the min interval — cleared but not reported
        }
        self.last_tilt = Some(now);
        Ok(true)
    }

    /// Consume the driver and hand the I²C bus back to the caller.
    pub fn release(self) -> I2C {
        self.i2c
    }

    fn read_reg(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut b = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg], &mut b)?;
        Ok(b[0])
    }

    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[reg, val])
    }
}

/// Convert a little-endian, left-justified output register pair to milli-g for
/// normal mode (10-bit, ±2 g → 4 mg per count after the 6-bit right shift).
fn to_mg(lo: u8, hi: u8) -> i16 {
    let raw = i16::from_le_bytes([lo, hi]) >> 6; // 10-bit signed count
    (raw as i32 * 4) as i16
}
