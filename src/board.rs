//! HARDWARIO TOWER Core Module board support.
//!
//! Most apps use [`Board::take`] via the [`app!`](crate::app) macro and never touch the
//! lower-level [`init`]. `take` performs the *always on* setup — clocks, the USB-gated
//! serial [`console`](crate::console), and putting the TMP112 into shutdown (one-shot)
//! mode so it does not free-run and waste power — then hands the app a [`Board`] of
//! ready-to-use resources.

use embassy_executor::Spawner;
use embassy_stm32::exti::{ExtiInput, InterruptHandler};
use embassy_stm32::flash::Flash;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::peripherals::{DMA1_CH3, IWDG, PA1, PA15, PB3, PB4, PB5, PB7, PH1, SPI1, TIM2};
use embassy_stm32::rcc::{LsConfig, Sysclk};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Peri, Peripherals, bind_interrupts, interrupt};
use embassy_time::Duration;
use embedded_hal::i2c::{ErrorType, I2c as I2cTrait, Operation};
use log::LevelFilter;

use crate::storage::{Nv, Storage};
use crate::tmp112::{self, Tmp112};

// PA8 (button), PA12 (VBUS_SENSE) and PA7 (SPIRIT1 nIRQ) are EXTI lines 8/12/7
// — all on EXTI4_15, no line collision (PB6 accel INT1 is line 6, also here).
bind_interrupts!(struct Irqs {
    EXTI4_15 => InterruptHandler<interrupt::typelevel::EXTI4_15>;
});
// The USART1 interrupt (console UART) is bound in `console` (`ConsoleIrqs`), which owns
// the UART and rebuilds it dynamically as USB is plugged/unplugged.

/// Default console verbosity set by [`Board::take`]; lower it at runtime with
/// `log::set_max_level`.
const LOG_LEVEL: LevelFilter = LevelFilter::Trace;

// I2C2 addresses of the always-on bus devices `Board::take` quiesces at boot.
const ACCEL_ADDR: u8 = 0x19; // LIS2DH12 accelerometer (SA0 tied high)
const LIS2DH12_CTRL_REG1: u8 = 0x20; // ODR/enable; 0x00 = power-down
const ATSHA_ADDR: u8 = 0x64; // ATSHA204A crypto (word address 0x01 = Sleep)

/// The board's shared I²C2 bus (blocking), wrapped in an [`AtshaGuard`] so every
/// transaction automatically re-sleeps the ATSHA204A. This is the bus type the app
/// receives (via [`Board::tmp112`] and `release()`).
pub type GuardedI2c = AtshaGuard<I2c<'static, Blocking, Master>>;

/// The board's TMP112 on the (ATSHA-guarded) I²C2 bus (blocking), in shutdown / one-shot mode.
pub type Tmp112Sensor = Tmp112<GuardedI2c>;

/// Ready-to-use board resources handed to an app by [`Board::take`].
///
/// The console is already running and the [`tmp112`](Board::tmp112) sensor is in
/// shutdown mode (call `oneshot()` to read). The remaining fields are the
/// board's user peripherals — build the blocks you need from them.
pub struct Board {
    /// Task spawner for the running executor.
    pub spawner: Spawner,
    /// TMP112 temperature sensor, shut down and ready for `oneshot()` reads.
    pub tmp112: Tmp112Sensor,
    /// On-board LED pin (PH1, active-high) — pass to [`led::init`](crate::led::init).
    pub led: Peri<'static, PH1>,
    /// On-board button (PA8, active-high, pull-down), EXTI-bound — pass to
    /// [`button::init_exti`](crate::button::init_exti) with [`Polarity::ActiveHigh`](crate::button::Polarity::ActiveHigh).
    pub button: ExtiInput<'static, Async>,
    /// WS2812 strip timer (TIM2) — pass to [`strip::Strip::new`](crate::strip::Strip::new).
    pub strip_tim: Peri<'static, TIM2>,
    /// WS2812 strip data pin (PA1).
    pub strip_data: Peri<'static, PA1>,
    /// WS2812 strip DMA channel (DMA1_CH3).
    pub strip_dma: Peri<'static, DMA1_CH3>,
    /// LIS2DH12 accelerometer INT1 line (PB6, active-high, pull-down), EXTI-bound
    /// — for the tilt/movement interrupt. The accelerometer shares the I²C2 bus
    /// with the TMP112; reclaim it with [`tmp112`](Self::tmp112)`.release()`.
    pub accel_int: ExtiInput<'static, Async>,
    /// The one shared key-value store over the data EEPROM — see [`storage`](crate::storage).
    /// `Copy`; hand the same handle to `Net`, the [`shell`](crate::shell), and the app at once
    /// (each call locks the one store).
    pub kv: Nv,
    /// Independent watchdog peripheral — **opt-in**. Pass to
    /// [`watchdog::enable`](crate::watchdog::enable) to arm a hardware reset on a firmware hang;
    /// apps that don't want it simply ignore this field. The IWDG runs from the LSI and keeps
    /// counting in STOP, so the feeder must pet it during idle (see the `watchdog` module).
    pub iwdg: Peri<'static, IWDG>,

    // --- SPIRIT1 sub-GHz radio (SPSGRF module) — see [`radio`](crate::radio). ---
    /// SPIRIT1 shutdown pin (PB7). Has a 1 MΩ hardware pull-up so the part boots
    /// into SHUTDOWN; the driver must drive it **low** to enable the radio.
    pub radio_sdn: Peri<'static, PB7>,
    /// SPIRIT1 SPI chip-select (PA15) — **software-controlled** (≥2 µs setup), so
    /// the radio driver owns it as a GPIO output, not the SPI peripheral's NSS.
    pub radio_cs: Peri<'static, PA15>,
    /// SPI1 peripheral for the radio bus (≤10 MHz, mode 0). Blocking — see the
    /// `"spi"` feature note in Cargo.toml.
    pub radio_spi: Peri<'static, SPI1>,
    /// SPI1 SCLK (PB3).
    pub radio_sck: Peri<'static, PB3>,
    /// SPI1 MOSI (PB5).
    pub radio_mosi: Peri<'static, PB5>,
    /// SPI1 MISO (PB4).
    pub radio_miso: Peri<'static, PB4>,
    /// SPIRIT1 GPIO0 / nIRQ (PA7, active-low), EXTI line 7 (on the bound
    /// `EXTI4_15` group) — wakes the driver on radio events. Pulled up so the
    /// idle (de-asserted) level is defined before the SPIRIT1 drives it.
    pub radio_irq: ExtiInput<'static, Async>,
}

/// Boot indicator timings — the common HARDWARIO TOWER power-on signature every
/// board plays before its application starts (driven by [`Board::boot_indicator`],
/// called from the [`app!`](crate::app) macro). Total 3 s.
const BOOT_SETTLE_MS: u64 = 500; // do nothing first — let the supply settle (a
// battery can bounce on its contacts at insertion), so a brown-out during this
// window re-runs cleanly rather than mid-application.
const BOOT_ON_MS: u64 = 2000; // LED solid on — "powered, booting".
const BOOT_TAIL_MS: u64 = 500; // LED off — a clear gap before the app takes the LED.

impl Board {
    /// Play the common TOWER power-on LED signature on the board LED (PH1,
    /// active-high): **500 ms off → 2 s on → 500 ms off**, then return. Called by
    /// the [`app!`](crate::app) macro after the SDK is up and before the app's
    /// `run`, so every board — whatever the app — shows the same 3 s boot
    /// indication. Borrows the LED pin for the duration and hands it back, so the
    /// app still owns `board.led` for its own [`led::init`](crate::led::init).
    ///
    /// Async, not blocking: the awaits let the console come up and drain the boot
    /// backlog during the signature (the `Hello`/boot logs are emitted meanwhile),
    /// so only the *application* is delayed by the 3 s, not the host link.
    pub async fn boot_indicator(&mut self) {
        let mut led = Output::new(self.led.reborrow(), Level::Low, Speed::Low);
        embassy_time::Timer::after_millis(BOOT_SETTLE_MS).await;
        led.set_high();
        embassy_time::Timer::after_millis(BOOT_ON_MS).await;
        led.set_low();
        embassy_time::Timer::after_millis(BOOT_TAIL_MS).await;
        // `led` drops here → the pin is released back to `self.led` for the app.
    }

    /// Initialise the board and return its resources. Call once at start-up —
    /// typically via [`app!`](crate::app). Performs the always-on setup: clocks,
    /// the serial console, TMP112 shutdown, and **USB-aware power management**
    /// (see below).
    ///
    /// The USB-presence-gated [`console::manager`](crate::console::manager) is spawned
    /// automatically, so every app gets the same low-power policy without wiring it up:
    /// the console UART is up (and interrupts responsive) while USB is connected, and is
    /// torn down when unplugged so the executor can reach STOP and idle at µA.
    pub fn take(spawner: Spawner) -> Self {
        let p = init();

        // Dynamic, USB-presence-gated console. On the STM32L0 an *enabled* USART holds
        // embassy's STOP refcount, so a permanently-on console would keep the low-power
        // executor out of STOP forever — an unplugged node would burn ~3.5 mA (WFI at
        // 16 MHz) instead of idling at µA. So the console UART is owned by
        // [`console::manager`], which builds it while USB is present and **drops** it on
        // unplug (releasing the refcount → STOP). `VBUS_SENSE` (PA12) is the gate: EXTI
        // line 12 wakes the MCU out of STOP on plug-in to bring the console back.
        // Pull-down so the sense line reads a defined low when unplugged (no false "USB
        // present" from a floating pin). When plugged, the FT231X drives PA12 high via its
        // CBUS3 output (a push-pull ~3.3 V logic level — not a 5 V divider), but only tens
        // of ms after power-up; the manager's VBUS poll covers that late assertion. PA12 is
        // also the USB DP pin, used here purely as a VBUS_SENSE GPIO (no USB peripheral).
        crate::console::install_logger(LOG_LEVEL);
        let vbus = ExtiInput::new(p.PA12, p.EXTI12, Pull::Down, Irqs);
        spawner.spawn(crate::console::manager(p.USART1, p.PA9, p.PA10, vbus).unwrap());

        // I2C2 sensor bus — quiesce every device on it so the bus costs ~no idle current
        // (each may be absent on some boards, so NACKs are ignored). Order matters: the
        // ATSHA sleep is the LAST bus op, since any I2C traffic can re-wake it.
        let mut i2c_config = I2cConfig::default();
        i2c_config.frequency = Hertz::khz(100);
        let i2c = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, i2c_config);

        // TMP112 → shutdown (one-shot) so it doesn't free-run.
        let mut sensor = Tmp112::new(i2c, tmp112::ADDR_VPLUS);
        let _ = sensor.shutdown();
        let mut i2c = sensor.release(); // reclaim the bus for the rest of the hygiene

        // LIS2DH12 accelerometer → power-down (CTRL_REG1 = 0x00). Its POR default is
        // already power-down (~0.5 µA), but a prior configuration can persist across a
        // warm reset, so force it — the board wires only the accel's INT line, never
        // configures the part.
        let _ = i2c.blocking_write(ACCEL_ADDR, &[LIS2DH12_CTRL_REG1, 0x00]);

        // ATSHA204A crypto → Sleep (the last raw bus op, since any I2C traffic can re-wake
        // it). From here on the app's bus is [`AtshaGuard`]-wrapped, so every subsequent
        // transaction re-sleeps it automatically — no app has to remember.
        atsha_sleep(&mut i2c);

        // Re-wrap the (already shut-down) TMP112 on the ATSHA-guarded bus for the app.
        let sensor = Tmp112::new(AtshaGuard::new(i2c), tmp112::ADDR_VPLUS);

        Board {
            spawner,
            tmp112: sensor,
            led: p.PH1,
            // The Core Module button is active-high (press drives PA8 high; the
            // line is externally pulled down), so it uses a pull-down and
            // `Polarity::ActiveHigh` at the app.
            button: ExtiInput::new(p.PA8, p.EXTI8, Pull::Down, Irqs),
            strip_tim: p.TIM2,
            strip_data: p.PA1,
            strip_dma: p.DMA1_CH3,
            // LIS2DH12 INT1 (PB6) — active-high push-pull, so pull-down + rising edge.
            accel_int: ExtiInput::new(p.PB6, p.EXTI6, Pull::Down, Irqs),
            kv: Nv::install(Storage::new(Flash::new_blocking(p.FLASH))),
            iwdg: p.IWDG,

            // SPIRIT1 radio resources. The driver (`Spirit1::new`) builds the
            // blocking SPI and drives SDN/CS itself; here we just hand over the
            // raw peripherals and the EXTI-bound nIRQ line.
            radio_sdn: p.PB7,
            radio_cs: p.PA15,
            radio_spi: p.SPI1,
            radio_sck: p.PB3,
            radio_mosi: p.PB5,
            radio_miso: p.PB4,
            radio_irq: ExtiInput::new(p.PA7, p.EXTI7, Pull::Up, Irqs),
        }
    }
}

/// Initialise clocks and low-power config for the Core Module, returning the
/// peripherals. (Used by [`Board::take`]; call directly only for custom setups.)
///
/// - **sysclk = HSI16 (16 MHz)** — fast enough for WS2812 PWM; only affects
///   run-mode current, STOP-mode idle is unchanged (clock off).
/// - **RTC ← LSE** (32.768 kHz crystal) — the STOP-mode timekeeper / wake source.
/// - **`min_stop_pause = 0`** — any await length safely uses RTC-backed STOP.
/// - **debug clock gated in STOP** for real low-power current.
pub fn init() -> Peripherals {
    // Snapshot RCC_CSR (RTC-domain state + hardware reset-cause flags) BEFORE embassy's clock
    // init touches it, then clear the sticky reset flags so every boot reports its own cause.
    // Boot forensics: shows whether the LSE/RTC domain survived the reset and which hardware
    // reset fired. (Added for the 2026-07-05 bimodal-boot investigation, where it RULED OUT
    // the LSE/backup domain — every slow boot inherited a fully-matched domain + pin reset;
    // the proven cause is the EEPROM KV compaction CPU stall in init_session, ≈5.2 s, see
    // docs/storage.md. Kept: cheap, and reset-cause + domain state are generally useful.)
    // Read it back with [`preinit_csr`]; `examples/lse_probe.rs` decodes and logs it each boot.
    let csr = embassy_stm32::pac::RCC.csr().read();
    PREINIT_CSR.store(csr.0, core::sync::atomic::Ordering::Relaxed);
    embassy_stm32::pac::RCC.csr().modify(|w| w.set_rmvf(true));

    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = true;
    config.rcc.sys = Sysclk::HSI;
    config.rcc.ls = LsConfig::default_lse();
    // LSE crystal drive strength, selectable per build (TOWER_FEATURES=lse-drive-…). Default =
    // embassy's default_lse() = MediumHigh. Higher drive trades ~0.1–0.5 µA of STOP current
    // for oscillation robustness. (Swept Low/MediumHigh/High on the bench 2026-07-05: drive
    // has NO effect on the ~5.2 s bimodal-boot delay — that is the KV compaction stall — but
    // the knob is kept for crystal-robustness experiments on marginal boards.)
    #[cfg(any(
        feature = "lse-drive-low",
        feature = "lse-drive-medium-low",
        feature = "lse-drive-high"
    ))]
    if let Some(lse) = &mut config.rcc.ls.lse {
        #[cfg(feature = "lse-drive-low")]
        {
            lse.mode = embassy_stm32::rcc::LseMode::Oscillator(embassy_stm32::rcc::LseDrive::Low);
        }
        #[cfg(feature = "lse-drive-medium-low")]
        {
            lse.mode = embassy_stm32::rcc::LseMode::Oscillator(embassy_stm32::rcc::LseDrive::MediumLow);
        }
        #[cfg(feature = "lse-drive-high")]
        {
            lse.mode = embassy_stm32::rcc::LseMode::Oscillator(embassy_stm32::rcc::LseDrive::High);
        }
    }
    config.min_stop_pause = Duration::from_ticks(0);
    config.enable_debug_during_sleep = false;
    let p = embassy_stm32::init(config);

    apply_stop_tuning();

    p
}

/// Raw `RCC_CSR` as sampled by [`init`] before embassy's clock init ran — the RTC-domain
/// state the boot *inherited* (LSEON/LSERDY/LSEDRV/RTCSEL/RTCEN) plus this reset's cause
/// flags (PIN/POR/SFT/IWDG/…; cleared right after sampling, so they are per-boot). Decode
/// with `embassy_stm32::pac::rcc::regs::Csr(raw)`.
pub fn preinit_csr() -> u32 {
    PREINIT_CSR.load(core::sync::atomic::Ordering::Relaxed)
}

/// See [`preinit_csr`].
static PREINIT_CSR: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// This unit's stable 32-bit radio address, derived from the STM32's 96-bit unique
/// device ID: CRC-32/IEEE over the UID, clamped to never be 0 (`0` is the "host not yet
/// known" dest in the pairing JOIN_REQ, and the "unset / auto" sentinel of the
/// [`shell` `address` setting](crate::shell::radio_address)). Collisions across a fleet
/// are birthday-bounded (~2⁻³² per pair) — acceptable for network addressing; an
/// operator can pin an explicit or random address instead (`system address`).
pub fn unique_id32() -> u32 {
    let id = tower_protocol::crc::crc32_ieee(&embassy_stm32::uid::uid());
    if id == 0 { 1 } else { id }
}

/// A best-effort random `u32` — **not cryptographic**. Seeds an xorshift from the chip
/// UID, the current uptime tick, and a rolling internal state, so successive calls (and
/// different units) diverge; the live entropy is *when* it is called. Enough to mint a
/// unique network address (`system address random`); for secret randomness use a real
/// TRNG.
pub fn rand_u32() -> u32 {
    use core::sync::atomic::{AtomicU32, Ordering};
    static STATE: AtomicU32 = AtomicU32::new(0);
    let mut x =
        STATE.load(Ordering::Relaxed) ^ unique_id32() ^ (embassy_time::Instant::now().as_ticks() as u32) | 1; // never seed the xorshift with 0 (it would stick at 0)
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    STATE.store(x, Ordering::Relaxed);
    x
}

/// Assert the STM32L0 STOP-mode power tuning in `PWR_CR`:
/// - **LPSDSR** — put the voltage regulator in *low-power* mode during deep sleep
///   (embassy's L0 `enter_stop` sets only PDDS/CWUF, otherwise leaving the *main*
///   regulator powered in STOP — tens–hundreds of µA of avoidable draw);
/// - **ULP** — disable VREFINT in Stop (its buffer costs ~1.5 µA).
///
/// **Must be re-applied after every wake.** embassy's `exit_stop` re-inits RCC on each
/// wake (`rcc::reinit_saved` → `rcc/l.rs` voltage-scale step), which does a *full-register*
/// `PWR.cr().write(set_vos)` — that zeroes LPSDSR (bit 0) and ULP (bit 9). So a single
/// init-time write only holds for the first STOP; from the second STOP on the bits are
/// gone. [`crate::console::manager`] re-calls this on each wake of its VBUS-poll loop, so
/// the idle (unplugged) STOP always re-enters with the bits set. Caveat: with `ULP` set,
/// VREFINT is unavailable for a short window after wake (regulator/VREFINT startup) — fine
/// here as the SDK uses no ADC/VREFINT; code that adds battery sensing must account for it.
pub(crate) fn apply_stop_tuning() {
    embassy_stm32::pac::PWR.cr().modify(|w| {
        w.set_lpsdsr(embassy_stm32::pac::pwr::vals::Mode::LOW_POWER_MODE);
        w.set_ulp(true);
    });
}

/// Put the on-board **ATSHA204A** crypto (I²C 0x64) back to Sleep (word address `0x01`,
/// idle ~30 nA). Awake it draws far more (up to ~200 µA idle) — enough to dwarf the µA
/// STOP-mode floor. **Assume any I²C transaction on the shared bus can wake it**, so it
/// must be re-slept after *every* other transaction; the [`AtshaGuard`] bus wrapper does
/// this automatically, so most code never calls this directly. A NACK (already asleep, or
/// part absent on some board variants) is ignored.
pub fn atsha_sleep(i2c: &mut I2c<'static, Blocking, Master>) {
    let _ = i2c.blocking_write(ATSHA_ADDR, &[0x01]);
}

/// I²C bus wrapper that re-sleeps the on-board **ATSHA204A** (0x64) after **every**
/// transaction to another device. The ATSHA shares the I²C2 bus and — conservatively —
/// *any* traffic can wake it into ~200 µA idle (its watchdog only auto-sleeps it ~1.3 s
/// after a wake), which would dwarf the µA STOP-mode floor. Wrapping the bus makes the
/// "re-sleep after each transaction" policy automatic, so the sensor drivers and apps
/// can't forget it. Transactions addressed to the ATSHA itself are passed through
/// untouched, so a deliberate crypto sequence still manages its own wake/sleep.
///
/// It implements [`embedded_hal::i2c::I2c`], so it drops in wherever a raw bus went (the
/// [`Tmp112`] / [`Lis2dh12`](crate::lis2dh12::Lis2dh12) drivers are
/// generic over it). Use [`into_inner`](Self::into_inner) for deliberate *unguarded*
/// access (e.g. a raw bus scan).
pub struct AtshaGuard<T> {
    inner: T,
}

impl<T> AtshaGuard<T> {
    /// Wrap `inner` so every non-ATSHA transaction is followed by an ATSHA sleep.
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    /// Unwrap and return the raw (unguarded) bus.
    pub fn into_inner(self) -> T {
        self.inner
    }
}

impl<T: ErrorType> ErrorType for AtshaGuard<T> {
    type Error = T::Error;
}

impl<T: I2cTrait> I2cTrait for AtshaGuard<T> {
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        let r = self.inner.transaction(address, operations);
        // Re-sleep the ATSHA after traffic to any *other* device. Issued on `inner`
        // directly (never recurses through the guard); a NACK (already asleep) is ignored.
        if address != ATSHA_ADDR {
            let _ = self.inner.write(ATSHA_ADDR, &[0x01]);
        }
        r
    }
}
