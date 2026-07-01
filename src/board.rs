//! HARDWARIO TOWER Core Module board support.
//!
//! Most apps use [`Board::take`] via the [`app!`](crate::app) macro and never
//! touch the lower-level [`init`]/[`console::init`](crate::console::init). `take`
//! performs the *always
//! on* setup — clocks, the serial console, and putting the TMP112 into shutdown
//! (one-shot) mode so it does not free-run and waste power — then hands the app
//! a [`Board`] of ready-to-use resources.

use embassy_executor::Spawner;
use embassy_stm32::exti::{ExtiInput, InterruptHandler};
use embassy_stm32::flash::Flash;
use embassy_stm32::gpio::Pull;
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::peripherals::{DMA1_CH3, PA1, PA15, PB3, PB4, PB5, PB7, PH1, SPI1, TIM2};
use embassy_stm32::rcc::{LsConfig, Sysclk};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Peri, Peripherals, bind_interrupts, interrupt};
use embassy_time::Duration;
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

/// The board's TMP112 on the I²C2 bus (blocking), in shutdown / one-shot mode.
pub type Tmp112Sensor = Tmp112<I2c<'static, Blocking, Master>>;

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
    /// `Copy`; hand the same handle to `Net`, the [`shell`](crate::shell), and FOTA at once (each
    /// call locks the one store). For raw program-flash access (FOTA staging) use
    /// [`Nv::with_flash`](crate::storage::Nv::with_flash).
    pub kv: Nv,

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

impl Board {
    /// Initialise the board and return its resources. Call once at start-up —
    /// typically via [`app!`](crate::app). Performs the always-on setup: clocks,
    /// the serial console, TMP112 shutdown, and **USB-aware power management**
    /// (see below).
    ///
    /// A [`power::vbus_task`](crate::power::vbus_task) is spawned automatically so
    /// every app gets the same low-power policy: stay awake (Sleep) while USB is
    /// connected — keeping the console and interrupts responsive for debugging —
    /// and allow STOP when running unplugged. Apps don't wire this up themselves.
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
        // present" from a floating pin); the external divider drives it high when plugged
        // (docs/console.md notes the V_IH caveat to verify on HW).
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

        // ATSHA204A crypto → Sleep (word address 0x01, ~30 nA). Its idle draw is ~200 µA
        // and any bus activity can wake it; the on-chip watchdog auto-sleeps it ~1.3 s
        // after a wake, but we sleep it explicitly here (the last bus op) so it's down
        // immediately. Apps that poll the I2C bus should re-sleep it after each batch.
        let _ = i2c.blocking_write(ATSHA_ADDR, &[0x01]);

        // Re-wrap the (already shut-down) TMP112 for the app's one-shot reads.
        let sensor = Tmp112::new(i2c, tmp112::ADDR_VPLUS);

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

            // SPIRIT1 radio resources. The driver (radio::init) builds the
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
    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = true;
    config.rcc.sys = Sysclk::HSI;
    config.rcc.ls = LsConfig::default_lse();
    config.min_stop_pause = Duration::from_ticks(0);
    config.enable_debug_during_sleep = false;
    let p = embassy_stm32::init(config);

    // STOP-mode regulator: embassy's L0 low-power `enter_stop` sets only PDDS/CWUF,
    // leaving the *main* voltage regulator powered in STOP (tens–hundreds of µA).
    // Switch the regulator to low-power mode in deep sleep (PWR_CR.LPSDSR) and
    // disable VREFINT in Stop (PWR_CR.ULP) for the datasheet ~µA STOP current.
    // embassy's later read-modify-write of PWR_CR (PDDS/CWUF on stop entry)
    // preserves these bits, and the fast-wakeup path re-enables VREFINT as needed.
    embassy_stm32::pac::PWR.cr().modify(|w| {
        w.set_lpsdsr(embassy_stm32::pac::pwr::vals::Mode::LOW_POWER_MODE);
        w.set_ulp(true);
    });

    p
}
