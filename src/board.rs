//! HARDWARIO TOWER Core Module board support.
//!
//! Most apps use [`Board::take`] via the [`app!`](crate::app) macro and never
//! touch the lower-level [`init`]/[`init_console`]. `take` performs the *always
//! on* setup — clocks, the serial console, and putting the TMP112 into shutdown
//! (one-shot) mode so it does not free-run and waste power — then hands the app
//! a [`Board`] of ready-to-use resources.

use embassy_executor::Spawner;
use embassy_stm32::exti::{ExtiInput, InterruptHandler};
use embassy_stm32::gpio::Pull;
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::peripherals::{DMA1_CH3, PA1, PA9, PH1, TIM2, USART1};
use embassy_stm32::rcc::{LsConfig, Sysclk};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_stm32::{Peri, Peripherals, bind_interrupts, interrupt};
use embassy_time::Duration;
use log::LevelFilter;

use crate::tmp112::{self, Tmp112};

// PA8 (button) and PA12 (VBUS_SENSE) are EXTI lines 8/12 — both on EXTI4_15.
bind_interrupts!(struct Irqs {
    EXTI4_15 => InterruptHandler<interrupt::typelevel::EXTI4_15>;
});

/// Default console verbosity set by [`Board::take`]; lower it at runtime with
/// `log::set_max_level`.
const LOG_LEVEL: LevelFilter = LevelFilter::Trace;

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

        // Console — always on.
        init_console(p.USART1, p.PA9, LOG_LEVEL);

        // TMP112 — put it in shutdown (one-shot) mode so it doesn't free-run.
        let mut i2c_config = I2cConfig::default();
        i2c_config.frequency = Hertz::khz(100);
        let i2c = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, i2c_config);
        let mut sensor = Tmp112::new(i2c, tmp112::ADDR_VPLUS);
        let _ = sensor.shutdown(); // ignore: sensor may be absent on some boards

        // USB-aware power management, for every app: inhibit STOP while VBUS
        // (PA12) is high so the console/EXTI stay live; allow STOP when unplugged.
        let vbus = ExtiInput::new(p.PA12, p.EXTI12, Pull::None, Irqs);
        spawner.spawn(crate::power::vbus_task(vbus).unwrap());

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
    embassy_stm32::init(config)
}

/// Bring up the serial log console on USART1 TX (PA9) at 115200 8N1 and install
/// it as the global [`log`] backend at `max_level`.
pub fn init_console(usart1: Peri<'static, USART1>, tx: Peri<'static, PA9>, max_level: LevelFilter) {
    let uart =
        UartTx::new_blocking(usart1, tx, UartConfig::default()).expect("USART1 115200 8N1 config");
    crate::console::init(uart, max_level);
}
