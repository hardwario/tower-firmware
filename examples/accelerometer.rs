//! accelerometer — LIS2DH12 dice orientation + tilt alert
//! ([`lis2dh12`](tower::lis2dh12) block).
//!
//! Reports which die face is up (1–6) as you turn the board, like a digital die.
//! Opposite faces sum to 7. Watch with `just run example accelerometer`.
//!
//! Tilt detection is **opt-in** via the `TILT` constant below: set it to a
//! [`Sensitivity`](tower::lis2dh12::Sensitivity) to have the accelerometer's
//! hardware interrupt (INT1 → PB6) flag any manipulation — logged as `tilt!` and
//! flashed on the LED — or `None` to disable it.
//!
//! The accelerometer shares the I²C2 bus with the TMP112, so the app reclaims the
//! bus from the (shut-down) sensor via `release()`.
//!
//!   just run example accelerometer

#![no_std]
#![no_main]

use embassy_futures::select::{Either, select};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};
use log::{error, info, warn};
use tower::lis2dh12::{Lis2dh12, Sensitivity, TiltConfig};
use tower::{app, board::Board, led, lis2dh12};

/// Tilt / manipulation detection: `Some(config)` to enable, `None` to skip.
/// `min_interval` rate-limits reported tilts so one shake is a single event.
const TILT: Option<TiltConfig> = Some(TiltConfig {
    sensitivity: Sensitivity::Medium,
    min_interval: Duration::from_millis(500),
});

/// How often orientation is sampled (matches the 10 Hz ODR).
const SAMPLE_MS: u64 = 100;

static LED_CH: led::LedChannel = led::LedChannel::new();
/// Quick triple-blink shown when a tilt is detected.
static ALERT: led::Pattern = &[
    led::Step::on(50),
    led::Step::off(50),
    led::Step::on(50),
    led::Step::off(50),
    led::Step::on(50),
];

async fn run(b: Board) {
    let led = led::init(
        b.spawner,
        Output::new(b.led, Level::Low, Speed::Low),
        &LED_CH,
        led::Polarity::ActiveHigh,
    );

    let mut int1 = b.accel_int;
    // Reclaim the shared I²C2 bus from the TMP112 driver for the accelerometer.
    let mut accel = Lis2dh12::new(b.tmp112.release(), lis2dh12::ADDR_DEFAULT);

    match accel.who_am_i() {
        Ok(lis2dh12::WHO_AM_I_ID) => {}
        Ok(id) => warn!(target: "accel", "unexpected WHO_AM_I 0x{:02X} (continuing)", id),
        Err(e) => error!(target: "accel", "not responding: {:?}", e),
    }
    if let Err(e) = accel.init() {
        error!(target: "accel", "init failed: {:?}", e);
    }

    if let Some(cfg) = TILT {
        match accel.enable_tilt(cfg) {
            Ok(()) => info!(target: "accel", "tilt detection enabled"),
            Err(e) => error!(target: "accel", "tilt enable failed: {:?}", e),
        }
    }

    info!(target: "accel", "turn the board to roll the dice (face-up = 1..6)");

    let mut last: Option<u8> = None;
    loop {
        // Wait for the next sample tick, or (when enabled) a tilt interrupt. INT1
        // is latched and active-high, so `wait_for_high` also catches an edge we
        // missed while busy: it stays high until `tilt_triggered` clears it.
        let tilted = if TILT.is_some() {
            matches!(
                select(Timer::after_millis(SAMPLE_MS), int1.wait_for_high()).await,
                Either::Second(())
            )
        } else {
            Timer::after_millis(SAMPLE_MS).await;
            false
        };

        if tilted {
            if accel.tilt_triggered().unwrap_or(false) {
                warn!(target: "accel", "tilt! device moved");
                led.play(ALERT);
            }
            continue;
        }

        match accel.read() {
            // Report only settled face changes; ignore transient "moving" states.
            Ok(sample) => {
                if let Some(face) = sample.dice()
                    && last != Some(face)
                {
                    info!(target: "accel", "dice: {}", face);
                    last = Some(face);
                }
            }
            Err(e) => error!(target: "accel", "read failed: {:?}", e),
        }
    }
}

app!(run);
