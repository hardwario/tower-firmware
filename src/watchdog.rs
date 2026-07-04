//! Optional independent watchdog (IWDG).
//!
//! Opt-in hardware safety net: an app calls [`enable`] once at start-up to arm a reset that
//! fires if the firmware wedges (an infinite loop, a deadlock, a peripheral that never returns).
//!
//! **Low-power aware.** The STM32L0 IWDG clocks from the LSI and *keeps counting in STOP*, so a
//! naive watchdog would reset a healthy battery node the moment it idled past the timeout. Instead
//! [`enable`] spawns a feeder task that pets the watchdog every `timeout/2`: the low-power executor
//! wakes for that timer even from STOP, pets, and drops back to STOP — so an idle node stays alive
//! while a genuinely hung one (its executor no longer running the feeder) is reset after `timeout`.
//!
//! The L0 IWDG maxes out near ~28 s (12-bit reload / 256 prescaler at the ~37 kHz LSI); [`enable`]
//! clamps `timeout` to a safe ceiling so an over-long request can't trip embassy's prescaler assert.

use embassy_executor::Spawner;
use embassy_stm32::Peri;
use embassy_stm32::peripherals::IWDG;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_time::{Duration, Timer};

/// Hardware ceiling for the L0 IWDG timeout, with margin below the ~28 s max so embassy's
/// prescaler computation never asserts.
const MAX_TIMEOUT_US: u32 = 26_000_000;

/// Arm the independent watchdog with `timeout`, then spawn a task that pets it every `timeout/2`.
///
/// Call once, early in the app (before any long-running work). After this, the firmware must keep
/// the async executor alive — a hang that stops the feeder triggers a hardware reset after
/// `timeout`. `timeout` is clamped to the L0 hardware maximum (~26 s).
///
/// ```ignore
/// let b = Board::take(spawner);
/// watchdog::enable(b.iwdg, spawner, Duration::from_secs(8));
/// ```
pub fn enable(iwdg: Peri<'static, IWDG>, spawner: Spawner, timeout: Duration) {
    let us = (timeout.as_micros().min(MAX_TIMEOUT_US as u64)) as u32;
    let mut wdg = IndependentWatchdog::new(iwdg, us);
    wdg.unleash();
    // Pet at half the (clamped) timeout so a healthy system always has a full period of margin.
    let interval = Duration::from_micros((us / 2) as u64);
    // Spawn failure here means the feeder pool is already claimed (enable called twice) — a
    // programming error, and leaving an unfed armed watchdog would reset the board, so fail loud.
    spawner.spawn(feeder(wdg, interval).unwrap());
}

#[embassy_executor::task]
async fn feeder(mut wdg: IndependentWatchdog<'static, IWDG>, interval: Duration) {
    loop {
        wdg.pet();
        Timer::after(interval).await;
    }
}
