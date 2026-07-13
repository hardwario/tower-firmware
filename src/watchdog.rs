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
//!
//! **Liveness gating.** The feeder alone only proves the *executor* runs — a single wedged task
//! (the 2026-07-13 gateway: its main loop parked forever on console backpressure while the feeder
//! kept petting) never trips it. An app whose one main loop IS the product opts in with
//! [`require_checkin`] + [`checkin`]: the feeder then skips the pet once check-ins go stale, so a
//! parked main loop becomes a bounded IWDG reset instead of a device that needs a manual NRST.
//! Battery nodes that legitimately STOP for minutes simply don't opt in — [`enable`] alone keeps
//! the old behaviour.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_stm32::Peri;
use embassy_stm32::peripherals::IWDG;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_time::{Duration, Instant, Timer};

/// Hardware ceiling for the L0 IWDG timeout, with margin below the ~28 s max so embassy's
/// prescaler computation never asserts.
const MAX_TIMEOUT_US: u32 = 26_000_000;

/// Max check-in age (ticks) before the feeder withholds the pet; 0 = liveness gating off.
/// Load/store only (M0+ has no CAS). Tick-truncated to u32 and wrapping-compared, same as
/// the console's `LAST_HOST_RX` — the ~36 h wrap at 32 kHz ticks is harmless at these ages.
static CHECKIN_MAX_TICKS: AtomicU32 = AtomicU32::new(0);
/// Tick timestamp (low 32 bits) of the most recent [`checkin`].
static LAST_CHECKIN: AtomicU32 = AtomicU32::new(0);

/// Gate the feeder on app liveness: once called, the feeder pets the IWDG only while the
/// latest [`checkin`] is younger than `max_age` — a main loop parked past that lets the
/// hardware reset fire. Call after [`enable`], then [`checkin`] from the loop being
/// guarded. `max_age` should comfortably exceed the loop's longest legitimate iteration
/// (radio slices, EEPROM writes), and the worst-case reset lands ~`timeout × 1.5` after
/// the hang (one already-fed period + the unfed one).
pub fn require_checkin(max_age: Duration) {
    checkin(); // arm fresh so the gate can't fire before the first loop iteration
    CHECKIN_MAX_TICKS.store(max_age.as_ticks().max(1) as u32, Ordering::Relaxed);
}

/// Record main-loop liveness (see [`require_checkin`]). Cheap: one atomic store.
pub fn checkin() {
    LAST_CHECKIN.store(Instant::now().as_ticks() as u32, Ordering::Relaxed);
}

/// Whether the feeder may pet: no gating configured, or the last check-in is fresh.
fn live() -> bool {
    let max = CHECKIN_MAX_TICKS.load(Ordering::Relaxed);
    if max == 0 {
        return true;
    }
    let age = (Instant::now().as_ticks() as u32).wrapping_sub(LAST_CHECKIN.load(Ordering::Relaxed));
    age <= max
}

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
        if live() {
            wdg.pet();
        } else {
            // Stale check-ins: withhold the pet and let the IWDG reset fire. Say why on the
            // console first — this log is the only breadcrumb a wedge (not a fault) leaves.
            log::error!(target: "watchdog", "main loop unresponsive — letting the IWDG reset");
        }
        Timer::after(interval).await;
    }
}
