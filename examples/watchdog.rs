//! watchdog — the opt-in independent watchdog (IWDG).
//!
//! Arms an 8 s watchdog, then blinks the LED and logs a heartbeat. The feeder task pets the
//! watchdog every 4 s (half the timeout), so a healthy run never resets — even across STOP-mode
//! idle, since the low-power executor wakes for the feeder's timer. To watch it bite, replace the
//! loop body with `loop {}` (a hang): the feeder stops and the MCU resets after ~8 s.
//!
//!   just flash example watchdog     (then: just logs)

#![no_std]
#![no_main]

use embassy_time::{Duration, Timer};
use log::info;
use tower::{app, board::Board, watchdog};

async fn run(b: Board) {
    // Arm the watchdog: a firmware hang now triggers a hardware reset after 8 s.
    watchdog::enable(b.iwdg, b.spawner, Duration::from_secs(8));

    let mut beat = 0u32;
    loop {
        info!(target: "wdg", "alive: heartbeat {beat} (watchdog fed by the feeder task)");
        beat += 1;
        Timer::after(Duration::from_secs(2)).await;
    }
}

app!(run);
