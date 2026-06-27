//! console_panic — Phase 1 panic-path test. Counts down, then panics: the panic
//! handler must emit one framed error record via the PAC (the executor is dead) so it
//! still renders in `tower logs`. The countdown gives the host time to attach.

#![no_std]
#![no_main]

use embassy_time::Timer;
use log::info;
use tower::{app, board::Board};

async fn run(_b: Board) {
    for i in (1..=10).rev() {
        info!("alive — deliberate panic in {}s", i);
        Timer::after_secs(1).await;
    }
    panic!("deliberate Phase-1 panic test");
}

app!(run);
