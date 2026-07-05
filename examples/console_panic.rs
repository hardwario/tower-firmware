//! console_panic — panic-path test. Counts down, then panics: the panic
//! handler must emit one framed error record via the PAC (the executor is dead) so it
//! still renders in `tower logs`. The countdown gives the host time to attach.
//!
//! Since the reset-on-fault policy: after that error frame the unit RESETS (it does not
//! halt), so expect the boot banner again plus a `crash` module ERROR frame re-reporting
//! the panic from the reset-surviving breadcrumb (also: `/system/crash print`). The
//! countdown then restarts — this example crash-loops by design; the bootguard backoff
//! kicks in after 8 fast resets.

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
    panic!("deliberate panic-path test");
}

app!(run);
