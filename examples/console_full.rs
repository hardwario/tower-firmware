//! console_full — the showcase for `tower console` (the TUI). Serves the shell and
//! emits live logs (every level) plus periodic structured events, so all four panes
//! — Device Events / Shell Command / Shell Responses / Device Logs — have content.
//!
//! Drive it with:  `tower console`
//!   try `/system identity print`, `/system/resource print`, TAB completion,
//!   F3 zoom, Shift-Tab to move focus, F5 pause, F10 quit.

#![no_std]
#![no_main]

use core::fmt::Write;
use embassy_time::Timer;
use heapless::String;
use log::{debug, error, info, trace, warn};
use tower::{app, board::Board, console, println};

async fn run(b: Board) {
    // The shell is served automatically by `app!` (Responses/Command panes), over the shared KV.
    info!("console_full ready — drive the shell, e.g. `/system identity print`");

    let mut n: u32 = 0;
    loop {
        // Logs across levels so the Logs pane shows colour + variety.
        info!("heartbeat {}", n);
        if n.is_multiple_of(3) {
            warn!("periodic warning at tick {}", n);
            debug!("debug detail at tick {}", n);
            trace!("trace detail at tick {}", n);
        }
        if n.is_multiple_of(5) {
            error!("sample error at tick {}", n);
            println!("raw println at tick {}", n);
        }

        // A structured event so the Events pane has content.
        let mut count = String::<12>::new();
        let _ = write!(count, "{}", n);
        console::event("tick", &[("n", count.as_str()), ("src", "console_full")]).await;

        n = n.wrapping_add(1);
        Timer::after_secs(2).await;
    }
}

app!(run);
