//! events_demo — structured, self-describing events interleaved with logs.
//!
//! Emits a `measurement` event each second (plus a periodic `heartbeat`) and normal
//! logs. Render events with `tower events` and logs with `tower logs` — both decode
//! the same framed stream.

#![no_std]
#![no_main]

use core::fmt::Write;
use embassy_time::Timer;
use heapless::String;
use log::{info, warn};
use tower::{app, board::Board, console};

async fn run(_b: Board) {
    let mut n: u32 = 0;
    loop {
        info!("loop tick {}", n); // a normal log, to show interleaving

        let mut count = String::<12>::new();
        let _ = write!(count, "{}", n);
        let mut temp = String::<12>::new();
        let _ = write!(temp, "{}", 2000 + (n % 50)); // fake centi-degrees

        console::event(
            "measurement",
            &[
                ("count", count.as_str()),
                ("temp_c", temp.as_str()),
                ("unit", "cdeg"),
            ],
        )
        .await;

        if n.is_multiple_of(5) {
            console::event("heartbeat", &[("uptime", count.as_str())]).await;
            warn!("warning at tick {}", n);
        }

        n = n.wrapping_add(1);
        Timer::after_secs(1).await;
    }
}

app!(run);
