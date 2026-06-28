//! console_demo — framed-console test.
//!
//! Logs at every level + a `println!`, every 2 s, after a boot burst that
//! deliberately overflows the TX queue so the host shows a `Dropped` marker.
//! Render with the host CLI:  `tower logs`  (raw `jolt monitor` shows binary now).

#![no_std]
#![no_main]

use embassy_time::Timer;
use log::{debug, error, info, trace, warn};
use tower::{app, board::Board, println};

async fn run(_b: Board) {
    let mut n: u32 = 0;
    loop {
        // One of each level + a raw println!, paced so the writer drains them (none
        // dropped) — exercises every render path.
        error!("error sample #{}", n);
        warn!("warn sample #{}", n);
        info!("info sample #{}", n);
        debug!("debug sample #{}", n);
        trace!("trace sample #{}", n);
        println!("raw println! line #{}", n);
        // A line far over the 192-char buffer: must arrive *truncated*, not empty
        // (the clip path — a plain heapless write would reject it wholesale).
        const A: &str = "abcdefghijklmnopqrstuvwxyz0123456789";
        info!("longline #{n}: {A}{A}{A}{A}{A}{A}");
        Timer::after_millis(400).await; // let the queue drain before the burst

        // Then a burst far larger than the TX queue depth with no awaits between, so
        // the queue overflows and the host shows a `Dropped` marker each round.
        for i in 0..30 {
            info!("burst {} (round {})", i, n);
        }

        n = n.wrapping_add(1);
        Timer::after_secs(2).await;
    }
}

app!(run);
