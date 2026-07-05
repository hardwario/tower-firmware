//! thermometer — log the TMP112 temperature every 2 s.
//!
//! The console and the TMP112 (one-shot mode) are already set up by the board,
//! so the app is just the read loop. Watch it with `tower logs`.
//!
//!   just flash example thermometer

#![no_std]
#![no_main]

use embassy_time::Timer;
use log::{error, info};
use tower::{app, board::Board, tmp112};

async fn run(mut b: Board) {
    loop {
        match b.tmp112.oneshot().await {
            Ok(raw) => {
                let mc = tmp112::raw_to_millicelsius(raw);
                let sign = if mc < 0 { "-" } else { "" };
                // milli-degrees -> centi-degrees, rounded, for two decimal places.
                let centi = (mc.unsigned_abs() + 5) / 10;
                info!(target: "tmp112", "{}{}.{:02} deg. C", sign, centi / 100, centi % 100);
            }
            Err(e) => error!(target: "tmp112", "read failed: {:?}", e),
        }
        Timer::after_secs(2).await;
    }
}

app!(run);
