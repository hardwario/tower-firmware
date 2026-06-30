//! radio_climate_monitor — TOWER IoT Kit product firmware (SKELETON).
//!
//! A battery climate node: it wakes on an interval, measures temperature (the on-board
//! TMP112), (will) send a secured radio reading to the gateway, then returns to STOP
//! low-power until the next measurement.
//!
//! This is a starting skeleton — the measure loop runs and logs; the radio send is marked
//! TODO. The `net_*` examples and `docs/radio.md` (the `net` node role) show the full
//! pattern (`radio::init` from `b.radio_*`, pair with a gateway, then `send`).
//!
//!   just build app radio_climate_monitor
//!   just run   app radio_climate_monitor

#![no_std]
#![no_main]

use embassy_time::Timer;
use log::{error, info};
use tower::{app, board::Board, tmp112};

// Measurement interval. A real battery product would space these out (minutes) to save
// power; kept short here so the skeleton is easy to watch over the console.
const MEASURE_INTERVAL_SECS: u64 = 60;

async fn run(mut b: Board) {
    // TODO: bring up the SPIRIT1 radio as a node and pair with the gateway. See the
    // `net_*` examples + the `net` node role in `docs/radio.md`.
    info!(target: "climate", "radio_climate_monitor skeleton — measuring every {MEASURE_INTERVAL_SECS} s");

    loop {
        match b.tmp112.oneshot().await {
            Ok(raw) => {
                let mc = tmp112::raw_to_millicelsius(raw);
                let sign = if mc < 0 { "-" } else { "" };
                let centi = (mc.unsigned_abs() + 5) / 10; // milli- -> centi-deg, rounded
                info!(target: "climate", "{}{}.{:02} deg. C", sign, centi / 100, centi % 100);
                // TODO: send the reading to the gateway over the radio, e.g.
                //   net.send(&Reading { millicelsius: mc }).await;
            }
            Err(e) => error!(target: "climate", "tmp112 read failed: {:?}", e),
        }
        Timer::after_secs(MEASURE_INTERVAL_SECS).await;
    }
}

app!(run);
