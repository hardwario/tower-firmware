//! i2cscan — probe the I2C2 bus (PB10/PB11) and log responding addresses.
//!
//! A maker diagnostic: confirms the bus works and finds device addresses (e.g.
//! which address the TMP112 is strapped to). Watch with `just monitor`.
//!
//! Uses the standard `Board` setup like the other samples, then reclaims the raw
//! I2C2 bus from the (shut-down) TMP112 driver via `release()` to probe it.
//!
//!   just flash i2cscan

#![no_std]
#![no_main]

use embassy_time::Timer;
use log::info;
use tower::{app, board::Board};

async fn run(b: Board) {
    // Take the I2C2 bus back from the TMP112 driver for raw address probing.
    let mut i2c = b.tmp112.release();

    info!(target: "i2c", "scanning I2C2 @ 100 kHz ...");
    let mut found = 0;
    for addr in 0x08u8..=0x77 {
        // A 1-byte read ACKs the address if a device is present.
        if i2c.blocking_read(addr, &mut [0u8; 1]).is_ok() {
            info!(target: "i2c", "device at 0x{:02X}", addr);
            found += 1;
        }
    }
    info!(target: "i2c", "scan complete - {} device(s)", found);

    loop {
        Timer::after_secs(60).await;
    }
}

app!(run);
