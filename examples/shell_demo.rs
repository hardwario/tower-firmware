//! shell_demo — Phase 3: the RouterOS-style shell, coexisting with live logs.
//!
//! Drive it with the host CLI:  `tower shell`
//!   /system identity print
//!   /system identity set name=tower-01
//!   /system/resource print
//!   /export
//!   /system reboot

#![no_std]
#![no_main]

use embassy_time::Timer;
use log::info;
use tower::{app, board::Board, shell};

async fn run(b: Board) {
    // Hand the EEPROM to the shell (it owns the KV store for settings) and spawn it.
    shell::serve(b.spawner, b.storage);
    info!("shell ready — try `/system identity print` via `tower shell`");

    // Logs keep flowing alongside the shell on the same framed link.
    let mut n: u32 = 0;
    loop {
        info!("heartbeat {}", n);
        n = n.wrapping_add(1);
        Timer::after_secs(5).await;
    }
}

app!(run);
