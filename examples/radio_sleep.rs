//! radio_sleep — low-power SLEEP / SHUTDOWN between transfers (docs/radio.md).
//!
//!   TOWER_FEATURES=role-node    just flash example radio_sleep   # duty-cycled sender
//!   TOWER_FEATURES=role-gateway just flash example radio_sleep   # always-on receiver
//!
//! The node wakes, transmits a frame, then drops the radio to a low-power state
//! between transfers — alternating SLEEP (config retained, fast wake) and SHUTDOWN
//! (powered down, needs a re-init on wake). It logs the wake latency for each so
//! the SLEEP→READY vs SHUTDOWN→READY+reconfig cost is visible. The gateway just
//! keeps receiving and prints each frame — proving the node re-links correctly
//! after BOTH sleep modes.
//!
//! Note: with USB connected the MCU stays out of STOP (USB inhibits it — see the
//! boot log). Unplug USB to let the MCU itself drop to STOP during the wait; the
//! radio's SLEEP/SHUTDOWN behaviour shown here is independent of that.

#![no_std]
#![no_main]

use embassy_time::Duration;
use log::{error, info};
use tower::radio::{RfConfig, Spirit1, config};
use tower::{app, board::Board};
#[cfg(feature = "role-node")]
use {embassy_time::Instant, embassy_time::Timer, log::warn};

const CHANNEL: u8 = 0;

async fn run(b: Board) {
    let mut radio = Spirit1::new(
        b.radio_spi,
        b.radio_sck,
        b.radio_mosi,
        b.radio_miso,
        b.radio_cs,
        b.radio_sdn,
        b.radio_irq,
    );
    if let Err(e) = radio.exit_shutdown().await {
        error!(target: "sleep", "exit_shutdown: {e}");
    }
    if let Err(e) = radio.read_device_id() {
        error!(target: "sleep", "device id: {e}");
    }
    let cfg = RfConfig {
        band: config::Band::DEFAULT,
        channel: CHANNEL,
    };
    if let Err(e) = config::apply(&mut radio, &cfg).await {
        error!(target: "sleep", "config: {e}");
    }

    #[cfg(feature = "role-node")]
    node(&mut radio, &cfg).await;
    #[cfg(not(feature = "role-node"))]
    gateway(&mut radio).await;
}

#[cfg(feature = "role-node")]
async fn node(radio: &mut Spirit1, cfg: &RfConfig) -> ! {
    info!(target: "sleep", "NODE: TX then sleep, alternating SLEEP / SHUTDOWN on ch{}", CHANNEL);
    let mut seq: u32 = 0;
    loop {
        // Transmit (radio is READY here — freshly woken on later iterations).
        let mut frame = [0x5Au8; 12];
        frame[..4].copy_from_slice(&seq.to_le_bytes());
        match radio.tx(&frame, false, Duration::from_millis(200)).await {
            Ok(()) => info!(target: "sleep", "seq={} tx ok", seq),
            Err(e) => warn!(target: "sleep", "seq={} tx {e}", seq),
        }

        // Sleep between transfers, alternating the two low-power modes.
        let use_shutdown = seq % 2 == 1;
        if use_shutdown {
            radio.enter_shutdown();
            info!(target: "sleep", "→ SHUTDOWN (powered down)");
        } else {
            let _ = radio.to_sleep().await;
            info!(target: "sleep", "→ SLEEP (config retained)");
        }
        Timer::after_secs(3).await; // (MCU would drop to STOP here if USB unplugged)

        // Wake and re-link, timing the cost of each path.
        let t0 = Instant::now();
        if use_shutdown {
            let _ = radio.exit_shutdown().await;
            let _ = config::apply(radio, cfg).await; // SHUTDOWN loses config
        } else {
            let _ = radio.to_ready().await;
        }
        let us = t0.elapsed().as_micros();
        info!(
            target: "sleep",
            "woke from {} in {} µs",
            if use_shutdown { "SHUTDOWN (+reconfig)" } else { "SLEEP" },
            us
        );
        seq = seq.wrapping_add(1);
    }
}

#[cfg(not(feature = "role-node"))]
async fn gateway(radio: &mut Spirit1) -> ! {
    info!(target: "sleep", "GATEWAY: receiving across the node's sleep cycles on ch{}", CHANNEL);
    let mut buf = [0u8; 96];
    let mut last: Option<u32> = None;
    loop {
        match radio.rx(&mut buf, Duration::from_secs(8)).await {
            Ok((len, q)) => {
                let seq = if len >= 4 {
                    u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]])
                } else {
                    0
                };
                let relink = match last {
                    Some(p) if seq == p.wrapping_add(1) => " (re-linked after sleep)",
                    _ => "",
                };
                last = Some(seq);
                info!(target: "sleep", "rx seq={} len={} rssi={}dBm{}", seq, len, q.rssi, relink);
            }
            Err(tower::radio::RadioError::Timeout) => {
                info!(target: "sleep", "...idle (no frame in 8 s)");
            }
            Err(e) => info!(target: "sleep", "rx {e}"),
        }
    }
}

app!(run);
