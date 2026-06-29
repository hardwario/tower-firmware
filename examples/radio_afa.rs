//! radio_afa — EU 868 LBT + Adaptive Frequency Agility (EN 300 220).
//!
//!   TOWER_FEATURES=role-gateway just flash radio_afa   # scans the AFA set, auto-ACKs
//!   TOWER_FEATURES=role-node    just flash radio_afa   # LBT + agility transmitter
//!
//! Listen-Before-Talk + frequency agility is the EU technique that relaxes the 1 %
//! duty cap. The node listens before every TX (CCA) and hops to another channel
//! when one is busy or still in its post-TX off-time; the gateway just scans the
//! 8-channel set (865.2–868.0 MHz) and ACKs on whatever channel caught the frame —
//! no time-sync. Each node cycle does: (1) one **confirmed** send (LBT + delivery +
//! ACK — the node camps a channel, the gateway scans onto it), then (2) a short
//! **unconfirmed burst** whose per-channel off-time forces the channel to sweep,
//! demonstrating agility. Watch the node log the channel it used and the gateway log
//! the channel it received on — they move across the set under contention.

#![no_std]
#![no_main]

use log::{error, info};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{AfaConfig, AfaRole, Net, NetConfig};
use tower::storage::Kv;
use tower::{app, board::Board};

#[cfg(not(feature = "role-node"))]
use embassy_time::Duration;
#[cfg(feature = "role-node")]
use {embassy_time::Timer, log::warn, tower::radio::net::SendResult};

const NODE_ID: u32 = 0x1111_1111;
const GW_ID: u32 = 0x2222_2222;
const KEY: [u8; 16] = [
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
];

#[cfg(not(feature = "role-node"))]
fn seq_of(d: &[u8]) -> u32 {
    if d.len() >= 4 {
        u32::from_le_bytes([d[0], d[1], d[2], d[3]])
    } else {
        0
    }
}

async fn run(b: Board) {
    let radio = Spirit1::new(
        b.radio_spi,
        b.radio_sck,
        b.radio_mosi,
        b.radio_miso,
        b.radio_cs,
        b.radio_sdn,
        b.radio_irq,
    );
    let kv = Kv::new(b.storage);

    #[cfg(feature = "role-node")]
    let my_id = NODE_ID;
    #[cfg(not(feature = "role-node"))]
    let my_id = GW_ID;

    let mut net = match Net::new(
        radio,
        kv,
        NetConfig {
            my_id,
            key: KEY,
            band: Band::Eu868,
            channel: 0,
        },
    )
    .await
    {
        Ok(n) => n,
        Err(e) => {
            error!(target: "afa", "net init: {e}");
            return;
        }
    };

    #[cfg(feature = "role-node")]
    {
        net.add_peer(GW_ID, &KEY);
        if let Err(e) = net.enable_afa(AfaRole::Node, AfaConfig { primary: 0 }).await {
            error!(target: "afa", "enable_afa: {e}");
            return;
        }
        info!(target: "afa", "NODE: LBT+AFA over 8 EU channels (865.2–868.0 MHz)");
        let mut seq: u32 = 0;
        loop {
            // 1) Confirmed: LBT + delivery + ACK (node camps a channel; GW scans onto it).
            match net.afa_send(GW_ID, &seq.to_le_bytes(), true, 6).await {
                SendResult::Delivered => {
                    info!(target: "afa", "seq={} ch={} Delivered", seq, net.afa_channel())
                }
                r => warn!(target: "afa", "seq={} ch={} {r}", seq, net.afa_channel()),
            }
            seq = seq.wrapping_add(1);

            // 2) Unconfirmed burst: the per-channel off-time forces agility — watch ch sweep.
            for _ in 0..4 {
                let _ = net.afa_send(GW_ID, &seq.to_le_bytes(), false, 1).await;
                info!(target: "afa", "  agility burst seq={} ch={}", seq, net.afa_channel());
                seq = seq.wrapping_add(1);
            }
            Timer::after_secs(2).await;
        }
    }

    #[cfg(not(feature = "role-node"))]
    {
        net.add_peer(NODE_ID, &KEY);
        if let Err(e) = net.enable_afa(AfaRole::Gateway, AfaConfig { primary: 0 }).await {
            error!(target: "afa", "enable_afa: {e}");
            return;
        }
        info!(target: "afa", "GATEWAY: scanning 8 EU AFA channels, auto-ACK");
        loop {
            if let Some(rx) = net.afa_serve(Duration::from_secs(10)).await {
                info!(
                    target: "afa",
                    "rx seq={} on ch={} src={:08X} rssi={}dBm{}",
                    seq_of(rx.data()), net.afa_channel(), rx.src, rx.rssi_dbm,
                    if rx.confirmed { " (ACKed)" } else { "" }
                );
            }
        }
    }
}

app!(run);
