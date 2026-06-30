//! radio_gateway — reference gateway (shipped happy-path app, docs/radio.md).
//!
//!   TOWER_FEATURES=role-gateway just flash example radio_gateway
//!
//! Pairs with `radio_node`. Listens for confirmed telemetry frames, authenticates
//! and decrypts them (AES-CCM), applies the replay rule, auto-ACKs, and prints the
//! decoded sensor sample with link quality. Registers each node in the peer table
//! so every node is decoded under its own key with its own replay lane — add more
//! `add_peer(...)` calls to grow the star (up to 64 nodes).

#![no_std]
#![no_main]

use embassy_time::Duration;
use log::{error, info};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig};
use tower::{app, board::Board};

const NODE_ID: u32 = 0x1111_1111;
const GW_ID: u32 = 0x2222_2222;
const KEY: [u8; 16] = [
    0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe, 0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
];

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

    let mut net = match Net::new(
        radio,
        b.kv,
        NetConfig {
            my_id: GW_ID,
            key: KEY,
            band: Band::DEFAULT,
            channel: 0,
        },
    )
    .await
    {
        Ok(n) => n,
        Err(e) => {
            error!(target: "gw", "net init: {e}");
            return;
        }
    };
    net.add_peer(NODE_ID, &KEY); // register each node with its per-node key

    info!(target: "gw", "GATEWAY {:08X}: listening ({} node(s) registered)", GW_ID, net.peer_count());
    loop {
        if let Some(rx) = net.recv(Duration::from_secs(15)).await {
            let d = rx.data();
            if d.len() >= 8 {
                let seq = u32::from_le_bytes([d[0], d[1], d[2], d[3]]);
                let vbat = u16::from_le_bytes([d[4], d[5]]);
                let temp = i16::from_le_bytes([d[6], d[7]]);
                info!(
                    target: "gw",
                    "src={:08X} cnt={} seq={} vbat={}mV temp={}.{}°C rssi={}dBm (ACKed)",
                    rx.src, rx.counter, seq, vbat, temp / 10, (temp % 10).abs(), rx.rssi_dbm
                );
            } else {
                info!(target: "gw", "src={:08X} cnt={} {} B rssi={}dBm (ACKed)", rx.src, rx.counter, d.len(), rx.rssi_dbm);
            }
        }
    }
}

app!(run);
