//! net_star — star topology with per-node keys (RADIO.md §7.2/§7.4).
//!
//!   TOWER_FEATURES=role-gateway              just flash net_star  # hub (holds both nodes)
//!   TOWER_FEATURES=role-node                 just flash net_star  # node A
//!   TOWER_FEATURES=role-node,node-2          just flash net_star  # node B
//!
//! The gateway registers TWO peers, each under its OWN key (add_peer). A node is
//! flashed as A (default) or B (node-2) with that node's key and sends confirmed
//! uplinks. The gateway decrypts each node with the registered per-node key,
//! tracks a separate replay lane per node, and auto-ACKs. With two boards: flash
//! the gateway once, then the node as A — re-flash it as B to show the second
//! peer decoded under a different key by the same gateway. (B only decodes
//! because KEY_B is registered; the gateway's default key is KEY_A.)

#![no_std]
#![no_main]

use log::{error, info};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig};
use tower::storage::Kv;
use tower::{app, board::Board};

#[cfg(feature = "role-node")]
use {embassy_time::Timer, log::warn, tower::radio::net::SendResult};
#[cfg(not(feature = "role-node"))]
use embassy_time::Duration;

const GW_ID: u32 = 0x2222_2222;
#[cfg(any(not(feature = "role-node"), not(feature = "node-2")))]
const NODE_A: u32 = 0x0A0A_0A0A;
#[cfg(any(not(feature = "role-node"), feature = "node-2"))]
const NODE_B: u32 = 0x0B0B_0B0B;
#[cfg(any(not(feature = "role-node"), not(feature = "node-2")))]
const KEY_A: [u8; 16] = [0xA0; 16];
#[cfg(any(not(feature = "role-node"), feature = "node-2"))]
const KEY_B: [u8; 16] = [0xB0; 16];

// Node identity selected by the `node-2` feature.
#[cfg(all(feature = "role-node", not(feature = "node-2")))]
const NODE: (u32, [u8; 16], char) = (NODE_A, KEY_A, 'A');
#[cfg(all(feature = "role-node", feature = "node-2"))]
const NODE: (u32, [u8; 16], char) = (NODE_B, KEY_B, 'B');

async fn run(b: Board) {
    let radio = Spirit1::new(
        b.radio_spi, b.radio_sck, b.radio_mosi, b.radio_miso,
        b.radio_cs, b.radio_sdn, b.radio_irq,
    );
    let kv = Kv::new(b.storage);

    // The gateway's default key is KEY_A; each node is registered with its own.
    #[cfg(feature = "role-node")]
    let (my_id, key) = (NODE.0, NODE.1);
    #[cfg(not(feature = "role-node"))]
    let (my_id, key) = (GW_ID, KEY_A);

    let mut net = match Net::new(radio, kv, NetConfig { my_id, key, band: Band::DEFAULT, channel: 0 }).await {
        Ok(n) => n,
        Err(e) => {
            error!(target: "star", "net init: {:?}", e);
            return;
        }
    };

    #[cfg(not(feature = "role-node"))]
    {
        net.add_peer(NODE_A, &KEY_A);
        net.add_peer(NODE_B, &KEY_B);
        info!(target: "star", "GATEWAY {:08X}: {} peers registered (per-node keys)", GW_ID, net.peer_count());
        loop {
            if let Some(rx) = net.recv(Duration::from_secs(10)).await {
                let who = match rx.src {
                    NODE_A => 'A',
                    NODE_B => 'B',
                    _ => '?',
                };
                let text = core::str::from_utf8(rx.data()).unwrap_or("<bin>");
                info!(
                    target: "star",
                    "rx node {} (src={:08X}) cnt={} rssi={}dBm \"{}\" (ACKed, per-node key)",
                    who, rx.src, rx.counter, rx.rssi_dbm, text
                );
            }
        }
    }

    #[cfg(feature = "role-node")]
    {
        info!(target: "star", "NODE {} ({:08X}): confirmed uplink to GW under per-node key", NODE.2, NODE.0);
        let mut seq: u32 = 0;
        loop {
            let mut msg = [0u8; 6];
            msg[0] = NODE.2 as u8;
            msg[1] = b':';
            msg[2] = b'0' + ((seq / 100) % 10) as u8;
            msg[3] = b'0' + ((seq / 10) % 10) as u8;
            msg[4] = b'0' + (seq % 10) as u8;
            match net.send(GW_ID, &msg, true, 3).await {
                SendResult::Delivered => info!(target: "star", "node {} seq={} Delivered", NODE.2, seq),
                SendResult::NotDelivered => warn!(target: "star", "node {} seq={} NotDelivered", NODE.2, seq),
                other => warn!(target: "star", "node {} seq={} {:?}", NODE.2, seq, other),
            }
            seq = seq.wrapping_add(1);
            Timer::after_secs(2).await;
        }
    }
}

app!(run);
