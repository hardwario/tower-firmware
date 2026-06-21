//! net_pairing — OTA 3-way pairing under the public pairing key (RADIO.md §7.6).
//!
//!   TOWER_FEATURES=role-gateway just flash net_pairing   # host: opens a window
//!   TOWER_FEATURES=role-node    just flash net_pairing   # joiner: requests to join
//!
//! The host opens a 1-minute pairing window ([`PAIRING_WINDOW`]) and, on the first
//! JOIN_REQ, hands out a per-node key (JOIN_RESP) and waits for JOIN_CONFIRM. The
//! **joiner chooses its own ID** and keeps it — the host does NOT assign it; the
//! host only learns that ID and the key it handed out, to install the peer. Both
//! log the key (they must match) and the joiner's ID (the same on both sides),
//! proving the handshake. (The key is sniffable in-window by design; see §7.6.)

#![no_std]
#![no_main]

use embassy_time::Timer;
use log::{error, info, warn};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig, PAIRING_KEY, PAIRING_WINDOW};
use tower::storage::Kv;
use tower::{app, board::Board};

#[cfg(not(feature = "role-node"))]
const HOST_ID: u32 = 0x2222_2222;
// The joiner's OWN, self-chosen ID (kept after pairing — not assigned by the host).
#[cfg(feature = "role-node")]
const MY_ID: u32 = 0x0000_00BB;
// The per-node key the host hands out (would be random in production).
#[cfg(not(feature = "role-node"))]
const HANDED_KEY: [u8; 16] = [
    0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
];

async fn run(b: Board) {
    let radio = Spirit1::new(
        b.radio_spi, b.radio_sck, b.radio_mosi, b.radio_miso,
        b.radio_cs, b.radio_sdn, b.radio_irq,
    );
    let kv = Kv::new(b.storage);

    #[cfg(feature = "role-node")]
    let my_id = MY_ID;
    #[cfg(not(feature = "role-node"))]
    let my_id = HOST_ID;

    // The Net's own key is unused during pairing (JOIN frames use PAIRING_KEY).
    let mut net = match Net::new(radio, kv, NetConfig { my_id, key: PAIRING_KEY, band: Band::DEFAULT, channel: 0 }).await {
        Ok(n) => n,
        Err(e) => {
            error!(target: "pair", "net init: {:?}", e);
            return;
        }
    };

    #[cfg(not(feature = "role-node"))]
    {
        info!(target: "pair", "HOST {:08X}: opening pairing window (1 min)", HOST_ID);
        loop {
            match net.open_pairing(PAIRING_WINDOW, &HANDED_KEY).await {
                // The joiner brought its own id; the host installs (id, key).
                Some(node_id) => info!(
                    target: "pair",
                    "PAIRED *** node id={:08X} (joiner-chosen) key[..4]={:02x}{:02x}{:02x}{:02x}",
                    node_id, HANDED_KEY[0], HANDED_KEY[1], HANDED_KEY[2], HANDED_KEY[3]
                ),
                None => warn!(target: "pair", "pairing window closed (no joiner / lost confirm)"),
            }
            Timer::after_secs(2).await;
        }
    }

    #[cfg(feature = "role-node")]
    {
        info!(target: "pair", "JOINER: requesting to join with my own id {:08X} (1 min)", MY_ID);
        loop {
            match net.join(MY_ID, PAIRING_WINDOW).await {
                Some(key) => info!(
                    target: "pair",
                    "JOINED *** id={:08X} (mine) key[..4]={:02x}{:02x}{:02x}{:02x} (expect a0a1a2a3)",
                    MY_ID, key[0], key[1], key[2], key[3]
                ),
                None => warn!(target: "pair", "join failed (no host in range)"),
            }
            Timer::after_secs(3).await;
        }
    }
}

app!(run);
