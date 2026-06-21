//! net_pairing — OTA 3-way pairing under the public pairing key (RADIO.md §7.6).
//!
//!   TOWER_FEATURES=role-gateway just flash net_pairing   # host: opens a window
//!   TOWER_FEATURES=role-node    just flash net_pairing   # joiner: requests to join
//!
//! Host opens a 10 s pairing window and, on the first JOIN_REQ, assigns an ID +
//! per-node key (JOIN_RESP) and waits for JOIN_CONFIRM. Joiner sends JOIN_REQ,
//! receives the assignment, confirms, and commits. Both log the key — they must
//! match, proving the 3-way handshake delivered the per-node key. (The key is
//! sniffable in-window by design; see §7.6.)

#![no_std]
#![no_main]

use embassy_time::{Duration, Timer};
use log::{error, info, warn};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig, PAIRING_KEY};
use tower::storage::Kv;
use tower::{app, board::Board};

#[cfg(not(feature = "role-node"))]
const HOST_ID: u32 = 0x2222_2222;
#[cfg(feature = "role-node")]
const PROPOSED_ID: u32 = 0x0000_00BB;
#[cfg(not(feature = "role-node"))]
const ASSIGN_ID: u32 = 0x0000_00AA;
// The per-node key the host hands out (would be random in production).
#[cfg(not(feature = "role-node"))]
const ASSIGN_KEY: [u8; 16] = [
    0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
];

async fn run(b: Board) {
    let radio = Spirit1::new(
        b.radio_spi, b.radio_sck, b.radio_mosi, b.radio_miso,
        b.radio_cs, b.radio_sdn, b.radio_irq,
    );
    let kv = Kv::new(b.storage);

    #[cfg(feature = "role-node")]
    let my_id = PROPOSED_ID;
    #[cfg(not(feature = "role-node"))]
    let my_id = HOST_ID;

    // The Net's own key is unused during pairing (JOIN frames use PAIRING_KEY).
    let mut net = match Net::new(radio, kv, NetConfig { my_id, key: PAIRING_KEY, band: Band::Eu868, channel: 0 }).await {
        Ok(n) => n,
        Err(e) => {
            error!(target: "pair", "net init: {:?}", e);
            return;
        }
    };

    #[cfg(not(feature = "role-node"))]
    {
        info!(target: "pair", "HOST {:08X}: opening 10 s pairing window", HOST_ID);
        loop {
            match net.open_pairing(Duration::from_secs(10), ASSIGN_ID, &ASSIGN_KEY).await {
                Some(proposed) => info!(
                    target: "pair",
                    "PAIRED *** joiner proposed={:08X} assigned={:08X} key[..4]={:02x}{:02x}{:02x}{:02x}",
                    proposed, ASSIGN_ID, ASSIGN_KEY[0], ASSIGN_KEY[1], ASSIGN_KEY[2], ASSIGN_KEY[3]
                ),
                None => warn!(target: "pair", "pairing window closed (no joiner / lost confirm)"),
            }
            Timer::after_secs(2).await;
        }
    }

    #[cfg(feature = "role-node")]
    {
        info!(target: "pair", "JOINER: requesting to join (proposed {:08X})", PROPOSED_ID);
        loop {
            match net.join(PROPOSED_ID, Duration::from_secs(10)).await {
                Some((assigned, key)) => info!(
                    target: "pair",
                    "JOINED *** assigned={:08X} key[..4]={:02x}{:02x}{:02x}{:02x} (expect a0a1a2a3)",
                    assigned, key[0], key[1], key[2], key[3]
                ),
                None => warn!(target: "pair", "join failed (no host in range)"),
            }
            Timer::after_secs(3).await;
        }
    }
}

app!(run);
