//! net_bulk — bulk transfer via the pull mechanism (RADIO.md §7.5).
//!
//!   TOWER_FEATURES=role-gateway just flash net_bulk   # sender: serves a blob
//!   TOWER_FEATURES=role-node    just flash net_bulk   # requester: pulls + verifies
//!
//! The sender announces a 200-byte blob (byte[i] = i) and answers BULK_REQ(index)
//! with BULK_DATA(index, ≤64 B); the requester pulls all 4 chunks, reassembles,
//! and verifies the pattern. Demonstrates announce → pull → reassemble with the
//! 24-bit chunk index in each chunk's nonce.

#![no_std]
#![no_main]

use embassy_time::Timer;
use log::{error, info};
#[cfg(feature = "role-node")]
use log::warn;
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig};
use tower::storage::Kv;
use tower::{app, board::Board};

const NODE_ID: u32 = 0x1111_1111;
const GW_ID: u32 = 0x2222_2222;
const KEY: [u8; 16] = [
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
];
#[cfg(not(feature = "role-node"))]
const BLOB_LEN: usize = 200;

async fn run(b: Board) {
    let radio = Spirit1::new(
        b.radio_spi, b.radio_sck, b.radio_mosi, b.radio_miso,
        b.radio_cs, b.radio_sdn, b.radio_irq,
    );
    let kv = Kv::new(b.storage);

    #[cfg(feature = "role-node")]
    let my_id = NODE_ID;
    #[cfg(not(feature = "role-node"))]
    let my_id = GW_ID;

    let mut net = match Net::new(radio, kv, NetConfig { my_id, key: KEY, band: Band::DEFAULT, channel: 0 }).await {
        Ok(n) => n,
        Err(e) => {
            error!(target: "bulk", "net init: {:?}", e);
            return;
        }
    };

    #[cfg(not(feature = "role-node"))]
    {
        info!(target: "bulk", "SENDER: serving a {}-byte blob", BLOB_LEN);
        loop {
            let mut blob = [0u8; BLOB_LEN];
            for (i, b) in blob.iter_mut().enumerate() {
                *b = i as u8;
            }
            let ok = net.bulk_serve(NODE_ID, &blob).await;
            info!(target: "bulk", "bulk_serve done (served_last={})", ok);
            Timer::after_secs(1).await;
        }
    }

    #[cfg(feature = "role-node")]
    {
        info!(target: "bulk", "REQUESTER: pulling the blob from {:08X}", GW_ID);
        loop {
            let mut out = [0u8; 256];
            match net.bulk_fetch(GW_ID, &mut out).await {
                Some(n) => {
                    let good = (0..n).all(|i| out[i] == i as u8);
                    info!(
                        target: "bulk",
                        "fetched {} bytes ({} chunks), verify {}",
                        n, n.div_ceil(64), if good { "OK ***" } else { "MISMATCH" }
                    );
                }
                None => warn!(target: "bulk", "bulk_fetch failed/timeout"),
            }
            Timer::after_secs(1).await;
        }
    }
}

app!(run);
