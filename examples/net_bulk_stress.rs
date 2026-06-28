//! net_bulk_stress — large bulk transfer stress (docs/radio.md).
//!
//!   TOWER_FEATURES=role-gateway just flash net_bulk_stress   # sender: serves a big blob
//!   TOWER_FEATURES=role-node    just flash net_bulk_stress   # requester: pulls + verifies
//!
//! Like `net_bulk` but a **multi-KB** blob over many chunks, to stress the pull
//! state machine (24-bit chunk index, per-chunk req/resp + retries, session-counter
//! nonces) and exercise sustained TX/RX at the 4 MHz SPI. The requester reassembles,
//! checks every byte against the pattern AND a CRC-32, and reports bytes / chunks /
//! elapsed / throughput / PASS-FAIL each round.
//!
//! Hardware-measured on two boards (4 MHz SPI):
//! - 4 KB / 64 chunks and 6 KB / 96 chunks complete reliably, CRC OK every round,
//!   ~4–6 kbps effective (dominated by the per-chunk req/resp round-trip, not airtime).
//! - **Limits.** This example's receive buffer is monolithic, so its cap is RAM, not
//!   the protocol (the 24-bit index allows 1 GB): 8 KB (~11.9 KB future → ~8.6 KB
//!   stack left) overflows the L0 stack — keep blobs ≲ 6 KB *here*. For larger
//!   transfers use the **streaming** `bulk_serve_from`/`bulk_fetch_into` API
//!   (`net_bulk_stream` demos it to 64 KB at constant RAM). On EU 868 the 1 % duty governor caps
//!   *sustained* bulk (correct regulatory behaviour): a ~4 KB transfer costs ~2.7 s
//!   of gateway airtime, the bucket holds 36 s (1 % of an hour) and refills at
//!   10 ms/s, so you can burst ~13 transfers, then it throttles to ~1 per ~4.5 min
//!   (the 1 % rate). A single transfer is always unaffected.

#![no_std]
#![no_main]

use log::{error, info};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig};
use tower::storage::Kv;
use tower::{app, board::Board};

use embassy_time::Timer;
#[cfg(feature = "role-node")]
use {embassy_time::Instant, log::warn};

const NODE_ID: u32 = 0x1111_1111;
const GW_ID: u32 = 0x2222_2222;
const KEY: [u8; 16] = [
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
];
/// Blob size to stress (64-byte chunks → BLOB_LEN/64 chunks). ≲ 6 KB on this 20 KB
/// L0 — a monolithic receive buffer larger than that starves the task stack.
const BLOB_LEN: usize = 4096;

/// Deterministic, position-dependent pattern (varies across all chunks, so a
/// swapped/duplicated/dropped chunk is caught): byte i = i ⊕ (i>>8) ⊕ 0xA5.
fn pat(i: usize) -> u8 {
    (i as u8) ^ ((i >> 8) as u8) ^ 0xA5
}

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            let m = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & m);
        }
    }
    !crc
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
            error!(target: "bulk", "net init: {:?}", e);
            return;
        }
    };

    #[cfg(not(feature = "role-node"))]
    {
        let mut blob = [0u8; BLOB_LEN];
        for (i, x) in blob.iter_mut().enumerate() {
            *x = pat(i);
        }
        info!(target: "bulk", "SENDER: serving a {}-byte blob ({} chunks), crc=0x{:08x}", BLOB_LEN, BLOB_LEN.div_ceil(64), crc32(&blob));
        loop {
            let ok = net.bulk_serve(NODE_ID, &blob).await;
            info!(target: "bulk", "bulk_serve done (served_last={})", ok);
            Timer::after_secs(1).await;
        }
    }

    #[cfg(feature = "role-node")]
    {
        info!(target: "bulk", "REQUESTER: pulling {} B from {:08X}", BLOB_LEN, GW_ID);
        let mut out = [0u8; BLOB_LEN];
        let mut round: u32 = 0;
        loop {
            let t0 = Instant::now();
            match net.bulk_fetch(GW_ID, &mut out).await {
                Some(n) => {
                    let ms = t0.elapsed().as_millis().max(1);
                    let bytes_ok = n == BLOB_LEN && (0..n).all(|i| out[i] == pat(i));
                    let crc = crc32(&out[..n]);
                    let crc_ok = crc == crc32_pattern();
                    let bps = (n as u64 * 8 * 1000) / ms;
                    if bytes_ok && crc_ok {
                        info!(target: "bulk", "round {} OK *** {} B ({} chunks) in {} ms = {} bps, crc=0x{:08x}",
                            round, n, n.div_ceil(64), ms, bps, crc);
                    } else {
                        error!(target: "bulk", "round {} FAIL: n={} bytes_ok={} crc_ok={} (crc=0x{:08x})",
                            round, n, bytes_ok, crc_ok, crc);
                    }
                }
                None => warn!(target: "bulk", "round {} bulk_fetch failed/timeout", round),
            }
            round = round.wrapping_add(1);
            Timer::after_secs(1).await;
        }
    }
}

/// CRC-32 of the full reference pattern (computed once, no big buffer needed).
#[cfg(feature = "role-node")]
fn crc32_pattern() -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for i in 0..BLOB_LEN {
        crc ^= pat(i) as u32;
        for _ in 0..8 {
            let m = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & m);
        }
    }
    !crc
}

app!(run);
