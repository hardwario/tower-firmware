//! radio_node — reference sensor node (shipped happy-path app, docs/radio.md).
//!
//!   TOWER_FEATURES=role-node just flash example radio_node
//!
//! Pairs with `radio_gateway`. Every 5 s the node sends a confirmed telemetry
//! frame (sequence, battery mV, temperature ×10 °C) to the gateway and reports
//! whether it was `Delivered` (ACKed) with the round-trip latency. This is the
//! canonical use of the network layer: secure (AES-CCM), confirmed, replay-safe,
//! duty-metered uplink in ~20 lines of application code.
//!
//! `radio_gateway` and `radio_node` share the IDs + key below; in a real
//! deployment the per-node key is provisioned via OTA pairing (see `net_pairing`).

#![no_std]
#![no_main]

use embassy_time::{Duration, Instant, Timer};
use log::{error, info, warn};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig, SendResult};
use tower::storage::Kv;
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
    let kv = Kv::new(b.storage);

    let mut net = match Net::new(
        radio,
        kv,
        NetConfig {
            my_id: NODE_ID,
            key: KEY,
            band: Band::DEFAULT,
            channel: 0,
        },
    )
    .await
    {
        Ok(n) => n,
        Err(e) => {
            error!(target: "node", "net init: {e}");
            return;
        }
    };
    net.add_peer(GW_ID, &KEY); // gateway under its per-link key

    info!(target: "node", "NODE {:08X}: confirmed telemetry → GW {:08X} every 5 s", NODE_ID, GW_ID);
    let mut seq: u32 = 0;
    let (mut delivered, mut lost) = (0u32, 0u32);
    loop {
        // Synthetic sensor sample (replace with a real TMP112 / battery read).
        let vbat_mv: u16 = 3300u16.saturating_sub((seq % 64) as u16);
        let temp_c10: i16 = 215 + (seq % 20) as i16; // 21.5 .. 23.4 °C

        let mut payload = [0u8; 8];
        payload[0..4].copy_from_slice(&seq.to_le_bytes());
        payload[4..6].copy_from_slice(&vbat_mv.to_le_bytes());
        payload[6..8].copy_from_slice(&temp_c10.to_le_bytes());

        let t0 = Instant::now();
        let r = net.send(GW_ID, &payload, /*confirmed=*/ true, /*reps=*/ 3).await;
        let ms = t0.elapsed().as_millis();
        match r {
            SendResult::Delivered => {
                delivered += 1;
                info!(target: "node", "seq={} Delivered ({} ms) vbat={}mV temp={}.{}°C [ok={} lost={}]",
                    seq, ms, vbat_mv, temp_c10 / 10, temp_c10 % 10, delivered, lost);
            }
            SendResult::NotDelivered => {
                lost += 1;
                warn!(target: "node", "seq={} NotDelivered ({} ms) [ok={} lost={}]", seq, ms, delivered, lost);
            }
            other => warn!(target: "node", "seq={} {other}", seq),
        }
        seq = seq.wrapping_add(1);
        Timer::after(Duration::from_secs(5)).await;
    }
}

app!(run);
