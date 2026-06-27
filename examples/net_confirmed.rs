//! net_confirmed — confirmed delivery with ACK + retransmit (docs/radio.md).
//!
//!   TOWER_FEATURES=role-node    just flash net_confirmed   # confirmed sender
//!   TOWER_FEATURES=role-gateway just flash net_confirmed   # receiver + auto-ACK
//!
//! Node: sends a confirmed message every 2 s and reports Delivered / NotDelivered
//! with the latency. Gateway: receives, auto-ACKs, and logs each accepted message
//! (a retransmit re-sends the cached ACK without re-delivering; a replay is dropped).
//! Watch both monitors: the Node should report `Delivered` once the Gateway ACKs.

#![no_std]
#![no_main]

use log::{error, info};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig};
use tower::storage::Kv;
use tower::{app, board::Board};

#[cfg(feature = "role-node")]
use {embassy_time::Instant, log::warn, tower::radio::net::SendResult};
#[cfg(not(feature = "role-node"))]
use embassy_time::Duration;

#[cfg(feature = "role-node")]
const NODE_ID: u32 = 0x1111_1111;
const GW_ID: u32 = 0x2222_2222;
const KEY: [u8; 16] = [
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
];

async fn run(b: Board) {
    let radio = Spirit1::new(
        b.radio_spi, b.radio_sck, b.radio_mosi, b.radio_miso,
        b.radio_cs, b.radio_sdn, b.radio_irq,
    );

    #[cfg(feature = "role-node")]
    let my_id = NODE_ID;
    #[cfg(not(feature = "role-node"))]
    let my_id = GW_ID;

    let kv = Kv::new(b.storage);
    let mut net = match Net::new(radio, kv, NetConfig { my_id, key: KEY, band: Band::DEFAULT, channel: 0 }).await {
        Ok(n) => n,
        Err(e) => {
            error!(target: "confirmed", "net init: {:?}", e);
            return;
        }
    };

    #[cfg(feature = "role-node")]
    node(&mut net).await;
    #[cfg(not(feature = "role-node"))]
    gateway(&mut net).await;
}

#[cfg(feature = "role-node")]
async fn node(net: &mut Net) -> ! {
    info!(target: "confirmed", "NODE: confirmed send every 2 s (reps=3)");
    let mut seq: u32 = 0;
    loop {
        let mut msg = [0u8; 8];
        msg[..4].copy_from_slice(b"msg ");
        msg[4] = b'0' + ((seq / 100) % 10) as u8;
        msg[5] = b'0' + ((seq / 10) % 10) as u8;
        msg[6] = b'0' + (seq % 10) as u8;

        let t0 = Instant::now();
        let r = net.send(GW_ID, &msg, true, 3).await;
        let ms = t0.elapsed().as_millis();
        match r {
            SendResult::Delivered => info!(target: "confirmed", "seq={} Delivered ({} ms)", seq, ms),
            SendResult::NotDelivered => warn!(target: "confirmed", "seq={} NotDelivered ({} ms)", seq, ms),
            SendResult::Busy => warn!(target: "confirmed", "seq={} Busy", seq),
            SendResult::DutyLimited => warn!(target: "confirmed", "seq={} DutyLimited", seq),
            SendResult::Error(e) => error!(target: "confirmed", "seq={} Error {:?}", seq, e),
            // WrongMode/NotSynced only arise in AFA/FHSS modes (not plain send).
            other => warn!(target: "confirmed", "seq={} {:?}", seq, other),
        }
        seq = seq.wrapping_add(1);
        embassy_time::Timer::after_secs(2).await;
    }
}

#[cfg(not(feature = "role-node"))]
async fn gateway(net: &mut Net) -> ! {
    info!(target: "confirmed", "GATEWAY: receiving + auto-ACK");
    loop {
        if let Some(rx) = net.recv(Duration::from_secs(10)).await {
            let text = core::str::from_utf8(rx.data()).unwrap_or("<bin>");
            info!(
                target: "confirmed",
                "rx src={:08X} cnt={} rssi={}dBm confirmed={} \"{}\" (ACKed)",
                rx.src, rx.counter, rx.rssi_dbm, rx.confirmed, text
            );
        }
    }
}

app!(run);
