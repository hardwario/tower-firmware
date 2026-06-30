//! net_p2p — peer-to-peer confirmed exchange under a shared link key (docs/radio.md).
//!
//!   TOWER_FEATURES=role-peer-a just flash example net_p2p   # initiator (PING)
//!   TOWER_FEATURES=role-peer-b just flash example net_p2p   # responder (PONG)
//!
//! Two boards as symmetric peers: each registers the other with add_peer under
//! the shared per-link key and exchanges confirmed messages BOTH directions.
//! A pings B (confirmed, gets B's ACK), then listens; B receives (auto-ACKs) and
//! pongs back confirmed (gets A's ACK). Both should report Delivered each round
//! and print the peer's payload — demonstrating bidirectional confirmed P2P over
//! the half-duplex radio plus the peer table (each side holds one peer).

#![no_std]
#![no_main]

use embassy_time::Duration;
#[cfg(not(feature = "role-peer-b"))]
use embassy_time::Timer;
use log::{error, info, warn};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig, SendResult};
use tower::storage::Kv;
use tower::{app, board::Board};

const PEER_A: u32 = 0x0000_00A1;
const PEER_B: u32 = 0x0000_00B1;
const LINK_KEY: [u8; 16] = [
    0x50, 0x32, 0x50, 0x4b, 0x45, 0x59, 0x21, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
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

    #[cfg(feature = "role-peer-b")]
    let (my_id, peer_id) = (PEER_B, PEER_A);
    #[cfg(not(feature = "role-peer-b"))]
    let (my_id, peer_id) = (PEER_A, PEER_B);

    let mut net = match Net::new(
        radio,
        kv,
        NetConfig {
            my_id,
            key: LINK_KEY,
            band: Band::DEFAULT,
            channel: 0,
        },
    )
    .await
    {
        Ok(n) => n,
        Err(e) => {
            error!(target: "p2p", "net init: {e}");
            return;
        }
    };
    net.add_peer(peer_id, &LINK_KEY);
    info!(target: "p2p", "{:08X}: peer {:08X} registered ({} peer)", my_id, peer_id, net.peer_count());

    // Peer A is the initiator; peer B is the responder. Both directions are
    // confirmed, so each round produces an ACK in each direction.
    #[cfg(not(feature = "role-peer-b"))]
    {
        let mut seq: u32 = 0;
        loop {
            let mut msg = [0u8; 8];
            msg[..4].copy_from_slice(b"PING");
            msg[4] = b'0' + ((seq / 100) % 10) as u8;
            msg[5] = b'0' + ((seq / 10) % 10) as u8;
            msg[6] = b'0' + (seq % 10) as u8;
            match net.send(peer_id, &msg, true, 3).await {
                SendResult::Delivered => info!(target: "p2p", "A: PING {} Delivered", seq),
                r => warn!(target: "p2p", "A: PING {} {r}", seq),
            }
            // Listen for B's PONG (it auto-ACKs us via recv).
            if let Some(rx) = net.recv(Duration::from_millis(1500)).await {
                let text = core::str::from_utf8(rx.data()).unwrap_or("<bin>");
                info!(target: "p2p", "A: rx \"{}\" from {:08X} rssi={}dBm", text, rx.src, rx.rssi_dbm);
            }
            seq = seq.wrapping_add(1);
            Timer::after_secs(2).await;
        }
    }

    #[cfg(feature = "role-peer-b")]
    {
        let mut seq: u32 = 0;
        loop {
            if let Some(rx) = net.recv(Duration::from_secs(5)).await {
                let text = core::str::from_utf8(rx.data()).unwrap_or("<bin>");
                info!(target: "p2p", "B: rx \"{}\" from {:08X} rssi={}dBm (ACKed)", text, rx.src, rx.rssi_dbm);
                // Pong back, confirmed.
                let mut msg = [0u8; 8];
                msg[..4].copy_from_slice(b"PONG");
                msg[4] = b'0' + ((seq / 100) % 10) as u8;
                msg[5] = b'0' + ((seq / 10) % 10) as u8;
                msg[6] = b'0' + (seq % 10) as u8;
                match net.send(peer_id, &msg, true, 3).await {
                    SendResult::Delivered => info!(target: "p2p", "B: PONG {} Delivered", seq),
                    r => warn!(target: "p2p", "B: PONG {} {r}", seq),
                }
                seq = seq.wrapping_add(1);
            }
        }
    }
}

app!(run);
