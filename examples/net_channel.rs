//! net_channel — secured confirmed link on a NON-default channel (docs/radio.md).
//!
//!   TOWER_FEATURES=role-node    just flash net_channel   # sender on ch2
//!   TOWER_FEATURES=role-gateway just flash net_channel   # receiver on ch2
//!
//! Same confirmed-delivery link as `radio_node`/`radio_gateway`, but on channel 2
//! instead of 0. Bringing the radio up on a non-zero channel exercises the channel
//! programming (CHNUM/CHSPACE) and the per-channel VCO auto-calibration in
//! `config::apply` — if the synthesizer didn't re-lock on the new channel, nothing
//! would link. Re-flash with `CHANNEL` = 0/1/2 to sweep the EU 868 sub-band and
//! confirm each is usable (the "shared-channel rule": both ends must agree, docs/radio.md).
//! Both boards MUST use the same channel.

#![no_std]
#![no_main]

use log::{error, info};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig};
use tower::storage::Kv;
use tower::{app, board::Board};

#[cfg(not(feature = "role-node"))]
use embassy_time::Duration;
#[cfg(feature = "role-node")]
use {embassy_time::Timer, log::warn, tower::radio::net::SendResult};

/// Sweep this 0/1/2 by re-flashing to verify each EU 868 channel.
const CHANNEL: u8 = 2;
const NODE_ID: u32 = 0x1111_1111;
const GW_ID: u32 = 0x2222_2222;
const KEY: [u8; 16] = [
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
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
            band: Band::DEFAULT,
            channel: CHANNEL,
        },
    )
    .await
    {
        Ok(n) => n,
        Err(e) => {
            error!(target: "chan", "net init on ch{}: {:?}", CHANNEL, e);
            return;
        }
    };

    #[cfg(feature = "role-node")]
    {
        net.add_peer(GW_ID, &KEY);
        info!(target: "chan", "NODE: confirmed sends on ch{} (VCO calibrated for this channel)", CHANNEL);
        let mut seq: u32 = 0;
        loop {
            match net.send(GW_ID, &seq.to_le_bytes(), true, 3).await {
                SendResult::Delivered => info!(target: "chan", "ch{} seq={} Delivered", CHANNEL, seq),
                r => warn!(target: "chan", "ch{} seq={} {:?}", CHANNEL, seq, r),
            }
            seq = seq.wrapping_add(1);
            Timer::after_secs(2).await;
        }
    }

    #[cfg(not(feature = "role-node"))]
    {
        net.add_peer(NODE_ID, &KEY);
        info!(target: "chan", "GATEWAY: receiving on ch{}", CHANNEL);
        loop {
            if let Some(rx) = net.recv(Duration::from_secs(10)).await {
                let seq = if rx.data().len() >= 4 {
                    u32::from_le_bytes([rx.data()[0], rx.data()[1], rx.data()[2], rx.data()[3]])
                } else {
                    0
                };
                info!(target: "chan", "ch{} rx src={:08X} seq={} cnt={} rssi={}dBm (ACKed)", CHANNEL, rx.src, seq, rx.counter, rx.rssi_dbm);
            }
        }
    }
}

app!(run);
