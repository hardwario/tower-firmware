//! net_persist — TX-counter persistence across reboot (docs/radio.md).
//!
//! Single board. On boot it logs the resumed TX counter, the persisted reserve
//! watermark, and last-seen, then sends unconfirmed frames (advancing the
//! counter). Power-cycle / reset the board: the counter must **resume at the
//! previous watermark** (jumping ahead, never reusing a value) — e.g. boot 1
//! starts at 1 (watermark 1025); after a reset it resumes at 1025 (watermark
//! 2049), skipping the unused tail of the reserved block.
//!
//!   just flash example net_persist     (then: just logs; power-cycle to watch the counter resume)

#![no_std]
#![no_main]

use embassy_time::Timer;
use log::{error, info};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig};
use tower::storage::Kv;
use tower::{app, board::Board};

const MY_ID: u32 = 0x1111_1111;
const PEER_ID: u32 = 0x2222_2222;
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
    let mut net = match Net::new(
        radio,
        kv,
        NetConfig {
            my_id: MY_ID,
            key: KEY,
            band: Band::DEFAULT,
            channel: 0,
        },
    )
    .await
    {
        Ok(n) => n,
        Err(e) => {
            error!(target: "persist", "net init: {e}");
            return;
        }
    };

    info!(
        target: "persist",
        "BOOT: resumed tx_counter={} reserve_watermark={} last_seen={}",
        net.tx_counter(), net.reserve_watermark(), net.last_seen()
    );
    info!(target: "persist", "power-cycle the board (then: just logs) to see the counter resume at the watermark");

    // Advance the counter with unconfirmed sends (no ACK needed).
    loop {
        let _ = net.send(PEER_ID, b"x", false, 1).await;
        info!(target: "persist", "sent; tx_counter now {} (watermark {})", net.tx_counter(), net.reserve_watermark());
        Timer::after_secs(2).await;
    }
}

app!(run);
