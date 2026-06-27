//! radio_csma — CSMA/CCA: defer TX while the channel is busy (docs/radio.md).
//!
//!   TOWER_FEATURES=role-gateway just flash radio_csma   # jammer: CW 3 s on / 3 s off
//!   TOWER_FEATURES=role-node    just flash radio_csma   # sender: TX with CSMA enabled
//!
//! The jammer holds an unmodulated carrier for 3 s, then releases it for 3 s,
//! repeating. The sender transmits a small frame every 600 ms with CSMA/CCA
//! enabled: while the carrier is up its RSSI exceeds the −90 dBm threshold, so the
//! radio backs off and (after MAX_NB attempts) reports `Busy` — it never blocks.
//! When the carrier drops, the very next CSMA TX goes out `ok`. Watch the sender:
//! it should print runs of `Busy` during the jam and `ok` when the channel clears.

#![no_std]
#![no_main]

use embassy_time::Timer;
use log::{error, info};
#[cfg(feature = "role-node")]
use {embassy_time::Duration, log::warn, tower::radio::RadioError};
use tower::radio::{RfConfig, Spirit1, config};
use tower::{app, board::Board};

const CHANNEL: u8 = 0;

async fn run(b: Board) {
    let mut radio = Spirit1::new(
        b.radio_spi, b.radio_sck, b.radio_mosi, b.radio_miso,
        b.radio_cs, b.radio_sdn, b.radio_irq,
    );
    if let Err(e) = radio.exit_shutdown().await {
        error!(target: "csma", "exit_shutdown: {:?}", e);
    }
    if let Err(e) = radio.read_device_id() {
        error!(target: "csma", "device id: {:?}", e);
    }
    if let Err(e) = config::apply(&mut radio, &RfConfig { band: config::Band::DEFAULT, channel: CHANNEL }).await {
        error!(target: "csma", "config: {:?}", e);
    }

    #[cfg(feature = "role-node")]
    sender(&mut radio).await;
    #[cfg(not(feature = "role-node"))]
    jammer(&mut radio).await;
}

#[cfg(feature = "role-node")]
async fn sender(radio: &mut Spirit1) -> ! {
    info!(target: "csma", "SENDER: CSMA TX every 600 ms on ch{} (expect Busy while jammed)", CHANNEL);
    let mut seq: u32 = 0;
    let (mut ok, mut busy) = (0u32, 0u32);
    loop {
        let mut frame = [0xC5u8; 16];
        frame[..4].copy_from_slice(&seq.to_le_bytes());
        match radio.tx(&frame, /*use_csma=*/ true, Duration::from_millis(400)).await {
            Ok(()) => {
                ok += 1;
                info!(target: "csma", "seq={} ok (channel clear) [ok={} busy={}]", seq, ok, busy);
            }
            Err(RadioError::Busy) => {
                busy += 1;
                warn!(target: "csma", "seq={} Busy — CCA backed off (channel held) [ok={} busy={}]", seq, ok, busy);
            }
            Err(e) => warn!(target: "csma", "seq={} {:?}", seq, e),
        }
        seq = seq.wrapping_add(1);
        Timer::after_millis(600).await;
    }
}

#[cfg(not(feature = "role-node"))]
async fn jammer(radio: &mut Spirit1) -> ! {
    info!(target: "csma", "JAMMER: holding CW carrier 3 s on / 3 s off on ch{}", CHANNEL);
    loop {
        info!(target: "csma", "carrier ON (channel busy)");
        let _ = radio.cw_test(true).await;
        Timer::after_secs(3).await;
        info!(target: "csma", "carrier OFF (channel clear)");
        let _ = radio.cw_test(false).await;
        Timer::after_secs(3).await;
    }
}

app!(run);
