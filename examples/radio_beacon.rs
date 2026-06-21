//! radio_beacon / radio_sniffer — first real modulated two-board link.
//!
//! Build twice:
//!   TOWER_FEATURES=role-node    just flash radio_beacon   # TX: sends a counter
//!                                                          #     frame every 1 s
//!   TOWER_FEATURES=role-gateway just flash radio_beacon   # RX: logs each frame
//!                                                          #     + RSSI/LQI/SQI/AFC
//! (no feature defaults to the RX/sniffer role).
//!
//! The TX sends a small `[seq:4][0xA5 pad...]` payload; the RX prints the received
//! bytes, the decoded sequence number (checking for gaps), and per-packet signal
//! quality. This proves the data rate, deviation and channel filter (the digital
//! domain) on top of the Step 3 carrier — the first end-to-end packet link.

#![no_std]
#![no_main]

use embassy_time::Duration;
use log::{error, info, warn};
use tower::radio::{RfConfig, Spirit1, config};
use tower::{app, board::Board};

const CHANNEL: u8 = 0;
#[cfg(feature = "role-node")]
const PAYLOAD: usize = 16;

async fn run(b: Board) {
    let mut radio = Spirit1::new(
        b.radio_spi,
        b.radio_sck,
        b.radio_mosi,
        b.radio_miso,
        b.radio_cs,
        b.radio_sdn,
        b.radio_irq,
    );

    if let Err(e) = radio.exit_shutdown().await {
        error!(target: "radio", "exit_shutdown: {:?}", e);
    }
    if let Err(e) = radio.read_device_id() {
        error!(target: "radio", "device id: {:?}", e);
    }
    let cfg = RfConfig {
        band: config::Band::Eu868,
        channel: CHANNEL,
    };
    if let Err(e) = config::apply(&mut radio, &cfg).await {
        error!(target: "radio", "config: {:?}", e);
    }

    #[cfg(feature = "role-node")]
    beacon(&mut radio).await;
    #[cfg(not(feature = "role-node"))]
    sniffer(&mut radio).await;
}

#[cfg(feature = "role-node")]
async fn beacon(radio: &mut Spirit1) -> ! {
    info!(target: "beacon", "TX: sending a {}-byte frame every 1 s on ch{}", PAYLOAD, CHANNEL);
    let mut seq: u32 = 0;
    loop {
        let mut frame = [0xA5u8; PAYLOAD];
        frame[..4].copy_from_slice(&seq.to_le_bytes());
        match radio.tx(&frame, false, Duration::from_millis(200)).await {
            Ok(()) => info!(target: "beacon", "tx seq={} ok", seq),
            Err(e) => warn!(target: "beacon", "tx seq={} failed: {:?}", seq, e),
        }
        seq = seq.wrapping_add(1);
        embassy_time::Timer::after_secs(1).await;
    }
}

#[cfg(not(feature = "role-node"))]
async fn sniffer(radio: &mut Spirit1) -> ! {
    info!(target: "sniffer", "RX: listening on ch{} (RSSI/LQI/SQI/AFC per packet)", CHANNEL);
    let mut buf = [0u8; 96];
    let mut last_seq: Option<u32> = None;
    loop {
        match radio.rx(&mut buf, Duration::from_secs(5)).await {
            Ok((len, q)) => {
                let seq = if len >= 4 {
                    u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]])
                } else {
                    0
                };
                // Gap detection across sequence numbers.
                let gap = match last_seq {
                    Some(p) if seq > p.wrapping_add(1) => seq.wrapping_sub(p).wrapping_sub(1),
                    _ => 0,
                };
                last_seq = Some(seq);
                info!(
                    target: "sniffer",
                    "rx len={} seq={} rssi={}dBm(0x{:02X}) pqi={} sqi={} afc={}{}",
                    len, seq, q.rssi_dbm, q.rssi_raw, q.lqi, q.sqi, q.afc_raw,
                    if gap > 0 { " <gap!>" } else { "" }
                );
            }
            Err(tower::radio::RadioError::Timeout) => {
                info!(target: "sniffer", "...idle (no packet in 5 s)");
            }
            Err(e) => warn!(target: "sniffer", "rx error: {:?}", e),
        }
    }
}

app!(run);
