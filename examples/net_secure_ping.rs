//! net_secure_ping — the full stack end-to-end over the air: build a CCM-sealed
//! DATA frame, transmit it, receive + authenticate + decrypt on the other board.
//!
//!   TOWER_FEATURES=role-node    just flash net_secure_ping   # sends sealed frames
//!   TOWER_FEATURES=role-gateway just flash net_secure_ping   # receives + verifies
//!
//! Proves radio link + frame codec + AES-CCM together: the Gateway logs the
//! decrypted payload only if the CCM tag authenticates (a forged/tampered frame
//! is rejected, and a CRC-corrupt frame is dropped by the radio before that).

#![no_std]
#![no_main]

use embassy_time::Duration;
use log::{error, info, warn};
use tower::radio::ccm::Ccm;
use tower::radio::frame::{self, flags};
use tower::radio::{RfConfig, Spirit1, config};
use tower::{app, board::Board};

// Throwaway shared test identity/key (real keys come from provisioning, §7.6).
#[cfg(feature = "role-node")]
const NODE_ID: u32 = 0x1111_1111;
const GW_ID: u32 = 0x2222_2222;
const KEY: [u8; 16] = [
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
];

async fn run(b: Board) {
    let mut radio = Spirit1::new(
        b.radio_spi, b.radio_sck, b.radio_mosi, b.radio_miso,
        b.radio_cs, b.radio_sdn, b.radio_irq,
    );
    let _ = radio.exit_shutdown().await;
    if radio.read_device_id().is_err() {
        error!(target: "secping", "device id mismatch");
    }
    if let Err(e) = config::apply(&mut radio, &RfConfig { band: config::Band::DEFAULT, channel: 0 }).await {
        error!(target: "secping", "config: {:?}", e);
    }
    let mut ccm = Ccm::new();

    #[cfg(feature = "role-node")]
    node(&mut radio, &mut ccm).await;
    #[cfg(not(feature = "role-node"))]
    gateway(&mut radio, &mut ccm).await;
}

#[cfg(feature = "role-node")]
async fn node(radio: &mut Spirit1, ccm: &mut Ccm) -> ! {
    info!(target: "secping", "NODE {:08X}: sending CCM-sealed DATA every 1 s", NODE_ID);
    let mut counter: u32 = 1; // counter 0 = "never sent" (§6)
    loop {
        let mut payload = [0u8; 12];
        payload[..5].copy_from_slice(b"ping ");
        // append the counter as ascii-ish for readability
        let c = counter;
        payload[5] = b'0' + ((c / 100) % 10) as u8;
        payload[6] = b'0' + ((c / 10) % 10) as u8;
        payload[7] = b'0' + (c % 10) as u8;

        let hdr = frame::Header {
            frame_type: frame::FrameType::Data,
            flags: flags::CONFIRMED,
            src: NODE_ID,
            dest: GW_ID,
            counter,
            bulk_index: None,
        };
        let mut buf = [0u8; frame::MAX_FRAME];
        match frame::seal_frame(ccm, &KEY, &hdr, &payload, &mut buf) {
            Ok(n) => match radio.tx(&buf[..n], false, Duration::from_millis(200)).await {
                Ok(()) => info!(target: "secping", "tx cnt={} ok ({} B on air)", counter, n),
                Err(e) => warn!(target: "secping", "tx cnt={} failed: {:?}", counter, e),
            },
            Err(e) => error!(target: "secping", "seal: {:?}", e),
        }
        counter = counter.wrapping_add(1);
        embassy_time::Timer::after_secs(1).await;
    }
}

#[cfg(not(feature = "role-node"))]
async fn gateway(radio: &mut Spirit1, ccm: &mut Ccm) -> ! {
    info!(target: "secping", "GATEWAY {:08X}: receiving + authenticating", GW_ID);
    let mut buf = [0u8; frame::MAX_FRAME];
    loop {
        match radio.rx(&mut buf, Duration::from_secs(5)).await {
            Ok((len, q)) => match frame::open_frame(ccm, &KEY, &mut buf[..len]) {
                Ok((hdr, range)) => {
                    let pt = &buf[range];
                    let text = core::str::from_utf8(pt).unwrap_or("<bin>");
                    info!(
                        target: "secping",
                        "AUTH OK: src={:08X} cnt={} confirmed={} rssi={}dBm | \"{}\"",
                        hdr.src, hdr.counter, hdr.flags & flags::CONFIRMED != 0,
                        q.rssi_dbm, text
                    );
                }
                Err(frame::FrameError::AuthFail) => warn!(target: "secping", "CCM auth FAIL — dropped"),
                Err(e) => warn!(target: "secping", "frame error: {:?} — dropped", e),
            },
            Err(tower::radio::RadioError::Timeout) => {
                info!(target: "secping", "...idle")
            }
            Err(e) => warn!(target: "secping", "rx: {:?}", e),
        }
    }
}

app!(run);
