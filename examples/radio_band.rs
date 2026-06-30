//! radio_band — runtime band switching via `net.set_band()` (868 ↔ 915 MHz).
//!
//!   TOWER_FEATURES=role-node    just flash example radio_band   # dwells on each band, sends
//!   TOWER_FEATURES=role-gateway just flash example radio_band   # scans both bands, receives
//!
//! One firmware image, both bands, selected at **runtime** — no rebuild, no Cargo
//! feature. The node retunes with `set_band`, dwells on each band for ~4 s and
//! sends confirmed frames tagged with the band; the gateway scans the two bands
//! (~0.9 s each), also via `set_band`, and logs which band each frame arrived on.
//! You should see deliveries on BOTH 868 and 915 — proof the live retune works.
//! (915 is for bench testing only; it is not FCC 15.247-compliant — see `Band`.)

#![no_std]
#![no_main]

#[cfg(not(feature = "role-node"))]
use embassy_time::Duration;
use log::{error, info};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig};
use tower::storage::Kv;
use tower::{app, board::Board};

#[cfg(feature = "role-node")]
use {embassy_time::Timer, log::warn, tower::radio::net::SendResult};

const NODE_ID: u32 = 0x1111_1111;
const GW_ID: u32 = 0x2222_2222;
const KEY: [u8; 16] = [
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
];
const BANDS: [Band; 2] = [Band::Eu868, Band::Us915];

fn band_mhz(b: Band) -> &'static str {
    match b {
        Band::Eu868 => "868",
        Band::Us915 => "915",
    }
}
#[cfg(feature = "role-node")]
fn band_tag(b: Band) -> u8 {
    match b {
        Band::Eu868 => 0,
        Band::Us915 => 1,
    }
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
            error!(target: "band", "net init: {e}");
            return;
        }
    };

    #[cfg(feature = "role-node")]
    {
        net.add_peer(GW_ID, &KEY);
        info!(target: "band", "NODE: cycling 868 ↔ 915 at runtime (4 s/band)");
        let mut seq: u32 = 0;
        loop {
            for &band in &BANDS {
                if let Err(e) = net.set_band(band, 0).await {
                    error!(target: "band", "set_band {}: {e}", band_mhz(band));
                    continue;
                }
                info!(target: "band", "NODE now on {} MHz", band_mhz(band));
                for _ in 0..8 {
                    let payload = [band_tag(band)];
                    match net.send(GW_ID, &payload, true, 2).await {
                        SendResult::Delivered => {
                            info!(target: "band", "  {} MHz seq={} Delivered", band_mhz(band), seq)
                        }
                        r => warn!(target: "band", "  {} MHz seq={} {r}", band_mhz(band), seq),
                    }
                    seq = seq.wrapping_add(1);
                    Timer::after_millis(500).await;
                }
            }
        }
    }

    #[cfg(not(feature = "role-node"))]
    {
        net.add_peer(NODE_ID, &KEY);
        info!(target: "band", "GATEWAY: scanning 868/915 at runtime (set_band per hop)");
        loop {
            for &band in &BANDS {
                if net.set_band(band, 0).await.is_err() {
                    continue;
                }
                if let Some(rx) = net.recv(Duration::from_millis(900)).await {
                    let tag = rx.data().first().copied().unwrap_or(0xFF);
                    info!(
                        target: "band",
                        "rx on {} MHz: src={:08X} cnt={} node_tag={} rssi={}dBm (ACKed)",
                        band_mhz(band), rx.src, rx.counter, tag, rx.rssi_dbm
                    );
                }
            }
        }
    }
}

app!(run);
