//! fhss_compliance — F10: §15.247 compliance evidence from a board log (1 board).
//!
//!   just flash example fhss_compliance
//!
//! Runs the FHSS hop-master for >1 full cycle (80 slots) and then reports, from the
//! per-channel airtime counters, the three §15.247(a)(1)(i) facts the hop schedule
//! guarantees: (1) ≥50 distinct channels are used, (2) no channel's transmitted
//! airtime approaches the 0.4 s/20 s limit, and (3) channel use is even (each
//! channel is visited exactly once per cycle by the Fisher-Yates permutation). This
//! is master-only (beacons just here, no node), so the airtime per channel is one
//! beacon ToA — the structural bound (cycle 24 s > 20 s ⇒ ≤ one 300 ms slot/channel/
//! 20 s) is what keeps a *loaded* channel compliant too. Capture with `--reset`.

#![no_std]
#![no_main]

use log::{error, info};
use tower::radio::Spirit1;
use tower::radio::config::{FHSS_N, fhss_freq_hz};
use tower::radio::net::{FhssConfig, FhssRole, Net, NetConfig};
use tower::{app, board::Board};

const GW_ADDR: u32 = 0x2222_2222;
const KEY: [u8; 16] = [
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
];
/// §15.247(a)(1)(i): ≤ 0.4 s average occupancy per channel per 20 s.
const LIMIT_MS: u32 = 400;
/// Slots to run: > one 80-slot cycle so every channel is visited.
const SLOTS: u32 = 90;

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
    let mut net = match Net::new(
        radio,
        b.kv,
        NetConfig {
            addr: GW_ADDR,
            key: KEY,
            band: tower::radio::config::Band::Us915,
            channel: 0,
        },
    )
    .await
    {
        Ok(n) => n,
        Err(e) => {
            error!(target: "fhssc", "net init: {e}");
            return;
        }
    };
    if let Err(e) = net.enable_fhss(FhssRole::Master, FhssConfig::default()).await {
        error!(target: "fhssc", "enable_fhss: {e}");
        return;
    }
    info!(target: "fhssc", "running hop-master for {} slots (>1 cycle of {})…", SLOTS, FHSS_N);

    let mut visited = [false; FHSS_N as usize];
    for _ in 0..SLOTS {
        let Some(s) = net.fhss_master_tick().await else {
            continue; // master mode not active (shouldn't happen here — we enabled it above)
        };
        if (s.channel as usize) < FHSS_N as usize {
            visited[s.channel as usize] = true;
        }
    }

    // Tally: distinct channels used + per-channel airtime extremes.
    let used = visited.iter().filter(|&&v| v).count();
    let (mut max_ms, mut min_ms, mut sum_ms) = (0u32, u32::MAX, 0u32);
    for ch in 0..FHSS_N {
        let a = net.fhss_channel_airtime_ms(ch);
        max_ms = max_ms.max(a);
        min_ms = min_ms.min(a);
        sum_ms += a;
    }
    let mean_ms = sum_ms / FHSS_N as u32;
    let edges_ok = visited[0] && visited[FHSS_N as usize - 1]; // band edges exercised

    info!(target: "fhssc", "channels used = {} / {}  (≥50: {})", used, FHSS_N, used >= 50);
    info!(
        target: "fhssc",
        "per-channel airtime: max={}ms min={}ms mean={}ms  (≤{}ms: {})",
        max_ms, min_ms, mean_ms, LIMIT_MS, max_ms <= LIMIT_MS
    );
    info!(target: "fhssc", "band edges used: ch0 ({}Hz) + ch{} ({}Hz): {}", fhss_freq_hz(0), FHSS_N - 1, fhss_freq_hz(FHSS_N - 1), edges_ok);

    let pass = used >= 50 && max_ms <= LIMIT_MS && edges_ok;
    if pass {
        info!(target: "fhssc", "F10 COMPLIANCE: PASS *** ({} ch, max {}ms ≤ {}ms)", used, max_ms, LIMIT_MS);
    } else {
        error!(target: "fhssc", "F10 COMPLIANCE: FAIL");
    }
    loop {
        embassy_time::Timer::after_secs(5).await;
    }
}

app!(run);
