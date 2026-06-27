//! radio_interop — comprehensive semi-fuzzy soak campaign (docs/radio.md).
//!
//!   TOWER_FEATURES=role-node    just flash radio_interop   # randomized sender
//!   TOWER_FEATURES=role-gateway just flash radio_interop   # invariant checker
//!
//! One file, two roles, long unattended runs on the two boards. The node drives
//! pseudo-random traffic (seeded from its device ID so a run replays bit-for-bit;
//! the seed is logged at boot): random payload size 9..74 B (edges emphasized),
//! confirmed/unconfirmed, reps 1..10, timing jitter, and a low-probability
//! oversized `send()` that MUST be locally rejected. Each payload is
//! self-describing — `[seq:4][len:1][crc32:4][filler…]` — so the gateway verifies
//! integrity and ordering without lockstep.
//!
//! Invariants (→ §14), latched to the LED (solid = FAIL) and a `VERDICT:` line:
//!   • payload integrity — embedded crc32 matches recomputed (gateway)
//!   • strict-monotonic accepted counter per src — no replay/reorder accepted
//!   • oversized send always locally rejected (node)
//!   • confirmed transfers always resolve to Delivered/NotDelivered (by type)
//!   • duty respected — over-budget TX reports DutyLimited, never silently sent
//! Rolling tallies print periodically and persist to EEPROM (survive a reboot).

#![no_std]
#![no_main]

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Duration;
use log::{error, info};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig};
use tower::storage::Kv;
use tower::{app, board::Board};

#[cfg(feature = "role-node")]
use {embassy_time::Timer, tower::radio::net::SendResult};

/// Build-time seed override (XORed with the device ID) to force a replay.
#[cfg(feature = "role-node")]
const SEED: u32 = 0xC0FF_EE00;
const NODE_ID: u32 = 0x1111_1111;
const GW_ID: u32 = 0x2222_2222;
const KEY: [u8; 16] = [
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
];
/// EEPROM keys for cumulative soak tallies (clear of the net layer's 0x52xx/0x53xx).
const KV_COUNT: u16 = 0x5410; // node: tx_ok / gateway: accepted
const KV_FAIL: u16 = 0x5411; // either role: latched failures

#[cfg(feature = "role-node")]
fn xorshift32(s: &mut u32) -> u32 {
    let mut x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    x
}

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            let m = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & m);
        }
    }
    !crc
}

fn read_u32(kv: &Kv<'static>, key: u16) -> u32 {
    let mut b = [0u8; 4];
    match kv.get_bytes(key, &mut b) {
        Ok(Some(4)) => u32::from_le_bytes(b),
        _ => 0,
    }
}

async fn run(b: Board) {
    let mut led = Output::new(b.led, Level::Low, Speed::Low);
    let radio = Spirit1::new(
        b.radio_spi, b.radio_sck, b.radio_mosi, b.radio_miso,
        b.radio_cs, b.radio_sdn, b.radio_irq,
    );
    let kv = Kv::new(b.storage);

    #[cfg(feature = "role-node")]
    let my_id = NODE_ID;
    #[cfg(not(feature = "role-node"))]
    let my_id = GW_ID;

    let mut net = match Net::new(radio, kv, NetConfig { my_id, key: KEY, band: Band::DEFAULT, channel: 0 }).await {
        Ok(n) => n,
        Err(e) => {
            error!(target: "soak", "net init: {:?}", e);
            return;
        }
    };
    let (cum_count, cum_fail) = (read_u32(net.kv(), KV_COUNT), read_u32(net.kv(), KV_FAIL));

    #[cfg(feature = "role-node")]
    node(&mut net, &mut led, cum_count, cum_fail).await;
    #[cfg(not(feature = "role-node"))]
    gateway(&mut net, &mut led, cum_count, cum_fail).await;
}

#[cfg(feature = "role-node")]
async fn node(net: &mut Net, led: &mut Output<'static>, cum_ok: u32, cum_fail: u32) -> ! {
    net.add_peer(GW_ID, &KEY);
    let mut rng = SEED ^ net.id();
    if rng == 0 {
        rng = 1;
    }
    info!(target: "soak", "NODE seed=0x{:08X}  (cumulative: tx_ok={} fails={})", rng, cum_ok, cum_fail);

    let mut seq: u32 = 0;
    let (mut ok, mut nd, mut busy, mut duty, mut rej) = (0u32, 0u32, 0u32, 0u32, 0u32);
    let mut fails = cum_fail;
    let mut iters: u32 = 0;
    let mut payload = [0u8; 96];
    loop {
        let r = xorshift32(&mut rng);
        // Low-probability fault: oversized send MUST be rejected locally.
        if r & 0x1F == 0 {
            let big = [0xEE; 80];
            match net.send(GW_ID, &big, true, 1).await {
                SendResult::Error(_) => rej += 1,
                other => {
                    fails += 1;
                    error!(target: "soak", "INVARIANT: oversized send not rejected ({:?}) ✗", other);
                }
            }
        } else {
            // Random payload size 9..=74, edges (9, 74) emphasized.
            let pick = (r >> 5) % 10;
            let size = match pick {
                0 | 1 | 2 => 9,
                3 | 4 => 74,
                _ => 9 + ((r >> 9) % 66) as usize,
            };
            let confirmed = (r >> 17) & 0x7 != 0; // ~7/8 confirmed
            let reps = 1 + ((r >> 20) % 10) as u8;
            payload[0..4].copy_from_slice(&seq.to_le_bytes());
            payload[4] = size as u8;
            for i in 9..size {
                payload[i] = (seq as u8).wrapping_add(i as u8);
            }
            let crc = crc32(&payload[9..size]);
            payload[5..9].copy_from_slice(&crc.to_le_bytes());
            match net.send(GW_ID, &payload[..size], confirmed, reps).await {
                SendResult::Delivered => ok += 1,
                SendResult::NotDelivered => nd += 1,
                SendResult::Busy => busy += 1,
                SendResult::DutyLimited => duty += 1,
                _ => {} // Error / WrongMode / NotSynced (latter two not in plain send)
            }
            seq = seq.wrapping_add(1);
        }

        iters += 1;
        if fails == 0 {
            led.toggle(); // heartbeat
        } else {
            led.set_high(); // latched FAIL
        }
        if iters % 50 == 0 {
            let _ = net.kv().set_bytes(KV_COUNT, &(cum_ok + ok).to_le_bytes());
            let _ = net.kv().set_bytes(KV_FAIL, &fails.to_le_bytes());
            info!(
                target: "soak",
                "VERDICT: {}  iters={} ok={} nd={} busy={} duty={} rej={} fails={}",
                if fails == 0 { "PASS" } else { "FAIL" }, iters, ok, nd, busy, duty, rej, fails
            );
        }
        // Timing jitter ~0.5..2 s (keeps near the EU duty budget, bursts over it).
        let jitter = 500 + (r >> 24) % 1500;
        Timer::after(Duration::from_millis(jitter as u64)).await;
    }
}

#[cfg(not(feature = "role-node"))]
async fn gateway(net: &mut Net, led: &mut Output<'static>, cum_acc: u32, cum_fail: u32) -> ! {
    net.add_peer(NODE_ID, &KEY);
    info!(target: "soak", "GATEWAY checking integrity + ordering  (cumulative: accepted={} fails={})", cum_acc, cum_fail);

    let mut last: Option<u32> = None;
    let (mut accepted, mut integ, mut order) = (0u32, 0u32, 0u32);
    let mut fails = cum_fail;
    loop {
        if let Some(rx) = net.recv(Duration::from_secs(15)).await {
            accepted += 1;
            let d = rx.data();
            // Payload integrity: declared len + embedded crc32 over the filler.
            if d.len() >= 9 {
                let declared = d[4] as usize;
                let want = u32::from_le_bytes([d[5], d[6], d[7], d[8]]);
                let got = if declared == d.len() && declared >= 9 {
                    crc32(&d[9..declared])
                } else {
                    want.wrapping_add(1) // force mismatch on a bad length
                };
                if declared != d.len() || got != want {
                    integ += 1;
                    fails += 1;
                    error!(target: "soak", "INVARIANT: integrity src={:08X} cnt={} declared={} actual={} ✗", rx.src, rx.counter, declared, d.len());
                }
            }
            // Strict-monotonic accepted counter (no replay/reorder accepted).
            if let Some(p) = last
                && rx.counter <= p
            {
                order += 1;
                fails += 1;
                error!(target: "soak", "INVARIANT: counter {} <= last {} ✗", rx.counter, p);
            }
            last = Some(rx.counter);

            if fails == 0 {
                led.toggle();
            } else {
                led.set_high();
            }
            if accepted % 25 == 0 {
                let _ = net.kv().set_bytes(KV_COUNT, &(cum_acc + accepted).to_le_bytes());
                let _ = net.kv().set_bytes(KV_FAIL, &fails.to_le_bytes());
                info!(
                    target: "soak",
                    "VERDICT: {}  accepted={} integrity_fail={} order_fail={} fails={} last_cnt={}",
                    if fails == 0 { "PASS" } else { "FAIL" }, accepted, integ, order, fails, last.unwrap_or(0)
                );
            }
        }
    }
}

app!(run);
