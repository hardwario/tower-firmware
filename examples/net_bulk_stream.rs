//! net_bulk_stream — streaming bulk transfer, no RAM ceiling (docs/radio.md).
//!
//!   TOWER_FEATURES=role-gateway just flash net_bulk_stream   # sender: streams a blob from a source
//!   TOWER_FEATURES=role-node    just flash net_bulk_stream   # requester: streams to a verifying sink
//!
//! Proves the streaming path (`bulk_serve_from` / `bulk_fetch_into`) carries
//! transfers far larger than the old monolithic-buffer ceiling (~6 KB on this 20 KB
//! L0). **Neither board ever buffers the whole transfer:** the sender generates each
//! 64 B chunk on demand from a [`BulkSource`] and the requester verifies + CRCs each
//! chunk on the fly through a [`BulkSink`], then discards it. RAM is constant
//! regardless of size — exactly what a flash-backed FOTA source/sink would use; here
//! the sink just checks bytes instead of writing flash.
//!
//! The sender cycles **firmware-sized** blobs — 4 / 16 / 32 / 64 KB — and the
//! requester is purely announce-driven (it verifies whatever length the announce
//! declares against the reference pattern + a streamed CRC-32, so it self-syncs to
//! the sender's size each round). Per round it logs bytes / chunks / ms / bps / PASS.
//!
//! Uses **Us915** (unrestricted bench duty) so the 64 KB blob completes in ~1.5 min
//! rather than the ~12 min an EU 868 1 % duty cycle would impose — this is a
//! protocol/RAM stress, not an RF-compliance demo (single-channel Us915 is bench-only).

#![no_std]
#![no_main]

use log::{error, info};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig};
use tower::storage::Kv;
use tower::{app, board::Board};

#[cfg(feature = "role-node")]
use {embassy_time::Instant, log::warn};
#[cfg(not(feature = "role-node"))]
use embassy_time::Timer;

const NODE_ID: u32 = 0x1111_1111;
const GW_ID: u32 = 0x2222_2222;
const KEY: [u8; 16] = [
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
];

/// Firmware-sized blobs the sender cycles through (64-byte chunks → size/64 chunks).
/// 64 KB exceeds the old ~6 KB monolithic ceiling >10×; here it costs no extra RAM.
#[cfg(not(feature = "role-node"))]
const SIZES: [usize; 4] = [4096, 16384, 32768, 65536];

/// Deterministic, position-dependent pattern (varies across all chunks, so a
/// swapped/duplicated/dropped chunk is caught): byte i = i ⊕ (i>>8) ⊕ 0xA5.
fn pat(i: usize) -> u8 {
    (i as u8) ^ ((i >> 8) as u8) ^ 0xA5
}

/// CRC-32 (IEEE) of the reference pattern's first `n` bytes, computed on the fly
/// (no buffer): the sender logs it per blob, the requester checks the streamed CRC.
fn crc32_pattern(n: usize) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for i in 0..n {
        crc ^= pat(i) as u32;
        for _ in 0..8 {
            let m = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & m);
        }
    }
    !crc
}

/// Sender side: generate pattern bytes on demand — no blob is ever held in RAM.
#[cfg(not(feature = "role-node"))]
struct PatternSource {
    len: usize,
}

#[cfg(not(feature = "role-node"))]
impl tower::radio::net::BulkSource for PatternSource {
    fn total_len(&self) -> usize {
        self.len
    }
    async fn read(&mut self, offset: usize, out: &mut [u8]) -> usize {
        for (j, b) in out.iter_mut().enumerate() {
            *b = pat(offset + j);
        }
        out.len()
    }
}

/// Requester side: verify + CRC each chunk as it streams in, then discard it — no
/// reassembly buffer, so transfer size is bounded by neither RAM nor the protocol.
#[cfg(feature = "role-node")]
struct CrcCheckSink {
    total: usize,
    received: usize,
    byte_errors: u32,
    crc: u32,
}

#[cfg(feature = "role-node")]
impl CrcCheckSink {
    const fn new() -> Self {
        Self { total: 0, received: 0, byte_errors: 0, crc: 0xFFFF_FFFF }
    }
    fn final_crc(&self) -> u32 {
        !self.crc
    }
}

#[cfg(feature = "role-node")]
impl tower::radio::net::BulkSink for CrcCheckSink {
    async fn begin(&mut self, total_len: usize) -> bool {
        self.total = total_len;
        self.received = 0;
        self.byte_errors = 0;
        self.crc = 0xFFFF_FFFF;
        true
    }
    async fn consume(&mut self, offset: usize, chunk: &[u8]) -> bool {
        for (j, &b) in chunk.iter().enumerate() {
            if b != pat(offset + j) {
                self.byte_errors += 1;
            }
            self.crc ^= b as u32;
            for _ in 0..8 {
                let m = (self.crc & 1).wrapping_neg();
                self.crc = (self.crc >> 1) ^ (0xEDB8_8320 & m);
            }
        }
        self.received += chunk.len();
        true
    }
}

async fn run(b: Board) {
    let radio = Spirit1::new(
        b.radio_spi, b.radio_sck, b.radio_mosi, b.radio_miso,
        b.radio_cs, b.radio_sdn, b.radio_irq,
    );
    let kv = Kv::new(b.storage);

    #[cfg(feature = "role-node")]
    let my_id = NODE_ID;
    #[cfg(not(feature = "role-node"))]
    let my_id = GW_ID;

    let mut net = match Net::new(radio, kv, NetConfig { my_id, key: KEY, band: Band::Us915, channel: 0 }).await {
        Ok(n) => n,
        Err(e) => {
            error!(target: "stream", "net init: {:?}", e);
            return;
        }
    };

    #[cfg(not(feature = "role-node"))]
    {
        info!(target: "stream", "SENDER: streaming {:?} B to {:08X} (constant RAM, no blob buffer)", SIZES, NODE_ID);
        let mut i = 0usize;
        loop {
            let size = SIZES[i % SIZES.len()];
            let mut src = PatternSource { len: size };
            info!(target: "stream", "serving {} B ({} chunks) crc=0x{:08x}", size, size.div_ceil(64), crc32_pattern(size));
            let ok = net.bulk_serve_from(NODE_ID, &mut src).await;
            info!(target: "stream", "served {} B (served_last={})", size, ok);
            i = i.wrapping_add(1);
            Timer::after_secs(2).await;
        }
    }

    #[cfg(feature = "role-node")]
    {
        info!(target: "stream", "REQUESTER: streaming-fetch from {:08X} (constant RAM, verify-on-the-fly)", GW_ID);
        let mut sink = CrcCheckSink::new();
        let mut round: u32 = 0;
        loop {
            let t0 = Instant::now();
            match net.bulk_fetch_into(GW_ID, &mut sink).await {
                Some(n) => {
                    let ms = t0.elapsed().as_millis().max(1);
                    let crc = sink.final_crc();
                    let expect = crc32_pattern(n);
                    let ok = n == sink.total && sink.received == n && sink.byte_errors == 0 && crc == expect;
                    let bps = (n as u64 * 8 * 1000) / ms;
                    if ok {
                        info!(target: "stream", "round {} OK *** {} B ({} chunks) in {} ms = {} bps, crc=0x{:08x}",
                            round, n, n.div_ceil(64), ms, bps, crc);
                    } else {
                        error!(target: "stream", "round {} FAIL: n={} recv={} errs={} crc=0x{:08x} expect=0x{:08x}",
                            round, n, sink.received, sink.byte_errors, crc, expect);
                    }
                }
                None => warn!(target: "stream", "round {} fetch failed/timeout", round),
            }
            round = round.wrapping_add(1);
        }
    }
}

app!(run);
