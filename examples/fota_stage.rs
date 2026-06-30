//! fota_stage — staging verify: program flash <- generated image (docs/fota.md).
//!
//!   just flash example fota_stage      # single board, no radio, no role feature
//!   just run example fota_stage      # flash + open the monitor (catches the boot banner)
//!
//! Proves the device-side FOTA write path end-to-end **without the radio**: a
//! deterministic, firmware-sized blob is streamed 64 B at a time through
//! [`FlashSink`] into the **DFU** slot (`tower::fota::DFU_OFFSET`), exactly as
//! `bulk_fetch_into` would feed it over the air — erasing pages, programming words,
//! and folding a running SHA-256. Then the image is **read back from flash** and
//! independently re-hashed + CRC'd + byte-compared against the reference pattern.
//!
//! What it validates (the staging exit criteria): a full-size image lands in DFU
//! flash byte-perfect; the SHA the sink computed matches the SHA recomputed from
//! flash; survives the real erase/program path at size; RAM is constant (only a
//! chunk, a page read buffer, and two hash states — no image buffer).
//!
//! The odd `65535` size deliberately exercises the **partial-tail** path (a 63 B last
//! chunk padded to a 64-bit... 4-byte word with `0xFF`). Per round it logs
//! bytes / chunks / ms / SHA prefix / CRC / PASS|FAIL.

#![no_std]
#![no_main]

use embassy_time::{Instant, Timer};
use log::{error, info};
use sha2::{Digest, Sha512};
use tower::fota::{DFU_OFFSET, DFU_SIZE, FlashSink, Stage};
use tower::radio::net::{BULK_CHUNK, BulkSink};
use tower::{app, board::Board};
use tower_protocol::crc;

/// Reference image digest matching [`FlashSink::finish`]: SHA-512 truncated to 256 bits.
fn digest32(h: Sha512) -> [u8; 32] {
    let full: [u8; 64] = h.finalize().into();
    let mut out = [0u8; 32];
    out.copy_from_slice(&full[..32]);
    out
}

/// Image sizes to stage (all ≤ DFU slot). `65535` is not a multiple of 64 or 4, so it
/// drives the last-chunk pad path; the rest are clean firmware-sized blobs. L0 flash
/// programs word-by-word (slow), so a 64 KB stage takes a few seconds — progress is logged.
const SIZES: [usize; 3] = [4096, 16384, 65535];

/// Deterministic, position-dependent pattern (varies across every chunk, so a
/// swapped/dropped/duplicated chunk is caught): byte i = i ⊕ (i>>8) ⊕ 0xA5.
/// Same shape as `net_bulk_stream`'s `pat`, so the two demos cross-check each other.
fn pat(i: usize) -> u8 {
    (i as u8) ^ ((i >> 8) as u8) ^ 0xA5
}

async fn run(b: Board) {
    // Reclaim the single Flash handle from the shared KV — FOTA stages in program flash on the
    // same peripheral (disjoint region). Sole flash owner: no radio, no Net, no shell (`no_shell`).
    let mut flash = b.kv.into_owned_flash();

    info!(target: "fota", "STAGE TEST: DFU slot at offset 0x{:05x}, size {} B", DFU_OFFSET, DFU_SIZE);
    // Yield so the writer task can flush the banner + line above: flash program/erase below
    // is blocking and never yields, which would otherwise starve the async console.
    Timer::after_millis(50).await;

    let mut round: u32 = 0;
    loop {
        let mut all_ok = true;
        for &size in &SIZES {
            let t0 = Instant::now();

            // --- stage: stream the generated image through FlashSink into DFU ---
            let stage = Stage::new(&mut flash, DFU_OFFSET, DFU_SIZE);
            let mut sink = FlashSink::new(stage);
            if !sink.begin(size).await {
                error!(target: "fota", "{} B: begin refused (slot {} B)", size, DFU_SIZE);
                all_ok = false;
                continue;
            }

            let n_chunks = size.div_ceil(BULK_CHUNK).max(1);
            // Log progress ~every 25% (and yield there) so a multi-second stage is visibly
            // alive and the console drains — the blocking flash writes never yield on their own.
            let step = (n_chunks / 4).max(1);
            let mut ref_hasher = Sha512::new();
            let mut expect_crc = 0xFFFF_FFFFu32;
            let mut feed_ok = true;
            let mut chunk = [0u8; BULK_CHUNK];
            for k in 0..n_chunks {
                let off = k * BULK_CHUNK;
                let len = (size - off).min(BULK_CHUNK);
                for (j, c) in chunk[..len].iter_mut().enumerate() {
                    *c = pat(off + j);
                }
                ref_hasher.update(&chunk[..len]);
                expect_crc = crc::crc32_update(expect_crc, &chunk[..len]);
                if !sink.consume(off, &chunk[..len]).await {
                    error!(target: "fota", "{} B: consume failed at off {}", size, off);
                    feed_ok = false;
                    break;
                }
                if k > 0 && k.is_multiple_of(step) && n_chunks > 64 {
                    info!(target: "fota", "{} B: staging {}%", size, k * 100 / n_chunks);
                    Timer::after_millis(10).await; // let the console writer task run
                }
            }

            let received = sink.received() as usize;
            let staged_sha = sink.finish(); // releases the &mut flash borrow
            let expect_sha = digest32(ref_hasher);
            let expect_crc = !expect_crc;

            if !feed_ok {
                all_ok = false;
                continue;
            }

            // --- read back from flash and re-verify independently ---
            let mut rstage = Stage::new(&mut flash, DFU_OFFSET, DFU_SIZE);
            let mut rb_hasher = Sha512::new();
            let mut rb_crc = 0xFFFF_FFFFu32;
            let mut byte_errors: u32 = 0;
            let mut read_ok = true;
            let mut buf = [0u8; 128];
            let mut o = 0usize;
            while o < size {
                let n = (size - o).min(buf.len());
                if let Err(e) = rstage.read(o as u32, &mut buf[..n]) {
                    error!(target: "fota", "{} B: readback at {} failed: {e}", size, o);
                    read_ok = false;
                    break;
                }
                for (j, &got) in buf[..n].iter().enumerate() {
                    if got != pat(o + j) {
                        byte_errors += 1;
                    }
                }
                rb_hasher.update(&buf[..n]);
                rb_crc = crc::crc32_update(rb_crc, &buf[..n]);
                o += n;
            }
            let rb_sha = digest32(rb_hasher);
            let rb_crc = !rb_crc;

            let ms = t0.elapsed().as_millis().max(1);
            let ok = read_ok
                && received == size
                && byte_errors == 0
                && staged_sha == expect_sha
                && rb_sha == expect_sha
                && rb_crc == expect_crc;
            all_ok &= ok;

            if ok {
                info!(target: "fota",
                    "{} B ({} chunks) in {} ms: PASS  sha={:02x}{:02x}{:02x}{:02x}.. crc=0x{:08x}",
                    size, n_chunks, ms, rb_sha[0], rb_sha[1], rb_sha[2], rb_sha[3], rb_crc);
            } else {
                error!(target: "fota",
                    "{} B: FAIL recv={} berr={} staged_sha_ok={} rb_sha_ok={} crc_ok={}",
                    size, received, byte_errors,
                    staged_sha == expect_sha, rb_sha == expect_sha, rb_crc == expect_crc);
            }
            Timer::after_millis(50).await; // flush this size's result before the next blocking stage
        }

        info!(target: "fota", "round {} {}", round, if all_ok { "*** ALL PASS ***" } else { "had FAILURES" });
        round = round.wrapping_add(1);
        Timer::after_secs(5).await;
    }
}

app!(run, no_shell);
