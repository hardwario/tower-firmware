//! fota_app — A/B swap bench app: links into ACTIVE, boots under the bootloader, and exercises
//! the full A/B swap path on the bench (docs/fota.md). **Button-free** — drives itself
//! from the boot state, so it runs on the Radio Dongle (no user button) as well as the
//! Core Module.
//!
//!   # The Radio Dongle has no SWD, so the bootloader + this app (linked into ACTIVE) are
//!   # merged into one image (tools/fota_merge.py) and flashed over the UART bootloader,
//!   # then watched with the `tower` CLI:
//!   just flash-fota fota_app   # build bootloader + app, merge, flash (`tower flash`)
//!   just logs              # stream the framed console (`tower logs`)
//!
//! What it does, with **no button**, one self-swap per power cycle:
//!
//! - **Fresh boot** (`State::Boot`): a few seconds after start it streams a byte-identical
//!   copy of its own running ACTIVE image into DFU (via embassy-boot's `write_firmware`),
//!   reads it back to confirm the SHA-256 matches, then `mark_updated()` + resets so the
//!   bootloader swaps. The staged image is the running, known-good one, so the swap is
//!   **brick-safe + revert-safe** — it proves erase→program→swap→confirm on real hardware
//!   with no v2 image and no radio.
//! - **After the swap** (`State::Swap`): calls `mark_booted()` and prints
//!   `*** SWAP CONFIRMED ***`, then idles. Power-cycle to run another swap.
//! - **`fota-no-confirm` feature**: skips `mark_booted` so the *revert* path can be tested —
//!   power-cycle after a swap and the bootloader rolls back (`State::Revert`).
//!
//! In the real OTA flow (`fota_ota`) the source isn't the self-image but chunks pulled over
//! the radio, and the swap is gated by a signed `Manifest` verified in the **bootloader**
//! (not app-side, and not by this self-SHA) — see `docs/fota.md`.

#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m::peripheral::SCB;
use embassy_boot_stm32::{AlignedBuffer, BlockingFirmwareUpdater, FirmwareUpdaterConfig, State};
use embassy_stm32::flash::WRITE_SIZE;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Timer;
use embedded_storage::nor_flash::NorFlash;
use log::{error, info, warn};
use sha2::{Digest, Sha256};
use tower::fota::{ACTIVE_OFFSET, ACTIVE_SIZE, FLASH_BASE};
use tower::{app, board::Board};

/// Reported firmware version — bumped by the `fota-v2` feature (cosmetic; a real v1→v2
/// change comes from staging a different image over OTA — see fota_ota).
#[cfg(not(feature = "fota-v2"))]
const VERSION: u32 = 1;
#[cfg(feature = "fota-v2")]
const VERSION: u32 = 2;

/// Stage/verify granularity — one L0 flash page (erase unit); `ACTIVE_SIZE` is a multiple.
const CHUNK: usize = 128;
/// Grace period after a fresh boot before the self-swap — long enough to attach the
/// `tower` console monitor after flashing and watch the running version first.
const SWAP_DELAY_S: u64 = 8;

async fn run(b: Board) {
    info!(target: "fota-app", "firmware v{VERSION} running from the ACTIVE slot");

    // The one L0 Flash, reclaimed from the shared KV (sole owner — `no_shell`, no radio) and split
    // into its single bank region, shared through a blocking mutex — what embassy-boot's
    // FirmwareUpdater drives (it owns the DFU + STATE partitions over this same handle).
    let regions = b.kv.into_owned_flash().into_blocking_regions();
    let flash = Mutex::<NoopRawMutex, _>::new(RefCell::new(regions.bank1_region));
    let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash, &flash);
    let mut aligned = AlignedBuffer([0u8; WRITE_SIZE]);
    let mut updater = BlockingFirmwareUpdater::new(config, &mut aligned.0);

    // Boot-state machine (button-free): confirm a just-swapped image, note a revert, or —
    // on a plain good boot — arm one self-swap test. `fresh` gates the trigger.
    let fresh = match updater.get_state() {
        Ok(State::Swap) => {
            #[cfg(not(feature = "fota-no-confirm"))]
            match updater.mark_booted() {
                Ok(()) => {
                    info!(target: "fota-app", "*** SWAP CONFIRMED *** booted the swapped image, marked good")
                }
                Err(e) => warn!(target: "fota-app", "mark_booted failed: {e:?}"),
            }
            #[cfg(feature = "fota-no-confirm")]
            warn!(target: "fota-app", "swapped but NOT confirming (revert test) — power-cycle to roll back");
            false
        }
        Ok(State::Revert) => {
            info!(target: "fota-app", "*** REVERTED *** an unconfirmed image was rolled back to the previous one");
            false
        }
        Ok(State::Boot) => true, // plain good boot → run one self-swap test
        Ok(other) => {
            info!(target: "fota-app", "boot state: {other:?} (idling)");
            false
        }
        Err(e) => {
            warn!(target: "fota-app", "get_state failed: {e:?}");
            false
        }
    };

    if fresh {
        info!(target: "fota-app", "v{VERSION} healthy; self-swap test in {SWAP_DELAY_S}s (power-cycle to re-run)");
        Timer::after_secs(SWAP_DELAY_S).await;
        if stage_self_image(&mut updater).await {
            match updater.mark_updated() {
                Ok(()) => {
                    info!(target: "fota-app", "swap armed — resetting; bootloader will swap on boot");
                    Timer::after_millis(50).await; // let the console drain first
                    SCB::sys_reset();
                }
                Err(e) => error!(target: "fota-app", "mark_updated failed: {e:?}"),
            }
        }
    }

    // Idle heartbeat so the monitor shows the live version between cycles.
    let mut tick: u32 = 0;
    loop {
        info!(target: "fota-app", "v{VERSION} alive (tick {tick})");
        tick = tick.wrapping_add(1);
        Timer::after_secs(5).await;
    }
}

/// Stage a byte-identical copy of the running ACTIVE image into the DFU slot through the
/// bootloader's `write_firmware`, then read it back and confirm the SHA-256 matches.
/// Returns whether DFU now holds a verified, swap-ready image (the caller arms the swap).
///
/// The source is the currently-running, known-good image (read from memory-mapped flash),
/// so a resulting swap can't brick the device — this exercises the real flash path safely.
/// Generic over the updater's partition flash so it needn't name the concrete L0 region.
async fn stage_self_image<DFU: NorFlash, STATE: NorFlash>(
    updater: &mut BlockingFirmwareUpdater<'_, DFU, STATE>,
) -> bool {
    let len = ACTIVE_SIZE as usize; // copy the whole ACTIVE slot; DFU is larger, so it fits
    info!(target: "fota-app", "staging self-image ({} KB) into DFU — a few seconds...", len / 1024);

    // SAFETY: ACTIVE is this app's own program flash; reading it memory-mapped is sound.
    let src = unsafe { core::slice::from_raw_parts((FLASH_BASE + ACTIVE_OFFSET) as *const u8, len) };

    // 1) Program DFU from the source, folding a SHA over exactly what we wrote. The writes
    //    are blocking and never yield, so log + yield ~every 25% to keep the async console
    //    alive (else the console writer is starved and the monitor looks frozen).
    let quarter = (len / 4).max(CHUNK);
    let mut next_mark = quarter;
    let mut src_hash = Sha256::new();
    let mut off = 0usize;
    while off < len {
        let end = (off + CHUNK).min(len);
        if let Err(e) = updater.write_firmware(off, &src[off..end]) {
            error!(target: "fota-app", "write_firmware at {off} failed: {e:?}");
            return false;
        }
        src_hash.update(&src[off..end]);
        off = end;
        if off >= next_mark {
            info!(target: "fota-app", "staging {}%", off * 100 / len);
            Timer::after_millis(10).await; // let the console writer task run
            next_mark += quarter;
        }
    }

    // 2) Read DFU back and fold a second SHA — proves the bytes actually landed in flash
    //    before we arm the swap (in the real OTA flow the bootloader's signed-manifest + SHA
    //    check is the gate instead — see fota_ota / docs/fota.md).
    let mut dfu_hash = Sha256::new();
    let mut buf = [0u8; CHUNK];
    let mut o = 0usize;
    while o < len {
        let n = (len - o).min(CHUNK);
        if let Err(e) = updater.read_dfu(o as u32, &mut buf[..n]) {
            error!(target: "fota-app", "read_dfu at {o} failed: {e:?}");
            return false;
        }
        dfu_hash.update(&buf[..n]);
        o += n;
    }

    let src_sha: [u8; 32] = src_hash.finalize().into();
    let dfu_sha: [u8; 32] = dfu_hash.finalize().into();
    if src_sha == dfu_sha {
        info!(target: "fota-app", "DFU verified: sha {:02x}{:02x}{:02x}{:02x}..",
            dfu_sha[0], dfu_sha[1], dfu_sha[2], dfu_sha[3]);
        true
    } else {
        error!(target: "fota-app", "DFU SHA mismatch after staging — NOT arming swap");
        false
    }
}

app!(run, no_shell);
