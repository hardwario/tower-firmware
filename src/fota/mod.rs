//! FOTA — firmware-over-the-air staging (see the repo's `docs/fota.md`).
//!
//! This module is the **device side** of FOTA: the flash layout the bootloader and
//! app agree on, the program-flash writer the app uses to stage a downloaded image,
//! and the durable state it keeps in EEPROM across reboots. The signed [`Manifest`]
//! that gates the update lives in the shared [`tower_protocol::fota`] crate (so the
//! host signer and the device verifier cannot drift) and is re-exported here.
//!
//! ## What this module provides (the device side, as built)
//!
//! - [`Stage`] — a program-flash erase/program/read window over a slot (the piece
//!   [`storage`](crate::storage) lacks: it wraps the **EEPROM**, while FOTA needs
//!   **program-flash** erase/program on the same `Flash` handle).
//! - [`FlashSink`] — a [`BulkSink`](crate::radio::net::BulkSink) that streams a received
//!   image straight into the **DFU** slot with constant RAM, folding a running SHA-256
//!   (used standalone by `examples/fota_stage.rs`; the OTA pull instead writes through
//!   `Net`'s own flash — see [`Net::bulk_fetch_to_flash`](crate::radio::net::Net::bulk_fetch_to_flash)).
//! - [`pull_update`] / [`PullOutcome`] — the node OTA driver: pull the signed manifest,
//!   run cheap crypto-free policy (decode / rollback floor / fits-ACTIVE), stream the
//!   image into DFU **with resume**, and stash the signed manifest for the bootloader.
//! - [`HostProxySource`] — the gateway's image source (decision #7): it holds no image and
//!   fetches each chunk from the **host** over the console link on demand.
//!
//! The signature/hash check is **not** here — it runs in the immutable **bootloader**
//! (`crates/bootloader`), which reads the stashed manifest, verifies Ed25519 + SHA-256
//! against the staged DFU image, and only then arms the A/B swap. Keeping salty out of the
//! *duplicated* A/B app slots is the flash saving that makes a radio + crypto OTA node fit
//! the L083. See `docs/fota.md` for the full as-built picture (design, layout, and caveats).

mod flash;
mod hostproxy;
mod ota;
mod sink;

pub use flash::Stage;
pub use hostproxy::HostProxySource;
pub use ota::{PullOutcome, installed_version, pull_update, set_installed_version};
pub use sink::FlashSink;

// Re-export the shared signed-manifest types so device code says `tower::fota::Manifest`.
// The **bootloader** owns the Ed25519 verify (`verify_signed` + `VENDOR_PUBKEY`, behind
// tower-protocol's `verify` feature, which this lib does NOT enable — so salty stays out of
// the app). The app uses only the crypto-free pieces: `decode`/`supersedes` for policy, and
// the host-proxy `FOTA_MANIFEST_OFFSET`. `VENDOR_PUBKEY` is re-exported for reference.
pub use tower_protocol::fota::{
    FOTA_MANIFEST_OFFSET, MANIFEST_LEN, Manifest, SIG_LEN, SIGNED_LEN, VENDOR_PUBKEY, split_signed,
};

// ---------------------------------------------------------------------------
// Flash layout (docs/fota.md). Offsets are **from the flash start** (`0x0800_0000`) — that
// is what `embassy_stm32::flash::Flash::blocking_{read,write,erase}` take, NOT absolute
// addresses. Absolute address = `FLASH_BASE + offset`.
//
// The slots are NOT equal-sized: embassy-boot's swap (see the const guards below)
// requires DFU to be at least one page LARGER than ACTIVE, and STATE to hold per-page
// swap progress (4 B/page → ≈ ACTIVE/8 on the L0's 128 B pages). Getting these wrong makes
// the bootloader's `prepare_boot` panic at runtime → a silent/dead loader, so the guards
// below turn that into a build error.
//
// The BOOTLOADER is large (32 KB): it carries the Ed25519 verify (salty) + SHA-256, because
// the signature/hash check runs **bootloader-side**, not in the app (docs/fota.md). That keeps
// salty out of the *duplicated* A/B app slots — net flash saving — and runs the verify on a
// clean stack. The MANIFEST region is where the app stashes the signed
// manifest for the bootloader to read before a swap.
//
//   region            abs addr        offset     size    purpose
//   BOOTLOADER        0x0800_0000     0x00000    32 KB    loader + Ed25519/SHA verify + swap
//   BOOTLOADER_STATE  0x0800_8000     0x08000    12 KB    embassy-boot swap magic + progress
//   MANIFEST          0x0800_B000     0x0B000     2 KB    signed manifest staged for the loader
//   ACTIVE            0x0800_B800     0x0B800    70 KB    running app (linked here)
//   DFU               0x0801_D000     0x1D000    72 KB    staging slot (must be > ACTIVE)
//   (spare)           0x0802_F000     0x2F000     4 KB    margin / future
// ---------------------------------------------------------------------------

/// Program-flash base (absolute address of offset 0). The `blocking_*` flash API is
/// offset-relative to this; here only for documenting / computing absolute addresses.
pub const FLASH_BASE: u32 = 0x0800_0000;
/// L0 page (erase) granularity — matches embassy-stm32's `erase_size` for this part. Also
/// the bootloader's swap copy-buffer size (`BootLoader::prepare::<_,_,_,128>`).
pub const PAGE_SIZE: u32 = 128;
/// L0 word (program) granularity — matches embassy-stm32's `WRITE_SIZE` for this part.
pub const WRITE_SIZE: u32 = 4;

/// Bootloader region offset.
pub const BOOTLOADER_OFFSET: u32 = 0x0_0000;
/// Bootloader region size (holds the loader + salty Ed25519 verify + SHA-256).
pub const BOOTLOADER_SIZE: u32 = 32 * 1024;
/// embassy-boot swap-state region offset.
pub const STATE_OFFSET: u32 = 0x0_8000;
/// embassy-boot swap-state region size (sized for ACTIVE's per-page progress; see guards).
pub const STATE_SIZE: u32 = 12 * 1024;
/// MANIFEST region offset — where the app stashes the signed [`Manifest`] for the bootloader
/// to read + verify before swapping (the only image metadata that crosses the app↔loader
/// boundary). Read raw by the loader; written by the app via [`Stage`].
pub const MANIFEST_OFFSET: u32 = 0x0_B000;
/// MANIFEST region size (one or more pages; holds a [`SIGNED_LEN`]-byte signed manifest).
pub const MANIFEST_SIZE: u32 = 2 * 1024;
/// ACTIVE (running app) slot offset — the app's `memory.x` FLASH origin.
pub const ACTIVE_OFFSET: u32 = 0x0_B800;
/// ACTIVE (running app) slot size.
pub const ACTIVE_SIZE: u32 = 70 * 1024;
/// DFU (staging) slot offset — where a downloaded image is written before swap.
pub const DFU_OFFSET: u32 = 0x1_D000;
/// DFU (staging) slot size — **larger** than ACTIVE (embassy-boot swap needs the slack).
pub const DFU_SIZE: u32 = 72 * 1024;
/// Spare region offset (margin / future use).
pub const SPARE_OFFSET: u32 = 0x2_F000;
/// Total program flash on the STM32L083CZ.
pub const FLASH_TOTAL: u32 = 192 * 1024;

// Compile-time guards encoding embassy-boot's swap requirements (`assert_partitions` +
// `prepare_boot`), so a bad layout fails the BUILD instead of silently trapping the
// bootloader at runtime.
const _: () = {
    // Page alignment (L0 erases 128 B pages).
    assert!(BOOTLOADER_SIZE.is_multiple_of(PAGE_SIZE));
    assert!(STATE_OFFSET.is_multiple_of(PAGE_SIZE) && STATE_SIZE.is_multiple_of(PAGE_SIZE));
    assert!(MANIFEST_OFFSET.is_multiple_of(PAGE_SIZE) && MANIFEST_SIZE.is_multiple_of(PAGE_SIZE));
    assert!(ACTIVE_OFFSET.is_multiple_of(PAGE_SIZE) && ACTIVE_SIZE.is_multiple_of(PAGE_SIZE));
    assert!(DFU_OFFSET.is_multiple_of(PAGE_SIZE) && DFU_SIZE.is_multiple_of(PAGE_SIZE));
    // Contiguous, non-overlapping, fits in flash.
    assert!(STATE_OFFSET == BOOTLOADER_OFFSET + BOOTLOADER_SIZE);
    assert!(MANIFEST_OFFSET == STATE_OFFSET + STATE_SIZE);
    assert!(ACTIVE_OFFSET == MANIFEST_OFFSET + MANIFEST_SIZE);
    assert!(DFU_OFFSET == ACTIVE_OFFSET + ACTIVE_SIZE);
    assert!(SPARE_OFFSET == DFU_OFFSET + DFU_SIZE);
    assert!(SPARE_OFFSET <= FLASH_TOTAL);
    // The MANIFEST region must hold a full signed manifest.
    assert!(MANIFEST_SIZE >= SIGNED_LEN as u32);
    // Word-aligned programming is a subset of page alignment.
    assert!(PAGE_SIZE.is_multiple_of(WRITE_SIZE));
    // embassy-boot swap: DFU must be at least one page LARGER than ACTIVE.
    assert!(DFU_SIZE >= ACTIVE_SIZE + PAGE_SIZE);
    // embassy-boot swap state: STATE must hold the magic + 4 B of progress per ACTIVE page
    // (`2 + 4*pages` WRITE_SIZE-words). On the L0's 128 B pages that is ≈ ACTIVE/8.
    assert!(2 + 4 * (ACTIVE_SIZE / PAGE_SIZE) <= STATE_SIZE / WRITE_SIZE);
};

// ---------------------------------------------------------------------------
// FOTA EEPROM key range (docs/fota.md: reserve `0x5400+`; net uses `0x52xx/0x53xx`,
// console `0x55xx`). Values are stored via `storage::Kv` (postcard or raw bytes).
// ---------------------------------------------------------------------------

/// Base of the FOTA EEPROM key range. `0x5400` itself is currently reserved/unused — the
/// first live key is [`KEY_DOWNLOAD_HWM`] at `0x5401`.
pub const KEY_BASE: u16 = 0x5400;
/// KV key: download high-water mark (u32 LE) — bytes contiguously staged in DFU, for resume
/// (docs/fota.md: high-water mark, restart from the last contiguous chunk). Updated
/// periodically during a pull so a duty stall or power-cut resumes instead of restarting.
pub const KEY_DOWNLOAD_HWM: u16 = 0x5401;
/// KV key: installed firmware version (u32 LE) — rollback protection rejects an image
/// whose `version <= installed` (docs/fota.md).
pub const KEY_INSTALLED_VERSION: u16 = 0x5402;
/// KV key: the in-progress download's image version (u32 LE) — pairs with
/// [`KEY_DOWNLOAD_HWM`] so a resume only continues the *same* image (a different version on
/// offer ⇒ start fresh, re-erase DFU). The bootloader's SHA check is the final backstop.
pub const KEY_DOWNLOAD_IDENT: u16 = 0x5403;

/// A FOTA staging error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// An underlying program-flash erase/program/read failed.
    Flash(embassy_stm32::flash::Error),
    /// The image (or an access) would exceed the slot size.
    TooLarge,
    /// A program offset or length was not [`WRITE_SIZE`]-aligned (internal invariant).
    Unaligned,
}

/// Round `n` up to the next multiple of `to` (a power of two ≥ 1).
pub(crate) const fn round_up(n: u32, to: u32) -> u32 {
    (n + to - 1) & !(to - 1)
}
