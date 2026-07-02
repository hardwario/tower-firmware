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
//!   image straight into the **DFU** slot with constant RAM, folding the running image digest
//!   (used standalone by `examples/fota_stage.rs`; the OTA pull instead writes through
//!   `Net`'s own flash — see [`Net::bulk_fetch_to_flash`](crate::radio::net::Net::bulk_fetch_to_flash)).
//! - [`pull_update`] / [`PullOutcome`] — the node OTA driver: pull the signed manifest,
//!   run cheap crypto-free policy (decode / rollback floor / fits-ACTIVE), stream the
//!   image into DFU **with resume**, and stash the signed manifest for the bootloader.
//! - [`HostProxySource`] — the gateway's image source (decision #7): it holds no image and
//!   fetches each chunk from the **host** over the console link on demand.
//!
//! The signature/hash check is **not** here — it runs in the immutable **bootloader**
//! (`crates/bootloader`), which reads the stashed manifest, verifies Ed25519 + the image
//! digest against the staged DFU image, and only then arms the A/B swap. Keeping salty out of the
//! *duplicated* A/B app slots is the flash saving that makes a radio + crypto OTA node fit
//! the L083. See `docs/fota.md` for the full as-built picture (design, layout, and caveats).

mod flash;
mod hostproxy;
mod ota;
mod sink;

pub use flash::Stage;
pub use hostproxy::HostProxySource;
pub use ota::{
    PullOutcome, installed_version, promote_pending_version, pull_update, set_installed_version,
};
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
// Flash layout (docs/fota.md). The partition table + the embassy-boot swap guards live in the
// shared `fota-layout` leaf crate, so the app (here) and the standalone bootloader (which can't
// depend on `tower`) pull the SAME numbers and a drift fails BOTH builds — see that crate's docs
// for the full table and rationale. Re-exported here so device code keeps saying
// `tower::fota::DFU_OFFSET`, `super::WRITE_SIZE`, etc. (no API change from the dedup).
// ---------------------------------------------------------------------------
pub use fota_layout::{
    ACTIVE_OFFSET, ACTIVE_SIZE, BOOTLOADER_OFFSET, BOOTLOADER_SIZE, DFU_OFFSET, DFU_SIZE,
    FLASH_BASE, FLASH_TOTAL, MANIFEST_OFFSET, MANIFEST_SIZE, PAGE_SIZE, SPARE_OFFSET, STATE_OFFSET,
    STATE_SIZE, WRITE_SIZE,
};

// The one guard that needs a tower-protocol type (the MANIFEST region must hold a full signed
// manifest) stays here rather than in `fota-layout`, so that crate can remain a dependency-free
// leaf and not add a fifth tower-protocol pin to the lockstep set. All the layout-internal guards
// (alignment / contiguity / embassy-boot swap sizing) are enforced in `fota-layout`.
const _: () = assert!(MANIFEST_SIZE >= SIGNED_LEN as u32);

// ---------------------------------------------------------------------------
// FOTA EEPROM key range (docs/fota.md: reserve `0x5400+`; net uses `0x52xx/0x53xx`,
// console `0x55xx`). Values are stored via `storage::Kv` (postcard or raw bytes).
// ---------------------------------------------------------------------------

use crate::storage::{NS_FOTA, key};

// FOTA keys cross into `Net::bulk_fetch_to_flash` (a raw-`u16` progress key), so they are composed
// with [`key`](crate::storage::key) rather than held as a [`Scoped`](crate::storage::Scoped) handle.
/// KV key: download high-water mark (u32 LE) — bytes contiguously staged in DFU, for resume
/// (docs/fota.md: high-water mark, restart from the last contiguous chunk). Updated
/// periodically during a pull so a duty stall or power-cut resumes instead of restarting.
pub const KEY_DOWNLOAD_HWM: u16 = key(NS_FOTA, 0x00);
/// KV key: installed firmware version (u32 LE) — rollback protection rejects an image
/// whose `version <= installed` (docs/fota.md).
pub const KEY_INSTALLED_VERSION: u16 = key(NS_FOTA, 0x01);
/// KV key: the in-progress download's image version (u32 LE) — pairs with
/// [`KEY_DOWNLOAD_HWM`] so a resume only continues the *same* image (a different version on
/// offer ⇒ start fresh, re-erase DFU). The bootloader's SHA check is the final backstop.
pub const KEY_DOWNLOAD_IDENT: u16 = key(NS_FOTA, 0x02);
/// KV key: the *staged* image's manifest version (u32 LE), written when a manifest is stashed
/// and **promoted** to [`KEY_INSTALLED_VERSION`] once the swapped image confirms. This is how
/// the rollback floor learns the version of the image actually installed: after the swap+reset
/// the running image can no longer see the manifest, and its own compile-time constant may
/// differ from the signed `--version` — persisting the manifest version here decouples the
/// floor from that constant and prevents an endless reinstall loop (docs/fota.md).
pub const KEY_PENDING_VERSION: u16 = key(NS_FOTA, 0x03);

/// A FOTA staging error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// An underlying program-flash erase/program/read failed.
    Flash(embassy_stm32::flash::Error),
    /// The image (or an access) would exceed the slot size.
    TooLarge,
    /// A program offset or length was not [`WRITE_SIZE`]-aligned (internal invariant).
    Unaligned,
    /// A programmed chunk did not read back as written — the write was silently aborted by
    /// hardware (e.g. the L0 NOTZEROERR/FWWERR the blocking driver doesn't surface). The bytes
    /// did not land in flash. See [`Stage::program_verified`](crate::fota::Stage::program_verified).
    Verify,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Error::Flash(_) => "program-flash access failed",
            Error::TooLarge => "image exceeds the slot",
            Error::Unaligned => "offset/length not word-aligned",
            Error::Verify => "programmed bytes did not read back (silent write failure)",
        })
    }
}

/// Round `n` up to the next multiple of `to` (a power of two ≥ 1).
pub(crate) const fn round_up(n: u32, to: u32) -> u32 {
    (n + to - 1) & !(to - 1)
}
