//! OTA install driver (docs/fota.md) — the node-side control flow that turns a
//! "downlink pending" advertisement into a staged, swap-ready image.
//!
//! The Ed25519 signature + SHA-256 verify lives in the **bootloader** (docs/fota.md) — that
//! keeps salty out of the duplicated A/B app slots (so a radio + crypto OTA
//! node fits the L083) and runs the verify on the loader's clean stack. So the app's job is
//! light and crypto-free:
//!
//! 1. pull the 116-byte signed [`Manifest`] ([`Net::bulk_fetch`]);
//! 2. cheap **policy** checks — `Manifest::decode` + rollback [`supersedes`](Manifest::supersedes)
//!    + fits-ACTIVE (no signature check here);
//! 3. stream the image into the DFU slot ([`Net::bulk_fetch_to_flash`]);
//! 4. **stash the signed manifest** in the MANIFEST region for the bootloader, *last* — so a
//!    crash before this leaves no manifest → no swap → the node simply re-pulls.
//!
//! The app does **not** arm the swap. On the next reset the bootloader reads the stashed
//! manifest, verifies signature + hash against the DFU image, and only then swaps. A
//! forged/corrupt image the app happens to stage is therefore rejected by the bootloader
//! (no swap); the node keeps running the old image and notices it's still the old version.
//!
//! Resume across a power-cut or duty stall is **wired in**: [`pull_update`] continues from the
//! persisted high-water mark (`KEY_DOWNLOAD_HWM`, paired with `KEY_DOWNLOAD_IDENT` so only the
//! *same* image resumes) instead of restarting, and returns [`PullOutcome::InProgress`] when a
//! pull stops short — call it again to continue (no re-erase, no re-download).

use super::{
    ACTIVE_SIZE, DFU_OFFSET, DFU_SIZE, KEY_DOWNLOAD_HWM, KEY_DOWNLOAD_IDENT, KEY_INSTALLED_VERSION,
    MANIFEST_LEN, MANIFEST_OFFSET, MANIFEST_SIZE, Manifest, SIGNED_LEN, Stage,
};
use crate::radio::net::Net;
use crate::storage::Kv;

/// Bench diagnostics (`fota-diag` feature): log a pull stage, then drain the console so the
/// line reaches the UART before the next step (the task-based console can't flush after a
/// fault, so the last line printed localizes a halt). Expands to nothing without the feature.
macro_rules! diag {
    ($($a:tt)*) => {{
        #[cfg(feature = "fota-diag")]
        {
            log::info!(target: "fota-diag", $($a)*);
            embassy_time::Timer::after_millis(500).await;
        }
    }};
}

/// Why [`pull_update`] stopped — or that an image is staged + the manifest stashed (the
/// bootloader will verify + swap on the next reset).
#[derive(Debug, Clone, PartialEq, Eq)]
#[must_use]
pub enum PullOutcome {
    /// No signed manifest arrived (the announce/transfer failed, or it was too short).
    NoManifest,
    /// The manifest didn't decode (bad magic/format) — corrupt or not a manifest.
    Malformed,
    /// `version <= installed` — refuse the rollback (the cheap app-side policy gate).
    NotNewer { version: u32, installed: u32 },
    /// The declared image size won't fit the ACTIVE slot.
    TooLarge { size: u32 },
    /// The manifest couldn't be stashed (a flash fault after a complete download).
    ImageFailed,
    /// The image is **partly** staged: this pull made progress (or none) but didn't finish —
    /// e.g. the EU duty budget ran out, or chunks were lost. The high-water mark is persisted;
    /// **call `pull_update` again** (when idle / after the duty bucket refills) to resume from
    /// where it stopped, without re-downloading or re-erasing.
    InProgress { staged: u32, total: u32 },
    /// The full image is in DFU and the signed manifest is stashed — reset and the bootloader
    /// will verify (Ed25519 + SHA-256) and swap. The caller persists `manifest.version` after a
    /// confirmed boot with [`set_installed_version`].
    Staged { manifest: Manifest },
}

/// Pull + stage one update from `gateway` (docs/fota.md), **resuming** a partial download.
/// Fetches the signed manifest, runs the cheap policy checks (decode / rollback / fits-ACTIVE —
/// **no crypto**), then streams the image into DFU from the persisted high-water mark (so a
/// duty stall or reboot continues instead of restarting), and on completion stashes the signed
/// manifest for the bootloader. Returns [`PullOutcome::Staged`] when the full image is staged
/// (the bootloader verifies + swaps on reset), or [`PullOutcome::InProgress`] when more is
/// needed — **call again to resume**. Touches flash only via the DFU + MANIFEST regions + the
/// EEPROM resume keys; the caller still owns `net`.
pub async fn pull_update(net: &mut Net, gateway: u32) -> PullOutcome {
    let installed = installed_version(net.kv());
    diag!("start: installed=v{installed}, fetching manifest");

    // 1) Pull the signed manifest (116 B) — kept verbatim for the bootloader to verify.
    let mut signed = [0u8; SIGNED_LEN];
    match net.bulk_fetch(gateway, &mut signed).await {
        Some(n) if n >= SIGNED_LEN => {}
        _ => return PullOutcome::NoManifest,
    }
    // 2) Cheap policy (no signature check — that's the bootloader's job).
    let manifest = match Manifest::decode(&signed[..MANIFEST_LEN]) {
        Some(m) => m,
        None => return PullOutcome::Malformed,
    };
    if !manifest.supersedes(installed) {
        return PullOutcome::NotNewer {
            version: manifest.version,
            installed,
        };
    }
    if manifest.size > ACTIVE_SIZE {
        return PullOutcome::TooLarge { size: manifest.size };
    }

    // 3) Decide the resume point: continue this image if the persisted download identity
    //    matches, else start fresh (which re-erases DFU on the first chunk).
    let start = resume_offset(net, manifest.version, manifest.size);
    diag!(
        "manifest v{} size={}, resume from {start}",
        manifest.version,
        manifest.size
    );

    // 4) Stream into DFU from `start`, persisting the high-water mark as it goes.
    let staged = net
        .bulk_fetch_to_flash(gateway, DFU_OFFSET, DFU_SIZE, start, Some(KEY_DOWNLOAD_HWM))
        .await as u32;
    if staged < manifest.size {
        diag!("partial: {staged}/{} B staged — resume next cycle", manifest.size);
        return PullOutcome::InProgress {
            staged,
            total: manifest.size,
        };
    }

    // 5) Complete: clear the resume state, then stash the manifest LAST (a crash before this
    //    leaves no manifest → the bootloader won't swap and the node re-pulls).
    clear_download(net);
    diag!("image staged: {staged} B, stashing manifest");
    if !stash_manifest(net, &signed) {
        return PullOutcome::ImageFailed;
    }
    diag!("staged — reset to let the bootloader verify + swap");
    PullOutcome::Staged { manifest }
}

/// Resume point for `version`/`size`: the persisted high-water mark if the in-progress
/// download is the **same** image ([`KEY_DOWNLOAD_IDENT`] == `version`), else `0` after
/// recording this image as the new in-progress one (so `bulk_fetch_to_flash` re-erases DFU).
fn resume_offset(net: &mut Net, version: u32, size: u32) -> u32 {
    if get_u32(net, KEY_DOWNLOAD_IDENT) == Some(version) {
        get_u32(net, KEY_DOWNLOAD_HWM).unwrap_or(0).min(size)
    } else {
        let _ = net.kv().set_bytes(KEY_DOWNLOAD_IDENT, &version.to_le_bytes());
        let _ = net.kv().set_bytes(KEY_DOWNLOAD_HWM, &0u32.to_le_bytes());
        0
    }
}

/// Clear the resume state after a completed download, so it isn't mistaken for a partial one
/// and a later (different) image starts fresh.
fn clear_download(net: &mut Net) {
    let _ = net.kv().set_bytes(KEY_DOWNLOAD_HWM, &0u32.to_le_bytes());
    let _ = net.kv().set_bytes(KEY_DOWNLOAD_IDENT, &0u32.to_le_bytes());
}

/// Read a u32 LE value from EEPROM key `key`, or `None` if absent.
fn get_u32(net: &mut Net, key: u16) -> Option<u32> {
    let mut buf = [0u8; 4];
    match net.kv().get_bytes(key, &mut buf) {
        Ok(Some(n)) if n >= 4 => Some(u32::from_le_bytes(buf)),
        _ => None,
    }
}

/// Write the signed manifest into the MANIFEST region (erase the page, program the blob) so
/// the bootloader can read + verify it before swapping. `SIGNED_LEN` (116) is word-aligned.
fn stash_manifest(net: &mut Net, signed: &[u8; SIGNED_LEN]) -> bool {
    let mut stage = Stage::new(net.kv().storage_mut().flash_mut(), MANIFEST_OFFSET, MANIFEST_SIZE);
    stage.erase(SIGNED_LEN as u32).is_ok() && stage.program(0, signed).is_ok()
}

/// The installed firmware version from EEPROM (`KEY_INSTALLED_VERSION`, u32 LE), or `0` if
/// never written — the rollback floor (an image must be strictly newer than this to install).
///
/// Takes the [`Kv`] directly rather than `Net`, so it is callable at **confirm time** — in the
/// synchronous boot-state path where only the reclaimed `Kv` is in hand and the radio `Net`
/// does not exist yet. While transceiving, pass `net.kv()`.
pub fn installed_version(kv: &Kv<'_>) -> u32 {
    let mut buf = [0u8; 4];
    match kv.get_bytes(KEY_INSTALLED_VERSION, &mut buf) {
        Ok(Some(n)) if n >= 4 => u32::from_le_bytes(buf),
        _ => 0,
    }
}

/// Persist the installed firmware version (`KEY_INSTALLED_VERSION`). Call **after** a swapped
/// image has booted and self-confirmed, so a later validly-signed *older* image is refused.
/// Returns whether the write succeeded. Takes [`Kv`] (not `Net`) for the same reason as
/// [`installed_version`]: the confirm happens before `Net` is constructed.
pub fn set_installed_version(kv: &mut Kv<'_>, version: u32) -> bool {
    kv.set_bytes(KEY_INSTALLED_VERSION, &version.to_le_bytes())
        .is_ok()
}
