//! OTA install driver (docs/fota.md) — the node-side control flow that turns a
//! "downlink pending" advertisement into a staged, swap-ready image.
//!
//! The Ed25519 signature + image-digest verify lives in the **bootloader** (docs/fota.md) — that
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
    KEY_PENDING_VERSION, MANIFEST_LEN, MANIFEST_OFFSET, MANIFEST_SIZE, Manifest, SIGNED_LEN, Stage,
};
use crate::radio::net::Net;
use crate::storage::Nv;

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
    /// The rollback floor ([`KEY_INSTALLED_VERSION`]) could not be read (EEPROM fault / corrupt
    /// record). Refuse to install — **fail closed** — rather than assume floor 0 and risk
    /// accepting a downgrade to an older, validly-signed, vulnerable image.
    FloorUnavailable,
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
    /// will verify (Ed25519 + image digest) and swap. The staged image's `manifest.version` is
    /// persisted as the *pending* version; after the swapped image confirms, promote it to the
    /// rollback floor with [`promote_pending_version`] (do **not** persist a compile-time constant
    /// — that can mismatch the signed `--version` and cause an endless reinstall loop).
    Staged { manifest: Manifest },
}

impl core::fmt::Display for PullOutcome {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            PullOutcome::NoManifest => f.write_str("no signed manifest received"),
            PullOutcome::Malformed => f.write_str("manifest malformed"),
            PullOutcome::NotNewer { version, installed } => {
                write!(f, "v{version} not newer than installed v{installed}")
            }
            PullOutcome::FloorUnavailable => {
                f.write_str("rollback floor unreadable — refusing to install (fail closed)")
            }
            PullOutcome::TooLarge { size } => write!(f, "image too large ({size} B)"),
            PullOutcome::ImageFailed => f.write_str("manifest stash failed after download"),
            PullOutcome::InProgress { staged, total } => {
                write!(f, "in progress ({staged}/{total} B)")
            }
            PullOutcome::Staged { manifest } => {
                write!(f, "staged v{} ({} B)", manifest.version, manifest.size)
            }
        }
    }
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
    // Read the rollback floor, distinguishing an absent key (factory floor 0) from an EEPROM
    // *fault* — on a fault we fail closed rather than assume 0 and accept a downgrade (C13).
    let installed = match read_installed_floor(net.kv()) {
        Ok(v) => v,
        Err(()) => {
            diag!("rollback floor unreadable (EEPROM fault) — refusing to install (fail closed)");
            return PullOutcome::FloorUnavailable;
        }
    };
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
    // Record the staged image's manifest version so the confirming image can promote it to the
    // rollback floor (C5). After the swap+reset the running image can't see the manifest, and its
    // own compile-time VERSION may differ from the signed `--version`; persisting it here breaks
    // the reinstall loop that arose from setting the floor to a mismatched compile-time constant.
    let _ = net.kv().set_bytes(KEY_PENDING_VERSION, &manifest.version.to_le_bytes());
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
    net.kv().with_flash(|f| {
        let mut stage = Stage::new(f, MANIFEST_OFFSET, MANIFEST_SIZE);
        stage.erase(SIGNED_LEN as u32).is_ok() && stage.program(0, signed).is_ok()
    })
}

/// The installed firmware version from EEPROM (`KEY_INSTALLED_VERSION`, u32 LE), or `0` if
/// never written — the rollback floor (an image must be strictly newer than this to install).
/// Returns `0` for both an absent key and a read fault; for the install *decision* use the
/// fail-closed [`read_installed_floor`] instead, which distinguishes the two.
///
/// Takes the shared [`Nv`] handle, so it is callable at **confirm time** — pass `b.kv` in the
/// boot-state path before the radio `Net` exists; while transceiving, pass `net.kv()`.
pub fn installed_version(kv: Nv) -> u32 {
    read_installed_floor(kv).unwrap_or(0)
}

/// The rollback floor for the install decision, distinguishing a genuinely *absent* key
/// (`Ok(0)` — a factory device that has never completed an OTA) from an EEPROM read *fault* or
/// corrupt record (`Err(())` — refuse to install, fail closed, rather than assume `0` and accept
/// a downgrade to an older validly-signed image). See [`PullOutcome::FloorUnavailable`].
fn read_installed_floor(kv: Nv) -> Result<u32, ()> {
    let mut buf = [0u8; 4];
    match kv.get_bytes(KEY_INSTALLED_VERSION, &mut buf) {
        Ok(Some(n)) if n >= 4 => Ok(u32::from_le_bytes(buf)),
        Ok(Some(_)) => Err(()), // short/corrupt record — don't trust it as a floor
        Ok(None) => Ok(0),      // never written — genuine factory floor 0
        Err(_) => Err(()),      // EEPROM fault — fail closed
    }
}

/// Promote the *pending* staged version (recorded by [`pull_update`] at stash time) to the
/// installed rollback floor, then clear it. Call this **after** a swapped image has booted and
/// self-confirmed, instead of persisting the image's compile-time constant: the pending value is
/// the actual signed manifest version of the running image, so the floor can't drift below what
/// was advertised (which caused an endless reinstall loop). Returns the resulting floor. If no
/// pending version is recorded (an ordinary boot, not a post-OTA confirm) the floor is unchanged.
pub fn promote_pending_version(kv: Nv) -> u32 {
    let mut buf = [0u8; 4];
    let pending = match kv.get_bytes(KEY_PENDING_VERSION, &mut buf) {
        Ok(Some(n)) if n >= 4 => Some(u32::from_le_bytes(buf)),
        _ => None,
    };
    match pending {
        Some(v) => {
            let _ = kv.set_bytes(KEY_INSTALLED_VERSION, &v.to_le_bytes());
            let _ = kv.set_bytes(KEY_PENDING_VERSION, &0u32.to_le_bytes());
            v
        }
        None => installed_version(kv),
    }
}

/// Persist the installed firmware version (`KEY_INSTALLED_VERSION`). Call **after** a swapped
/// image has booted and self-confirmed, so a later validly-signed *older* image is refused.
/// Returns whether the write succeeded. Takes the shared [`Nv`] handle (not `Net`) for the same
/// reason as [`installed_version`]: the confirm happens before `Net` is constructed.
pub fn set_installed_version(kv: Nv, version: u32) -> bool {
    kv.set_bytes(KEY_INSTALLED_VERSION, &version.to_le_bytes())
        .is_ok()
}
