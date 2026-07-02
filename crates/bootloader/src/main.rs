//! HARDWARIO TOWER Core Module bootloader (embassy-boot, docs/fota.md).
//!
//! Blocking loader that is also the **FOTA authenticity gate**. On a clean boot it checks
//! the MANIFEST region: if the app staged a signed manifest there, the loader reads the DFU
//! image, verifies the **Ed25519 signature** over the manifest and that the **image digest**
//! (a 256-bit truncation of SHA-512) matches `manifest.sha256`, and only then arms the
//! embassy-boot swap (`mark_updated`). It clears the manifest request *before* arming, so a
//! power loss never re-installs. Then it lets embassy-boot perform (or resume, or revert) the
//! A/B swap and jumps to ACTIVE. The app confirms a good boot with `mark_booted`.
//!
//! The image digest reuses **salty's SHA-512** — the hash the Ed25519 verify already links —
//! truncated to 32 bytes, so the loader carries one hash engine instead of two (no separate
//! `sha2`). The signer (`tools/fota-sign`) computes the identical truncated SHA-512.
//!
//! Verifying **here** (not in the app) keeps salty out of the duplicated A/B app slots — the
//! flash win that makes a radio + crypto OTA node fit the L083 — and runs the verify on the
//! loader's clean, deep stack. Rollback (version) stays app-side policy (it's app EEPROM
//! state); the loader gates authenticity + integrity.
//!
//! Resume-safety: the verify happens only on a clean `State::Boot` with a staged manifest. A
//! swap already in progress (`State::Swap`/`Revert`) is resumed/reverted by embassy-boot
//! without re-verifying (the manifest was already cleared when the swap was armed).
//!
//! The single-bank L0 concern (docs/fota.md) is handled by embassy-boot-stm32. **Vendor key:**
//! [`VENDOR_PUBKEY`] is selected in tower-protocol by its `dev-key` feature — a default-off-in-
//! production guard. A bring-up build (default features) bakes the public DEV key (forgeable —
//! its seed ships in `tools/fota-sign`); a **production** build pins tower-protocol with
//! `default-features = false` and supplies the real key via the `TOWER_VENDOR_PUBKEY` env var
//! (`fota-sign pubkey --hex`), or the crate fails to compile. So shipping the dev key is a
//! mechanical error, not a silent omission — nothing to remember to "replace" here.

#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m_rt::{entry, exception};
use embassy_boot_stm32::{AlignedBuffer, BlockingFirmwareState, BootLoader, BootLoaderConfig, State};
use embassy_stm32::flash::{BANK1_REGION, Flash, WRITE_SIZE};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use salty::Sha512;
use tower_protocol::fota::{SIGNED_LEN, VENDOR_PUBKEY, split_signed, verify_signed};

// Flash offsets come from the shared `fota-layout` crate (which the `tower` lib's src/fota also
// re-exports), so the loader and the app can never drift on the layout — a mismatch fails BOTH
// builds via fota-layout's compile-time guards. (These were previously hand-duplicated here with
// no guards; the guards now live in one place.) The DFU image is read raw; the MANIFEST region
// holds the staged signed manifest.
//
// NB: `memory.x` (the linker input, FLASH origin = BOOTLOADER region) can't `use` this crate, so
// it stays a separate copy — its comments cite fota-layout's table.
//
// The image is *staged* in DFU (78 KB, deliberately one page larger than ACTIVE for the
// embassy-boot swap) but *runs* from ACTIVE after the swap, so the real bound on image size is
// the ACTIVE slot, not the DFU slot. Verifying against ACTIVE_SIZE rejects an image in the
// (ACTIVE_SIZE, DFU_SIZE] gap that would pass the hash yet be truncated when copied to ACTIVE.
use fota_layout::{ACTIVE_SIZE, DFU_OFFSET, MANIFEST_OFFSET, MANIFEST_SIZE};

/// Per-product hardware id this loader accepts images for. `0` (the default) accepts an image
/// with **any** `manifest.hw_id` — the single-product case (the Core Module). Set this to a
/// product-specific id in a *multi-product* deployment that shares one vendor key: then only a
/// generic image (`hw_id == 0`) or one targeting this product (`hw_id == DEVICE_HW_ID`) installs,
/// so an image signed for a *different* product is rejected even though its signature is valid.
const DEVICE_HW_ID: u32 = 0;

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(Default::default());

    // One blocking flash region (the single L0 bank) shared by the active/dfu/state
    // partitions and our raw manifest/DFU reads, via a blocking mutex + RefCell.
    let layout = Flash::new_blocking(p.FLASH).into_blocking_regions();
    let flash = Mutex::new(RefCell::new(layout.bank1_region));
    let mut aligned = AlignedBuffer([0u8; WRITE_SIZE]);

    // Peek the embassy-boot state without swapping.
    let state = {
        let cfg = BootLoaderConfig::from_linkerfile_blocking(&flash, &flash, &flash);
        let mut fw = BlockingFirmwareState::new(cfg.state, &mut aligned.0);
        fw.get_state().unwrap_or(State::Boot)
    };

    // Only on a clean boot do we consider a freshly staged image. A swap already in flight
    // (Swap/Revert) is resumed/reverted below without re-verifying.
    if state == State::Boot {
        let mut signed = [0u8; SIGNED_LEN];
        let present = flash
            .lock(|c| c.borrow_mut().read(MANIFEST_OFFSET, &mut signed))
            .is_ok()
            && split_signed(&signed).is_some();

        if present {
            let valid = verify_staged(&flash, &signed);
            // Clear the request BEFORE arming: a power loss after this (but before/at
            // mark_updated) then leaves no manifest → the app re-stages; it never
            // re-installs a stale image.
            let _ = flash.lock(|c| {
                c.borrow_mut()
                    .erase(MANIFEST_OFFSET, MANIFEST_OFFSET + MANIFEST_SIZE)
            });
            if valid {
                let cfg = BootLoaderConfig::from_linkerfile_blocking(&flash, &flash, &flash);
                let mut fw = BlockingFirmwareState::new(cfg.state, &mut aligned.0);
                let _ = fw.mark_updated();
            }
        }
    }

    // Perform (or resume, or revert) the A/B swap, then jump to ACTIVE. The swap copy buffer
    // must divide PAGE_SIZE = 128 (a bigger value makes prepare_boot panic → silent loader).
    let cfg = BootLoaderConfig::from_linkerfile_blocking(&flash, &flash, &flash);
    let active_offset = cfg.active.offset();
    let bl = BootLoader::prepare::<_, _, _, 128>(cfg);
    unsafe { bl.load(BANK1_REGION.base() + active_offset) }
}

/// Authenticate a staged image: the signed manifest must verify against [`VENDOR_PUBKEY`]
/// (Ed25519) **and** the image digest of the first `manifest.size` DFU bytes — a 256-bit
/// truncation of SHA-512, computed with salty's already-linked hash — must equal the manifest's
/// `sha256` field. Returns `false` on any mismatch or read error.
///
/// Scope of the gate: **authenticity + integrity + post-swap fit + (opt-in) cross-product**.
/// The `manifest.hw_id` check is gated on [`DEVICE_HW_ID`] (default `0` = accept any, the
/// single-product case). Rollback (version) is **out of scope** — it is app-side EEPROM policy
/// (see `tower::fota::installed_version`), since the loader holds no version state.
fn verify_staged<F: NorFlash + ReadNorFlash, M: RawMutex>(
    flash: &Mutex<M, RefCell<F>>,
    signed: &[u8; SIGNED_LEN],
) -> bool {
    let Some(manifest) = verify_signed(&VENDOR_PUBKEY, signed) else {
        return false; // bad / forged signature
    };
    // Cross-product gate (opt-in; see DEVICE_HW_ID): reject an image signed for a *different*
    // product. A generic image (hw_id 0) is always accepted.
    if DEVICE_HW_ID != 0 && manifest.hw_id != 0 && manifest.hw_id != DEVICE_HW_ID {
        return false;
    }
    // Bound by ACTIVE (where the image runs after swap), not DFU (where it is staged).
    if manifest.size > ACTIVE_SIZE {
        return false;
    }
    let mut hasher = Sha512::new();
    let mut buf = [0u8; 128];
    let mut off = 0u32;
    while off < manifest.size {
        let n = ((manifest.size - off) as usize).min(buf.len());
        if flash
            .lock(|c| c.borrow_mut().read(DFU_OFFSET + off, &mut buf[..n]))
            .is_err()
        {
            return false;
        }
        hasher.update(&buf[..n]);
        off += n as u32;
    }
    // Truncate the 64-byte SHA-512 to the manifest's 32-byte digest field.
    let full = hasher.finalize();
    full[..32] == manifest.sha256
}

#[unsafe(no_mangle)]
#[cfg_attr(target_os = "none", unsafe(link_section = ".HardFault.user"))]
unsafe extern "C" fn HardFault() -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

#[exception]
unsafe fn DefaultHandler(_: i16) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    cortex_m::asm::udf();
}
