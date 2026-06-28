//! HARDWARIO TOWER Core Module bootloader (embassy-boot, docs/fota.md).
//!
//! Blocking loader that is also the **FOTA authenticity gate**. On a clean boot it checks
//! the MANIFEST region: if the app staged a signed manifest there, the loader reads the DFU
//! image, verifies the **Ed25519 signature** over the manifest and that **SHA-256(DFU) ==
//! manifest.sha256**, and only then arms the embassy-boot swap (`mark_updated`). It clears
//! the manifest request *before* arming, so a power loss never re-installs. Then it lets
//! embassy-boot perform (or resume, or revert) the A/B swap and jumps to ACTIVE. The app
//! confirms a good boot with `mark_booted`.
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
//! The single-bank L0 concern (docs/fota.md) is handled by embassy-boot-stm32. **DEV vendor
//! key** — replace `tower_protocol::fota::VENDOR_PUBKEY` before shipping.

#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m_rt::{entry, exception};
use embassy_boot_stm32::{AlignedBuffer, BlockingFirmwareState, BootLoader, BootLoaderConfig, State};
use embassy_stm32::flash::{BANK1_REGION, Flash, WRITE_SIZE};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use sha2::{Digest, Sha256};
use tower_protocol::fota::{SIGNED_LEN, VENDOR_PUBKEY, split_signed, verify_signed};

// Flash offsets — kept in lockstep with src/fota/mod.rs and the memory.x files (the
// bootloader can't depend on the `tower` lib, so the layout is duplicated here, like
// memory.x). The DFU image is read raw; the MANIFEST region holds the staged signed manifest.
const MANIFEST_OFFSET: u32 = 0x0_B000;
const MANIFEST_SIZE: u32 = 2 * 1024;
const DFU_OFFSET: u32 = 0x1_D000;
const DFU_SIZE: u32 = 72 * 1024;

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
            let _ = flash.lock(|c| c.borrow_mut().erase(MANIFEST_OFFSET, MANIFEST_OFFSET + MANIFEST_SIZE));
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
/// (Ed25519) **and** the SHA-256 of the first `manifest.size` DFU bytes must equal the
/// manifest's `sha256`. Returns `false` on any mismatch or read error.
fn verify_staged<F: NorFlash + ReadNorFlash, M: RawMutex>(
    flash: &Mutex<M, RefCell<F>>,
    signed: &[u8; SIGNED_LEN],
) -> bool {
    let Some(manifest) = verify_signed(&VENDOR_PUBKEY, signed) else {
        return false; // bad / forged signature
    };
    if manifest.size > DFU_SIZE {
        return false;
    }
    let mut hasher = Sha256::new();
    let mut buf = [0u8; 128];
    let mut off = 0u32;
    while off < manifest.size {
        let n = ((manifest.size - off) as usize).min(buf.len());
        if flash.lock(|c| c.borrow_mut().read(DFU_OFFSET + off, &mut buf[..n])).is_err() {
            return false;
        }
        hasher.update(&buf[..n]);
        off += n as u32;
    }
    let got: [u8; 32] = hasher.finalize().into();
    got == manifest.sha256
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
