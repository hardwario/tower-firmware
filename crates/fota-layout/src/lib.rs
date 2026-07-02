//! FOTA flash partition table for the HARDWARIO TOWER Core Module (docs/fota.md).
//!
//! The single source of truth for the A/B FOTA flash layout, shared by the `tower` lib
//! (`src/fota`) and the standalone `tower-bootloader` binary. The bootloader cannot depend on
//! the `tower` lib (it carries its own `memory.x` and a disjoint dependency set), so before this
//! crate the offsets were **hand-duplicated** in `crates/bootloader/src/main.rs` — with no
//! guards there. Pulling both consumers through this leaf crate means a layout edit lands in one
//! place and any drift fails **both** builds via the compile-time guards below.
//!
//! Offsets are **from the flash start** (`FLASH_BASE` = `0x0800_0000`) — that is what
//! `embassy_stm32::flash::Flash::blocking_{read,write,erase}` take, NOT absolute addresses.
//! Absolute address = `FLASH_BASE + offset`.
//!
//! The slots are NOT equal-sized: embassy-boot's swap (see the const guards below)
//! requires DFU to be at least one page LARGER than ACTIVE, and STATE to hold per-page
//! swap progress (4 B/page → ≈ ACTIVE/8 on the L0's 128 B pages). Getting these wrong makes
//! the bootloader's `prepare_boot` panic at runtime → a silent/dead loader, so the guards
//! below turn that into a build error.
//!
//! The BOOTLOADER is 20 KB: it carries the Ed25519 verify (salty) + a SHA-512-based image
//! digest, because the signature/hash check runs **bootloader-side**, not in the app
//! (docs/fota.md). That keeps salty out of the *duplicated* A/B app slots — net flash saving —
//! and runs the verify on a clean stack. (It reuses salty's SHA-512 for the image digest rather
//! than linking a second hash crate; the loader binary is ≈16 KB, so 20 KB leaves ~4 KB margin —
//! guarded by `just size-check` so it can't silently erode toward the brick-on-overflow limit.)
//! The 12 KB reclaimed from the old 32 KB region went to ACTIVE/DFU, and MANIFEST is trimmed to
//! 256 B (just over the 116-byte signed manifest) so its slack goes to ACTIVE too. The MANIFEST
//! region is where the app stashes the signed manifest for the bootloader to read before a swap.
//!
//!   region            abs addr        offset     size      purpose
//!   BOOTLOADER        0x0800_0000     0x00000    20 KB     loader + Ed25519/SHA-512 verify + swap
//!   BOOTLOADER_STATE  0x0800_5000     0x05000    12 KB     embassy-boot swap magic + progress
//!   MANIFEST          0x0800_8000     0x08000    256 B     signed manifest staged for the loader
//!   ACTIVE            0x0800_8100     0x08100    77.75 KB  running app (256 B-aligned for VTOR)
//!   DFU               0x0801_B800     0x1B800    78 KB     staging slot (must be > ACTIVE)
//!   (spare)           0x0802_F000     0x2F000    4 KB      margin / future
//!
//! **Keep the `memory.x` files in sync with these** — `crates/bootloader/memory.x` (FLASH origin
//! = BOOTLOADER region) and the FOTA app's ACTIVE-origin `memory.x` (via `build.rs`). Those are
//! linker inputs so they can't `use` this crate; the guards here bound the numbers, the `memory.x`
//! comments cite this table.

#![no_std]

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
/// Bootloader region size (holds the loader + salty Ed25519 verify + SHA-512 image digest).
pub const BOOTLOADER_SIZE: u32 = 20 * 1024;
/// embassy-boot swap-state region offset.
pub const STATE_OFFSET: u32 = 0x0_5000;
/// embassy-boot swap-state region size (sized for ACTIVE's per-page progress; see guards).
pub const STATE_SIZE: u32 = 12 * 1024;
/// MANIFEST region offset — where the app stashes the signed `Manifest` for the bootloader
/// to read + verify before swapping (the only image metadata that crosses the app↔loader
/// boundary). Read raw by the loader; written by the app via `Stage`.
pub const MANIFEST_OFFSET: u32 = 0x0_8000;
/// MANIFEST region size: 256 B (two L0 pages), just over the 116-byte signed manifest.
/// Trimmed from 2 KB so the slack goes to ACTIVE; the loader erases this whole region.
/// (That it holds a full signed manifest — `MANIFEST_SIZE >= SIGNED_LEN` — is guarded in
/// `src/fota/mod.rs`, which has the tower-protocol dep; this crate stays dependency-free.)
pub const MANIFEST_SIZE: u32 = 256;
/// ACTIVE (running app) slot offset — the app's `memory.x` FLASH origin. **256 B-aligned** so the
/// app's vector table is a valid VTOR base on the Cortex-M0+ (the actual hardware requirement).
pub const ACTIVE_OFFSET: u32 = 0x0_8100;
/// ACTIVE (running app) slot size — 76 KB + the 1792 B reclaimed from the trimmed MANIFEST.
pub const ACTIVE_SIZE: u32 = 79_616;
/// DFU (staging) slot offset — where a downloaded image is written before swap.
pub const DFU_OFFSET: u32 = 0x1_B800;
/// DFU (staging) slot size — **larger** than ACTIVE (embassy-boot swap needs the slack).
pub const DFU_SIZE: u32 = 78 * 1024;
/// Spare region offset (margin / future use).
pub const SPARE_OFFSET: u32 = 0x2_F000;
/// Total program flash on the STM32L083CZ.
pub const FLASH_TOTAL: u32 = 192 * 1024;

// Compile-time guards encoding embassy-boot's swap requirements (`assert_partitions` +
// `prepare_boot`), so a bad layout fails the BUILD instead of silently trapping the
// bootloader at runtime. These live in the shared crate so BOTH consumers get them.
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
    // Word-aligned programming is a subset of page alignment.
    assert!(PAGE_SIZE.is_multiple_of(WRITE_SIZE));
    // embassy-boot swap: DFU must be at least one page LARGER than ACTIVE.
    assert!(DFU_SIZE >= ACTIVE_SIZE + PAGE_SIZE);
    // embassy-boot swap state: STATE must hold the magic + 4 B of progress per ACTIVE page
    // (`2 + 4*pages` WRITE_SIZE-words). On the L0's 128 B pages that is ≈ ACTIVE/8.
    assert!(2 + 4 * (ACTIVE_SIZE / PAGE_SIZE) <= STATE_SIZE / WRITE_SIZE);
};
