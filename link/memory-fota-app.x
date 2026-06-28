/* FOTA app layout — for an app linked into the ACTIVE slot, booted by crates/bootloader
   (docs/fota.md, kept in lockstep with src/fota/mod.rs). Selected by the `fota-active` cargo
   feature; see build.rs. FLASH = ACTIVE, so the reset vector lands where the bootloader
   jumps. Partition symbols are offsets from the flash base (ORIGIN(BOOTLOADER) =
   0x0800_0000), which embassy-boot's FirmwareUpdater/FirmwareState from_linkerfile reads
   (the app uses them for get_state/mark_booted — it no longer verifies; the bootloader does).
   Slots are NOT equal — DFU is larger than ACTIVE (embassy-boot swap requirement). The app
   stashes the signed manifest in the MANIFEST region (offset 0x08000, not a linker region
   here) for the bootloader to verify before swapping.

   NOTE 1 K = 1 KiBi = 1024 bytes. */
MEMORY
{
  BOOTLOADER       : ORIGIN = 0x08000000, LENGTH = 20K
  BOOTLOADER_STATE : ORIGIN = 0x08005000, LENGTH = 12K
  FLASH            : ORIGIN = 0x08008800, LENGTH = 76K  /* ACTIVE — the app runs here */
  DFU              : ORIGIN = 0x0801B800, LENGTH = 78K
  RAM        (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(BOOTLOADER);
__bootloader_state_end   = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(BOOTLOADER);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(BOOTLOADER);
__bootloader_dfu_end   = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(BOOTLOADER);
