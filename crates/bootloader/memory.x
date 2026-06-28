/* HARDWARIO TOWER Core Module bootloader — STM32L083CZ (192K flash / 20K RAM).
   A/B FOTA partition table (docs/fota.md, kept in lockstep with src/fota/mod.rs). FLASH =
   the BOOTLOADER region: the loader runs here, verifies a staged image (Ed25519 + SHA-256),
   swaps it in, and jumps to ACTIVE. All boundaries are 128 B-page aligned. The slots are
   NOT equal: embassy-boot's swap needs DFU larger than ACTIVE and STATE big enough for
   per-page progress (≈ ACTIVE/8 on the L0's 128 B pages). BOOTLOADER is 32K because it
   carries the Ed25519 verify (salty) + SHA-256 (docs/fota.md). The MANIFEST
   region (between STATE and ACTIVE, read raw at MANIFEST_OFFSET in main.rs) is where the app
   stashes the signed manifest for the loader — intentionally NOT a linker region here.

   NOTE 1 K = 1 KiBi = 1024 bytes. Partition symbols are offsets from the flash base
   (ORIGIN(FLASH) = 0x0800_0000); embassy-boot's from_linkerfile reads them. */
MEMORY
{
  FLASH            : ORIGIN = 0x08000000, LENGTH = 32K  /* BOOTLOADER (loader + verify) */
  BOOTLOADER_STATE : ORIGIN = 0x08008000, LENGTH = 12K  /* swap magic + progress */
  ACTIVE           : ORIGIN = 0x0800B800, LENGTH = 70K  /* running app (after the 2K manifest gap) */
  DFU              : ORIGIN = 0x0801D000, LENGTH = 72K  /* staged image (> ACTIVE) */
  RAM        (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(FLASH);
__bootloader_state_end   = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(FLASH);

__bootloader_active_start = ORIGIN(ACTIVE) - ORIGIN(FLASH);
__bootloader_active_end   = ORIGIN(ACTIVE) + LENGTH(ACTIVE) - ORIGIN(FLASH);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(FLASH);
__bootloader_dfu_end   = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(FLASH);
