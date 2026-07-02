//! Program-flash staging window — the piece [`storage`](crate::storage) lacks.
//!
//! `Storage`/`Kv` wrap the L0 **data EEPROM**; FOTA stages an image in **program
//! flash**, which is a different access path (page erase + word program) on the *same*
//! `Flash` peripheral. [`Stage`] is that path, scoped to one slot (e.g. the DFU
//! staging slot, [`DFU_OFFSET`](super::DFU_OFFSET)`..`): erase pages, program words,
//! read back. It borrows the single `Flash` handle (reclaim it from `Storage` with
//! [`Storage::into_flash`](crate::storage::Storage::into_flash)).
//!
//! All offsets are **relative to the slot start** so a caller never deals with the
//! slot's absolute placement. The L0 stalls the core during erase/program; that is a
//! Phase-2 bootloader concern (the swap routine may need to run from RAM), not a
//! concern here, where staging runs from the app with the radio idle between chunks.

use embassy_stm32::flash::{Blocking, Flash};

use super::{Error, PAGE_SIZE, WRITE_SIZE, round_up};

/// A program-flash erase/program/read window over a single slot, borrowing the one
/// blocking [`Flash`] handle.
pub struct Stage<'f, 'd> {
    flash: &'f mut Flash<'d, Blocking>,
    /// Slot start as an offset from the flash base (what the `blocking_*` API wants).
    base: u32,
    /// Slot length in bytes (page-aligned).
    size: u32,
}

impl<'f, 'd> Stage<'f, 'd> {
    /// Open a staging window of `size` bytes at `base` (both offsets from the flash
    /// base, page-aligned — e.g. [`DFU_OFFSET`](super::DFU_OFFSET) /
    /// [`DFU_SIZE`](super::DFU_SIZE)).
    pub fn new(flash: &'f mut Flash<'d, Blocking>, base: u32, size: u32) -> Self {
        debug_assert!(base.is_multiple_of(PAGE_SIZE), "slot base must be page-aligned");
        debug_assert!(size.is_multiple_of(PAGE_SIZE), "slot size must be page-aligned");
        Self { flash, base, size }
    }

    /// The slot size in bytes.
    #[must_use]
    pub const fn size(&self) -> u32 {
        self.size
    }

    /// Erase enough whole pages from the slot start to hold `len` bytes (rounded up to
    /// a [`PAGE_SIZE`] page). `len == 0` erases nothing. Errors with
    /// [`Error::TooLarge`] if `len` exceeds the slot.
    pub fn erase(&mut self, len: u32) -> Result<(), Error> {
        if len > self.size {
            return Err(Error::TooLarge);
        }
        let end = round_up(len, PAGE_SIZE); // page-aligned; ≤ size (size is page-aligned)
        if end == 0 {
            return Ok(());
        }
        self.flash
            .blocking_erase(self.base, self.base + end)
            .map_err(Error::Flash)
    }

    /// Erase the entire slot.
    pub fn erase_all(&mut self) -> Result<(), Error> {
        self.erase(self.size)
    }

    /// Program `bytes` at `rel_off` within the slot. `rel_off` and `bytes.len()` must
    /// both be multiples of [`WRITE_SIZE`] (the caller pads a partial tail to a word).
    /// The target pages must already be erased.
    pub fn program(&mut self, rel_off: u32, bytes: &[u8]) -> Result<(), Error> {
        if !rel_off.is_multiple_of(WRITE_SIZE) || !(bytes.len() as u32).is_multiple_of(WRITE_SIZE) {
            return Err(Error::Unaligned);
        }
        if rel_off + bytes.len() as u32 > self.size {
            return Err(Error::TooLarge);
        }
        self.flash
            .blocking_write(self.base + rel_off, bytes)
            .map_err(Error::Flash)
    }

    /// Program `bytes` at `rel_off`, then read them back and confirm they landed — guarding
    /// against a *silent* programming failure. The embassy-stm32 L0 flash driver checks
    /// WRPERR/PGAERR/SIZERR but not the L0-specific NOTZEROERR/FWWERR, so programming a
    /// non-erased word can be aborted by hardware yet still return `Ok`; a resume that
    /// re-programs over stale/torn data would otherwise keep garbage undetected until the
    /// bootloader's whole-image hash fails (forcing a full re-download). Returns
    /// [`Error::Verify`] on a read-back mismatch. Verifies in ≤64-byte pieces to bound the
    /// stack buffer.
    pub fn program_verified(&mut self, rel_off: u32, bytes: &[u8]) -> Result<(), Error> {
        self.program(rel_off, bytes)?;
        let mut back = [0u8; 64];
        let mut done = 0usize;
        while done < bytes.len() {
            let n = (bytes.len() - done).min(back.len());
            self.read(rel_off + done as u32, &mut back[..n])?;
            if back[..n] != bytes[done..done + n] {
                return Err(Error::Verify);
            }
            done += n;
        }
        Ok(())
    }

    /// Read `buf.len()` bytes from `rel_off` within the slot (a memory-mapped copy).
    pub fn read(&mut self, rel_off: u32, buf: &mut [u8]) -> Result<(), Error> {
        if rel_off + buf.len() as u32 > self.size {
            return Err(Error::TooLarge);
        }
        self.flash
            .blocking_read(self.base + rel_off, buf)
            .map_err(Error::Flash)
    }
}
