//! [`FlashSink`] — stream a received image straight into the DFU slot.
//!
//! This is the FOTA counterpart to `examples/net_bulk_stream.rs`'s `CrcCheckSink`:
//! a [`BulkSink`] fed one ≤64 B chunk at a time by
//! [`Net::bulk_fetch_into`](crate::radio::net::Net::bulk_fetch_into), but instead of
//! merely checking bytes it **programs them into program flash** via [`Stage`] and
//! folds a running **SHA-256** over the image. RAM stays constant regardless of image
//! size — only the chunk, a one-word pad buffer, and the hash state are held.
//!
//! The whole-image hash it computes is what the bootloader will check against the
//! signed [`Manifest`](super::Manifest) before swapping (docs/fota.md). Per-chunk wire
//! integrity is already handled by the CCM-authenticated transport; this hash is the
//! end-to-end check that the bytes landed in flash intact.

use log::error;
use sha2::{Digest, Sha256};

use super::{Error, Stage, WRITE_SIZE, round_up};
use crate::radio::net::{BULK_CHUNK, BulkSink};

/// A [`BulkSink`] that writes the received image into a flash slot (the DFU staging
/// slot) and hashes it on the fly. Construct over the slot, hand to `bulk_fetch_into`,
/// then read the size with [`received`](Self::received) and the digest with
/// [`finish`](Self::finish).
pub struct FlashSink<'f, 'd> {
    stage: Stage<'f, 'd>,
    hasher: Sha256,
    /// Bytes consumed and programmed so far (also the next expected offset).
    written: u32,
    /// Total announced for this transfer (set in [`begin`](BulkSink::begin)).
    total: u32,
    /// Sticky failure flag — set on the first erase/program error or protocol misuse.
    failed: bool,
}

impl<'f, 'd> FlashSink<'f, 'd> {
    /// Build a sink over the staging slot `stage` writes into.
    pub fn new(stage: Stage<'f, 'd>) -> Self {
        Self {
            stage,
            hasher: Sha256::new(),
            written: 0,
            total: 0,
            failed: false,
        }
    }

    /// Bytes received and programmed so far (the staged image length once complete).
    #[must_use]
    pub fn received(&self) -> u32 {
        self.written
    }

    /// Whether a flash error or protocol misuse aborted the transfer.
    #[must_use]
    pub fn failed(&self) -> bool {
        self.failed
    }

    /// Finalize and return the SHA-256 of the bytes consumed (the staged image hash).
    /// Consumes the sink — call [`received`](Self::received) first if you need the size.
    #[must_use]
    pub fn finish(self) -> [u8; 32] {
        let digest = self.hasher.finalize();
        let mut out = [0u8; 32];
        out.copy_from_slice(&digest);
        out
    }

    /// Program one chunk at `off`, padding a partial tail word so the write is word-aligned.
    /// The pad bytes lie **beyond** the image's real length, so they are never hashed (the
    /// digest folds only the real `chunk` bytes) nor read back — their value is don't-care.
    /// Padded with `0x00` (the L0's erased value, matching `Net::bulk_fetch_to_flash`).
    /// Separated from the trait method so the `?`-style flow reads cleanly.
    fn program_chunk(&mut self, off: u32, chunk: &[u8]) -> Result<(), Error> {
        if (chunk.len() as u32).is_multiple_of(WRITE_SIZE) {
            self.stage.program(off, chunk)
        } else {
            // Only the last chunk can be non-word-multiple, and it is < BULK_CHUNK,
            // so the padded length never exceeds one chunk buffer.
            let mut buf = [0u8; BULK_CHUNK];
            buf[..chunk.len()].copy_from_slice(chunk);
            let padded = round_up(chunk.len() as u32, WRITE_SIZE) as usize;
            self.stage.program(off, &buf[..padded])
        }
    }
}

impl BulkSink for FlashSink<'_, '_> {
    async fn begin(&mut self, total_len: usize) -> bool {
        let total = total_len as u32;
        if total > self.stage.size() {
            error!(target: "fota", "image {} B exceeds DFU slot {} B", total, self.stage.size());
            self.failed = true;
            return false;
        }
        if let Err(e) = self.stage.erase(total) {
            error!(target: "fota", "DFU erase for {} B failed: {:?}", total, e);
            self.failed = true;
            return false;
        }
        self.hasher = Sha256::new();
        self.written = 0;
        self.total = total;
        self.failed = false;
        true
    }

    async fn consume(&mut self, offset: usize, chunk: &[u8]) -> bool {
        let off = offset as u32;
        // bulk_fetch_into delivers chunks in increasing, contiguous offset order; a gap
        // would corrupt the image (and the SHA), so refuse rather than write past it.
        if off != self.written {
            error!(target: "fota", "out-of-order chunk: off={} expected={}", off, self.written);
            self.failed = true;
            return false;
        }
        // Hash the real bytes (never the pad) so the digest matches the manifest.
        self.hasher.update(chunk);
        if let Err(e) = self.program_chunk(off, chunk) {
            error!(target: "fota", "DFU program at {} ({} B) failed: {:?}", off, chunk.len(), e);
            self.failed = true;
            return false;
        }
        self.written += chunk.len() as u32;
        true
    }
}
