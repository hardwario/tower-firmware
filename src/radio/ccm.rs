//! AES-128-CCM (NIST SP 800-38C / RFC 3610) built in firmware on the L0
//! hardware AES ECB primitive ([`aes`](super::aes)).
//!
//! Fixed parameters for the TOWER radio: **13-byte nonce (N=13, L=2)** and an
//! **8-byte tag (M=8)** — confidentiality + integrity in one AEAD. CCM = CBC-MAC
//! over the AAD + plaintext for the tag, plus CTR for confidentiality, both from
//! single-block AES encryption. The nonce is derived from the clear header (see
//! [`frame`](super::frame)); the AAD is the whole cleartext header.
//!
//! The CCM **construction** (CBC-MAC + CTR) lives in the host-testable
//! [`tower_radio_core::ccm`] leaf crate, generic over an
//! [`AesBlock`](tower_radio_core::ccm::AesBlock) single-block cipher; it is verified there
//! against RFC 3610 Packet Vector #1. This module is the thin device binding: it plugs the L0
//! hardware AES ([`HwAes`]) in as that block cipher and re-exposes the same `Ccm` API the radio
//! stack already uses. **Zero** behavioural change on target — the hardware binding caches the
//! key and reloads it only when it changes, so a whole seal/open (one key, many blocks) issues
//! the exact same register sequence as the original set-key-once code.

use super::aes::Aes;
use tower_radio_core::ccm::{AesBlock, Ccm as CoreCcm};

// Re-export the fixed CCM parameters from the core so callers keep saying `ccm::NONCE_LEN` etc.
pub use tower_radio_core::ccm::{MAX_AAD, NONCE_LEN, TAG_LEN};

/// The L0 hardware AES engine as an [`AesBlock`], with a **key cache**: the hardware `set_key`
/// runs only when the key actually changes. Across a single seal/open the key is constant, so
/// this reproduces the original "set the key once, then encrypt N blocks" register sequence
/// exactly — no extra key loads, no behavioural change on device.
struct HwAes {
    aes: Aes,
    loaded_key: Option<[u8; 16]>,
}

impl HwAes {
    fn new() -> Self {
        Self {
            aes: Aes::new(),
            loaded_key: None,
        }
    }
}

impl AesBlock for HwAes {
    fn encrypt_block(&mut self, key: &[u8; 16], block: &mut [u8; 16]) {
        if self.loaded_key.as_ref() != Some(key) {
            self.aes.set_key(key);
            self.loaded_key = Some(*key);
        }
        self.aes.encrypt_block(block);
    }
}

/// AES-128-CCM context holding the L0 hardware AES engine. A thin newtype over
/// [`tower_radio_core::ccm::Ccm`] so the radio stack's `Ccm` type/API is unchanged.
pub struct Ccm {
    core: CoreCcm<HwAes>,
}

impl Ccm {
    /// Create a CCM context (enables the AES clock).
    pub fn new() -> Self {
        Self {
            core: CoreCcm::new(HwAes::new()),
        }
    }

    /// Encrypt `data` in place and return the authentication tag. `aad` is
    /// authenticated but not encrypted. `nonce` must be unique per (key, frame).
    pub fn seal(
        &mut self,
        key: &[u8; 16],
        nonce: &[u8; NONCE_LEN],
        aad: &[u8],
        data: &mut [u8],
    ) -> [u8; TAG_LEN] {
        self.core.seal(key, nonce, aad, data)
    }

    /// Decrypt `data` in place and verify the tag. Returns `true` on success;
    /// on `false` (authentication failure) `data` must be discarded.
    #[must_use]
    pub fn open(
        &mut self,
        key: &[u8; 16],
        nonce: &[u8; NONCE_LEN],
        aad: &[u8],
        data: &mut [u8],
        tag: &[u8; TAG_LEN],
    ) -> bool {
        self.core.open(key, nonce, aad, data, tag)
    }
}

impl Default for Ccm {
    fn default() -> Self {
        Self::new()
    }
}
