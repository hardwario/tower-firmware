//! AES-128-CCM (NIST SP 800-38C / RFC 3610), the **pure** algorithm.
//!
//! Extracted from `src/radio/ccm.rs` so the CCM construction can be tested on the host against
//! the RFC 3610 vectors (the firmware is no_std / thumbv6m and can't `cargo test`). It is
//! generic over an [`AesBlock`] — the single-block ECB primitive — so the firmware plugs in the
//! L0 hardware AES engine and the host tests plug in a pure-Rust AES. The math (CBC-MAC + CTR,
//! N=13/L=2/M=8) is byte-for-byte identical to the on-target original: **zero** behavioural
//! change on device, this only makes the block cipher a parameter.
//!
//! Fixed parameters for the TOWER radio: **13-byte nonce (N=13, L=2)** and an
//! **8-byte tag (M=8)** — confidentiality + integrity in one AEAD. CCM = CBC-MAC
//! over the AAD + plaintext for the tag, plus CTR for confidentiality, both from
//! single-block AES encryption. The nonce is derived from the clear header; the AAD is the
//! whole cleartext header.

/// CCM length-field size (bytes used for the message length / counter).
const L: usize = 2;
/// Nonce length (15 - L).
pub const NONCE_LEN: usize = 13;
/// Authentication tag length.
pub const TAG_LEN: usize = 8;
/// Maximum supported associated-data length. The CBC-MAC prefixes AAD with a
/// 2-byte length, so the encoding buffer is `2 + MAX_AAD`. The stack's only AAD
/// is the frame header (≤ 17 B), so this is generous; callers must not exceed it.
pub const MAX_AAD: usize = 32;

/// The single-block AES-128 ECB primitive CCM is built on. `key` is the 16-byte key and
/// `block` is encrypted in place. The firmware's hardware impl caches the key and only reloads
/// it when it changes, so re-passing the (constant, per-frame) key here costs nothing and the
/// register-access sequence is identical to the original set-key-once code.
pub trait AesBlock {
    /// Encrypt one 16-byte block in place under `key` (ECB).
    fn encrypt_block(&mut self, key: &[u8; 16], block: &mut [u8; 16]);
}

/// AES-128-CCM context holding the AES engine `A`.
pub struct Ccm<A: AesBlock> {
    aes: A,
}

impl<A: AesBlock> Ccm<A> {
    /// Create a CCM context over the given AES engine.
    pub fn new(aes: A) -> Self {
        Self { aes }
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
        let mac = self.cbc_mac(key, nonce, aad, data);
        // CTR: S0 encrypts the tag; S1.. encrypt the payload.
        self.ctr_apply(key, nonce, data);
        let s0 = self.ctr_keystream(key, nonce, 0);
        let mut tag = [0u8; TAG_LEN];
        for j in 0..TAG_LEN {
            tag[j] = mac[j] ^ s0[j];
        }
        tag
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
        // CTR is symmetric: decrypt first, then MAC the recovered plaintext.
        self.ctr_apply(key, nonce, data);
        let mac = self.cbc_mac(key, nonce, aad, data);
        let s0 = self.ctr_keystream(key, nonce, 0);
        let mut expected = [0u8; TAG_LEN];
        for j in 0..TAG_LEN {
            expected[j] = mac[j] ^ s0[j];
        }
        ct_eq(&expected, tag)
    }

    /// CBC-MAC over B0 ‖ (len-prefixed AAD, padded) ‖ (payload, padded).
    /// Returns the raw MAC block (the tag is its first `TAG_LEN` bytes XOR S0).
    fn cbc_mac(&mut self, key: &[u8; 16], nonce: &[u8; NONCE_LEN], aad: &[u8], data: &[u8]) -> [u8; 16] {
        let mut x = [0u8; 16];

        // B0 = flags ‖ nonce ‖ l(m).  flags = 64·Adata + 8·((M-2)/2) + (L-1).
        let adata = !aad.is_empty();
        let mut b0 = [0u8; 16];
        b0[0] = (if adata { 0x40 } else { 0 }) | ((((TAG_LEN - 2) / 2) as u8) << 3) | ((L - 1) as u8);
        b0[1..1 + NONCE_LEN].copy_from_slice(nonce);
        let mlen = data.len() as u16;
        b0[14..16].copy_from_slice(&mlen.to_be_bytes());
        block_xor(&mut x, &b0);
        self.aes.encrypt_block(key, &mut x);

        // Associated data: 2-byte big-endian length prefix, then AAD, zero-padded.
        if adata {
            debug_assert!(aad.len() <= MAX_AAD, "AAD exceeds MAX_AAD");
            let mut a = [0u8; 2 + MAX_AAD];
            let alen = aad.len() as u16;
            a[0..2].copy_from_slice(&alen.to_be_bytes());
            a[2..2 + aad.len()].copy_from_slice(aad);
            self.mac_stream(key, &mut x, &a[..2 + aad.len()]);
        }

        // Payload, zero-padded to a block boundary.
        self.mac_stream(key, &mut x, data);
        x
    }

    /// XOR `stream` into the running CBC-MAC `x` in 16-byte blocks (final block
    /// zero-padded), encrypting after each.
    fn mac_stream(&mut self, key: &[u8; 16], x: &mut [u8; 16], stream: &[u8]) {
        for chunk in stream.chunks(16) {
            for (j, &b) in chunk.iter().enumerate() {
                x[j] ^= b;
            }
            self.aes.encrypt_block(key, x);
        }
    }

    /// Apply the CTR keystream (S1, S2, …) to `data` in place.
    fn ctr_apply(&mut self, key: &[u8; 16], nonce: &[u8; NONCE_LEN], data: &mut [u8]) {
        for (i, chunk) in data.chunks_mut(16).enumerate() {
            let s = self.ctr_keystream(key, nonce, (i + 1) as u16);
            for (b, k) in chunk.iter_mut().zip(s.iter()) {
                *b ^= k;
            }
        }
    }

    /// CTR keystream block S_counter = E(A_counter), A = flags ‖ nonce ‖ counter.
    fn ctr_keystream(&mut self, key: &[u8; 16], nonce: &[u8; NONCE_LEN], counter: u16) -> [u8; 16] {
        let mut a = [0u8; 16];
        a[0] = (L - 1) as u8; // flags = L-1
        a[1..1 + NONCE_LEN].copy_from_slice(nonce);
        a[14..16].copy_from_slice(&counter.to_be_bytes());
        self.aes.encrypt_block(key, &mut a);
        a
    }
}

/// XOR `src` into `dst` (16 bytes).
fn block_xor(dst: &mut [u8; 16], src: &[u8; 16]) {
    for i in 0..16 {
        dst[i] ^= src[i];
    }
}

/// Constant-time equality for the tag comparison.
fn ct_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut diff = 0u8;
    for i in 0..a.len() {
        diff |= a[i] ^ b[i];
    }
    diff == 0
}

#[cfg(test)]
mod tests;
