//! AES-128-CCM (NIST SP 800-38C / RFC 3610) built in firmware on the L0
//! hardware AES ECB primitive ([`aes`](super::aes)).
//!
//! Fixed parameters for the TOWER radio: **13-byte nonce (N=13, L=2)** and an
//! **8-byte tag (M=8)** — confidentiality + integrity in one AEAD. CCM = CBC-MAC
//! over the AAD + plaintext for the tag, plus CTR for confidentiality, both from
//! single-block AES encryption. The nonce is derived from the clear header (see
//! [`frame`](super::frame)); the AAD is the whole cleartext header.


use super::aes::Aes;

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

/// AES-128-CCM context holding the AES engine.
pub struct Ccm {
    aes: Aes,
}

impl Ccm {
    /// Create a CCM context (enables the AES clock).
    pub fn new() -> Self {
        Self { aes: Aes::new() }
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
        self.aes.set_key(key);
        let mac = self.cbc_mac(nonce, aad, data);
        // CTR: S0 encrypts the tag; S1.. encrypt the payload.
        self.ctr_apply(nonce, data);
        let s0 = self.ctr_keystream(nonce, 0);
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
        self.aes.set_key(key);
        // CTR is symmetric: decrypt first, then MAC the recovered plaintext.
        self.ctr_apply(nonce, data);
        let mac = self.cbc_mac(nonce, aad, data);
        let s0 = self.ctr_keystream(nonce, 0);
        let mut expected = [0u8; TAG_LEN];
        for j in 0..TAG_LEN {
            expected[j] = mac[j] ^ s0[j];
        }
        ct_eq(&expected, tag)
    }

    /// CBC-MAC over B0 ‖ (len-prefixed AAD, padded) ‖ (payload, padded).
    /// Returns the raw MAC block (the tag is its first `TAG_LEN` bytes XOR S0).
    fn cbc_mac(&mut self, nonce: &[u8; NONCE_LEN], aad: &[u8], data: &[u8]) -> [u8; 16] {
        let mut x = [0u8; 16];

        // B0 = flags ‖ nonce ‖ l(m).  flags = 64·Adata + 8·((M-2)/2) + (L-1).
        let adata = !aad.is_empty();
        let mut b0 = [0u8; 16];
        b0[0] = (if adata { 0x40 } else { 0 }) | ((((TAG_LEN - 2) / 2) as u8) << 3) | ((L - 1) as u8);
        b0[1..1 + NONCE_LEN].copy_from_slice(nonce);
        let mlen = data.len() as u16;
        b0[14..16].copy_from_slice(&mlen.to_be_bytes());
        block_xor(&mut x, &b0);
        self.aes.encrypt_block(&mut x);

        // Associated data: 2-byte big-endian length prefix, then AAD, zero-padded.
        if adata {
            debug_assert!(aad.len() <= MAX_AAD, "AAD exceeds MAX_AAD");
            let mut a = [0u8; 2 + MAX_AAD];
            let alen = aad.len() as u16;
            a[0..2].copy_from_slice(&alen.to_be_bytes());
            a[2..2 + aad.len()].copy_from_slice(aad);
            self.mac_stream(&mut x, &a[..2 + aad.len()]);
        }

        // Payload, zero-padded to a block boundary.
        self.mac_stream(&mut x, data);
        x
    }

    /// XOR `stream` into the running CBC-MAC `x` in 16-byte blocks (final block
    /// zero-padded), encrypting after each.
    fn mac_stream(&mut self, x: &mut [u8; 16], stream: &[u8]) {
        for chunk in stream.chunks(16) {
            for (j, &b) in chunk.iter().enumerate() {
                x[j] ^= b;
            }
            self.aes.encrypt_block(x);
        }
    }

    /// Apply the CTR keystream (S1, S2, …) to `data` in place.
    fn ctr_apply(&mut self, nonce: &[u8; NONCE_LEN], data: &mut [u8]) {
        for (i, chunk) in data.chunks_mut(16).enumerate() {
            let s = self.ctr_keystream(nonce, (i + 1) as u16);
            for (b, k) in chunk.iter_mut().zip(s.iter()) {
                *b ^= k;
            }
        }
    }

    /// CTR keystream block S_counter = E(A_counter), A = flags ‖ nonce ‖ counter.
    fn ctr_keystream(&mut self, nonce: &[u8; NONCE_LEN], counter: u16) -> [u8; 16] {
        let mut a = [0u8; 16];
        a[0] = (L - 1) as u8; // flags = L-1
        a[1..1 + NONCE_LEN].copy_from_slice(nonce);
        a[14..16].copy_from_slice(&counter.to_be_bytes());
        self.aes.encrypt_block(&mut a);
        a
    }
}

impl Default for Ccm {
    fn default() -> Self {
        Self::new()
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
