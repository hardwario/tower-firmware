//! Host tests for the pure AES-128-CCM core.
//!
//! The firmware can't `cargo test` (no_std / thumbv6m), so its CCM was previously only
//! exercised on-device (`examples/crypto_ccm_kat.rs`). Here we plug a pure-Rust AES block into
//! the same [`Ccm`] the firmware uses (the L0 hardware AES is the *other* [`AesBlock`] impl) and
//! check it against RFC 3610 Packet Vector #1, plus a seal/open round-trip and single-byte
//! tamper rejection. This is the value of the trait extraction: the CCM math is now covered on
//! the host with **zero** change to the on-device crypto path.

use super::{AesBlock, Ccm, NONCE_LEN, TAG_LEN};
use aes::Aes128;
use aes::cipher::{BlockEncrypt, KeyInit, generic_array::GenericArray};

/// Pure-Rust single-block AES-128 (the `aes` crate) as an [`AesBlock`], for host tests only.
struct SoftAes;

impl AesBlock for SoftAes {
    fn encrypt_block(&mut self, key: &[u8; 16], block: &mut [u8; 16]) {
        let cipher = Aes128::new(GenericArray::from_slice(key));
        let mut b = GenericArray::clone_from_slice(block);
        cipher.encrypt_block(&mut b);
        block.copy_from_slice(&b);
    }
}

/// RFC 3610 §8 Packet Vector #1 (M=8, L=2, N=13 — exactly the TOWER parameters).
///
/// AAD = the 8-byte header; the remaining 23 bytes are the plaintext. The expected ciphertext +
/// 8-byte MIC are copied verbatim from the RFC, so this pins the CCM construction (CBC-MAC over
/// B0 ‖ len-prefixed AAD ‖ payload, then CTR with S0 masking the tag) byte-for-byte.
#[test]
fn rfc3610_packet_vector_1() {
    let key: [u8; 16] = [
        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE,
        0xCF,
    ];
    // 13-byte nonce: flags/L handled internally; RFC's "nonce" is the 13 bytes after the flags.
    let nonce: [u8; NONCE_LEN] = [
        0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x00, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5,
    ];
    let aad: [u8; 8] = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
    let mut data: [u8; 23] = [
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
        0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E,
    ];
    // Expected encrypted payload (23 B) and 8-byte MIC, from RFC 3610.
    let expected_ct: [u8; 23] = [
        0x58, 0x8C, 0x97, 0x9A, 0x61, 0xC6, 0x63, 0xD2, 0xF0, 0x66, 0xD0, 0xC2, 0xC0, 0xF9, 0x89,
        0x80, 0x6D, 0x5F, 0x6B, 0x61, 0xDA, 0xC3, 0x84,
    ];
    let expected_tag: [u8; TAG_LEN] = [0x17, 0xE8, 0xD1, 0x2C, 0xFD, 0xF9, 0x26, 0xE0];

    let mut ccm = Ccm::new(SoftAes);
    let tag = ccm.seal(&key, &nonce, &aad, &mut data);
    assert_eq!(data, expected_ct, "RFC 3610 #1 ciphertext mismatch");
    assert_eq!(tag, expected_tag, "RFC 3610 #1 MIC mismatch");

    // Round-trip: open recovers the plaintext and accepts the tag.
    let ok = ccm.open(&key, &nonce, &aad, &mut data, &tag);
    assert!(ok, "RFC 3610 #1 open must authenticate");
    assert_eq!(
        data,
        [
            0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
            0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E,
        ],
        "open must recover the original plaintext"
    );
}

/// seal → open round-trips arbitrary payloads (incl. empty and non-block-multiple lengths),
/// with and without AAD. `no_std` crate, so we work in a fixed buffer and slice it per length.
#[test]
fn seal_open_roundtrip() {
    let key = [0x11u8; 16];
    let nonce = [0x22u8; NONCE_LEN];
    let mut plain = [0u8; 40];
    for (i, b) in plain.iter_mut().enumerate() {
        *b = i as u8;
    }
    for aad in [&[][..], &[0xAA, 0xBB, 0xCC][..]] {
        for &len in &[0usize, 1, 15, 16, 17, 31, 32, 40] {
            let mut ccm = Ccm::new(SoftAes);
            let mut data = [0u8; 40];
            data[..len].copy_from_slice(&plain[..len]);
            let tag = ccm.seal(&key, &nonce, aad, &mut data[..len]);
            // Ciphertext differs from plaintext for non-empty payloads.
            if len > 0 {
                assert_ne!(&data[..len], &plain[..len], "seal must encrypt (len {len})");
            }
            let ok = ccm.open(&key, &nonce, aad, &mut data[..len], &tag);
            assert!(ok, "round-trip open failed (len {len}, aad {})", aad.len());
            assert_eq!(&data[..len], &plain[..len], "round-trip plaintext mismatch (len {len})");
        }
    }
}

/// A single-bit/byte flip anywhere in the ciphertext, tag, nonce, AAD, or key must make `open`
/// reject (the CCM tag is the integrity guarantee the radio relies on).
#[test]
fn single_byte_tamper_rejected() {
    let key = [0x33u8; 16];
    let nonce = [0x44u8; NONCE_LEN];
    let aad = [0x01u8, 0x02, 0x03, 0x04];
    let mut ccm = Ccm::new(SoftAes);

    let mut sealed = *b"tamper-me: 16byte payload!!";
    let tag = ccm.seal(&key, &nonce, &aad, &mut sealed);

    // Tamper the ciphertext.
    {
        let mut d = sealed;
        d[5] ^= 0x01;
        assert!(!ccm.open(&key, &nonce, &aad, &mut d, &tag), "flipped ciphertext must fail");
    }
    // Tamper the tag.
    {
        let mut d = sealed;
        let mut t = tag;
        t[0] ^= 0x80;
        assert!(!ccm.open(&key, &nonce, &aad, &mut d, &t), "flipped tag must fail");
    }
    // Tamper the AAD.
    {
        let mut d = sealed;
        let mut a = aad;
        a[2] ^= 0x10;
        assert!(!ccm.open(&key, &nonce, &a, &mut d, &tag), "flipped AAD must fail");
    }
    // Tamper the nonce.
    {
        let mut d = sealed;
        let mut n = nonce;
        n[1] ^= 0x01;
        assert!(!ccm.open(&key, &n, &aad, &mut d, &tag), "wrong nonce must fail");
    }
    // Wrong key.
    {
        let mut d = sealed;
        let mut wrong = key;
        wrong[15] ^= 0x01;
        assert!(!ccm.open(&wrong, &nonce, &aad, &mut d, &tag), "wrong key must fail");
    }
    // Untampered opens fine (sanity).
    {
        let mut d = sealed;
        assert!(ccm.open(&key, &nonce, &aad, &mut d, &tag), "untampered must authenticate");
    }
}
