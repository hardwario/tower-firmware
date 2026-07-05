//! CCM nonce construction (moved from `src/radio/frame.rs`; docs/radio.md §Security model).
//!
//! The 13-byte nonce is *derived* from the clear frame header — never transmitted — which is
//! why it's reconstructable on receive and unique per (key, frame): the key is per-node and
//! `src` fixes the sender (the two directions never collide even at equal counter values — no
//! `dir` field needed); the 32-bit `counter` advances one per transfer; `bulk_index` separates
//! the chunks of one transfer; and a retransmission re-sends the byte-identical frame (same
//! counter ⇒ same ciphertext ⇒ safe). The host tests here pin the exact byte layout (golden
//! test) and prove injectivity over the input space by round-trip decoding.

/// Byte length of the CCM nonce (the TOWER radio's fixed CCM parameters: N=13, L=2). Mirrors
/// `tower_radio_core::ccm::NONCE_LEN` — this crate is dependency-free, so the value is stated
/// here and const-asserted equal in the firmware's `frame.rs`.
pub const NONCE_LEN: usize = 13;

/// Derive the 13-byte CCM nonce from the header fields:
/// `src[4] ‖ counter[4] ‖ bulk_index[3] ‖ 0x0000` (little-endian). The single
/// audited nonce source for the whole stack.
pub fn nonce_for(src: u32, counter: u32, bulk_index: u32) -> [u8; NONCE_LEN] {
    let mut n = [0u8; NONCE_LEN];
    n[0..4].copy_from_slice(&src.to_le_bytes());
    n[4..8].copy_from_slice(&counter.to_le_bytes());
    let b = bulk_index.to_le_bytes();
    n[8..11].copy_from_slice(&b[..3]);
    // n[11..13] = 0x0000
    n
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Decode a nonce back into its `(src, counter, bulk_index_low24)` inputs. The existence of
    /// this inverse *is* the uniqueness proof: distinct `(src, counter, bulk_index & 0xFF_FFFF)`
    /// triples can never map to the same nonce bytes.
    fn decode(n: &[u8; NONCE_LEN]) -> (u32, u32, u32) {
        (
            u32::from_le_bytes([n[0], n[1], n[2], n[3]]),
            u32::from_le_bytes([n[4], n[5], n[6], n[7]]),
            u32::from_le_bytes([n[8], n[9], n[10], 0]),
        )
    }

    /// Golden byte layout: `src ‖ counter ‖ bulk_index[..3] ‖ 0x0000`, little-endian. A layout
    /// change (field order, endianness, trailing zeros) must fail THIS test loudly — the layout
    /// is the wire-compatibility and nonce-uniqueness contract (docs/radio.md).
    #[test]
    fn golden_layout() {
        let n = nonce_for(0x0403_0201, 0x0807_0605, 0x000B_0A09);
        assert_eq!(
            n,
            [
                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x00, 0x00
            ]
        );
    }

    /// The two trailing bytes are always zero (the `‖ 0x0000` in the documented layout), for
    /// every corner of the input space.
    #[test]
    fn trailing_bytes_always_zero() {
        for &src in &[0u32, 1, 0xFFFF_FFFF] {
            for &ctr in &[0u32, 1, 0xFFFF_FFFF] {
                for &bulk in &[0u32, 1, 0x00FF_FFFF, 0xFFFF_FFFF] {
                    let n = nonce_for(src, ctr, bulk);
                    assert_eq!(&n[11..13], &[0, 0]);
                }
            }
        }
    }

    /// Round-trip: every `(src, counter, bulk_index)` decodes back to itself (bulk index modulo
    /// its 24 on-wire bits), over a boundary-heavy grid — injectivity across the input space.
    #[test]
    fn round_trip_injective_over_grid() {
        let edges = [
            0u32,
            1,
            2,
            0x7F,
            0x80,
            0xFF,
            0x100,
            0xFFFF,
            0x1_0000,
            0xFFFF_FFFE,
            0xFFFF_FFFF,
        ];
        for &src in &edges {
            for &ctr in &edges {
                for &bulk in &edges {
                    let n = nonce_for(src, ctr, bulk);
                    assert_eq!(decode(&n), (src, ctr, bulk & 0x00FF_FFFF));
                }
            }
        }
    }

    /// Direction/id separation: two distinct senders never share a nonce, whatever the counter —
    /// this is why the same per-node key can protect both directions (docs/radio.md: "`src`
    /// fixes the sender").
    #[test]
    fn distinct_src_never_collides() {
        let ids = [0x1111_1111u32, 0x2222_2222, 0, 1, 0xFFFF_FFFF];
        for &a in &ids {
            for &b in &ids {
                if a == b {
                    continue;
                }
                for ctr in 0..200u32 {
                    assert_ne!(nonce_for(a, ctr, 0), nonce_for(b, ctr, 0));
                }
            }
        }
    }

    /// Counter separation under one sender: successive counters give distinct nonces (loop
    /// property across a window and across the u32 boundary region).
    #[test]
    fn distinct_counter_never_collides() {
        for base in [0u32, 1000, 0xFFFF_FF00] {
            for i in 0..200u32 {
                for j in 0..200u32 {
                    if i == j {
                        continue;
                    }
                    let (a, b) = (base.wrapping_add(i), base.wrapping_add(j));
                    if a == b {
                        continue;
                    }
                    assert_ne!(nonce_for(7, a, 0), nonce_for(7, b, 0));
                }
            }
        }
    }

    /// Bulk-chunk separation: the chunks of one transfer (same src, same session counter) get
    /// distinct nonces from their 24-bit `bulk_index`; and a non-bulk frame (index 0) matches
    /// bulk index 0 ONLY at the same counter — which the bulk protocol never reuses (the
    /// announce and chunk 0 ride *different* counters; see `net/bulk.rs`).
    #[test]
    fn distinct_bulk_index_never_collides() {
        for i in 0..300u32 {
            for j in 0..300u32 {
                if i == j {
                    continue;
                }
                assert_ne!(nonce_for(7, 42, i), nonce_for(7, 42, j));
            }
        }
        // The 24-bit truncation boundary: indices ≥ 2^24 alias low ones — the protocol caps the
        // index at 24 bits (16 MiB), so pin the aliasing as the documented contract.
        assert_eq!(nonce_for(7, 42, 0x0100_0005), nonce_for(7, 42, 5));
    }
}
