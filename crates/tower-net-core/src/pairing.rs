//! OTA-pairing JOIN_CONFIRM freshness/acceptance rule (moved from `src/radio/net/pairing.rs`;
//! docs/radio.md).
//!
//! The host mints a fresh per-session **challenge** in the JOIN_RESP that the joiner
//! must echo in the JOIN_CONFIRM. This is anti-*replay* within the window (a confirm
//! captured from a prior session carries a stale challenge and is rejected) on top of
//! CCM integrity — it does NOT add confidentiality or mutual auth (the challenge rides
//! the public-key frames in the clear-after-decrypt payload).
//!
//! Only this acceptance predicate is extracted; the challenge *minting* (the firmware's
//! non-cryptographic xorshift32 PRNG), the pairing windows/turnarounds and the JOIN frame
//! exchange stay in the firmware's radio flow.

/// Validate a JOIN_CONFIRM payload against this session's expectations. The payload echoes
/// `addr` (4 LE) ‖ `challenge` (4 LE) — commit only when both match, so a confirm replayed
/// from a prior session (stale challenge) is rejected.
#[must_use]
pub fn confirm_matches(payload: &[u8], addr: u32, challenge: u32) -> bool {
    payload.len() >= 8
        && u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]) == addr
        && u32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]) == challenge
}

#[cfg(test)]
mod tests {
    use super::*;

    fn confirm(addr: u32, challenge: u32) -> [u8; 8] {
        let mut p = [0u8; 8];
        p[..4].copy_from_slice(&addr.to_le_bytes());
        p[4..].copy_from_slice(&challenge.to_le_bytes());
        p
    }

    /// The genuine confirm — correct id, this session's challenge — commits.
    #[test]
    fn genuine_confirm_accepted() {
        assert!(confirm_matches(
            &confirm(0xAABB_CCDD, 0x1234_5678),
            0xAABB_CCDD,
            0x1234_5678
        ));
    }

    /// A confirm replayed from a PRIOR session carries that session's (stale) challenge — the
    /// freshness rule rejects it even though the node id matches and the frame authenticated
    /// under the (public) pairing key.
    #[test]
    fn stale_challenge_rejected() {
        let old_session = confirm(0xAABB_CCDD, 0x1111_1111);
        assert!(!confirm_matches(&old_session, 0xAABB_CCDD, 0x2222_2222));
    }

    /// A confirm echoing the wrong node address (another joiner's confirm in the window) is not a
    /// commit for THIS exchange.
    #[test]
    fn wrong_addr_rejected() {
        assert!(!confirm_matches(
            &confirm(0x0000_0001, 0x1234_5678),
            0x0000_0002,
            0x1234_5678
        ));
    }

    /// Both must match — one right field is not enough.
    #[test]
    fn one_matching_field_is_not_enough() {
        assert!(!confirm_matches(&confirm(1, 1), 1, 2));
        assert!(!confirm_matches(&confirm(1, 1), 2, 1));
        assert!(!confirm_matches(&confirm(1, 1), 2, 2));
    }

    /// Short payloads (missing/truncated echo) never commit.
    #[test]
    fn short_payload_rejected() {
        let full = confirm(1, 2);
        for len in 0..8 {
            assert!(!confirm_matches(&full[..len], 1, 2), "len {len}");
        }
    }

    /// The rule is `len >= 8`: trailing extra bytes are tolerated (pin the current contract).
    #[test]
    fn longer_payload_tolerated() {
        let mut long = [0u8; 12];
        long[..8].copy_from_slice(&confirm(1, 2));
        assert!(confirm_matches(&long, 1, 2));
    }

    /// Exactness over the full u32 corners of both fields.
    #[test]
    fn u32_corners_exact() {
        for &id in &[0u32, 1, u32::MAX] {
            for &ch in &[0u32, 1, u32::MAX] {
                assert!(confirm_matches(&confirm(id, ch), id, ch));
                assert!(!confirm_matches(&confirm(id, ch), id, ch.wrapping_add(1)));
                assert!(!confirm_matches(&confirm(id, ch), id.wrapping_add(1), ch));
            }
        }
    }
}
