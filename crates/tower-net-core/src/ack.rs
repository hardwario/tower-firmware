//! ACK acceptance / delivery resolution for the sender's ACK window (moved from
//! `src/radio/net/mod.rs::await_ack`; docs/radio.md).
//!
//! The sender of a confirmed frame opens an ACK window and offers every frame it hears to this
//! kernel; the kernel decides which single frame resolves the delivery. Everything else — a
//! foreign/undecodable frame, a non-ACK, an ACK from/to the wrong device, an ACK for a
//! *different* counter (stale/duplicate) — is **ignored**, i.e. the caller keeps listening for
//! the rest of the window rather than treating it as a failed delivery. Once resolved, further
//! (duplicate) ACKs are ignored too: a delivery resolves at most once.

/// ACK payload flags byte (byte 5, appended in wire v3). Absent on older peers'
/// 4/5-byte ACKs — the `len >= 4` acceptance rule (pinned below) is what makes the
/// append interop-safe in a mixed-version network.
pub mod ack_flags {
    /// The ACKer (a gateway) holds a queued downlink for the ACKed sender, who
    /// should keep its radio in RX for a short window instead of going to sleep.
    pub const PENDING: u8 = 1 << 0;
}

/// Metadata parsed out of an ACK payload — the receiver-side RSSI report and the
/// wire-v3 pending-downlink flag. Layout: `acked(4 LE) ‖ rssi(1, i8) ‖ flags(1)`,
/// with both trailing bytes optional (older peers send 4- or 5-byte ACKs).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AckMeta {
    /// RSSI at which the receiver heard our frame, as it packed it (clamped i8 dBm);
    /// `None` on a 4-byte (no-RSSI) ACK.
    pub rssi: Option<i8>,
    /// The ACKer holds a queued downlink for us ([`ack_flags::PENDING`]); always
    /// `false` on a pre-v3 (≤ 5-byte) ACK.
    pub pending: bool,
}

/// Parse the metadata bytes of an ACK payload (pure; the caller has already matched
/// the ACK via [`AckWait::offer`], so `payload` is the accepted ACK's).
#[must_use]
pub fn ack_meta(payload: &[u8]) -> AckMeta {
    AckMeta {
        rssi: payload.get(4).map(|&b| b as i8),
        pending: payload.get(5).is_some_and(|&f| f & ack_flags::PENDING != 0),
    }
}

/// The verdict for one frame heard inside the ACK window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AckVerdict {
    /// This is our ACK — the confirmed transfer is delivered (exactly once).
    Resolved,
    /// Not our ACK (foreign, non-ACK, wrong peer/dest, stale counter, or already resolved) —
    /// keep waiting for the rest of the window.
    Ignore,
}

/// One pending confirmed delivery: who must acknowledge what, and whether it already resolved.
#[derive(Debug, Clone, Copy)]
pub struct AckWait {
    /// Our own address — the ACK must be addressed to us.
    me: u32,
    /// The peer we sent to — the ACK must come from it.
    peer: u32,
    /// The TX counter of the frame awaiting acknowledgement.
    counter: u32,
    /// Delivery already resolved: any further ACK copy is a duplicate (ignored).
    resolved: bool,
}

impl AckWait {
    /// Start waiting for `peer`'s ACK of our frame `counter` (we are `me`).
    #[must_use]
    pub fn new(me: u32, peer: u32, counter: u32) -> Self {
        Self {
            me,
            peer,
            counter,
            resolved: false,
        }
    }

    /// Offer one authenticated frame heard in the window: its type (`is_ack`), clear-header
    /// `src`/`dest`, and decrypted payload.
    ///
    /// ACK payload: acked counter (4 LE) + rssi (1). A valid ACK for a *different* counter
    /// (a stale/duplicate ACK) also isn't ours — keep waiting for the right one.
    pub fn offer(&mut self, is_ack: bool, src: u32, dest: u32, payload: &[u8]) -> AckVerdict {
        if self.resolved {
            return AckVerdict::Ignore; // a delivery resolves at most once
        }
        if !is_ack || src != self.peer || dest != self.me {
            return AckVerdict::Ignore;
        }
        if payload.len() >= 4
            && u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]) == self.counter
        {
            self.resolved = true;
            AckVerdict::Resolved
        } else {
            AckVerdict::Ignore
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ME: u32 = 0x1111_1111;
    const PEER: u32 = 0x2222_2222;

    /// A well-formed ACK payload: acked counter (4 LE) + rssi (1).
    fn ack_payload(counter: u32, rssi: u8) -> [u8; 5] {
        let mut p = [0u8; 5];
        p[..4].copy_from_slice(&counter.to_le_bytes());
        p[4] = rssi;
        p
    }

    /// The matching ACK resolves the delivery.
    #[test]
    fn matching_ack_resolves() {
        let mut w = AckWait::new(ME, PEER, 42);
        assert_eq!(
            w.offer(true, PEER, ME, &ack_payload(42, 0xC8)),
            AckVerdict::Resolved
        );
    }

    /// A duplicate ACK after resolution is ignored — the delivery never double-resolves.
    #[test]
    fn duplicate_ack_does_not_double_resolve() {
        let mut w = AckWait::new(ME, PEER, 42);
        assert_eq!(
            w.offer(true, PEER, ME, &ack_payload(42, 0xC8)),
            AckVerdict::Resolved
        );
        assert_eq!(
            w.offer(true, PEER, ME, &ack_payload(42, 0xC8)),
            AckVerdict::Ignore
        );
        assert_eq!(
            w.offer(true, PEER, ME, &ack_payload(42, 0x10)),
            AckVerdict::Ignore
        );
    }

    /// A late/stale ACK for an old counter is ignored — the window keeps waiting for the right
    /// one instead of failing (the star-contention false-NotDelivered fix, docs/radio.md).
    #[test]
    fn stale_counter_ignored_then_right_one_resolves() {
        let mut w = AckWait::new(ME, PEER, 42);
        assert_eq!(w.offer(true, PEER, ME, &ack_payload(41, 0)), AckVerdict::Ignore);
        assert_eq!(w.offer(true, PEER, ME, &ack_payload(7, 0)), AckVerdict::Ignore);
        assert_eq!(w.offer(true, PEER, ME, &ack_payload(42, 0)), AckVerdict::Resolved);
    }

    /// Non-ACK frames are ignored, whoever they're from (a neighbouring uplink in our window).
    #[test]
    fn non_ack_frames_ignored() {
        let mut w = AckWait::new(ME, PEER, 42);
        assert_eq!(w.offer(false, PEER, ME, &ack_payload(42, 0)), AckVerdict::Ignore);
        assert_eq!(w.offer(false, 0x3333_3333, ME, b"data"), AckVerdict::Ignore);
    }

    /// An ACK from the wrong peer, or addressed to someone else, is not ours.
    #[test]
    fn wrong_src_or_dest_ignored() {
        let mut w = AckWait::new(ME, PEER, 42);
        assert_eq!(
            w.offer(true, 0x3333_3333, ME, &ack_payload(42, 0)),
            AckVerdict::Ignore
        );
        assert_eq!(
            w.offer(true, PEER, 0x3333_3333, &ack_payload(42, 0)),
            AckVerdict::Ignore
        );
        // Even our own counter echoed by a third party doesn't resolve.
        assert_eq!(w.offer(true, PEER, ME, &ack_payload(42, 0)), AckVerdict::Resolved);
    }

    /// A short payload (< 4 bytes — no full acked-counter field) never resolves.
    #[test]
    fn short_payload_ignored() {
        let mut w = AckWait::new(ME, PEER, 42);
        assert_eq!(w.offer(true, PEER, ME, &[]), AckVerdict::Ignore);
        assert_eq!(w.offer(true, PEER, ME, &[42]), AckVerdict::Ignore);
        assert_eq!(w.offer(true, PEER, ME, &[42, 0, 0]), AckVerdict::Ignore);
    }

    /// The rule is `len >= 4`: exactly 4 bytes (no rssi byte) still matches, and trailing bytes
    /// beyond the rssi are tolerated — pin the current contract.
    #[test]
    fn payload_length_rule_is_at_least_four() {
        let mut w = AckWait::new(ME, PEER, 42);
        assert_eq!(
            w.offer(true, PEER, ME, &42u32.to_le_bytes()),
            AckVerdict::Resolved
        );
        let mut w2 = AckWait::new(ME, PEER, 42);
        let mut long = [0u8; 8];
        long[..4].copy_from_slice(&42u32.to_le_bytes());
        assert_eq!(w2.offer(true, PEER, ME, &long), AckVerdict::Resolved);
    }

    /// `ack_meta` over every historical ACK length: 4 bytes (no RSSI, pre-RSSI era),
    /// 5 bytes (RSSI, wire v2), 6 bytes (RSSI + flags, wire v3). Older-peer ACKs must
    /// parse as "no pending" — never as a spurious RX window on the node.
    #[test]
    fn ack_meta_all_lengths() {
        assert_eq!(
            ack_meta(&42u32.to_le_bytes()),
            AckMeta {
                rssi: None,
                pending: false
            }
        );

        let mut p5 = [0u8; 5];
        p5[..4].copy_from_slice(&42u32.to_le_bytes());
        p5[4] = -67i8 as u8;
        assert_eq!(
            ack_meta(&p5),
            AckMeta {
                rssi: Some(-67),
                pending: false
            }
        );

        let mut p6 = [0u8; 6];
        p6[..5].copy_from_slice(&p5);
        p6[5] = ack_flags::PENDING;
        assert_eq!(
            ack_meta(&p6),
            AckMeta {
                rssi: Some(-67),
                pending: true
            }
        );

        // Flags byte present but pending bit clear.
        p6[5] = 0;
        assert_eq!(
            ack_meta(&p6),
            AckMeta {
                rssi: Some(-67),
                pending: false
            }
        );

        // Unknown future flag bits don't read as pending.
        p6[5] = 0xFE;
        assert!(!ack_meta(&p6).pending);
    }

    /// A 6-byte (flags-bearing) ACK still resolves the wait — the `len >= 4` rule.
    #[test]
    fn six_byte_ack_still_resolves() {
        let mut w = AckWait::new(ME, PEER, 42);
        let mut p6 = [0u8; 6];
        p6[..4].copy_from_slice(&42u32.to_le_bytes());
        p6[5] = ack_flags::PENDING;
        assert_eq!(w.offer(true, PEER, ME, &p6), AckVerdict::Resolved);
    }

    /// Counter matching is exact over the full u32 range (LE decode of the payload).
    #[test]
    fn counter_match_is_exact_u32() {
        for counter in [0u32, 1, 0x8000_0000, u32::MAX - 1, u32::MAX] {
            let mut w = AckWait::new(ME, PEER, counter);
            assert_eq!(
                w.offer(true, PEER, ME, &ack_payload(counter.wrapping_add(1), 0)),
                AckVerdict::Ignore
            );
            assert_eq!(
                w.offer(true, PEER, ME, &ack_payload(counter, 0)),
                AckVerdict::Resolved
            );
        }
    }
}
