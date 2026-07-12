//! Per-peer replay-acceptance rule (moved from `src/radio/net/mod.rs::recv`; docs/radio.md
//! §Security model).
//!
//! One [`ReplayLane`] per sender: its last-seen counter plus the lazy-persist cadence. The rule
//! is **strictly monotonic** — `counter > last_seen` is fresh, `== last_seen` is the benign
//! retransmit of the most-recently-accepted transfer (re-ACK, don't re-deliver), `< last_seen`
//! is a replay (drop silently, replay state untouched). There is no in-RAM acceptance window
//! below last-seen; the "replay window ≤ P across a reboot" (docs/storage.md) is purely the
//! persistence lag — after a reset a lane restored from EEPROM can be up to [`P`] accepts
//! behind, so up to `P` already-seen counters are re-accepted once each.
//!
//! **Ordering contract (docs/radio.md):** on receive, CCM-verify FIRST (this authenticates the
//! header, including the counter), *then* [`classify`](ReplayLane::classify) and — only for a
//! fresh frame — [`accept`](ReplayLane::accept). A forged high counter therefore can't poison
//! replay state: it never reaches this kernel, because CCM rejects the frame before the network
//! layer acts. Callers must preserve that order.

/// Receiver last-seen lazy-persist period: the replay window across a reboot is
/// ≤ `P` transfers (docs/radio.md).
pub const P: u32 = 32;

/// The replay-rule verdict for one authenticated frame (see the module docs for the exact
/// semantics of each arm).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayVerdict {
    /// `counter > last_seen`: fresh — deliver, advance the lane, ACK if requested.
    Fresh,
    /// `counter == last_seen`: benign retransmit of this peer's most-recently-accepted
    /// counter (its ACK was lost) — re-ACK deterministically, do NOT re-deliver.
    Retransmit,
    /// `counter < last_seen`: replay — drop silently (replay state untouched).
    Replay,
}

/// One sender's replay lane: last-seen counter + accepted-transfer count since the last
/// persist. On key install last-seen starts at **0** (the TX counter starts at 1; `0` = "never
/// sent"); a re-key resets both ends (a new key is a disjoint nonce space, docs/radio.md).
#[derive(Debug, Clone, Copy)]
pub struct ReplayLane {
    /// Replay last-seen: the highest counter accepted from this sender.
    last_seen: u32,
    /// Accepted-transfer count on this lane since its last persist.
    accepts: u32,
}

impl ReplayLane {
    /// A lane resuming at `last_seen` (0 for a fresh lane; the persisted value on restore).
    #[must_use]
    pub fn new(last_seen: u32) -> Self {
        Self {
            last_seen,
            accepts: 0,
        }
    }

    /// The lane's last-seen counter (diagnostics / persistence).
    #[must_use]
    pub fn last_seen(&self) -> u32 {
        self.last_seen
    }

    /// Classify an **authenticated** frame's counter against this lane (CCM-verify first — see
    /// the module docs). Pure: does not touch the lane.
    ///
    /// Counter `0` is unconditionally a [`Replay`](ReplayVerdict::Replay): `0` means "never
    /// sent" (TX counters start at 1), so no legitimate frame ever carries it. Without this
    /// rule a zero-counter frame on a *fresh* lane (last-seen 0) would compare equal and be
    /// re-ACKed as a "retransmit" of a transfer that never happened.
    #[must_use]
    pub fn classify(&self, counter: u32) -> ReplayVerdict {
        if counter == 0 {
            ReplayVerdict::Replay
        } else if counter > self.last_seen {
            ReplayVerdict::Fresh
        } else if counter == self.last_seen {
            ReplayVerdict::Retransmit
        } else {
            ReplayVerdict::Replay
        }
    }

    /// Record acceptance of `counter` (a [`Fresh`](ReplayVerdict::Fresh) frame): advance the
    /// lane's last-seen. Returns whether the caller must lazy-persist the lane now — true once
    /// every [`P`] accepts (replay window ≤ P across a reboot, docs/radio.md).
    #[must_use = "a true return means the lane must be persisted now"]
    pub fn accept(&mut self, counter: u32) -> bool {
        self.last_seen = counter;
        self.accepts = self.accepts.wrapping_add(1);
        self.accepts.is_multiple_of(P)
    }
}

/// Where a peer's replay lane is persisted, and whether it was restored or freshly claimed.
/// The result of [`assign_lane`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LaneBinding {
    /// Index (`0..records.len()`) of the persisted lane local this peer's lane lives at.
    pub index: usize,
    /// Restored last-seen (the persisted value on restore; `0` for a freshly-claimed lane).
    pub seen: u32,
    /// `true` = a fresh local was claimed — the caller must persist `(addr, 0)` there;
    /// `false` = the peer's own record was restored (no write needed).
    pub fresh: bool,
}

/// Bind a peer address to one of `records.len()` persisted replay-lane locals, **keyed by peer
/// addr, not by the caller's table slot**.
///
/// This is the fix for a real bug: the lane used to be persisted at `base + table_slot`, but a
/// peer's table slot is assigned by registration order, which *changes* when an earlier peer is
/// removed and the registry compacts. On the next reboot the surviving later peers landed on
/// different slots → their addr-tagged records no longer matched → their replay lanes reset to 0,
/// reopening the window until they next transmitted. Keying by addr makes a peer find its own lane
/// wherever it sits.
///
/// `records[l]` = the stored `(stored_addr, last_seen)` at local `l` (`None` if unwritten). `used`
/// bit `l` marks a local already bound to a *currently registered* peer this session (so two
/// live peers never share a local, and a removed peer's record stays for resume-on-re-pair while
/// its local frees for reuse). Priority: (1) restore this `addr`'s record at a free local; else
/// (2) claim the lowest free local. `None` only if every local is bound to a live peer — which
/// the caller prevents by only calling when it has a free table slot. `records.len()` ≤ 32
/// (the `used` bitmask width).
#[must_use]
pub fn assign_lane(addr: u32, records: &[Option<(u32, u32)>], used: u32) -> Option<LaneBinding> {
    // 1. Restore this addr's own record, at a local no live peer already occupies.
    for (l, rec) in records.iter().enumerate() {
        if let Some((stored_addr, seen)) = *rec {
            if stored_addr == addr && used & (1 << l) == 0 {
                return Some(LaneBinding {
                    index: l,
                    seen,
                    fresh: false,
                });
            }
        }
    }
    // 2. Claim the lowest free local (the caller persists `(addr, 0)` there).
    for l in 0..records.len() {
        if used & (1 << l) == 0 {
            return Some(LaneBinding {
                index: l,
                seen: 0,
                fresh: true,
            });
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// First message on a fresh lane (last-seen 0, sender's counter starts at 1) is fresh.
    #[test]
    fn first_message_accepted() {
        let lane = ReplayLane::new(0);
        assert_eq!(lane.classify(1), ReplayVerdict::Fresh);
    }

    /// An exact duplicate of the last-accepted counter is the benign-retransmit case: re-ACK,
    /// no re-delivery.
    #[test]
    fn exact_duplicate_is_retransmit() {
        let mut lane = ReplayLane::new(0);
        assert_eq!(lane.classify(5), ReplayVerdict::Fresh);
        let _ = lane.accept(5);
        assert_eq!(lane.classify(5), ReplayVerdict::Retransmit);
        // Still a retransmit on every further copy — the verdict is deterministic.
        assert_eq!(lane.classify(5), ReplayVerdict::Retransmit);
    }

    /// The rule is strictly monotonic: ANY counter below last-seen is a replay — there is no
    /// in-RAM acceptance window for "slightly older" frames (out-of-order delivery below the
    /// high-water mark is rejected by design; docs/radio.md).
    #[test]
    fn older_counters_rejected() {
        let mut lane = ReplayLane::new(0);
        let _ = lane.accept(10);
        for c in 0..10 {
            assert_eq!(lane.classify(c), ReplayVerdict::Replay, "counter {c}");
        }
    }

    /// Far-behind counters (e.g. a capture replayed much later) are equally rejected.
    #[test]
    fn behind_window_rejected() {
        let mut lane = ReplayLane::new(0);
        let _ = lane.accept(100_000);
        assert_eq!(lane.classify(1), ReplayVerdict::Replay);
        assert_eq!(lane.classify(99_999), ReplayVerdict::Replay);
    }

    /// A large forward jump (sender rebooted onto its reserve watermark, counter jumped by up
    /// to RESERVE) is fresh, and the lane slides forward — everything at or below the jump is
    /// then dead.
    #[test]
    fn large_forward_jump_accepted_and_slides() {
        let mut lane = ReplayLane::new(0);
        let _ = lane.accept(3);
        assert_eq!(lane.classify(3 + 1024), ReplayVerdict::Fresh);
        let _ = lane.accept(3 + 1024);
        assert_eq!(lane.last_seen(), 3 + 1024);
        assert_eq!(lane.classify(4), ReplayVerdict::Replay);
        assert_eq!(lane.classify(3 + 1024), ReplayVerdict::Retransmit);
        assert_eq!(lane.classify(3 + 1025), ReplayVerdict::Fresh);
    }

    /// Counter 0 is a Replay in EVERY lane state (fixed 2026-07-05; previously a fresh lane
    /// compared 0 == 0 and re-ACKed it as a "retransmit" of a transfer that never happened):
    /// 0 means "never sent" (TX counters start at 1), so a zero-counter frame is dropped
    /// silently — never delivered, never ACKed. (Per the ordering contract, such a frame only
    /// reaches the kernel at all if it authenticated under the key.)
    #[test]
    fn counter_zero_is_always_replay() {
        let fresh = ReplayLane::new(0);
        assert_eq!(fresh.classify(0), ReplayVerdict::Replay);
        let mut used = ReplayLane::new(0);
        let _ = used.accept(7);
        assert_eq!(used.classify(0), ReplayVerdict::Replay);
    }

    /// A forged huge counter never reaches the lane (CCM rejects first — the ordering contract
    /// in the module docs). What the kernel itself guarantees: `classify` is pure, so merely
    /// *looking* at any counter, however huge, cannot poison the lane.
    #[test]
    fn classify_never_mutates() {
        let mut lane = ReplayLane::new(0);
        let _ = lane.accept(7);
        let _ = lane.classify(u32::MAX);
        assert_eq!(lane.last_seen(), 7);
        assert_eq!(lane.classify(8), ReplayVerdict::Fresh); // lane unchanged by the probe
    }

    /// Reboot-window semantics (docs/storage.md): the lane persisted its last-seen up to P
    /// accepts ago, so a lane restored from that stale value re-accepts the last ≤ P counters
    /// once each — and each re-accept slides the lane so a *second* copy is rejected.
    #[test]
    fn reboot_replay_window_is_persistence_lag() {
        // Live lane accepted 1..=40; the persist fired at the 32nd accept (counter 32).
        let mut live = ReplayLane::new(0);
        let mut persisted = None;
        for c in 1..=40u32 {
            if live.accept(c) {
                persisted = Some(live.last_seen());
            }
        }
        assert_eq!(persisted, Some(32));
        // "Reboot": restore from the persisted value. Counters 33..=40 (≤ P of them) are
        // accepted AGAIN — that is exactly the documented ≤ P window…
        let mut rebooted = ReplayLane::new(persisted.unwrap());
        for c in 33..=40u32 {
            assert_eq!(rebooted.classify(c), ReplayVerdict::Fresh, "counter {c}");
            let _ = rebooted.accept(c);
            // …once each: the second copy of the same counter is no longer fresh.
            assert_eq!(rebooted.classify(c), ReplayVerdict::Retransmit);
        }
        // And anything at or below the persisted mark stays dead.
        assert_eq!(rebooted.classify(32), ReplayVerdict::Replay);
    }

    /// The lazy-persist cadence: `accept` returns true exactly on every P-th accept (the 32nd,
    /// 64th, …), independent of the counter values accepted.
    #[test]
    fn persist_cadence_every_p_accepts() {
        let mut lane = ReplayLane::new(0);
        let mut counter = 0u32;
        for n in 1..=(3 * P) {
            counter += 7; // arbitrary stride — cadence follows accepts, not counters
            let persist = lane.accept(counter);
            assert_eq!(persist, n % P == 0, "accept #{n}");
        }
    }

    /// u32 ceiling: once last-seen sits at 2³²−1 nothing can ever be fresh again — the strict
    /// `counter > last_seen` rule rejects every further frame, so the link fails **closed**
    /// (docs/radio.md: the TX side saturates at the same ceiling; re-key well before).
    #[test]
    fn u32_boundary_fails_closed() {
        let mut lane = ReplayLane::new(u32::MAX - 1);
        assert_eq!(lane.classify(u32::MAX), ReplayVerdict::Fresh);
        let _ = lane.accept(u32::MAX);
        assert_eq!(lane.classify(u32::MAX), ReplayVerdict::Retransmit);
        assert_eq!(lane.classify(0), ReplayVerdict::Replay);
        assert_eq!(lane.classify(u32::MAX - 1), ReplayVerdict::Replay);
        // No wrap-around acceptance exists anywhere in the space.
        for c in [1u32, 1000, u32::MAX / 2] {
            assert_eq!(lane.classify(c), ReplayVerdict::Replay);
        }
    }

    /// A restored lane (peer re-added, addr-matched record) resumes its window rather than
    /// reopening it: nothing at or below the restored last-seen is accepted.
    #[test]
    fn restored_lane_resumes_window() {
        let lane = ReplayLane::new(500);
        assert_eq!(lane.classify(500), ReplayVerdict::Retransmit);
        assert_eq!(lane.classify(499), ReplayVerdict::Replay);
        assert_eq!(lane.classify(501), ReplayVerdict::Fresh);
    }

    // --- assign_lane (addr-keyed replay-lane binding) ------------------------------------------

    /// An unknown peer with no record claims the lowest free local (fresh); an occupied local
    /// is skipped.
    #[test]
    fn assign_lane_claims_lowest_free() {
        let records = [None, None, None, None];
        assert_eq!(
            assign_lane(0xAAAA, &records, 0),
            Some(LaneBinding {
                index: 0,
                seen: 0,
                fresh: true
            })
        );
        assert_eq!(
            assign_lane(0xAAAA, &records, 0b0001),
            Some(LaneBinding {
                index: 1,
                seen: 0,
                fresh: true
            })
        );
    }

    /// A peer finds its OWN record by addr, at whatever local it sits — the restore path.
    #[test]
    fn assign_lane_restores_own_record_by_addr() {
        let records = [Some((0xA, 10)), Some((0xB, 20)), Some((0xC, 30)), Some((0xD, 40))];
        assert_eq!(
            assign_lane(0xC, &records, 0),
            Some(LaneBinding {
                index: 2,
                seen: 30,
                fresh: false
            })
        );
        assert_eq!(
            assign_lane(0xD, &records, 0),
            Some(LaneBinding {
                index: 3,
                seen: 40,
                fresh: false
            })
        );
    }

    /// THE regression: A,B,C,D registered (records A@0,B@1,C@2,D@3) → B removed (its record
    /// persists) → on reboot the boot mirror adds A,C,D in registry order; C and D must RESTORE
    /// their own lanes even though their table slots shifted. The old slot-indexed scheme reset
    /// C and D to 0 here — reopening their replay windows.
    #[test]
    fn assign_lane_survives_remove_then_reboot() {
        let records = [Some((0xA, 10)), Some((0xB, 20)), Some((0xC, 30)), Some((0xD, 40))];
        let mut used = 0u32;
        let a = assign_lane(0xA, &records, used).unwrap();
        assert_eq!((a.index, a.seen, a.fresh), (0, 10, false));
        used |= 1 << a.index;
        let c = assign_lane(0xC, &records, used).unwrap();
        assert_eq!(
            (c.index, c.seen, c.fresh),
            (2, 30, false),
            "C must restore, not reset"
        );
        used |= 1 << c.index;
        let d = assign_lane(0xD, &records, used).unwrap();
        assert_eq!(
            (d.index, d.seen, d.fresh),
            (3, 40, false),
            "D must restore, not reset"
        );
    }

    /// A new peer claims the local freed by a removed one (its stale record is overwritten).
    #[test]
    fn assign_lane_new_peer_claims_freed_local() {
        let records = [Some((0xA, 10)), Some((0xB, 20)), Some((0xC, 30)), Some((0xD, 40))];
        let used = 0b1101; // A@0, C@2, D@3 live; local 1 free (B removed).
        let e = assign_lane(0xE, &records, used).unwrap();
        assert_eq!((e.index, e.fresh), (1, true));
    }

    /// No free local (every local bound to a live peer) → None.
    #[test]
    fn assign_lane_none_when_full() {
        let records = [Some((0xA, 1)), Some((0xB, 2))];
        assert_eq!(assign_lane(0xC, &records, 0b11), None);
    }

    /// An addr-match at a local already held by a live peer is skipped (no two live peers on one
    /// local) — fall through to a claim.
    #[test]
    fn assign_lane_skips_addr_match_at_occupied_local() {
        let records = [Some((0xA, 10)), None];
        assert_eq!(
            assign_lane(0xA, &records, 0b01),
            Some(LaneBinding {
                index: 1,
                seen: 0,
                fresh: true
            })
        );
    }
}
