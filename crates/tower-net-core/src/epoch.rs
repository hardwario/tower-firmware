//! FHSS beacon-epoch acceptance machine (moved from `src/radio/net/fhss.rs`; FCC §15.247
//! hopping — the sync-security half).
//!
//! CCM auth proves a beacon is genuine but NOT fresh — a captured beacon replayed later still
//! authenticates — so freshness is enforced here: the monotonic gateway **epoch** (boot-id,
//! reject-backwards), the **two-beacon consistency guard** during acquisition, and the
//! **prediction-slack** check while tracking. The [`EpochGate`] is the pure decision machine;
//! the firmware keeps the radio, the slot clock (`Instant` anchor) and the EEPROM epoch
//! persistence, and feeds received `(epoch, slot)` pairs plus elapsed-time measurements in.

/// Slots of slack allowed between the slot advance of two acquisition beacons and the real time
/// elapsed between them. During acquisition the node parks on the rendezvous channel and hears the
/// gateway once per cycle, so two successive beacons are ~`FHSS_N` slots (one 24 s cycle) apart;
/// their `slot_abs` advance must match the elapsed wall-time (± this slack). A *replayed* capture
/// carries a FIXED `slot_abs`, so a second copy fails this check (its slot didn't advance while
/// time did) and the anchor is never committed — the two-beacon consistency guard.
pub const ACQUIRE_SLOT_SLACK: u32 = 3;

/// Slots of slack allowed between a beacon's slot index and the node's prediction before that
/// beacon is trusted to re-anchor the clock (while Synced). Real beacons land on the predicted
/// slot — drift across [`REACQUIRE_LIMIT`] is ≪ one slot — so this is tight; a beacon far off
/// prediction (e.g. a stale *replayed* capture) is ignored as a miss rather than yanking the
/// clock. A genuinely restarted gateway (epoch reset to ~0) is still recovered the normal way:
/// misses accrue → rescan → a fresh acquisition (no anchor held) accepts any beacon.
pub const FHSS_RESYNC_SLACK: u32 = 2;

/// Consecutive missed beacons the node rides through on its predicted channel
/// (keeping its clock anchor) before giving up and re-scanning from the rendezvous
/// channel. The anchor's drift over this span (24·300 ms·100 ppm ≈ 720 µs) stays far
/// inside the beacon RX window, so the prediction is still correct and a fade up to
/// ~24·300 ms ≈ 7 s re-locks within one slot once RF returns — *without* a sync loss
/// or a ~1-cycle rendezvous. A genuinely dead/restarted gateway is detected in ~7 s,
/// then recovered via the rendezvous channel.
pub const REACQUIRE_LIMIT: u32 = 24;

/// Master epoch bump rule: the next session's epoch is the stored one + 1 (starting from 1 on a
/// virgin store), saturating — strictly newer than any previous session's, so nodes reject the
/// older sessions' beacons as replays.
#[must_use]
pub fn next_master_epoch(stored: Option<u32>) -> u32 {
    stored.unwrap_or(0).saturating_add(1)
}

/// Verdict on a beacon received while tracking (a clock anchor is held).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackVerdict {
    /// Genuine and on-prediction: re-anchor the clock to it (→ Synced, misses reset).
    ReAnchor,
    /// From an older session (replay) or off-prediction — do NOT touch the clock; the caller
    /// treats the slot as a miss ([`EpochGate::beacon_missed`]).
    Ignore,
}

/// Verdict after a slot whose beacon did not re-anchor (missed or ignored).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissVerdict {
    /// Ride through on the kept anchor (drift ≪ the RX window for many slots).
    KeepPredicting,
    /// The anchor is too stale to trust ([`REACQUIRE_LIMIT`] consecutive misses) — drop it and
    /// fall back to scanning the rendezvous channel.
    Rescan,
}

/// Verdict on a beacon received while scanning (no anchor held — acquisition).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanVerdict {
    /// From an older gateway session (epoch below the highest we've locked to) — a replay;
    /// ignored entirely, not even held as a candidate.
    Reject,
    /// Keep scanning for a time-consistent successor. `replaced` = this beacon became the held
    /// candidate (first beacon, or it advanced past the one held), so the caller re-stamps its
    /// candidate-arrival clock; `false` = the held candidate stands (this beacon was behind it —
    /// e.g. a replayed capture), so the clock must NOT move or a periodic replay would reset the
    /// elapsed-time reference and deny acquisition indefinitely.
    Hold { replaced: bool },
    /// Two-beacon consistency satisfied: commit the anchor to THIS beacon (→ Synced).
    Lock,
}

/// The node/master epoch state plus the node's miss counter and acquisition candidate.
///
/// Master: `epoch` is this gateway session's monotonic epoch (boot-id) stamped into every beacon.
/// Node: `epoch` is the highest epoch it has anchored to (0 = none seen yet). A node ignores any
/// beacon with `epoch < self.epoch` (a stale replay from an older gateway session); a genuine
/// gateway restart carries a strictly higher epoch and is accepted.
#[derive(Debug, Clone, Copy)]
pub struct EpochGate {
    epoch: u32,
    /// Consecutive tracked slots without an accepted beacon (see [`REACQUIRE_LIMIT`]).
    miss: u32,
    /// Node acquisition scratch: the `(epoch, slot_abs)` of the first beacon seen while
    /// unanchored, held until a second, time-consistent beacon confirms it (see
    /// [`ACQUIRE_SLOT_SLACK`]); its arrival time stays with the caller, which passes the
    /// elapsed slots back in. A replayed capture never produces a consistent successor, so it
    /// can never commit an anchor.
    candidate: Option<(u32, u32)>,
}

impl Default for EpochGate {
    fn default() -> Self {
        Self::new()
    }
}

impl EpochGate {
    /// A gate with no epoch seen (0), no misses, no candidate.
    #[must_use]
    pub fn new() -> Self {
        Self {
            epoch: 0,
            miss: 0,
            candidate: None,
        }
    }

    /// Reset for a new FHSS session: master passes its bumped session epoch
    /// ([`next_master_epoch`]), a node passes 0 (no gateway epoch seen yet this session).
    pub fn reset(&mut self, epoch: u32) {
        self.epoch = epoch;
        self.miss = 0;
        self.candidate = None;
    }

    /// The current epoch (master: stamped into every beacon; node: highest seen).
    #[must_use]
    pub fn epoch(&self) -> u32 {
        self.epoch
    }

    /// Offer a beacon received while tracking (anchor held), against the node's
    /// `predicted_slot`. Re-anchor only to a beacon that is (a) not from an older gateway
    /// session — a replay carrying `epoch < self.epoch` is ignored — and (b) consistent with
    /// our prediction: a stale replayed capture is far behind the predicted slot and falls to
    /// the miss path, so it can't yank our clock. A real beacon lands on the predicted slot
    /// (± a slot of timing slack, [`FHSS_RESYNC_SLACK`]). A genuinely restarted gateway carries
    /// a higher epoch but a slot near 0, so it fails the prediction check here and is recovered
    /// via the rescan path ([`beacon_missed`](Self::beacon_missed) → [`MissVerdict::Rescan`]).
    pub fn offer_tracked(&mut self, epoch: u32, slot_abs: u32, predicted_slot: u32) -> TrackVerdict {
        if epoch >= self.epoch && slot_abs.abs_diff(predicted_slot) <= FHSS_RESYNC_SLACK {
            self.epoch = epoch; // track the newest epoch seen
            self.miss = 0; // re-anchored
            TrackVerdict::ReAnchor
        } else {
            TrackVerdict::Ignore
        }
    }

    /// A tracked slot passed without an accepted beacon (missed, or it failed
    /// [`offer_tracked`](Self::offer_tracked)): count it, and decide whether the anchor is
    /// still trustworthy ([`MissVerdict::KeepPredicting`]) or too stale
    /// ([`MissVerdict::Rescan`], after [`REACQUIRE_LIMIT`] consecutive misses).
    pub fn beacon_missed(&mut self) -> MissVerdict {
        self.miss += 1;
        if self.miss >= REACQUIRE_LIMIT {
            MissVerdict::Rescan
        } else {
            MissVerdict::KeepPredicting
        }
    }

    /// Offer a beacon received while scanning (no anchor). `elapsed_slots` is the wall-clock
    /// time since the held candidate's arrival, in whole slots (`elapsed_ms / FHSS_SLOT_MS`,
    /// measured by the caller; `None` when no candidate is held).
    ///
    /// Two-beacon consistency: commit an anchor only once a SECOND beacon of the SAME epoch
    /// advances its slot in step with the real time elapsed since the first. A replayed capture
    /// repeats one fixed `slot_abs`, so a second copy's slot didn't advance while wall-time did
    /// — the check fails and no anchor is committed. A real gateway's successive rendezvous
    /// beacons advance one cycle (`FHSS_N` slots) per ~24 s, matching the elapsed time.
    pub fn offer_scanning(&mut self, epoch: u32, slot_abs: u32, elapsed_slots: Option<u32>) -> ScanVerdict {
        // A beacon from an older gateway session (epoch below the highest we've locked to) is a
        // replay — ignore it entirely, don't even hold it as a candidate.
        if epoch < self.epoch {
            return ScanVerdict::Reject;
        }
        let consistent = match (self.candidate, elapsed_slots) {
            (Some((e0, s0)), Some(time_adv)) => {
                e0 == epoch && slot_abs > s0 && (slot_abs - s0).abs_diff(time_adv) <= ACQUIRE_SLOT_SLACK
            }
            _ => false,
        };
        if consistent {
            self.candidate = None;
            self.epoch = epoch; // record the session we've now locked to
            self.miss = 0; // locked: misses reset
            ScanVerdict::Lock
        } else {
            // Hold this beacon as the candidate ONLY if it advances past the one we already hold.
            // A replayed capture repeats one fixed (older) slot_abs; if it could evict the genuine
            // candidate, one replay every few slots would deny acquisition forever — the genuine
            // successor never finds its predecessor still held (a keyless, cheaper-than-jamming
            // DoS). A newer epoch, or a higher slot within the same epoch, legitimately wins.
            let replaced = match self.candidate {
                Some((e0, s0)) => epoch > e0 || (epoch == e0 && slot_abs > s0),
                None => true,
            };
            if replaced {
                self.candidate = Some((epoch, slot_abs));
            }
            ScanVerdict::Hold { replaced }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// FHSS channel count the firmware uses (`config::FHSS_N`), mirrored so the acquisition
    /// tests exercise realistic one-cycle slot advances without importing the firmware crate.
    const FHSS_N: u32 = 80;

    /// `true` if a verdict is a Hold (of either `replaced` flavour) — for the many asserts that
    /// only care that acquisition hasn't locked/rejected yet, not which beacon is the candidate.
    fn held(v: ScanVerdict) -> bool {
        matches!(v, ScanVerdict::Hold { .. })
    }

    /// Acquire a fresh node gate onto `epoch`, starting at `slot`: first beacon held, second
    /// one cycle later (consistent) locks. Returns the locked gate.
    fn locked_gate(epoch: u32, slot: u32) -> EpochGate {
        let mut g = EpochGate::new();
        g.reset(0);
        assert!(held(g.offer_scanning(epoch, slot, None)));
        assert_eq!(
            g.offer_scanning(epoch, slot + FHSS_N, Some(FHSS_N)),
            ScanVerdict::Lock
        );
        g
    }

    // --- master epoch bump -----------------------------------------------------------------

    #[test]
    fn master_epoch_bumps_monotonically() {
        assert_eq!(next_master_epoch(None), 1); // virgin store → first session is epoch 1
        assert_eq!(next_master_epoch(Some(1)), 2);
        assert_eq!(next_master_epoch(Some(41)), 42);
        // Saturates rather than wrapping back to an old (replayable) epoch.
        assert_eq!(next_master_epoch(Some(u32::MAX)), u32::MAX);
    }

    // --- acquisition (scanning) ------------------------------------------------------------

    /// Two consistent beacons of the same epoch, one cycle apart, lock; the gate then holds
    /// that session's epoch.
    #[test]
    fn two_consistent_beacons_lock() {
        let g = locked_gate(5, 1000);
        assert_eq!(g.epoch(), 5);
    }

    /// A single beacon NEVER locks — whatever its epoch or slot. The two-beacon guard means a
    /// single replayed capture can't anchor the node.
    #[test]
    fn single_beacon_never_locks() {
        for (epoch, slot) in [(1u32, 0u32), (5, 1000), (u32::MAX, u32::MAX)] {
            let mut g = EpochGate::new();
            assert!(held(g.offer_scanning(epoch, slot, None)));
        }
    }

    /// A replayed capture repeats one fixed slot_abs: however often it is replayed, and whatever
    /// real time elapses between copies, the slot never advances (`slot_abs > s0` fails) — the
    /// anchor is never committed. Crucially each replay returns `replaced: false`, so it does NOT
    /// evict the genuine held candidate — the #5 DoS fix. (Before it, a periodic replay reset the
    /// candidate — and the caller's elapsed-time clock — every time, denying acquisition forever.)
    #[test]
    fn replayed_fixed_capture_never_anchors() {
        let mut g = EpochGate::new();
        assert_eq!(
            g.offer_scanning(7, 500, None),
            ScanVerdict::Hold { replaced: true }
        ); // first: held
        for elapsed in [0u32, 1, FHSS_N, 10 * FHSS_N] {
            assert_eq!(
                g.offer_scanning(7, 500, Some(elapsed)),
                ScanVerdict::Hold { replaced: false }, // does not displace the genuine candidate
                "elapsed {elapsed}"
            );
        }
    }

    /// The #5 regression, end to end: a genuine first beacon is held, then a periodic replay of
    /// an OLDER capture is interleaved with the genuine successor. The replay must not displace
    /// the held candidate, so the successor still finds it and LOCKS — acquisition is not denied.
    #[test]
    fn replay_does_not_deny_acquisition() {
        let mut g = EpochGate::new();
        // Genuine beacon #1 (epoch 5, slot 1000) → held as the candidate.
        assert_eq!(
            g.offer_scanning(5, 1000, None),
            ScanVerdict::Hold { replaced: true }
        );
        // Attacker replays an OLDER same-epoch capture (slot 700) between the genuine beacons.
        assert_eq!(
            g.offer_scanning(5, 700, Some(3)),
            ScanVerdict::Hold { replaced: false }
        );
        // Genuine beacon #2, one cycle after #1, consistent with the STILL-held candidate → lock.
        assert_eq!(
            g.offer_scanning(5, 1000 + FHSS_N, Some(FHSS_N)),
            ScanVerdict::Lock
        );
        assert_eq!(g.epoch(), 5);
    }

    /// A replayed *older-session* beacon (epoch below the highest locked-to) is rejected in the
    /// scanning state — not even held as a candidate that could displace a genuine one.
    #[test]
    fn scanning_rejects_older_epoch_entirely() {
        let mut g = locked_gate(5, 1000);
        // …gateway fades, node rescans (state machine handled by the caller; gate keeps epoch).
        for old in [0u32, 1, 4] {
            assert_eq!(
                g.offer_scanning(old, 50_000, Some(FHSS_N)),
                ScanVerdict::Reject,
                "epoch {old}"
            );
        }
        // The held candidate is untouched by rejected replays: a genuine pair still locks.
        assert!(held(g.offer_scanning(6, 2000, None)));
        assert_eq!(g.offer_scanning(4, 9999, Some(1)), ScanVerdict::Reject);
        assert_eq!(
            g.offer_scanning(6, 2000 + FHSS_N, Some(FHSS_N)),
            ScanVerdict::Lock
        );
    }

    /// Equal-epoch beacons must advance the slot in step with elapsed time, within
    /// ACQUIRE_SLOT_SLACK — the exact boundary: |slot_adv − time_adv| == SLACK locks,
    /// SLACK + 1 holds.
    #[test]
    fn acquire_slot_slack_exact_boundary() {
        for (delta, locks) in [
            (0u32, true),
            (ACQUIRE_SLOT_SLACK, true),
            (ACQUIRE_SLOT_SLACK + 1, false),
        ] {
            // Slot advanced by FHSS_N while time advanced FHSS_N ± delta.
            for time_adv in [FHSS_N - delta, FHSS_N + delta] {
                let mut g = EpochGate::new();
                assert!(held(g.offer_scanning(3, 100, None)));
                let v = g.offer_scanning(3, 100 + FHSS_N, Some(time_adv));
                if locks {
                    assert_eq!(v, ScanVerdict::Lock, "±{delta}");
                } else {
                    assert!(held(v), "±{delta}");
                }
            }
        }
    }

    /// The slot must advance strictly forwards: an equal or *backwards* slot never locks even
    /// if the elapsed time is tiny (slot_abs > s0 is required, not just slack-consistency).
    #[test]
    fn acquire_requires_forward_slot() {
        let mut g = EpochGate::new();
        assert_eq!(
            g.offer_scanning(3, 100, None),
            ScanVerdict::Hold { replaced: true }
        ); // first
        // Equal and backward slots hold but do NOT displace the candidate (`replaced: false`).
        assert_eq!(
            g.offer_scanning(3, 100, Some(0)),
            ScanVerdict::Hold { replaced: false }
        ); // equal
        assert_eq!(
            g.offer_scanning(3, 99, Some(0)),
            ScanVerdict::Hold { replaced: false }
        ); // backwards
    }

    /// A candidate/second-beacon epoch mismatch (both current-or-newer) can't lock; the newer
    /// beacon replaces the candidate, and a consistent successor of the SAME epoch then locks.
    #[test]
    fn epoch_mismatch_replaces_candidate() {
        let mut g = EpochGate::new();
        assert!(held(g.offer_scanning(3, 100, None)));
        // Newer session appears mid-acquisition: held instead (e0 == epoch fails → Hold).
        assert!(held(g.offer_scanning(4, 20, Some(FHSS_N))));
        assert_eq!(g.offer_scanning(4, 20 + FHSS_N, Some(FHSS_N)), ScanVerdict::Lock);
        assert_eq!(g.epoch(), 4);
    }

    /// An inconsistent pair replaces the candidate with the newest beacon (the guard restarts
    /// from it) — a following beacon consistent with the NEW candidate locks.
    #[test]
    fn inconsistent_pair_restarts_from_newest() {
        let mut g = EpochGate::new();
        assert!(held(g.offer_scanning(3, 100, None)));
        // Slot jumped far more than time elapsed → inconsistent, becomes the new candidate.
        assert!(held(g.offer_scanning(3, 100 + 10 * FHSS_N, Some(FHSS_N))));
        // Consistent with the NEW candidate (not the old one) → locks.
        let s = 100 + 10 * FHSS_N;
        assert_eq!(g.offer_scanning(3, s + FHSS_N, Some(FHSS_N)), ScanVerdict::Lock);
    }

    /// After a gateway restart (epoch bump), a locked-then-lost node re-locks onto the NEW
    /// session — and from then on the old session's beacons are replays.
    #[test]
    fn epoch_bump_relocks_and_invalidates_old_session() {
        let mut g = locked_gate(5, 1000);
        // New session: epoch 6 accepted through a fresh two-beacon acquisition.
        assert!(held(g.offer_scanning(6, 10, None)));
        assert_eq!(g.offer_scanning(6, 10 + FHSS_N, Some(FHSS_N)), ScanVerdict::Lock);
        assert_eq!(g.epoch(), 6);
        // Captures of the previous session are now dead in both states.
        assert_eq!(g.offer_scanning(5, 99_999, Some(FHSS_N)), ScanVerdict::Reject);
        assert_eq!(g.offer_tracked(5, 1000, 1000), TrackVerdict::Ignore);
    }

    // --- tracking (anchored) ---------------------------------------------------------------

    /// While tracking, an equal-epoch beacon on the predicted slot (± FHSS_RESYNC_SLACK)
    /// re-anchors; the exact boundary: slack slots off still re-anchors, slack + 1 is ignored.
    #[test]
    fn resync_slack_exact_boundary() {
        for (delta, accept) in [
            (0u32, true),
            (FHSS_RESYNC_SLACK, true),
            (FHSS_RESYNC_SLACK + 1, false),
        ] {
            let mut g = locked_gate(5, 1000);
            let predicted = 2000u32;
            for slot in [predicted - delta, predicted + delta] {
                let v = g.offer_tracked(5, slot, predicted);
                let want = if accept {
                    TrackVerdict::ReAnchor
                } else {
                    TrackVerdict::Ignore
                };
                assert_eq!(v, want, "slot {slot} vs predicted {predicted}");
            }
        }
    }

    /// A replayed older-epoch beacon is ignored in the tracked state even if it happens to land
    /// on the predicted slot — the epoch test comes first.
    #[test]
    fn tracked_rejects_older_epoch_on_prediction() {
        let mut g = locked_gate(5, 1000);
        assert_eq!(g.offer_tracked(4, 2000, 2000), TrackVerdict::Ignore);
        assert_eq!(g.offer_tracked(0, 2000, 2000), TrackVerdict::Ignore);
    }

    /// A restarted gateway (higher epoch, slot near 0) fails the prediction check while an
    /// anchor is held — it must be recovered via misses → rescan, never by yanking the clock.
    #[test]
    fn tracked_ignores_restarted_gateway_off_prediction() {
        let mut g = locked_gate(5, 100_000);
        assert_eq!(g.offer_tracked(6, 3, 101_000), TrackVerdict::Ignore);
    }

    /// A higher epoch ON prediction is accepted and tracked (the "track the newest epoch seen"
    /// rule) — after it, the previous epoch's beacons are replays.
    #[test]
    fn tracked_follows_newest_epoch() {
        let mut g = locked_gate(5, 1000);
        assert_eq!(g.offer_tracked(6, 2001, 2000), TrackVerdict::ReAnchor);
        assert_eq!(g.epoch(), 6);
        assert_eq!(g.offer_tracked(5, 3000, 3000), TrackVerdict::Ignore);
    }

    /// Misses accrue to REACQUIRE_LIMIT before the anchor is given up: 23 misses ride through,
    /// the 24th rescans. A re-anchor resets the run.
    #[test]
    fn miss_limit_exact() {
        let mut g = locked_gate(5, 1000);
        for i in 1..REACQUIRE_LIMIT {
            assert_eq!(g.beacon_missed(), MissVerdict::KeepPredicting, "miss {i}");
        }
        assert_eq!(g.beacon_missed(), MissVerdict::Rescan);

        // Fresh lock → the counter starts over.
        let mut g = locked_gate(5, 1000);
        for _ in 0..10 {
            assert_eq!(g.beacon_missed(), MissVerdict::KeepPredicting);
        }
        assert_eq!(g.offer_tracked(5, 2000, 2000), TrackVerdict::ReAnchor); // resets misses
        for i in 1..REACQUIRE_LIMIT {
            assert_eq!(
                g.beacon_missed(),
                MissVerdict::KeepPredicting,
                "miss {i} after re-anchor"
            );
        }
        assert_eq!(g.beacon_missed(), MissVerdict::Rescan);
    }

    /// An ignored beacon does not itself count a miss (the caller counts the slot once, via
    /// beacon_missed) — pin the split so double-counting can't creep in.
    #[test]
    fn ignored_beacon_does_not_count_twice() {
        let mut g = locked_gate(5, 1000);
        for _ in 0..(REACQUIRE_LIMIT - 1) {
            assert_eq!(g.offer_tracked(4, 2000, 2000), TrackVerdict::Ignore); // replay attempt…
            assert_eq!(g.beacon_missed(), MissVerdict::KeepPredicting); // …counted once by the caller
        }
        assert_eq!(g.beacon_missed(), MissVerdict::Rescan);
    }

    // --- session reset ---------------------------------------------------------------------

    /// `reset` re-arms the gate for a new session: node (epoch 0) accepts any epoch again;
    /// master carries its bumped boot-id.
    #[test]
    fn reset_rearms_the_gate() {
        let mut g = locked_gate(9, 1000);
        g.reset(0); // node re-enables FHSS: no gateway epoch seen yet this session
        assert_eq!(g.epoch(), 0);
        assert!(held(g.offer_scanning(1, 10, None))); // low epoch OK again
        g.reset(next_master_epoch(Some(9))); // master session
        assert_eq!(g.epoch(), 10);
    }
}
