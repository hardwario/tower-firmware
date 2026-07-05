//! TX-counter reserve-ahead watermark + fail-closed nonce-safety kernel (moved from
//! `src/radio/net/mod.rs`; docs/radio.md §Security model).
//!
//! The TX counter is the CCM nonce input, so the one invariant that must survive everything —
//! including power loss mid-write — is: **a counter value is never used twice under one key**.
//! The kernel guarantees it with a reserve-ahead watermark: a boot resumes the counter *at* the
//! last durably persisted watermark (strictly greater than any value actually sent) and
//! immediately reserves the next block, so at most one block of counter space is skipped per
//! reboot and no value repeats. Whenever durability can no longer be proven — the watermark
//! record is missing on a store that shows prior use, a watermark persist fails, or the counter
//! hits the 2³²−1 ceiling — TX **locks** (fail closed) rather than risk a nonce reuse.
//!
//! The kernel is pure: the caller (the firmware's `Net`) owns the EEPROM and reports persist
//! outcomes back via [`reserve_persisted`](TxCounter::reserve_persisted) / [`lock`](TxCounter::lock).

/// TX-counter reserve block: persist the watermark only once per `RESERVE`
/// transfers, and on boot resume *at* the watermark (> any value actually sent,
/// so a counter is never reused; ≤ one block is skipped per reboot, docs/radio.md).
pub const RESERVE: u32 = 1024;

/// The TX-counter state: live counter, reserved (persisted) limit, and the fail-closed lock.
#[derive(Debug, Clone, Copy)]
pub struct TxCounter {
    /// Monotonic TX counter, advanced by one per transfer (docs/radio.md).
    counter: u32,
    /// Highest reserved (persisted) counter value; `counter < reserve_limit`
    /// holds as an invariant only while [`locked`](Self::locked) is false —
    /// the counter is never used at or past the last *durably* persisted watermark.
    reserve_limit: u32,
    /// Set once a reserve-watermark persist fails: TX is refused (fail closed) so a
    /// counter past the last durable watermark can never go on air (which after a
    /// reboot resuming at the stale watermark would reuse a CCM nonce). See
    /// [`advance`](Self::advance) and the firmware's `RadioError::NonceLocked`.
    locked: bool,
}

impl TxCounter {
    /// Boot-time resume from the persisted state: `stored_watermark` is the watermark record
    /// (if readable), `prior_use` whether the store shows any other evidence of prior network
    /// use (the firmware passes "a persisted last-seen exists"). Resumes the counter *at* the
    /// watermark (1 on the very first boot, since 0 = "never sent") and reserves the next
    /// block. Returns the kernel plus the watermark the caller must now persist **and verify**
    /// — `None` when the boot must not write one (see below), in which case the kernel starts
    /// locked. On a successful, read-back-verified persist the caller does nothing further; on
    /// failure it must call [`lock`](Self::lock).
    ///
    /// A store that shows prior use (a persisted last-seen) but whose watermark is now
    /// unreadable has almost certainly lost the watermark record to EEPROM corruption
    /// (`scan_half` stops at the first bad record, orphaning everything after it — and the
    /// watermark, rewritten every RESERVE transfers, tends to sit late in the log). Resuming
    /// the TX counter at 1 under the unchanged key would then reuse every CCM nonce 1..old.
    /// Fail **closed** WITHOUT rewriting a fresh low watermark (that would poison a later boot
    /// into resuming low and reusing nonces). The device can still receive; sending needs a
    /// re-key / factory reset. A genuinely virgin store (no watermark AND no last-seen)
    /// legitimately starts at 1.
    #[must_use = "the returned watermark must be persisted and verified (lock() on failure)"]
    pub fn resume(stored_watermark: Option<u32>, prior_use: bool) -> (Self, Option<u32>) {
        let watermark_lost = stored_watermark.is_none() && prior_use;
        let resume = stored_watermark.unwrap_or(1).max(1);
        // Saturating, not wrapping: a stored watermark within RESERVE of u32::MAX must pin
        // the limit at the ceiling — wrapping would persist a LOW watermark, and the boot
        // after that would resume low and reuse CCM nonces 1..old (the exact break this type
        // exists to prevent). Saturated, the counter just runs into `advance`'s u32::MAX
        // ceiling lock within one block: the link ends its life failing closed, never
        // repeating a nonce.
        let reserve_limit = resume.saturating_add(RESERVE);
        let txc = Self {
            counter: resume,
            reserve_limit,
            locked: watermark_lost,
        };
        (txc, if watermark_lost { None } else { Some(reserve_limit) })
    }

    /// Current live TX counter — the value the next frame rides (and the CCM nonce input).
    #[must_use]
    pub fn counter(&self) -> u32 {
        self.counter
    }

    /// Current reserved (persisted) watermark.
    #[must_use]
    pub fn reserve_limit(&self) -> u32 {
        self.reserve_limit
    }

    /// Whether TX is locked (fail closed — no counter may be allocated / no frame emitted).
    #[must_use]
    pub fn locked(&self) -> bool {
        self.locked
    }

    /// Advance the TX counter by one transfer. Returns the next watermark to persist when the
    /// current reserve is exhausted — the caller must persist it and report back with
    /// [`reserve_persisted`](Self::reserve_persisted) (success) or [`lock`](Self::lock)
    /// (failure) before allocating another counter.
    ///
    /// **Saturating, not wrapping** — the counter is the CCM nonce input, so it must never
    /// wrap back to a reused value. At the 2³²−1 ceiling (≈136 yr at 1 Hz — practically
    /// unreachable) it **locks TX** (`locked = true`): every subsequent send fails closed
    /// rather than transmitting another frame under the pinned `u32::MAX` nonce. A reused
    /// `(key, nonce)` is a CCM confidentiality *and* integrity break (keystream + MAC reuse),
    /// not merely a delivery failure the replay rule would reject — so we must stop emitting,
    /// not rely on the peer to drop it. Re-key well before then.
    #[must_use = "a returned watermark must be persisted (reserve_persisted / lock on failure)"]
    pub fn advance(&mut self) -> Option<u32> {
        self.counter = self.counter.saturating_add(1);
        if self.counter == u32::MAX {
            // Ceiling reached: fail closed. Do not emit any further frame (the next send would
            // reuse the pinned MAX nonce) and stop churning the reserve watermark.
            self.locked = true;
            return None;
        }
        if self.counter >= self.reserve_limit {
            // Extend the reservation, but only trust it once the write lands: the caller
            // persists this value and reports back. See `reserve_persisted` / `lock`.
            return Some(self.reserve_limit.saturating_add(RESERVE));
        }
        None
    }

    /// The watermark returned by [`advance`](Self::advance) was durably persisted: commit it as
    /// the new reserve limit.
    pub fn reserve_persisted(&mut self, watermark: u32) {
        self.reserve_limit = watermark;
    }

    /// A watermark persist failed (or did not read back): lock TX, permanently. We must NOT
    /// advance `reserve_limit` on a failed persist — doing so would let us keep emitting
    /// counters past the last *durable* watermark, and a reboot that then resumes at the stale
    /// watermark would reuse those CCM nonces. Instead fail closed — the guard on every send
    /// path refuses to transmit while locked. Nothing unlocks a `TxCounter`.
    pub fn lock(&mut self) {
        self.locked = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drive a kernel through `n` transfers with an always-succeeding persist store, returning
    /// every counter value consumed. Mirrors the firmware's advance_tx_counter loop.
    fn run(txc: &mut TxCounter, store: &mut Option<u32>, n: u32, out: &mut impl FnMut(u32)) {
        for _ in 0..n {
            assert!(!txc.locked());
            out(txc.counter());
            if let Some(next) = txc.advance() {
                *store = Some(next); // persist OK
                txc.reserve_persisted(next);
            }
        }
    }

    /// Fresh start (virgin store): counter resumes at 1 ("0 = never sent"), the first block is
    /// reserved at 1 + RESERVE, and the boot asks for that watermark to be persisted.
    #[test]
    fn fresh_start() {
        let (txc, persist) = TxCounter::resume(None, false);
        assert!(!txc.locked());
        assert_eq!(txc.counter(), 1);
        assert_eq!(txc.reserve_limit(), 1 + RESERVE);
        assert_eq!(persist, Some(1 + RESERVE));
    }

    /// A stored watermark of 0 still resumes at 1 (the `.max(1)` floor — counter 0 is reserved
    /// for "never sent").
    #[test]
    fn stored_zero_watermark_resumes_at_one() {
        let (txc, persist) = TxCounter::resume(Some(0), true);
        assert_eq!(txc.counter(), 1);
        assert_eq!(persist, Some(1u32.wrapping_add(RESERVE)));
        assert!(!txc.locked());
    }

    /// Resume-from-watermark never reuses: the counter resumes AT the persisted watermark —
    /// strictly greater than anything sent in the previous life — and the new reserve jumps
    /// ahead by RESERVE.
    #[test]
    fn resume_at_watermark_never_reuses() {
        // Life 1: fresh boot, send 700 transfers (persisted watermark stays 1 + RESERVE).
        let mut store = None;
        let (mut txc, persist) = TxCounter::resume(None, false);
        store = persist.or(store);
        let mut max_sent = 0;
        run(&mut txc, &mut store, 700, &mut |c| max_sent = max_sent.max(c));
        assert_eq!(max_sent, 700);
        assert_eq!(store, Some(1 + RESERVE));

        // Life 2 (power lost without further persists): resumes at the watermark.
        let (txc2, persist2) = TxCounter::resume(store, true);
        assert!(txc2.counter() > max_sent, "resume must exceed anything sent");
        assert_eq!(txc2.counter(), 1 + RESERVE);
        assert_eq!(persist2, Some(1 + 2 * RESERVE)); // watermark jumps by RESERVE
    }

    /// Repeated power-loss cycles: counters are strictly increasing across lives, with no
    /// value ever consumed twice, and each boot advances the watermark monotonically.
    #[test]
    fn repeated_power_loss_is_monotonic() {
        let mut store: Option<u32> = None;
        let mut prior_use = false;
        let mut last_counter = 0u32;
        let mut last_watermark = 0u32;
        for life in 0u32..20 {
            let (mut txc, persist) = TxCounter::resume(store, prior_use);
            assert!(!txc.locked(), "life {life}");
            let wm = persist.expect("boot persists a watermark");
            assert!(wm > last_watermark, "watermark monotone (life {life})");
            store = Some(wm);
            last_watermark = wm;
            // Send an irregular number of transfers, some lives crossing a reserve boundary.
            let sends = 1 + (life * 397) % (RESERVE + 300);
            run(&mut txc, &mut store, sends, &mut |c| {
                assert!(c > last_counter, "counter {c} reused/regressed in life {life}");
                last_counter = c;
            });
            prior_use = true; // later lives have receive/lane history
        }
    }

    /// Watermark-lost boot (prior use but no readable watermark): locked, and NO watermark is
    /// written (a fresh low watermark would poison a later boot into resuming low).
    #[test]
    fn watermark_lost_fails_closed_without_writing() {
        let (txc, persist) = TxCounter::resume(None, true);
        assert!(txc.locked());
        assert_eq!(persist, None, "must not rewrite a fresh low watermark");
    }

    /// A genuinely virgin store (no watermark AND no prior use) legitimately starts unlocked.
    #[test]
    fn virgin_store_is_not_watermark_lost() {
        let (txc, persist) = TxCounter::resume(None, false);
        assert!(!txc.locked());
        assert!(persist.is_some());
    }

    /// Boot persist failure: the caller locks the kernel; it stays locked.
    #[test]
    fn boot_persist_failure_locks() {
        let (mut txc, persist) = TxCounter::resume(None, false);
        assert!(persist.is_some());
        txc.lock(); // write failed / did not read back
        assert!(txc.locked());
    }

    /// Off-by-one at the reserve boundary: from a fresh boot (counter 1, limit 1+RESERVE) the
    /// counter values 2..=RESERVE advance with no persist request; stepping onto the limit
    /// (counter == 1+RESERVE) requests the next watermark. While unlocked, the counter never
    /// passes the durable watermark.
    #[test]
    fn reserve_boundary_exact() {
        let (mut txc, _persist) = TxCounter::resume(None, false);
        let limit = txc.reserve_limit();
        // Advances that keep counter < limit request nothing.
        for _ in 0..(RESERVE - 1) {
            assert!(txc.counter() < limit);
            assert_eq!(txc.advance(), None);
        }
        assert_eq!(txc.counter(), RESERVE); // == limit − 1: last counter under the watermark
        // The advance that reaches the watermark must re-reserve BEFORE further sends.
        assert_eq!(txc.advance(), Some(limit + RESERVE));
        assert_eq!(txc.counter(), limit);
        txc.reserve_persisted(limit + RESERVE);
        assert_eq!(txc.reserve_limit(), limit + RESERVE);
        assert_eq!(txc.advance(), None); // back under the (new) watermark
    }

    /// Reserve-persist failure mid-life: the kernel locks and the reserve limit is NOT
    /// advanced (emitting past the last durable watermark is exactly the nonce-reuse hazard).
    #[test]
    fn reserve_persist_failure_locks_without_advancing_limit() {
        let (mut txc, _persist) = TxCounter::resume(None, false);
        let limit = txc.reserve_limit();
        for _ in 0..(RESERVE - 1) {
            assert_eq!(txc.advance(), None);
        }
        let req = txc.advance();
        assert_eq!(req, Some(limit + RESERVE));
        txc.lock(); // persist failed
        assert!(txc.locked());
        assert_eq!(
            txc.reserve_limit(),
            limit,
            "failed persist must not extend the reserve"
        );
    }

    /// Near-exhaustion: reaching the 2³²−1 ceiling locks TX — and it STAYS locked (nothing
    /// unlocks a TxCounter; further advances keep saturating at MAX and re-locking).
    #[test]
    fn exhaustion_locks_and_stays_locked() {
        // Start near the ceiling via a (huge) stored watermark.
        let (mut txc, _persist) = TxCounter::resume(Some(u32::MAX - 3), true);
        assert_eq!(txc.counter(), u32::MAX - 3);
        let mut advances = 0;
        while !txc.locked() {
            if let Some(next) = txc.advance() {
                txc.reserve_persisted(next);
            }
            advances += 1;
            assert!(advances < 10, "must lock within a few advances of the ceiling");
        }
        // Locked exactly when the counter pinned at MAX.
        assert_eq!(txc.counter(), u32::MAX);
        assert_eq!(advances, 3);
        // Sticky: even the (firmware-unreachable) advance-while-locked path keeps it locked.
        assert_eq!(txc.advance(), None);
        assert!(txc.locked());
        assert_eq!(txc.counter(), u32::MAX); // saturating: the MAX nonce is never exceeded…
    }

    /// The exact ceiling edge: advancing from MAX−1 pins the counter at MAX and locks in the
    /// same step, with no watermark churn (no persist request).
    #[test]
    fn ceiling_edge_off_by_one() {
        let (mut txc, _persist) = TxCounter::resume(Some(u32::MAX - 1), true);
        assert!(!txc.locked());
        assert_eq!(txc.counter(), u32::MAX - 1); // one frame may still ride MAX−1
        assert_eq!(txc.advance(), None, "ceiling lock must not churn the watermark");
        assert!(txc.locked());
        assert_eq!(txc.counter(), u32::MAX);
    }

    /// A near-MAX stored watermark must SATURATE the reserve limit at u32::MAX (fixed
    /// 2026-07-05; previously `wrapping_add` could persist a LOW watermark here, and the boot
    /// after that would resume low and reuse CCM nonces 1..old). Reachable only after ~2³²
    /// transfers, but the fail-closed guarantee now holds at the very edge too: the counter
    /// drains the final saturated block and runs into `advance`'s ceiling lock.
    #[test]
    fn near_max_resume_saturates_reserve_limit_and_ends_locked() {
        let (mut txc, persist) = TxCounter::resume(Some(u32::MAX - 100), true);
        assert_eq!(txc.counter(), u32::MAX - 100);
        assert_eq!(txc.reserve_limit(), u32::MAX);
        assert_eq!(
            persist,
            Some(u32::MAX),
            "the boot-persisted watermark must never be low"
        );
        assert!(!txc.locked());
        // Drain the final saturated block: no watermark extension is ever requested (the
        // limit already sits at the ceiling), and the ceiling locks TX — never a wrap.
        for _ in 0..200 {
            assert!(txc.advance().is_none(), "no extension past a saturated limit");
            if txc.locked() {
                break;
            }
        }
        assert!(
            txc.locked(),
            "the ceiling must lock TX — the link ends closed, never wraps"
        );
        assert_eq!(txc.counter(), u32::MAX);
    }
}
