//! Host unit tests for the pure radio-timing core.
//!
//! These are the regulatory-arithmetic properties the firmware relies on but cannot test
//! itself (no_std, thumbv6m default target, no libtest). They pin the invariants the
//! security/compliance fixes established: the duty-governor residue carry (no refill lost to
//! truncation), the fixed-permutation FHSS spacing (§15.247), and non-under-counted airtime.

use super::*;

/// FHSS channel count the firmware uses (`config::FHSS_N`). Mirrored here so the permutation
/// tests exercise the real size without importing the firmware crate.
const FHSS_N: usize = 80;
/// FHSS slot length (ms) from `src/radio/net/fhss.rs` (`FHSS_SLOT_MS`). The cycle is
/// `FHSS_N · FHSS_SLOT_MS`; the compliance test asserts a channel's successive visits are
/// exactly one cycle apart, which must exceed the 20 s averaging window.
const FHSS_SLOT_MS: u64 = 300;

// --- DutyGovernor: residue / refill-truncation property (the parent's fix) ---------------

/// 1000 calls of `refill_ms(50)` must credit *exactly* the same airtime as one
/// `refill_ms(50_000)` — no credit lost to per-call integer truncation. This is the residue
/// accumulator the parent added: at 1 % (permil = 10), a 50 ms call earns `50·10/1000 = 0` ms
/// of whole-millisecond refill and, without carrying the 500-permil-ms remainder, would credit
/// *nothing* over 1000 calls while a single 50 s call credits 500 ms.
#[test]
fn refill_residue_no_truncation_loss() {
    // Drain both buckets to the same known-empty state first (cap is EU_CAP_MS, permil 10).
    let mut many = DutyGovernor::eu();
    let mut once = DutyGovernor::eu();
    assert!(many.try_consume(EU_CAP_MS)); // empty
    assert!(once.try_consume(EU_CAP_MS)); // empty
    assert_eq!(many.budget_ms(), 0);
    assert_eq!(once.budget_ms(), 0);

    for _ in 0..1000 {
        many.refill_ms(50);
    }
    once.refill_ms(50_000);

    // 50_000 ms · 10‰ = 500 ms credited, both ways, to the millisecond.
    assert_eq!(many.budget_ms(), 500, "residue carry lost refill to truncation");
    assert_eq!(once.budget_ms(), 500);
    assert_eq!(many.budget_ms(), once.budget_ms());
}

/// The same identity across a range of split counts and intervals: N refills of `step`
/// always equal one refill of `N·step` (mod the shared cap), for several permils.
#[test]
fn refill_split_equals_single_over_many_shapes() {
    for &permil in &[1u32, 10, 33, 250, 1000] {
        for &(n, step) in &[(1000u32, 50u32), (100, 7), (10_000, 1), (3, 33333)] {
            let cap = EU_CAP_MS;
            let mut split = DutyGovernor::new(cap, permil);
            let mut single = DutyGovernor::new(cap, permil);
            assert!(split.try_consume(cap));
            assert!(single.try_consume(cap));
            for _ in 0..n {
                split.refill_ms(step);
            }
            single.refill_ms(n.saturating_mul(step));
            assert_eq!(
                split.budget_ms(),
                single.budget_ms(),
                "split refill mismatch: permil={permil} n={n} step={step}"
            );
        }
    }
}

/// The bucket never exceeds its cap regardless of how much wall-clock is credited.
#[test]
fn refill_never_exceeds_cap() {
    let mut g = DutyGovernor::new(FHSS_DWELL_BURST_MS, FHSS_PERMIL);
    // Full already; credit a huge span.
    g.refill_ms(u32::MAX);
    assert_eq!(g.budget_ms(), FHSS_DWELL_BURST_MS);
    // Consume a little, over-refill again, still clamped.
    assert!(g.try_consume(40));
    g.refill_ms(1_000_000);
    assert_eq!(g.budget_ms(), FHSS_DWELL_BURST_MS);
}

/// `try_consume` boundary: succeeds iff the request fits the current budget, and decrements
/// exactly. An exact-budget consume drains to zero; one more byte is refused.
#[test]
fn try_consume_boundary() {
    let mut g = DutyGovernor::new(100, 0); // permil 0 → never refills, easy to reason about
    assert!(g.try_consume(60));
    assert_eq!(g.budget_ms(), 40);
    assert!(!g.try_consume(41), "over-budget consume must fail");
    assert_eq!(g.budget_ms(), 40, "a refused consume must not change the budget");
    assert!(g.try_consume(40), "exact-budget consume must succeed");
    assert_eq!(g.budget_ms(), 0);
    assert!(!g.try_consume(1), "empty bucket refuses any TX");
    assert!(g.try_consume(0), "a zero-airtime consume always fits");
}

// --- hop_channel: perfect permutation + §15.247 fixed-spacing compliance ------------------

/// For many seeds, `hop_channel` over slots `0..N` is a perfect permutation of `0..N`:
/// every channel appears exactly once (⇒ equal use). This is the "by construction" claim in
/// the fhss.rs module docs.
#[test]
fn hop_channel_is_perfect_permutation() {
    for seed in 0u32..5000 {
        let mut seen = [false; FHSS_N];
        for i in 0..FHSS_N as u8 {
            let ch = hop_channel::<FHSS_N>(seed, 0, i);
            assert!((ch as usize) < FHSS_N, "channel {ch} out of range (seed {seed})");
            assert!(!seen[ch as usize], "channel {ch} repeated (seed {seed}) — not a permutation");
            seen[ch as usize] = true;
        }
        assert!(seen.iter().all(|&s| s), "not every channel visited (seed {seed})");
    }
}

/// The §15.247 compliance property the fixed-permutation fix guarantees: since the permutation
/// is cycle-invariant, a given channel occupies the **same slot offset every cycle**, so its
/// successive on-air visits are exactly `FHSS_N · FHSS_SLOT_MS` apart. That spacing must exceed
/// the 20 s occupancy window (so a channel is tuned at most once per window). We assert, over a
/// large seed sweep, that (a) the permutation is identical across cycles and (b) the resulting
/// inter-visit spacing is the full cycle and strictly greater than 20 000 ms.
#[test]
fn hop_channel_fixed_across_cycles_bounds_occupancy() {
    let cycle_ms = FHSS_N as u64 * FHSS_SLOT_MS;
    assert!(
        cycle_ms > 20_000,
        "cycle {cycle_ms} ms must exceed the 20 s window (compliance invariant)"
    );

    for seed in 0u32..8000 {
        // Build the reference (cycle 0) permutation and each channel's slot offset within it.
        let mut slot_of_channel = [usize::MAX; FHSS_N];
        for i in 0..FHSS_N as u8 {
            let ch = hop_channel::<FHSS_N>(seed, 0, i) as usize;
            slot_of_channel[ch] = i as usize;
        }

        // Check several later cycles land the same permutation → the same slot for each channel.
        for &cycle in &[1u32, 2, 7, 100, 4321] {
            for i in 0..FHSS_N as u8 {
                assert_eq!(
                    hop_channel::<FHSS_N>(seed, cycle, i),
                    hop_channel::<FHSS_N>(seed, 0, i),
                    "permutation changed across cycles (seed {seed}, slot {i}) — would break §15.247 spacing"
                );
            }
        }

        // Successive visits of any channel are exactly one cycle apart, > the 20 s window.
        for (ch, &slot) in slot_of_channel.iter().enumerate() {
            assert_ne!(slot, usize::MAX, "channel {ch} never scheduled (seed {seed})");
            // Absolute slot in cycle c is c·FHSS_N + slot; the on-air time is that ·FHSS_SLOT_MS.
            let t0 = slot as u64 * FHSS_SLOT_MS;
            let t1 = (FHSS_N + slot) as u64 * FHSS_SLOT_MS;
            let spacing = t1 - t0;
            assert_eq!(spacing, cycle_ms);
            assert!(
                spacing > FHSS_WINDOW_MS as u64,
                "channel {ch} visited twice inside the 20 s window (seed {seed})"
            );
        }
    }
}

// --- frame_toa_ms: never under-counts ----------------------------------------------------

/// Time-on-air is rounded **up** (div_ceil), so the reported airtime is always ≥ the exact
/// airtime — the regulatory budget is never under-counted. Verify across the full frame-length
/// range that `frame_toa_ms(len)` ≥ the exact ceiling of the on-air bit-time.
#[test]
fn frame_toa_never_undercounts() {
    for len in 0usize..=255 {
        let toa = frame_toa_ms(len) as u64;
        // True (fractional) airtime scaled by BITRATE, to compare without floating point:
        //   true_ms = bits · 1000 / BITRATE.  We assert (independently of the impl's div_ceil):
        //   (a) never under-counts:  toa · BITRATE ≥ bits · 1000
        //   (b) it is the *tight* ceiling, not over by a whole ms: (toa-1)·BITRATE < bits·1000
        let bits = (4 + 4 + 1 + len as u64 + 2) * 8;
        let true_x_bitrate = bits * 1000;
        assert!(
            toa * BITRATE as u64 >= true_x_bitrate,
            "toa under-counts airtime at len={len}"
        );
        assert!(
            toa == 0 || (toa - 1) * (BITRATE as u64) < true_x_bitrate,
            "toa over-counts by a whole ms at len={len} (not the tight ceiling)"
        );
    }
}

/// Monotonic: a longer frame never reports less airtime.
#[test]
fn frame_toa_monotonic() {
    let mut prev = 0;
    for len in 0usize..=255 {
        let toa = frame_toa_ms(len);
        assert!(toa >= prev, "toa decreased at len={len}");
        prev = toa;
    }
}
