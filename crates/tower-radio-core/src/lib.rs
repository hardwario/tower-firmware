//! Pure integer core of the TOWER radio timing/compliance math (docs/radio.md).
//!
//! Extracted from `src/radio/duty.rs` and `src/radio/net/fhss.rs` so the regulatory
//! arithmetic — the EU/US/FHSS duty token-bucket, frame time-on-air, and the fixed FHSS
//! hop permutation — can be **unit-tested on the host** (`cargo test`). The firmware itself
//! is `no_std` with a thumbv6m default target and has no libtest, exactly the reason
//! `crates/tower-kv` was split out; this follows that precedent.
//!
//! Everything here is `no_std` and free of any real-time dependency: the token bucket takes
//! `elapsed_ms` as an argument rather than reading a clock. The `embassy_time::Instant`-based
//! [`DutyGovernor::try_tx`] wrapper stays in `src/radio/duty.rs` and delegates to
//! [`DutyGovernor::refill_ms`] + [`DutyGovernor::try_consume`] here, so there is **zero**
//! behavioural change on the target and no external API change (the firmware re-exports these
//! from `src/radio`).

#![no_std]

pub mod ccm;

/// Nominal over-the-air bit rate.
pub const BITRATE: u32 = 19_200;
/// EU 1 % duty over a rolling hour → 36 000 ms of airtime budget.
pub const EU_CAP_MS: u32 = 36_000;
/// EU duty in parts-per-thousand (10‰ = 1 %).
pub const EU_PERMIL: u32 = 10;
/// FHSS per-channel airtime window (FCC §15.247 measures occupancy over 20 s).
pub const FHSS_WINDOW_MS: u32 = 20_000;
/// FHSS per-channel burst budget (token-bucket cap). With [`FHSS_PERMIL`], the
/// worst-case spend in any 20 s is `cap + permil·20 s = 100 + 200 = 300 ms`.
pub const FHSS_DWELL_BURST_MS: u32 = 100;
/// FHSS per-channel sustained refill: 1 % (200 ms / 20 s). See [`FHSS_DWELL_BURST_MS`].
pub const FHSS_PERMIL: u32 = 10;

/// Time-on-air (ms) of a frame whose FIFO payload is `frame_len` bytes, including
/// the HW-generated preamble (4) + sync (4) + length (1) + CRC (2) — docs/radio.md. Rounded
/// up so the regulatory budget is never under-counted.
#[must_use]
pub fn frame_toa_ms(frame_len: usize) -> u32 {
    let bytes = 4 + 4 + 1 + frame_len as u32 + 2;
    (bytes * 8 * 1000).div_ceil(BITRATE)
}

/// Token-bucket duty governor (pure core, no real-time dependency).
///
/// Refill is driven by [`refill_ms`](Self::refill_ms) (elapsed wall-clock passed in) rather
/// than reading a clock, so it is fully deterministic and host-testable. The firmware's
/// `Instant`-based `try_tx` lives in `src/radio/duty.rs` and delegates here.
pub struct DutyGovernor {
    budget_ms: u32,
    cap_ms: u32,
    permil: u32,
    /// Sub-millisecond refill carry, in *permil-ms* (< 1000). Without it, `refill_ms` truncates
    /// `elapsed·permil/1000` to 0 for any call spaced < `1000/permil` ms apart (100 ms at 1 %)
    /// AND discards that elapsed time, so back-to-back `try_tx` calls refill nothing and starve
    /// TX once the bucket drains. Carrying the remainder makes many short intervals sum to the
    /// same refill as one long interval.
    residue: u32,
}

impl DutyGovernor {
    /// New governor, bucket full, with the given cap and duty (parts-per-thousand).
    pub fn new(cap_ms: u32, permil: u32) -> Self {
        Self {
            budget_ms: cap_ms,
            cap_ms,
            permil,
            residue: 0,
        }
    }

    /// EU governor (1 %, 36 s cap).
    pub fn eu() -> Self {
        Self::new(EU_CAP_MS, EU_PERMIL)
    }

    /// US 915 governor: no EU-style duty limit (100 % refill). FCC 15.247 governs
    /// US operation by channel-dwell/PSD rather than a fixed duty cycle, so the
    /// bucket simply refills as fast as it drains. (A compliant product still owes
    /// FHSS/wideband — this is for bench testing; see `Band`.)
    pub fn us915() -> Self {
        Self::new(EU_CAP_MS, 1000)
    }

    /// FHSS **per-channel** dwell governor (§15.247): caps *transmitted* airtime on
    /// one hop channel to ≤ [`FHSS_DWELL_BURST_MS`] + 1 %·20 s = **300 ms in any
    /// 20 s** (a token bucket can spend at most `cap + permil·window`), which is 25 %
    /// under the 0.4 s/20 s limit. One bucket per channel; the FHSS layer replaces
    /// the band duty governor with these while hopping.
    pub fn fhss_channel() -> Self {
        Self::new(FHSS_DWELL_BURST_MS, FHSS_PERMIL)
    }

    /// Remaining airtime budget (ms).
    #[must_use]
    pub fn budget_ms(&self) -> u32 {
        self.budget_ms
    }

    /// Add the airtime earned over `elapsed_ms` of wall-clock (capped). Accumulates the
    /// sub-millisecond remainder in [`residue`](Self::residue) so repeated short intervals sum
    /// exactly to the true refill instead of each truncating to zero (see the field docs).
    pub fn refill_ms(&mut self, elapsed_ms: u32) {
        let total = elapsed_ms
            .saturating_mul(self.permil)
            .saturating_add(self.residue);
        let refill = total / 1000;
        self.residue = total % 1000;
        self.budget_ms = self.budget_ms.saturating_add(refill).min(self.cap_ms);
    }

    /// Consume `toa_ms` if available; returns whether it was allowed.
    pub fn try_consume(&mut self, toa_ms: u32) -> bool {
        if self.budget_ms >= toa_ms {
            self.budget_ms -= toa_ms;
            true
        } else {
            false
        }
    }
}

/// Channel for slot index `i` (0..`N`) of `cycle` under `seed`: a seeded Fisher-Yates
/// shuffle of `0..N` → a perfect permutation (every channel exactly once ⇒ equal use, by
/// construction), deterministic at both ends. `N` is the FHSS channel count (the firmware
/// instantiates it with `config::FHSS_N` = 80).
///
/// The permutation is **fixed across cycles** — it is seeded from `seed` alone and does
/// **not** mix in `cycle`. This is a §15.247 compliance requirement, not a stylistic
/// choice: a fresh per-cycle shuffle gives *no* minimum spacing across a cycle boundary,
/// so a channel could fall in the last slot of one cycle and the first slot of the next
/// (~300 ms apart) and thus be occupied twice inside a single 20 s averaging window,
/// exceeding the 0.4 s/20 s per-channel limit. With one fixed permutation every channel
/// recurs at the *same* slot offset each cycle — exactly one cycle
/// (`N · FHSS_SLOT_MS` = 24 s > 20 s) apart — so it is tuned at most once per window
/// by construction. `cycle` is kept in the signature (both ends still pass it) for
/// call-site symmetry with the slot decomposition and to leave room for a future
/// schedule that re-keys per cycle *without* reintroducing the adjacency hazard.
#[must_use]
pub fn hop_channel<const N: usize>(seed: u32, cycle: u32, i: u8) -> u8 {
    let _ = cycle; // intentionally unused: the permutation must be cycle-invariant (see above)
    let mut perm = [0u8; N];
    let mut k = 0usize;
    while k < N {
        perm[k] = k as u8;
        k += 1;
    }
    // xorshift32 seeded from `seed` ONLY (not the cycle) — a single fixed permutation, so a
    // channel's successive visits are exactly one 24 s cycle apart (> the 20 s window).
    let mut x = seed;
    if x == 0 {
        x = 0xA5A5_A5A5;
    }
    let mut j = N - 1;
    while j >= 1 {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        let r = (x as usize) % (j + 1);
        perm.swap(j, r);
        j -= 1;
    }
    perm[i as usize]
}

#[cfg(test)]
mod tests;
