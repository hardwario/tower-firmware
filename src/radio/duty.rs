//! EU duty-cycle governor (docs/radio.md).
//!
//! A token-bucket over **transmitted airtime**: the bucket holds up to `cap_ms`
//! of airtime and refills at `permil`‰ of wall-clock (10‰ = 1 %). A TX is allowed
//! only if its time-on-air fits the bucket; otherwise it's deferred/refused
//! (`DutyLimited`). This approximates a rolling-hour 1 % limit (cap = 1 % of an
//! hour = 36 000 ms) and counts **all** TX — data, ACK, bulk, retransmits, JOIN.
//! The gateway is governed too (regulatory, independent of mains power).


use embassy_time::Instant;

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
/// the HW-generated preamble (4) + sync (4) + length (1) + CRC (2) — §2.6. Rounded
/// up so the regulatory budget is never under-counted.
#[must_use]
pub fn frame_toa_ms(frame_len: usize) -> u32 {
    let bytes = 4 + 4 + 1 + frame_len as u32 + 2;
    (bytes * 8 * 1000).div_ceil(BITRATE)
}

/// Token-bucket duty governor.
pub struct DutyGovernor {
    budget_ms: u32,
    cap_ms: u32,
    permil: u32,
    last: Instant,
}

impl DutyGovernor {
    /// New governor, bucket full, with the given cap and duty (parts-per-thousand).
    pub fn new(cap_ms: u32, permil: u32) -> Self {
        Self {
            budget_ms: cap_ms,
            cap_ms,
            permil,
            last: Instant::now(),
        }
    }

    /// EU governor (1 %, 36 s cap).
    pub fn eu() -> Self {
        Self::new(EU_CAP_MS, EU_PERMIL)
    }

    /// US 915 governor: no EU-style duty limit (100 % refill). FCC 15.247 governs
    /// US operation by channel-dwell/PSD rather than a fixed duty cycle, so the
    /// bucket simply refills as fast as it drains. (A compliant product still owes
    /// FHSS/wideband — this is for bench testing; see [`Band`](super::config::Band).)
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

    /// Refill from real elapsed time, then try to consume `toa_ms` of airtime.
    /// Returns `false` (don't transmit) if the budget is insufficient.
    pub fn try_tx(&mut self, toa_ms: u32) -> bool {
        let now = Instant::now();
        let elapsed = now.saturating_duration_since(self.last).as_millis() as u32;
        self.last = now;
        self.refill_ms(elapsed);
        self.try_consume(toa_ms)
    }

    /// Remaining airtime budget (ms).
    pub fn budget_ms(&self) -> u32 {
        self.budget_ms
    }

    // --- testable core (no real-time dependency) ---

    /// Add the airtime earned over `elapsed_ms` of wall-clock (capped).
    pub fn refill_ms(&mut self, elapsed_ms: u32) {
        let refill = elapsed_ms.saturating_mul(self.permil) / 1000;
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
