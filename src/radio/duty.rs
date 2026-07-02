//! EU duty-cycle governor (docs/radio.md).
//!
//! A token-bucket over **transmitted airtime**: the bucket holds up to `cap_ms`
//! of airtime and refills at `permil`‰ of wall-clock (10‰ = 1 %). A TX is allowed
//! only if its time-on-air fits the bucket; otherwise it's deferred/refused
//! (`DutyLimited`). This approximates a rolling-hour 1 % limit (cap = 1 % of an
//! hour = 36 000 ms) and counts **all** TX — data, ACK, bulk, retransmits, JOIN.
//! The gateway is governed too (regulatory, independent of mains power).
//!
//! The **pure integer** math (refill/consume with the sub-ms residue carry, the frame
//! time-on-air, and the EU/US/FHSS constants) lives in the host-testable
//! [`tower_radio_core`] leaf crate; this module keeps only the thin `embassy_time::Instant`
//! real-time wrapper ([`DutyGovernor::try_tx`]) on top of it. Splitting the arithmetic out
//! lets `just test` unit-test the regulatory budget on the host (the firmware itself is
//! no_std and can't `cargo test`) — with **zero** behavioural change on the target.

use embassy_time::Instant;

// Re-export the pure core so the rest of the firmware keeps saying `duty::frame_toa_ms`,
// `duty::BITRATE`, etc. — no external API change from the extraction.
pub use tower_radio_core::{
    BITRATE, EU_CAP_MS, EU_PERMIL, FHSS_DWELL_BURST_MS, FHSS_PERMIL, FHSS_WINDOW_MS, frame_toa_ms,
};

/// Token-bucket duty governor: the host-testable [`tower_radio_core::DutyGovernor`] core plus a
/// `last`-[`Instant`] so [`try_tx`](Self::try_tx) can refill from real elapsed time. All the
/// integer accounting (cap, permil, residue carry) is in the core; this only measures wall-clock.
pub struct DutyGovernor {
    core: tower_radio_core::DutyGovernor,
    last: Instant,
}

impl DutyGovernor {
    /// New governor, bucket full, with the given cap and duty (parts-per-thousand).
    pub fn new(cap_ms: u32, permil: u32) -> Self {
        Self {
            core: tower_radio_core::DutyGovernor::new(cap_ms, permil),
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
        self.core.refill_ms(elapsed);
        self.core.try_consume(toa_ms)
    }

    /// Remaining airtime budget (ms).
    #[must_use]
    pub fn budget_ms(&self) -> u32 {
        self.core.budget_ms()
    }

    /// Add the airtime earned over `elapsed_ms` of wall-clock (capped) — the pure-core refill,
    /// exposed for callers that meter their own elapsed time. See the field docs on the core's
    /// residue carry (many short intervals sum exactly to one long interval).
    pub fn refill_ms(&mut self, elapsed_ms: u32) {
        self.core.refill_ms(elapsed_ms);
    }

    /// Consume `toa_ms` if available; returns whether it was allowed.
    pub fn try_consume(&mut self, toa_ms: u32) -> bool {
        self.core.try_consume(toa_ms)
    }
}
