//! US 915 **FHSS** (FCC §15.247 frequency hopping). `impl Net` block over
//! [`super::Net`].
//!
//! Star topology, gateway = hop time-master. The gateway runs a free-running hop
//! clock (an [`Instant`] epoch) and, each 300 ms slot, retunes to that slot's
//! pseudo-random channel and broadcasts a **Beacon** (slot index), then listens.
//! A node blind-rendezvous: it parks on a fixed channel ([`FHSS_RENDEZVOUS_CH`]),
//! and since the per-cycle permutation visits every channel exactly once, the
//! gateway sweeps onto the rendezvous channel within ≤ 1 cycle — guaranteed. From
//! the beacon's slot index + arrival time the node reconstructs the gateway's
//! epoch, then hops in lockstep, re-aligning on each beacon.
//!
//! **Compliance (sized for margin, not the limit):** 80 channels × 300 ms slot →
//! cycle 24 s > 20 s, so any channel is *tuned* at most once per 20 s window =
//! ≤ 0.3 s (25 % under the 0.4 s/20 s limit, strict reading). A per-channel dwell
//! governor independently caps *transmitted* airtime at ≤ 300 ms/20 s. GUARD = 10 ms
//! (measured retune+lock was 762 µs; floor dominates) ≫ the ~30 µs/slot clock drift.
//! (Exact §15.247 numbers are FCC-KDB config to **verify** before any product claim.)

use embassy_time::{Duration, Instant, Timer};

use super::{Access, Net, Received, SendResult, TX_TIMEOUT};
use crate::radio::ccm::TAG_LEN;
use crate::radio::config::{self, Band, FHSS_N, FHSS_RENDEZVOUS_CH, fhss_freq_hz};
use crate::radio::device::RadioError;
use crate::radio::duty::{self, DutyGovernor};
use crate::radio::frame::{self, FrameType, HDR_LEN, Header, MAX_FRAME, MAX_PAYLOAD};

/// Slot length (ms). Cycle = `FHSS_N · FHSS_SLOT_MS`; must exceed 20 s so each
/// channel is tuned at most once per compliance window (see module docs).
const FHSS_SLOT_MS: u64 = 300;
/// Per-slot guard (retune lead + clock-skew margin); measured retune ≪ this.
const GUARD: Duration = Duration::from_millis(10);
/// Node RX window for the per-slot beacon (covers beacon ToA + jitter).
const BEACON_RX_MS: u64 = 60;
/// Node park-and-listen window while scanning (> one slot so it can't miss the
/// gateway's pass over the rendezvous channel).
const RENDEZVOUS_RX_MS: u64 = 350;
/// Consecutive missed beacons before declaring loss of sync → re-scan.
const MISS_LIMIT: u32 = 4;
/// Beacon payload: cycle(4 LE) ‖ slot_index(1).
const BEACON_PL_LEN: usize = 5;
/// Broadcast destination (beacons are network-wide).
const BROADCAST: u32 = 0xFFFF_FFFF;

// The hop cycle must exceed the 20 s compliance window (strict-occupancy margin).
const _: () = assert!(FHSS_N as u64 * FHSS_SLOT_MS > 20_000);

/// Which side of the FHSS link this device plays.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FhssRole {
    /// Free-running hop clock + per-slot beacon ([`Net::fhss_master_tick`]).
    Master,
    /// Blind-rendezvous follower ([`Net::fhss_node_tick`]).
    Node,
}

/// Node synchronization state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FhssState {
    /// Parked on the rendezvous channel, waiting for a beacon (also the Lost state).
    Scanning,
    /// Locked to the gateway's hop clock; tracking + re-aligning each beacon.
    Synced,
}

/// FHSS configuration.
#[derive(Debug, Clone, Copy, Default)]
pub struct FhssConfig {
    /// Hop-sequence seed shared by both ends. `0` (default) ⇒ derive from the link key.
    pub seed: u32,
}

/// FHSS runtime state, held on [`Net`] (inert unless `access == Fhss`).
pub(crate) struct Fhss {
    role: FhssRole,
    seed: u32,
    state: FhssState,
    /// Slot-clock anchor `(slot, boundary_instant)`: slot `anchor.0` started at
    /// `anchor.1` on *this device's* clock. Re-anchored each beacon (node) or set
    /// once at enable (master, anchor = `(0, epoch)`). Using an anchor of small
    /// time-deltas — rather than an absolute epoch reconstructed by subtracting
    /// `slot·SLOT` — avoids `Instant` underflow when the gateway's slot count far
    /// exceeds the node's uptime.
    anchor: Option<(u32, Instant)>,
    last_beacon: Instant,
    miss: u32,
    cur_channel: u8,
    /// Per-channel transmitted-airtime governor (§15.247 dwell).
    dwell: [DutyGovernor; FHSS_N as usize],
}

impl Fhss {
    pub(crate) fn disabled() -> Self {
        Self {
            role: FhssRole::Node,
            seed: 0,
            state: FhssState::Scanning,
            anchor: None,
            last_beacon: Instant::now(),
            miss: 0,
            cur_channel: FHSS_RENDEZVOUS_CH,
            dwell: core::array::from_fn(|_| DutyGovernor::fhss_channel()),
        }
    }
}

/// One master slot result: the channel beaconed on and any uplink caught + ACKed.
pub struct MasterSlot {
    pub channel: u8,
    pub slot: u32,
    pub received: Option<Received>,
}

/// One node slot result: current sync state, channel, slot index, and whether the
/// slot's beacon was heard (a re-alignment).
pub struct NodeSlot {
    pub state: FhssState,
    pub channel: u8,
    pub slot: u32,
    pub got_beacon: bool,
}

/// Channel for slot index `i` (0..[`FHSS_N`)) of `cycle` under `seed`: a seeded
/// Fisher-Yates shuffle of `0..FHSS_N` → a perfect permutation each cycle (every
/// channel exactly once ⇒ equal use, by construction), deterministic at both ends.
#[must_use]
pub fn hop_channel(seed: u32, cycle: u32, i: u8) -> u8 {
    let mut perm = [0u8; FHSS_N as usize];
    let mut k = 0usize;
    while k < FHSS_N as usize {
        perm[k] = k as u8;
        k += 1;
    }
    // xorshift32 seeded from (seed, cycle) — same idiom as net::backoff_ms.
    let mut x = seed ^ cycle.wrapping_mul(0x9E37_79B9);
    if x == 0 {
        x = 0xA5A5_A5A5;
    }
    let mut j = FHSS_N as usize - 1;
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

/// Beacon time-on-air (ms): a sealed beacon is HDR_LEN + payload + tag bytes.
fn beacon_toa_ms() -> u64 {
    duty::frame_toa_ms(HDR_LEN + BEACON_PL_LEN + TAG_LEN) as u64
}

impl Net {
    /// Enable US 915 FHSS at runtime (mutually exclusive with other access modes).
    /// `Master` starts the free-running hop clock now; `Node` starts Scanning. The
    /// seed defaults to a key-derived value so both ends agree without sending it.
    pub async fn enable_fhss(&mut self, role: FhssRole, cfg: FhssConfig) -> Result<(), RadioError> {
        let seed = if cfg.seed != 0 {
            cfg.seed
        } else {
            u32::from_le_bytes([self.default_key[0], self.default_key[1], self.default_key[2], self.default_key[3]]) | 1
        };
        self.fhss.role = role;
        self.fhss.seed = seed;
        self.fhss.miss = 0;
        self.fhss.dwell = core::array::from_fn(|_| DutyGovernor::fhss_channel());
        self.access = Access::Fhss;
        match role {
            FhssRole::Master => {
                self.fhss.state = FhssState::Synced;
                self.fhss.anchor = Some((0, Instant::now()));
            }
            FhssRole::Node => {
                self.fhss.state = FhssState::Scanning;
                self.fhss.anchor = None;
                self.fhss.cur_channel = FHSS_RENDEZVOUS_CH;
            }
        }
        Ok(())
    }

    /// Leave FHSS → plain duty mode. The caller selects the next band via `set_band`.
    pub async fn disable_fhss(&mut self) -> Result<(), RadioError> {
        self.access = Access::Duty;
        config::set_band(&mut self.radio, Band::Eu868, 0).await
    }

    /// Current FHSS sync state (diagnostics).
    #[must_use]
    pub fn fhss_state(&self) -> FhssState {
        self.fhss.state
    }

    /// Current FHSS channel index (diagnostics).
    #[must_use]
    pub fn fhss_current_channel(&self) -> u8 {
        self.fhss.cur_channel
    }

    /// Transmitted airtime consumed on channel `ch` in the rolling window (ms),
    /// for the compliance histogram (`cap − remaining budget`).
    #[must_use]
    pub fn fhss_channel_airtime_ms(&self, ch: u8) -> u32 {
        let g = &self.fhss.dwell[ch as usize];
        duty::FHSS_DWELL_BURST_MS.saturating_sub(g.budget_ms())
    }

    /// Channel for slot `slot_abs` under `seed`.
    fn fhss_channel_for(seed: u32, slot_abs: u32) -> u8 {
        hop_channel(seed, slot_abs / FHSS_N as u32, (slot_abs % FHSS_N as u32) as u8)
    }

    /// Current absolute slot index from the anchor `(slot, time)` at `now`
    /// (`anchor.slot + elapsed_since_anchor / SLOT`). Small deltas only.
    fn fhss_cur_slot(anchor: (u32, Instant), now: Instant) -> u32 {
        let elapsed_ms = now.saturating_duration_since(anchor.1).as_millis();
        anchor.0 + (elapsed_ms / FHSS_SLOT_MS) as u32
    }

    /// Boundary instant of slot `slot_abs` (≥ `anchor.slot`) from the anchor.
    fn fhss_slot_start(anchor: (u32, Instant), slot_abs: u32) -> Instant {
        anchor.1 + Duration::from_millis((slot_abs - anchor.0) as u64 * FHSS_SLOT_MS)
    }

    /// **Master:** run one slot — retune to the next slot's channel during the
    /// guard, beacon exactly at the slot boundary, then listen the rest of the slot
    /// for an uplink (decoded + auto-ACKed via [`recv`](Self::recv)). Call in a loop.
    pub async fn fhss_master_tick(&mut self) -> MasterSlot {
        let seed = self.fhss.seed;
        let anchor = self.fhss.anchor.expect("master anchor");
        let now = Instant::now();
        let slot = Self::fhss_cur_slot(anchor, now) + 1; // beacon the upcoming slot at its boundary
        let ch = Self::fhss_channel_for(seed, slot);
        let t_start = Self::fhss_slot_start(anchor, slot);

        // Retune during the guard, then wait for the exact boundary.
        let _ = config::set_freq_hz(&mut self.radio, fhss_freq_hz(ch)).await;
        self.fhss.cur_channel = ch;
        Timer::at(t_start).await;

        self.fhss_tx_beacon(slot, ch).await;

        // Listen for the rest of the slot's active window for an uplink.
        let listen_until = t_start + Duration::from_millis(FHSS_SLOT_MS) - GUARD;
        let now = Instant::now();
        let received = if listen_until > now {
            self.recv(listen_until.saturating_duration_since(now)).await
        } else {
            None
        };
        MasterSlot { channel: ch, slot, received }
    }

    /// **Node:** run one slot — when Scanning, park on the rendezvous channel and
    /// listen for a beacon (locking on receipt); when Synced, retune to the slot's
    /// channel, wait for the boundary, and catch the beacon to re-align. After this
    /// returns Synced with `got_beacon`, the gateway is listening on `channel` for
    /// the rest of the slot, so the caller may [`fhss_send`](Self::fhss_send) now.
    pub async fn fhss_node_tick(&mut self) -> NodeSlot {
        let seed = self.fhss.seed;
        match self.fhss.state {
            FhssState::Scanning => {
                let _ = config::set_freq_hz(&mut self.radio, fhss_freq_hz(FHSS_RENDEZVOUS_CH)).await;
                self.fhss.cur_channel = FHSS_RENDEZVOUS_CH;
                let got = self.fhss_rx_beacon(Duration::from_millis(RENDEZVOUS_RX_MS)).await;
                if let Some((slot_abs, t_rx)) = got {
                    self.fhss_lock(slot_abs, t_rx);
                    NodeSlot { state: FhssState::Synced, channel: FHSS_RENDEZVOUS_CH, slot: slot_abs, got_beacon: true }
                } else {
                    NodeSlot { state: FhssState::Scanning, channel: FHSS_RENDEZVOUS_CH, slot: 0, got_beacon: false }
                }
            }
            FhssState::Synced => {
                let anchor = self.fhss.anchor.expect("node anchor");
                let now = Instant::now();
                let slot = Self::fhss_cur_slot(anchor, now) + 1;
                let ch = Self::fhss_channel_for(seed, slot);
                let t_start = Self::fhss_slot_start(anchor, slot);

                let _ = config::set_freq_hz(&mut self.radio, fhss_freq_hz(ch)).await;
                self.fhss.cur_channel = ch;
                Timer::at(t_start).await;

                let got = self.fhss_rx_beacon(Duration::from_millis(BEACON_RX_MS)).await;
                if let Some((slot_abs, t_rx)) = got {
                    self.fhss_lock(slot_abs, t_rx);
                    NodeSlot { state: FhssState::Synced, channel: ch, slot, got_beacon: true }
                } else {
                    self.fhss.miss += 1;
                    if self.fhss.miss >= MISS_LIMIT {
                        self.fhss.state = FhssState::Scanning;
                        self.fhss.anchor = None;
                    }
                    NodeSlot { state: self.fhss.state, channel: ch, slot, got_beacon: false }
                }
            }
        }
    }

    /// **Node:** send one frame on the *current* slot's channel (call right after a
    /// Synced [`fhss_node_tick`](Self::fhss_node_tick) — the gateway is listening on
    /// that channel). Dwell-metered; refuses if the slot has too little time left for
    /// the exchange (slot-straddle rule) or if not synced. One TX counter is consumed.
    pub async fn fhss_send(&mut self, dest: u32, data: &[u8], confirmed: bool) -> SendResult {
        if self.access != Access::Fhss {
            return SendResult::WrongMode;
        }
        if self.fhss.state != FhssState::Synced {
            return SendResult::NotSynced;
        }
        if data.len() > MAX_PAYLOAD {
            return SendResult::Error(RadioError::TooLong);
        }
        let anchor = match self.fhss.anchor {
            Some(a) => a,
            None => return SendResult::NotSynced,
        };
        let my_id = self.my_id;
        let counter = self.tx_counter;
        let key = self.key_for(dest);
        let hdr = Header {
            frame_type: FrameType::Data,
            flags: if confirmed { frame::flags::CONFIRMED } else { 0 },
            src: my_id,
            dest,
            counter,
            bulk_index: None,
        };
        let mut buf = [0u8; MAX_FRAME];
        let n = match frame::seal_frame(&mut self.ccm, &key, &hdr, data, &mut buf) {
            Ok(n) => n,
            Err(_) => return SendResult::Error(RadioError::TooLong),
        };

        // Slot-straddle rule: only transmit if data (+ACK turnaround +ACK) + GUARD
        // finishes before this slot's active window ends.
        let now = Instant::now();
        let cur = Self::fhss_cur_slot(anchor, now);
        let ch = Self::fhss_channel_for(self.fhss.seed, cur);
        let slot_end = Self::fhss_slot_start(anchor, cur + 1);
        let room = slot_end.saturating_duration_since(now);
        let data_toa = duty::frame_toa_ms(n) as u64;
        let need = if confirmed { data_toa + 20 + beacon_toa_ms() } else { data_toa } + GUARD.as_millis();
        if room.as_millis() < need {
            return SendResult::DutyLimited; // no room this slot; caller retries next slot
        }

        let _ = config::set_freq_hz(&mut self.radio, fhss_freq_hz(ch)).await;
        self.fhss.cur_channel = ch;
        if !self.fhss.dwell[ch as usize].try_tx(duty::frame_toa_ms(n)) {
            self.advance_tx_counter();
            return SendResult::DutyLimited;
        }
        let result = match self.radio.tx(&buf[..n], false, TX_TIMEOUT).await {
            // Unconfirmed → delivered once sent; confirmed → wait for the ACK.
            Ok(()) if !confirmed || self.await_ack(dest, counter).await => SendResult::Delivered,
            Ok(()) => SendResult::NotDelivered,
            Err(e) => SendResult::Error(e),
        };
        self.advance_tx_counter();
        result
    }

    /// Lock/re-align the node clock from a beacon for `slot_abs` received at `t_rx`:
    /// the beacon was sent at that slot's boundary, so the boundary instant on the
    /// node's clock is `t_rx − beacon_toa`. Anchor at `(slot_abs, boundary)` — small
    /// subtraction only, never `slot·SLOT` (which would underflow). Resets misses.
    fn fhss_lock(&mut self, slot_abs: u32, t_rx: Instant) {
        let boundary = t_rx
            .checked_sub(Duration::from_millis(beacon_toa_ms()))
            .unwrap_or(t_rx);
        self.fhss.anchor = Some((slot_abs, boundary));
        self.fhss.state = FhssState::Synced;
        self.fhss.last_beacon = t_rx;
        self.fhss.miss = 0;
    }

    /// Build + transmit the beacon for `slot_abs` (already tuned to `ch`), sealed
    /// under the network (default) key so every node can sync; dwell-metered.
    async fn fhss_tx_beacon(&mut self, slot_abs: u32, ch: u8) {
        let cycle = slot_abs / FHSS_N as u32;
        let i = (slot_abs % FHSS_N as u32) as u8;
        let mut pl = [0u8; BEACON_PL_LEN];
        pl[..4].copy_from_slice(&cycle.to_le_bytes());
        pl[4] = i;
        let hdr = Header {
            frame_type: FrameType::Beacon,
            flags: 0,
            src: self.my_id,
            dest: BROADCAST,
            counter: self.tx_counter,
            bulk_index: None,
        };
        let mut buf = [0u8; MAX_FRAME];
        let key = self.default_key;
        if let Ok(n) = frame::seal_frame(&mut self.ccm, &key, &hdr, &pl, &mut buf) {
            self.advance_tx_counter();
            if self.fhss.dwell[ch as usize].try_tx(duty::frame_toa_ms(n)) {
                let _ = self.radio.tx(&buf[..n], false, TX_TIMEOUT).await;
            }
        }
    }

    /// Receive + authenticate a beacon under the network key (broadcast dest OK;
    /// **no** replay-advance — a beacon is an idempotent time signal, so a rebooted
    /// gateway can't lock the node out). Returns `(slot_abs, t_rx)`.
    async fn fhss_rx_beacon(&mut self, timeout: Duration) -> Option<(u32, Instant)> {
        let mut buf = [0u8; MAX_FRAME];
        let (len, _) = self.radio.rx(&mut buf, timeout).await.ok()?;
        let t_rx = Instant::now();
        let (peek, _) = frame::Header::parse(&buf[..len]).ok()?;
        if peek.frame_type != FrameType::Beacon {
            return None;
        }
        let key = self.default_key;
        let (_, range) = frame::open_frame(&mut self.ccm, &key, &mut buf[..len]).ok()?;
        let pl = &buf[range];
        if pl.len() < BEACON_PL_LEN {
            return None;
        }
        let cycle = u32::from_le_bytes([pl[0], pl[1], pl[2], pl[3]]);
        let i = pl[4];
        let slot_abs = cycle * FHSS_N as u32 + i as u32;
        Some((slot_abs, t_rx))
    }
}
