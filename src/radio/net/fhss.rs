//! US 915 **FHSS** (FCC §15.247 frequency hopping). `impl Net` block over
//! [`super::Net`].
//!
//! Star topology, gateway = hop time-master. The gateway runs a free-running hop
//! clock (an [`Instant`] epoch) and, each 300 ms slot, retunes to that slot's
//! pseudo-random channel and broadcasts a **Beacon** (slot index), then listens.
//! A node blind-rendezvous: it parks on a fixed channel ([`FHSS_RENDEZVOUS_CH`]),
//! and since the permutation visits every channel exactly once per cycle, the
//! gateway sweeps onto the rendezvous channel within ≤ 1 cycle — guaranteed. From
//! the beacon's slot index + arrival time the node reconstructs the gateway's
//! epoch, then hops in lockstep, re-aligning on each beacon.
//!
//! **Compliance (sized for margin, not the limit):** 80 channels × 300 ms slot →
//! cycle 24 s > 20 s. The hop permutation is **fixed across cycles** (see
//! [`hop_channel`]), so a channel recurs at the *same* slot offset every cycle and its
//! successive visits are exactly 24 s apart — strictly more than the 20 s averaging
//! window — so it is *tuned* at most once per window = ≤ 0.3 s (25 % under the
//! 0.4 s/20 s limit, strict reading). This spacing is the load-bearing invariant: a
//! *per-cycle-reshuffled* schedule would NOT bound cross-boundary spacing and could
//! double a channel's occupancy inside one window (the defect this design fixes).
//! Occupancy is thus bounded *by the hop schedule*, not a governor; per-channel airtime
//! is still recorded for the compliance histogram / diagnostics. GUARD = 10 ms
//! (measured retune+lock was 762 µs; floor dominates) ≫ the ~30 µs/slot clock drift.
//! (Exact §15.247 numbers are FCC-KDB config to **verify** before any product claim.)

use embassy_time::{Duration, Instant, Timer};
// The beacon-epoch acceptance machine (reject-backwards epochs, the two-beacon acquisition
// consistency guard, prediction-slack re-anchoring, the miss/rescan rule) lives in the
// host-testable `tower_net_core::epoch` kernel, together with its tuning constants
// (`ACQUIRE_SLOT_SLACK`, `FHSS_RESYNC_SLACK`, `REACQUIRE_LIMIT`). This module keeps the radio,
// the `Instant`-based slot clock and the EEPROM epoch persistence, and delegates each
// accept/reject/lock decision there. Zero behavioural change on target.
use tower_net_core::epoch::{EpochGate, MissVerdict, ScanVerdict, TrackVerdict, next_master_epoch};

use super::{Access, KEY_FHSS_EPOCH, NS_NET, Net, Received, SendResult, TX_TIMEOUT, read_u32};
use crate::radio::ccm::TAG_LEN;
use crate::radio::config::{self, Band, FHSS_N, FHSS_RENDEZVOUS_CH, fhss_freq_hz};
use crate::radio::device::RadioError;
use crate::radio::duty;
use crate::radio::frame::{self, FrameType, HDR_LEN, Header, MAX_FRAME, MAX_PAYLOAD};

/// Slot length (ms). Cycle = `FHSS_N · FHSS_SLOT_MS`; must exceed 20 s so each
/// channel is tuned at most once per compliance window (see module docs).
const FHSS_SLOT_MS: u64 = 300;
/// Per-slot guard (retune lead + clock-skew margin); measured retune ≪ this. Also
/// the lead by which the node opens its beacon RX *before* the slot boundary, so RX
/// is armed before the gateway transmits (covers `rx()` setup latency — without it a
/// late-armed RX can miss the beacon preamble and drop sync).
const GUARD: Duration = Duration::from_millis(10);
/// Node RX window for the per-slot beacon: opened `GUARD` early, so it spans
/// `[boundary − GUARD, boundary − GUARD + BEACON_RX_MS]`, generously covering the
/// beacon ToA (~16 ms) + retune/clock jitter (drift ≪ GUARD even across MISS_LIMIT).
const BEACON_RX_MS: u64 = 100;
/// Node park-and-listen window while scanning (> one slot so it can't miss the
/// gateway's pass over the rendezvous channel).
const RENDEZVOUS_RX_MS: u64 = 350;
// `REACQUIRE_LIMIT` (misses ridden through before rescanning), `ACQUIRE_SLOT_SLACK` (the
// two-beacon acquisition consistency slack) and `FHSS_RESYNC_SLACK` (the tracked-beacon
// prediction slack) moved, docs and all, to `tower_net_core::epoch` with the decisions they tune.
/// Beacon payload: epoch(4 LE) ‖ cycle(4 LE) ‖ slot_index(1). The leading `epoch` is the
/// gateway's monotonic boot-id (see [`crate::radio::net`]'s `KEY_FHSS_EPOCH`): a node refuses to
/// anchor to an epoch older than the highest it has seen, so a captured beacon replayed from a
/// previous gateway session can't re-sync it.
const BEACON_PL_LEN: usize = 9;
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
    cur_channel: u8,
    /// The epoch/freshness decision machine (`tower_net_core::epoch`): the session epoch
    /// (master boot-id / node highest-seen), the miss counter, and the `(epoch, slot)` of the
    /// held acquisition candidate.
    gate: EpochGate,
    /// Arrival time of the gate's held acquisition candidate (the kernel is time-free, so the
    /// `Instant` stays here; elapsed slots are measured against it and passed in). `Some` iff
    /// the gate holds a candidate.
    acquire_t0: Option<Instant>,
    /// Per-channel transmitted airtime (ms) accumulated this session — a light
    /// *measurement* for the §15.247 compliance histogram. Occupancy itself is
    /// bounded *structurally* (N=80, cycle 24 s > 20 s ⇒ each channel tuned ≤ once
    /// per 20 s ⇒ ≤ one 300 ms slot), so no per-channel enforcement governor is
    /// needed. (A `[DutyGovernor; 80]` here cost 1.6 KB and overflowed the L0
    /// during deep async poll — this `[u16; 80]` is 160 B.)
    airtime_ms: [u16; FHSS_N as usize],
}

impl Fhss {
    pub(crate) fn disabled() -> Self {
        Self {
            role: FhssRole::Node,
            seed: 0,
            state: FhssState::Scanning,
            anchor: None,
            last_beacon: Instant::now(),
            cur_channel: FHSS_RENDEZVOUS_CH,
            gate: EpochGate::new(),
            acquire_t0: None,
            airtime_ms: [0; FHSS_N as usize],
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
/// Fisher-Yates shuffle of `0..FHSS_N` → a perfect permutation (every channel exactly
/// once ⇒ equal use, by construction), deterministic at both ends.
///
/// The permutation is **fixed across cycles** — it is seeded from `seed` alone and does
/// **not** mix in `cycle`. This is a §15.247 compliance requirement, not a stylistic
/// choice: a fresh per-cycle shuffle gives *no* minimum spacing across a cycle boundary,
/// so a channel could fall in the last slot of one cycle and the first slot of the next
/// (~300 ms apart) and thus be occupied twice inside a single 20 s averaging window,
/// exceeding the 0.4 s/20 s per-channel limit. With one fixed permutation every channel
/// recurs at the *same* slot offset each cycle — exactly one cycle
/// (`FHSS_N · FHSS_SLOT_MS` = 24 s > 20 s) apart — so it is tuned at most once per window
/// by construction. `cycle` is kept in the signature (both ends still pass it) for
/// call-site symmetry with the slot decomposition and to leave room for a future
/// schedule that re-keys per cycle *without* reintroducing the adjacency hazard.
#[must_use]
pub fn hop_channel(seed: u32, cycle: u32, i: u8) -> u8 {
    // The permutation math lives in the host-testable `tower_radio_core` leaf crate (generic
    // over the channel count), where the perfect-permutation + fixed-cycle-spacing §15.247
    // properties are unit-tested; here we just bind it to this radio's `FHSS_N`. No behavioural
    // change — the algorithm (seeded xorshift32 Fisher-Yates, cycle-invariant) is identical.
    tower_radio_core::hop_channel::<{ FHSS_N as usize }>(seed, cycle, i)
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
            u32::from_le_bytes([
                self.default_key[0],
                self.default_key[1],
                self.default_key[2],
                self.default_key[3],
            ]) | 1
        };
        self.fhss.role = role;
        self.fhss.seed = seed;
        self.fhss.acquire_t0 = None;
        self.fhss.airtime_ms = [0; FHSS_N as usize];
        self.access = Access::Fhss;
        match role {
            FhssRole::Master => {
                // Bump + persist the monotonic epoch so this session's beacons are strictly newer
                // than any previous session's — nodes reject the older ones as replays. Best
                // effort: if the write fails we still advance in RAM (a reboot might then reuse the
                // epoch, only weakening replay protection, never breaking sync). The bump rule is
                // the kernel's (`next_master_epoch`); the EEPROM read/write is ours.
                let nkv = self.kv.scope(NS_NET);
                let next = next_master_epoch(read_u32(nkv, KEY_FHSS_EPOCH));
                let _ = nkv.set_bytes(KEY_FHSS_EPOCH, &next.to_le_bytes());
                self.fhss.gate.reset(next);
                self.fhss.state = FhssState::Synced;
                self.fhss.anchor = Some((0, Instant::now()));
            }
            FhssRole::Node => {
                self.fhss.gate.reset(0); // no gateway epoch seen yet this session
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

    /// Transmitted airtime accumulated on channel `ch` this session (ms), for the
    /// compliance histogram.
    #[must_use]
    pub fn fhss_channel_airtime_ms(&self, ch: u8) -> u32 {
        // `.get`, not `[]`: a caller-supplied `ch >= FHSS_N` must return 0, not panic
        // (a panic on this public diagnostic getter is a HardFault reset on the L0).
        self.fhss.airtime_ms.get(ch as usize).map_or(0, |&v| v as u32)
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
    ///
    /// Returns `None` if FHSS master mode is not active (no clock anchor) — e.g. called
    /// before `enable_fhss(FhssRole::Master, …)` or on a node-role `Net`. This mirrors the
    /// typed refusal of [`send`](Self::send) (`WrongMode`) / [`fhss_send`](Self::fhss_send)
    /// (`NotSynced`) rather than panicking the device on API misuse.
    pub async fn fhss_master_tick(&mut self) -> Option<MasterSlot> {
        if self.fhss.role != FhssRole::Master {
            return None;
        }
        let seed = self.fhss.seed;
        let anchor = self.fhss.anchor?;
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
        Some(MasterSlot {
            channel: ch,
            slot,
            received,
        })
    }

    /// **Node:** run one slot. Whenever a clock anchor is held (Synced, or riding
    /// through a transient), predict this slot's channel, open RX a guard before the
    /// boundary, and catch the beacon to re-align. A missed beacon **keeps the anchor
    /// and keeps predicting** — drift stays ≪ the RX window for many slots, so a fade
    /// of up to [`REACQUIRE_LIMIT`](tower_net_core::epoch::REACQUIRE_LIMIT) slots is
    /// ridden through and re-locks within one slot once RF returns (it is *not* a sync
    /// loss). Only after the anchor is too
    /// stale to trust (the gap exceeds that, e.g. the gateway restarted onto a new
    /// epoch) is the anchor dropped and the node falls back to parking on the fixed
    /// rendezvous channel (the gateway's permutation sweeps onto it once per cycle).
    /// After this returns Synced with `got_beacon`, the gateway is listening on
    /// `channel` for the rest of the slot, so the caller may
    /// [`fhss_send`](Self::fhss_send) now.
    pub async fn fhss_node_tick(&mut self) -> NodeSlot {
        let seed = self.fhss.seed;

        if let Some(anchor) = self.fhss.anchor {
            // Track by prediction (initial Synced *and* fast re-acquire after a fade).
            let now = Instant::now();
            let slot = Self::fhss_cur_slot(anchor, now) + 1;
            let ch = Self::fhss_channel_for(seed, slot);
            let t_start = Self::fhss_slot_start(anchor, slot);

            let _ = config::set_freq_hz(&mut self.radio, fhss_freq_hz(ch)).await;
            self.fhss.cur_channel = ch;
            // Open RX a guard *before* the boundary so it's armed before the gateway
            // transmits (covers rx() setup latency).
            Timer::at(t_start.checked_sub(GUARD).unwrap_or(t_start)).await;

            // Re-anchor only to a beacon the gate accepts ([`EpochGate::offer_tracked`]): not
            // from an older gateway session (a replayed capture is ignored) and consistent with
            // our prediction, so a stale replay can't yank our clock — see the kernel docs for
            // the full argument. A genuinely restarted gateway carries a higher epoch but a slot
            // near 0, so it fails the prediction check here and is recovered via the rescan path
            // (below).
            if let Some((epoch, slot_abs, t_rx)) =
                self.fhss_rx_beacon(Duration::from_millis(BEACON_RX_MS)).await
                && self.fhss.gate.offer_tracked(epoch, slot_abs, slot) == TrackVerdict::ReAnchor
            {
                self.fhss_lock(slot_abs, t_rx); // → Synced, re-anchored (gate reset the misses)
                return NodeSlot {
                    state: FhssState::Synced,
                    channel: ch,
                    slot,
                    got_beacon: true,
                };
            }

            // Missed this slot's beacon (or it failed the gate's checks above) —
            // keep predicting on the (drift-tracked)
            // anchor. Stay Synced through a fade; only give up the anchor once it's
            // too stale to trust (the gate counts the misses), then re-scan from the
            // rendezvous channel.
            if self.fhss.gate.beacon_missed() == MissVerdict::Rescan {
                self.fhss.anchor = None;
                self.fhss.state = FhssState::Scanning;
                return NodeSlot {
                    state: FhssState::Scanning,
                    channel: ch,
                    slot,
                    got_beacon: false,
                };
            }
            return NodeSlot {
                state: FhssState::Synced,
                channel: ch,
                slot,
                got_beacon: false,
            };
        }

        // No anchor: initial acquisition or post-restart — park on the rendezvous
        // channel and listen a wide window for the gateway's once-per-cycle pass.
        let _ = config::set_freq_hz(&mut self.radio, fhss_freq_hz(FHSS_RENDEZVOUS_CH)).await;
        self.fhss.cur_channel = FHSS_RENDEZVOUS_CH;
        let scanning = NodeSlot {
            state: FhssState::Scanning,
            channel: FHSS_RENDEZVOUS_CH,
            slot: 0,
            got_beacon: false,
        };
        let Some((epoch, slot_abs, t_rx)) =
            self.fhss_rx_beacon(Duration::from_millis(RENDEZVOUS_RX_MS)).await
        else {
            return scanning;
        };
        // The reject/hold/lock decision — the older-session replay reject and the two-beacon
        // consistency guard — is the gate's ([`EpochGate::offer_scanning`]; see the kernel docs
        // for the full argument). We measure the elapsed wall-time since the held candidate in
        // whole slots and keep its `Instant` here (the kernel is time-free).
        let elapsed_slots = self
            .fhss
            .acquire_t0
            .map(|t0| (t_rx.saturating_duration_since(t0).as_millis() / FHSS_SLOT_MS) as u32);
        match self.fhss.gate.offer_scanning(epoch, slot_abs, elapsed_slots) {
            ScanVerdict::Reject => scanning,
            ScanVerdict::Lock => {
                self.fhss.acquire_t0 = None;
                self.fhss_lock(slot_abs, t_rx);
                NodeSlot {
                    state: FhssState::Synced,
                    channel: FHSS_RENDEZVOUS_CH,
                    slot: slot_abs,
                    got_beacon: true,
                }
            }
            ScanVerdict::Hold { replaced } => {
                // Re-stamp the candidate-arrival clock ONLY if this beacon became the held
                // candidate. A beacon behind the held one (`replaced == false` — e.g. a periodic
                // replay) must not reset the reference, or the elapsed-time consistency check
                // never converges and acquisition is denied.
                if replaced {
                    self.fhss.acquire_t0 = Some(t_rx);
                }
                NodeSlot {
                    state: FhssState::Scanning,
                    channel: FHSS_RENDEZVOUS_CH,
                    slot: slot_abs,
                    got_beacon: false,
                }
            }
        }
    }

    /// **Node:** send one frame on the *current* slot's channel (call right after a
    /// Synced [`fhss_node_tick`](Self::fhss_node_tick) — the gateway is listening on
    /// that channel). Refuses (`DutyLimited`) if the slot has too little time left for
    /// the exchange (slot-straddle rule), or `NotSynced` if not locked. Per-channel
    /// occupancy is bounded structurally (the hop schedule), so no airtime governor
    /// gates the TX; consumed airtime is recorded for the compliance histogram. One
    /// TX counter is consumed.
    pub async fn fhss_send(&mut self, dest: u32, data: &[u8], confirmed: bool) -> SendResult {
        if self.txc.locked() {
            return SendResult::Error(RadioError::NonceLocked); // fail closed (nonce safety)
        }
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
        let addr = self.addr;
        let counter = self.txc.counter();
        let key = self.key_for(dest);
        let hdr = Header {
            frame_type: FrameType::Data,
            flags: if confirmed { frame::flags::CONFIRMED } else { 0 },
            src: addr,
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
        let need = if confirmed {
            data_toa + 20 + beacon_toa_ms()
        } else {
            data_toa
        } + GUARD.as_millis();
        if room.as_millis() < need {
            return SendResult::DutyLimited; // no room this slot; caller retries next slot
        }

        let _ = config::set_freq_hz(&mut self.radio, fhss_freq_hz(ch)).await;
        self.fhss.cur_channel = ch;
        self.fhss.airtime_ms[ch as usize] =
            self.fhss.airtime_ms[ch as usize].saturating_add(duty::frame_toa_ms(n) as u16);
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
    /// subtraction only, never `slot·SLOT` (which would underflow). (The miss counter
    /// was already reset by the gate decision that admitted this beacon.)
    fn fhss_lock(&mut self, slot_abs: u32, t_rx: Instant) {
        let boundary = t_rx
            .checked_sub(Duration::from_millis(beacon_toa_ms()))
            .unwrap_or(t_rx);
        self.fhss.anchor = Some((slot_abs, boundary));
        self.fhss.state = FhssState::Synced;
        self.fhss.last_beacon = t_rx;
    }

    /// Build + transmit the beacon for `slot_abs` (already tuned to `ch`), sealed
    /// under the network (default) key so every node can sync; dwell-metered.
    async fn fhss_tx_beacon(&mut self, slot_abs: u32, ch: u8) {
        if self.txc.locked() {
            return; // fail closed: can't safely allocate a beacon counter (nonce safety)
        }
        let cycle = slot_abs / FHSS_N as u32;
        let i = (slot_abs % FHSS_N as u32) as u8;
        let mut pl = [0u8; BEACON_PL_LEN];
        pl[..4].copy_from_slice(&self.fhss.gate.epoch().to_le_bytes());
        pl[4..8].copy_from_slice(&cycle.to_le_bytes());
        pl[8] = i;
        let hdr = Header {
            frame_type: FrameType::Beacon,
            flags: 0,
            src: self.addr,
            dest: BROADCAST,
            counter: self.txc.counter(),
            bulk_index: None,
        };
        let mut buf = [0u8; MAX_FRAME];
        let key = self.default_key;
        if let Ok(n) = frame::seal_frame(&mut self.ccm, &key, &hdr, &pl, &mut buf) {
            self.advance_tx_counter();
            self.fhss.airtime_ms[ch as usize] =
                self.fhss.airtime_ms[ch as usize].saturating_add(duty::frame_toa_ms(n) as u16);
            let _ = self.radio.tx(&buf[..n], false, TX_TIMEOUT).await;
        }
    }

    /// Receive + authenticate a beacon under the network key (broadcast dest OK). CCM auth proves
    /// the beacon is genuine but NOT fresh — a captured beacon replayed later still authenticates —
    /// so the caller enforces freshness via the monotonic `epoch` (reject-backwards) and the
    /// two-beacon acquisition consistency check. Listens until a valid beacon arrives or the
    /// window elapses, ignoring any non-beacon frame in between (so a stray frame in the
    /// widened/early window doesn't count as a miss). Returns `(epoch, slot_abs, t_rx)`.
    async fn fhss_rx_beacon(&mut self, timeout: Duration) -> Option<(u32, u32, Instant)> {
        let deadline = Instant::now().checked_add(timeout)?;
        let key = self.default_key;
        let mut buf = [0u8; MAX_FRAME];
        loop {
            let remaining = deadline.checked_duration_since(Instant::now())?;
            let (len, _) = self.radio.rx(&mut buf, remaining).await.ok()?;
            let t_rx = Instant::now();
            // Only a frame that parses, is a Beacon, and CCM-opens counts; otherwise
            // keep listening for the rest of the window.
            if let Ok((peek, _)) = frame::Header::parse(&buf[..len])
                && peek.frame_type == FrameType::Beacon
                && let Ok((_, range)) = frame::open_frame(&mut self.ccm, &key, &mut buf[..len])
                && range.len() >= BEACON_PL_LEN
            {
                let pl = &buf[range];
                let epoch = u32::from_le_bytes([pl[0], pl[1], pl[2], pl[3]]);
                let cycle = u32::from_le_bytes([pl[4], pl[5], pl[6], pl[7]]);
                let slot_abs = cycle * FHSS_N as u32 + pl[8] as u32;
                return Some((epoch, slot_abs, t_rx));
            }
        }
    }
}
