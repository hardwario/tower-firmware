//! Network layer: confirmed delivery, ACK, retransmit and replay protection
//! over the secured frame codec (docs/radio.md).
//!
//! `Net` owns the radio + CCM and serializes one transfer at a time (docs/radio.md). A
//! *node* `send(confirmed)` transmits a DATA frame then opens a 200 ms ACK
//! window, retransmitting the byte-identical frame on timeout (random 0–100 ms
//! backoff, 1–10 reps). A *receiver* `recv()` authenticates the frame, applies
//! the counter/replay rule, and auto-ACKs a confirmed frame — caching the ACK so
//! a retransmit re-sends the identical bytes without re-delivering.
//!
//! Keys are per-peer: [`Net::add_peer`] binds a peer address to its own AES key and
//! replay lane (star ≤16 / P2P ≤8, docs/radio.md); any unregistered peer falls back to the
//! [`NetConfig::key`] default lane (the single-link case). The TX counter and each
//! lane's last-seen are EEPROM-persisted (reserve-ahead watermark / lazy-persist, docs/radio.md).
//!
//! This module holds the core (peer table + confirmed unicast). The two larger
//! sub-protocols live alongside it as `impl Net` blocks: [`bulk`] (pull-based bulk
//! transfer, docs/radio.md) and [`pairing`] (OTA 3-way JOIN, docs/radio.md).

mod afa;
mod bulk;
mod fhss;
mod pairing;

pub use afa::{AfaConfig, AfaRole};
pub use bulk::{BULK_CHUNK, BulkSink, BulkSource};
pub use fhss::{FhssConfig, FhssRole, FhssState, MasterSlot, NodeSlot, hop_channel};
pub use pairing::{PAIRING_KEY, PAIRING_WINDOW};

use embassy_time::{Duration, Instant, Timer};
// The security-critical *decision* kernels (replay rule, TX-counter watermark/fail-closed,
// ACK resolution) live in the host-testable `tower_net_core` leaf crate; this module keeps the
// radio/EEPROM flow and delegates each decision there. Zero behavioural change on target.
pub use tower_net_core::ack::AckMeta;
use tower_net_core::ack::{AckVerdict, AckWait, ack_flags, ack_meta};
use tower_net_core::replay::{ReplayLane, ReplayVerdict, assign_lane};
use tower_net_core::txctr::TxCounter;

use super::ccm::Ccm;
use super::config::{self, Band, RfConfig};
use super::device::{RadioError, Spirit1};
use super::duty::{self, DutyGovernor};
use super::frame::{self, FrameType, Header, MAX_FRAME, MAX_PAYLOAD, flags};
use crate::storage::{NS_NET, Nv, Scoped};

/// ACK window the sender waits for an acknowledgement (docs/radio.md). The measured ACK
/// round-trip is ~35 ms (turnaround + ACK ToA + RX set-up), so 200 ms is ample.
const ACK_WINDOW: Duration = Duration::from_millis(200);
/// Turnaround the receiver waits before sending the ACK, so the sender has
/// finished switching TX→RX and is listening (the ACK window is 200 ms, so
/// there's ample room). Without this the ACK preamble races the sender's RX
/// set-up (to_ready + flush + mask + strobe, several SPI ops) and is missed.
/// Shared by bulk + pairing for the same TX→RX turnaround reason.
pub(crate) const ACK_TURNAROUND: Duration = Duration::from_millis(20);
/// Per-TX timeout (CSMA + ToA budget); generous for a ≤96 B frame at 19.2 kbps.
pub(crate) const TX_TIMEOUT: Duration = Duration::from_millis(120);
/// Default confirmed-delivery repetitions.
pub const DEFAULT_REPS: u8 = 3;
/// Max inter-rep backoff (ms), randomised to de-sync collided senders.
const MAX_BACKOFF_MS: u32 = 100;

// The TX-counter reserve block (`RESERVE` = 1024) and the receiver lazy-persist period
// (`P` = 32) live with their kernels in `tower_net_core` (`txctr::RESERVE`, `replay::P`).
/// EEPROM local keys (within `NS_NET`) for the persisted counter state.
const KEY_WATERMARK: u8 = 0x00;
const KEY_LASTSEEN: u8 = 0x01;
/// FHSS gateway epoch (boot-id): a master bumps + persists it each session so its beacons are
/// strictly newer than any previous session's, and a node refuses to anchor to an older epoch —
/// defeating a replayed beacon capture. `0x02` is free (peer lanes start at `KEY_LASTSEEN_BASE`).
const KEY_FHSS_EPOCH: u8 = 0x02;

/// Peer-table capacity. A gateway in a star holds up to 16 nodes; a P2P device
/// holds up to 8 peers (docs/radio.md). One table size covers both — the topology
/// is a usage policy, not a different type. Sized at 16 (was 64, then 32): the
/// table (`[Option<Peer>; MAX_PEERS]`, ~36 B/slot) lives in every Net app's
/// statically-allocated future, and with flip-link each slot is stack headroom the
/// 20 KB part loses — the gateway product's boot peak is what forced the last
/// halving (2026-07-11: 32 slots left it ~0.7 KB under the ~9 KB measured Net-app
/// stack peak → HardFault loop before the first Hello). 16 also halves the
/// replay-lane KV-key census (tower_kv::MAX_KEYS = 64 is a global budget).
pub const MAX_PEERS: usize = 16;
/// Local base (within `NS_NET`) for per-peer last-seen lanes (slot `i` → `KEY_LASTSEEN_BASE + i`,
/// `i < MAX_PEERS = 16`, so lanes occupy locals `0x10..=0x1F`).
const KEY_LASTSEEN_BASE: u8 = 0x10;

/// A registered peer: its address, per-peer AES key, and replay state (docs/radio.md).
#[derive(Clone, Copy)]
struct Peer {
    addr: u32,
    key: [u8; 16],
    /// The peer's replay lane (last-seen + lazy-persist cadence, `tower_net_core::replay`).
    lane: ReplayLane,
    /// KV local (`NS_NET`) where this peer's replay lane is persisted. Bound by *peer addr*
    /// (`assign_lane`), NOT the table slot — so the lane survives a slot shift when an
    /// earlier peer is removed and the table compacts (see [`add_peer`](Net::add_peer)).
    lane_local: u8,
    /// A downlink is queued for this peer (app-maintained via [`Net::set_pending`]);
    /// advertised in the flags byte of every ACK we send it. Must be set *before* the
    /// uplink that should learn it arrives — the ACK goes out inside [`Net::recv`],
    /// before the app sees the frame.
    pending: bool,
}

/// Outcome of a [`Net::send`]. Inspect it — a `NotDelivered`/`Busy`/`DutyLimited`
/// result is a delivery failure the caller must handle, not silently drop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[must_use]
pub enum SendResult {
    /// Confirmed and ACKed (or unconfirmed and transmitted).
    Delivered,
    /// Confirmed but no ACK after all repetitions.
    NotDelivered,
    /// CSMA reported the channel busy.
    Busy,
    /// The duty governor refused the TX (would exceed the airtime budget), or in
    /// LBT+AFA mode every channel was busy/in-off-time (couldn't transmit politely).
    DutyLimited,
    /// A mode-specific send was called in the wrong [`Access`] mode (e.g. `afa_send`
    /// outside AFA mode), or a plain `send` was attempted while a hopping/agility
    /// mode owns the channel schedule.
    WrongMode,
    /// FHSS `fhss_send` was called while the node has not locked to the gateway's
    /// hop schedule (not Synced).
    NotSynced,
    /// A local error (bad length, radio fault).
    Error(RadioError),
}

impl core::fmt::Display for SendResult {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            SendResult::Delivered => f.write_str("delivered"),
            SendResult::NotDelivered => f.write_str("not delivered (no ACK)"),
            SendResult::Busy => f.write_str("channel busy"),
            SendResult::DutyLimited => f.write_str("duty-cycle limited"),
            SendResult::WrongMode => f.write_str("wrong access mode"),
            SendResult::NotSynced => f.write_str("not synced to hop schedule"),
            SendResult::Error(e) => write!(f, "{e}"),
        }
    }
}

/// Spectrum-access mode (mutually exclusive, runtime-switchable like `set_band`).
/// The default [`Duty`](Access::Duty) path is unchanged; AFA/FHSS add polite,
/// region-specific access on top (EU LBT+AFA / US FHSS).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    /// Plain channel access governed by the band duty cycle (EU 1 %). Default.
    Duty,
    /// EU 868 Listen-Before-Talk + Adaptive Frequency Agility (EN 300 220).
    Afa,
    /// US 915 frequency hopping (FCC §15.247). Plain `send` is refused in this mode
    /// (a static-channel TX while hopping would violate the hopping requirement);
    /// use [`Net::fhss_send`].
    Fhss,
}

/// A received, authenticated application message.
pub struct Received {
    pub src: u32,
    pub counter: u32,
    pub rssi: i16,
    /// Link/packet quality indicator (the SPIRIT1's PQI) for this reception.
    pub lqi: u8,
    /// Whether the sender requested confirmation (an ACK was sent back).
    pub confirmed: bool,
    len: usize,
    buf: [u8; MAX_PAYLOAD],
}

impl Received {
    /// The decrypted payload bytes.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

/// Network configuration for this device.
pub struct NetConfig {
    /// This device's 32-bit address (rides in the clear header).
    pub addr: u32,
    /// Default AES-128 key, used for any peer not explicitly registered via
    /// [`Net::add_peer`]. In a single-link setup this is the link key; in a star
    /// it is the gateway's own/fallback key (each node is registered with its
    /// per-node key).
    pub key: [u8; 16],
    pub band: Band,
    pub channel: u8,
}

/// The network layer over one SPIRIT1 radio.
pub struct Net {
    radio: Spirit1,
    ccm: Ccm,
    addr: u32,
    /// Default key for unregistered peers (see [`NetConfig::key`]).
    default_key: [u8; 16],
    /// Per-peer (addr, key, last-seen) table; a registered peer overrides the
    /// default key and gets its own replay lane (docs/radio.md).
    peers: [Option<Peer>; MAX_PEERS],
    /// Replay lane for senders not in the peer table (the single-link lane).
    default_lane: ReplayLane,
    /// The TX-counter kernel: monotonic counter + reserve-ahead watermark + the fail-closed
    /// lock (`tower_net_core::txctr` — the CCM nonce anti-reuse invariant lives there; this
    /// module owns its EEPROM persistence). See [`advance_tx_counter`](Self::advance_tx_counter)
    /// and `RadioError::NonceLocked`.
    txc: TxCounter,
    /// EEPROM-backed counter persistence (shared handle).
    kv: Nv,
    /// EU duty-cycle governor (airtime budget for all TX).
    duty: DutyGovernor,
    /// Simple LCG state for the retransmit backoff (seeded from addr).
    rng: u32,
    /// Active spectrum-access mode (Duty default; AFA/FHSS switch at runtime).
    access: Access,
    /// EU LBT+AFA state (inert unless `access == Afa`).
    afa: afa::Afa,
    /// US FHSS state (inert unless `access == Fhss`).
    fhss: fhss::Fhss,
    /// Metadata of the ACK that resolved the most recent confirmed [`send`](Self::send)
    /// (receiver-side RSSI + the pending-downlink flag). Cleared at the start of each
    /// send; `None` after an unconfirmed or failed one.
    last_ack: Option<AckMeta>,
}

impl Net {
    /// Bring the radio up, apply the RF config, and initialise counters from
    /// EEPROM (`kv`): resume the TX counter at the persisted reserve watermark and
    /// reserve the next block, and restore the default-lane last-seen (per-peer
    /// lanes are restored when their peer is registered via [`add_peer`](Self::add_peer)).
    pub async fn new(mut radio: Spirit1, kv: Nv, cfg: NetConfig) -> Result<Self, RadioError> {
        radio.exit_shutdown().await?;
        radio.read_device_id()?;
        config::apply(
            &mut radio,
            &RfConfig {
                band: cfg.band,
                channel: cfg.channel,
            },
        )
        .await?;

        // Reserve-ahead TX counter: resume *at* the persisted watermark (1 on the very first
        // boot, since 0 = "never sent"), then reserve the next block. The resume/fail-closed
        // decision (incl. the lost-watermark corruption case) is `tower_net_core::txctr` —
        // see `TxCounter::resume` for the full rationale.
        let nkv = kv.scope(NS_NET); // our namespaced view; the raw `kv` is kept for `Net::kv()`
        let stored_watermark = read_u32(nkv, KEY_WATERMARK);
        let stored_last_seen = read_u32(nkv, KEY_LASTSEEN);
        let last_seen = stored_last_seen.unwrap_or(0);
        let (mut txc, boot_watermark) = TxCounter::resume(stored_watermark, stored_last_seen.is_some());
        if let Some(watermark) = boot_watermark {
            // Persist the reserve watermark AND verify it read back. TX is only nonce-safe while a
            // watermark strictly greater than any counter we will ever send is durably stored. If
            // the write fails or does not read back (EEPROM full/faulted), start TX **locked**:
            // the device can still receive, but sending would risk resuming at a stale watermark
            // after a reboot and reusing a CCM nonce. (Fail closed — see `TxCounter`.)
            let persisted = nkv.set_bytes(KEY_WATERMARK, &watermark.to_le_bytes()).is_ok();
            if !(persisted && read_u32(nkv, KEY_WATERMARK) == Some(watermark)) {
                txc.lock();
            }
        }

        // Duty policy follows the band: EU 1 %, US 915 unrestricted (docs/radio.md).
        let duty = match cfg.band {
            Band::Eu868 => DutyGovernor::eu(),
            Band::Us915 => DutyGovernor::us915(),
        };

        Ok(Self {
            radio,
            ccm: Ccm::new(),
            addr: cfg.addr,
            default_key: cfg.key,
            peers: [None; MAX_PEERS],
            default_lane: ReplayLane::new(last_seen),
            txc,
            kv,
            duty,
            rng: cfg.addr | 1,
            access: Access::Duty,
            afa: afa::Afa::disabled(),
            fhss: fhss::Fhss::disabled(),
            last_ack: None,
        })
    }

    /// The active spectrum-access mode ([`Access::Duty`] unless AFA/FHSS was enabled).
    #[must_use]
    pub fn access(&self) -> Access {
        self.access
    }

    /// This device's own address.
    #[must_use]
    pub fn addr(&self) -> u32 {
        self.addr
    }

    /// The shared (unscoped) EEPROM handle, for application-level persistence. The network
    /// layer owns the `NS_NET` namespace; for your own keys take a [`Scoped`] view of a different
    /// namespace, e.g. `net.kv().scope(NS_APP)` (see [`Nv::scope`](crate::storage::Nv::scope)).
    pub fn kv(&self) -> Nv {
        self.kv
    }

    /// Current live TX counter (for diagnostics / persistence demos).
    #[must_use]
    pub fn tx_counter(&self) -> u32 {
        self.txc.counter()
    }

    /// Current persisted reserve watermark.
    #[must_use]
    pub fn reserve_watermark(&self) -> u32 {
        self.txc.reserve_limit()
    }

    /// Current last-seen counter on the default lane (single-link diagnostics).
    #[must_use]
    pub fn last_seen(&self) -> u32 {
        self.default_lane.last_seen()
    }

    /// Register (or re-key) a peer: an explicit `peer_addr` → per-peer `key` binding with
    /// its own replay lane. The peer's persisted last-seen is restored. Returns
    /// `false` only if the table is full (and the peer is new) — check the return in
    /// production code. Up to [`MAX_PEERS`] peers (star ≤16 / P2P ≤8 by policy, docs/radio.md).
    pub fn add_peer(&mut self, peer_addr: u32, key: &[u8; 16]) -> bool {
        if let Some(i) = self.peer_slot(peer_addr) {
            // Re-key in place. A *changed* key is a disjoint CCM nonce space (docs/radio.md:
            // "a re-key resets both ends"), so the old last-seen is meaningless — and if the
            // re-keyed peer restarts its counter (a re-pair), a stale-high lane would reject
            // its fresh frames as replays. Reset + persist the lane on a key change so a reboot
            // doesn't restore the pre-re-key value. An idempotent re-add with the SAME key
            // (e.g. the boot mirror then a NodeAdd of an already-known peer) keeps the lane —
            // resetting it there would needlessly reopen the replay window.
            let changed = self.peers[i].as_ref().unwrap().key != *key;
            {
                let p = self.peers[i].as_mut().unwrap();
                p.key = *key;
                if changed {
                    p.lane = ReplayLane::new(0);
                }
            }
            if changed {
                let local = self.peers[i].as_ref().unwrap().lane_local;
                let _ = self.kv.scope(NS_NET).set_bytes(local, &lane_record(peer_addr, 0));
            }
            return true;
        }
        let Some(slot) = self.peers.iter().position(|p| p.is_none()) else {
            return false; // table full
        };
        let nkv = self.kv.scope(NS_NET);
        // Bind the replay lane by peer *addr*, not by table slot. Snapshot the persisted lane
        // records and the locals already held by live peers, then let `assign_lane` decide
        // (restore this peer's own record wherever it sits, else claim the lowest free local).
        // This is why a survivor keeps its replay window when an earlier peer is removed and
        // the table compacts — its slot shifts but its lane record is found by addr.
        let mut records = [None; MAX_PEERS];
        for (l, rec) in records.iter_mut().enumerate() {
            *rec = read_u32_pair(nkv, KEY_LASTSEEN_BASE + l as u8);
        }
        let used = self
            .peers
            .iter()
            .flatten()
            .fold(0u32, |m, p| m | 1 << (p.lane_local - KEY_LASTSEEN_BASE));
        // A free table slot guarantees a free local (≤ MAX_PEERS-1 live peers here), so
        // `assign_lane` returns Some — but fail closed to no-op if that ever changes.
        let Some(bind) = assign_lane(peer_addr, &records, used) else {
            return false;
        };
        let local = KEY_LASTSEEN_BASE + bind.index as u8;
        if bind.fresh {
            let _ = nkv.set_bytes(local, &lane_record(peer_addr, 0)); // claim the local for this addr
        }
        self.peers[slot] = Some(Peer {
            addr: peer_addr,
            key: *key,
            lane: ReplayLane::new(bind.seen),
            lane_local: local,
            pending: false,
        });
        true
    }

    /// Remove a peer. Returns whether it was present. (Its persisted last-seen record is
    /// left in EEPROM tagged with its addr; re-adding the *same* peer — even into a different
    /// slot — resumes its replay window, while a different peer inheriting the slot starts
    /// fresh at 0, since restore is now addr-matched. See [`add_peer`](Self::add_peer).)
    pub fn remove_peer(&mut self, peer_addr: u32) -> bool {
        if let Some(i) = self.peer_slot(peer_addr) {
            self.peers[i] = None;
            true
        } else {
            false
        }
    }

    /// Number of registered peers.
    #[must_use]
    pub fn peer_count(&self) -> usize {
        self.peers.iter().filter(|p| p.is_some()).count()
    }

    /// Mark whether a downlink is queued for a registered peer — advertised in the
    /// flags byte of every ACK sent to it, so a sleeping node knows to hold an RX
    /// window open after its uplink. Set it **before** the uplink that should learn
    /// it: the ACK goes out inside [`recv`](Self::recv), before the app sees the
    /// frame. Returns `false` if the peer isn't registered.
    pub fn set_pending(&mut self, peer_addr: u32, pending: bool) -> bool {
        match self.peer_slot(peer_addr) {
            Some(i) => {
                self.peers[i].as_mut().unwrap().pending = pending;
                true
            }
            None => false,
        }
    }

    /// Metadata of the ACK that resolved the most recent confirmed [`send`](Self::send):
    /// the receiver-side RSSI it reported, and whether it advertises a queued downlink
    /// for us (the sleeping-node RX-window cue). `None` after an unconfirmed, failed,
    /// or not-yet-attempted send.
    #[must_use]
    pub fn last_ack(&self) -> Option<AckMeta> {
        self.last_ack
    }

    /// Last-seen counter for a registered peer (`None` if not registered).
    #[must_use]
    pub fn peer_last_seen(&self, peer_addr: u32) -> Option<u32> {
        self.peer_slot(peer_addr)
            .map(|i| self.peers[i].as_ref().unwrap().lane.last_seen())
    }

    /// Table slot holding `peer_addr`, if registered.
    fn peer_slot(&self, peer_addr: u32) -> Option<usize> {
        self.peers
            .iter()
            .position(|p| matches!(p, Some(pe) if pe.addr == peer_addr))
    }

    /// AES key for `peer_addr`: the peer's key if registered, else the default key.
    fn key_for(&self, peer_addr: u32) -> [u8; 16] {
        match self.peer_slot(peer_addr) {
            Some(i) => self.peers[i].as_ref().unwrap().key,
            None => self.default_key,
        }
    }

    /// The replay lane for `src` (peer lane if registered, else default) — a copy, for the
    /// pure [`classify`](ReplayLane::classify) decision.
    fn lane(&self, src: u32) -> ReplayLane {
        match self.peer_slot(src) {
            Some(i) => self.peers[i].as_ref().unwrap().lane,
            None => self.default_lane,
        }
    }

    /// Record acceptance of `counter` from `src`: advance that lane's last-seen
    /// and lazy-persist every `P` accepts (replay window ≤ P across a reboot, docs/radio.md).
    /// The cadence decision is the lane kernel's ([`ReplayLane::accept`]); the EEPROM write is ours.
    fn lane_accept(&mut self, src: u32, counter: u32) {
        match self.peer_slot(src) {
            Some(i) => {
                let p = self.peers[i].as_mut().unwrap();
                if p.lane.accept(counter) {
                    let (peer_addr, local) = (p.addr, p.lane_local); // addr-bound local, not the slot
                    let _ = self
                        .kv
                        .scope(NS_NET)
                        .set_bytes(local, &lane_record(peer_addr, counter));
                }
            }
            None => {
                if self.default_lane.accept(counter) {
                    let _ = self
                        .kv
                        .scope(NS_NET)
                        .set_bytes(KEY_LASTSEEN, &counter.to_le_bytes());
                }
            }
        }
    }

    /// Advance the TX counter, re-reserving + persisting the next block when the
    /// current reserve is exhausted (the only TX-counter persistence path).
    ///
    /// The counter arithmetic — saturating at the 2³²−1 ceiling and locking TX there, and the
    /// reserve-ahead watermark — is [`TxCounter::advance`] (see its docs for the full CCM
    /// nonce-reuse argument). Here we only execute the persist it requests: if the write
    /// lands, the reservation is committed; if it fails, TX locks (fail closed) — the guard on
    /// every send path refuses to transmit while locked.
    ///
    /// Latency note: this write (once per `RESERVE` = 1024 sends) could formerly absorb the
    /// whole ~5 s compaction stall if it happened to fill the KV half. With the default
    /// `storage::maintenance` task the store compacts incrementally ahead of time, so the
    /// persist stays a single small append; the synchronous flip is only the fallback path
    /// (docs/storage.md).
    fn advance_tx_counter(&mut self) {
        if let Some(next) = self.txc.advance() {
            match self
                .kv
                .scope(NS_NET)
                .set_bytes(KEY_WATERMARK, &next.to_le_bytes())
            {
                Ok(()) => self.txc.reserve_persisted(next),
                Err(_) => self.txc.lock(),
            }
        }
    }

    /// Send `data` to `dest`. Confirmed sends open an ACK window and retransmit
    /// the byte-identical frame up to `reps` times; unconfirmed sends transmit
    /// once. The transfer consumes exactly one TX counter value (docs/radio.md).
    pub async fn send(&mut self, dest: u32, data: &[u8], confirmed: bool, reps: u8) -> SendResult {
        self.last_ack = None; // stale ACK metadata must not outlive its send
        if self.txc.locked() {
            return SendResult::Error(RadioError::NonceLocked); // fail closed (nonce safety)
        }
        // In FHSS mode a static-channel TX would break the hopping requirement;
        // callers must use fhss_send (which hops). AFA still allows plain send.
        if self.access == Access::Fhss {
            return SendResult::WrongMode;
        }
        if data.len() > MAX_PAYLOAD {
            return SendResult::Error(RadioError::TooLong); // MTU: use bulk for >74 B (docs/radio.md)
        }
        let counter = self.txc.counter();
        let hdr = Header {
            frame_type: FrameType::Data,
            flags: if confirmed { flags::CONFIRMED } else { 0 },
            src: self.addr,
            dest,
            counter,
            bulk_index: None,
        };
        let key = self.key_for(dest);
        let mut frame_buf = [0u8; MAX_FRAME];
        let n = match frame::seal_frame(&mut self.ccm, &key, &hdr, data, &mut frame_buf) {
            Ok(n) => n,
            Err(_) => return SendResult::Error(RadioError::TooLong),
        };

        // Consume the counter NOW — after sealing this frame, before it goes on air.
        // The nonce is derived from `counter`, so the counter must be spent even if this
        // send-future is later cancelled (a `select(send, Timer)` dropped mid-window):
        // otherwise the next send would read the same value and reuse the (key, nonce)
        // pair — a full CCM break. Retransmits below reuse the already-sealed `frame_buf`,
        // not a fresh counter, so advancing here is correct. A never-transmitted counter
        // (duty-limited first rep) just leaves a harmless hole in the monotonic sequence.
        self.advance_tx_counter();

        let toa = duty::frame_toa_ms(n);
        let reps = if confirmed { reps.clamp(1, 10) } else { 1 };
        let mut result = SendResult::NotDelivered;
        for attempt in 0..reps {
            if attempt > 0 {
                // Random 0–100 ms backoff before a retransmit (docs/radio.md).
                Timer::after(Duration::from_millis(self.backoff_ms() as u64)).await;
            }
            // Duty governor: every TX (incl. retransmits) counts (docs/radio.md).
            if !self.duty.try_tx(toa) {
                result = SendResult::DutyLimited;
                break;
            }
            match self.radio.tx(&frame_buf[..n], false, TX_TIMEOUT).await {
                Ok(()) => {}
                Err(RadioError::Busy) => {
                    result = SendResult::Busy;
                    continue;
                }
                Err(e) => {
                    result = SendResult::Error(e);
                    break;
                }
            }
            if !confirmed {
                result = SendResult::Delivered;
                break;
            }
            // Open the ACK window and look for our ACK.
            if self.await_ack(dest, counter).await {
                result = SendResult::Delivered;
                break;
            }
        }
        result
    }

    /// Receive one frame (up to `timeout`). Authenticates it, applies the
    /// counter/replay rule, auto-ACKs a confirmed frame, and returns the message
    /// for a freshly-accepted frame (`None` for a replay, retransmit, frame not
    /// addressed to us, auth failure, or timeout).
    pub async fn recv(&mut self, timeout: Duration) -> Option<Received> {
        let mut buf = [0u8; MAX_FRAME];
        let (len, q) = self.radio.rx(&mut buf, timeout).await.ok()?;

        // Peek the clear header (AAD) to learn the src, then CCM-open with that
        // peer's key — a registered peer uses its own key, others the default.
        let (peek, _) = frame::Header::parse(&buf[..len]).ok()?;
        if peek.dest != self.addr {
            return None; // not for us
        }
        let key = self.key_for(peek.src);

        // CCM-verify first (authenticates the header incl. counter), then decide. The
        // counter/replay rule is the lane kernel's (`tower_net_core::replay`) — see its docs
        // for why classification must only ever run on an authenticated frame.
        let (hdr, range) = frame::open_frame(&mut self.ccm, &key, &mut buf[..len]).ok()?;

        // Only application DATA frames surface here. An authenticated ACK / Join / Bulk /
        // Beacon addressed to us — e.g. a node's ACK arriving *after* our send's window
        // closed — must NOT advance the replay lane or be returned as an "uplink": those
        // types are owned by await_ack / pairing / bulk / fhss, each with its own rx path.
        // (Before this filter, a late ACK's own counter would be classified Fresh, advance
        // the src's lane, and be forwarded by the gateway as a 6-byte bogus uplink.)
        if hdr.frame_type != FrameType::Data {
            return None;
        }

        match self.lane(hdr.src).classify(hdr.counter) {
            ReplayVerdict::Fresh => {
                // Fresh — accept, advance the sender's lane, ACK if requested.
                self.lane_accept(hdr.src, hdr.counter);
                let confirmed = hdr.flags & flags::CONFIRMED != 0;
                if confirmed {
                    self.send_ack(hdr.src, hdr.counter, q.rssi).await;
                }
                let plen = range.end - range.start;
                let mut out = [0u8; MAX_PAYLOAD];
                out[..plen].copy_from_slice(&buf[range]);
                Some(Received {
                    src: hdr.src,
                    counter: hdr.counter,
                    rssi: q.rssi,
                    lqi: q.lqi,
                    confirmed,
                    len: plen,
                    buf: out,
                })
            }
            ReplayVerdict::Retransmit => {
                // Benign retransmit: this peer's most-recently-accepted counter is being resent
                // because its ACK was lost. Re-ACK deterministically (identified by this src's own
                // lane last-seen), do NOT re-deliver. Keying the re-ACK by (src, counter) rather than
                // a single global cache means interleaved senders in a star can't evict each other's
                // pending ACK — the old single-slot cache reported false NotDelivered + app-level
                // duplicates under that race (see docs/radio.md). Only confirmed frames were ACKed,
                // so only re-ACK confirmed ones.
                if hdr.flags & flags::CONFIRMED != 0 {
                    self.send_ack(hdr.src, hdr.counter, q.rssi).await;
                }
                None
            }
            // counter < last-seen → replay; drop silently (replay state untouched).
            ReplayVerdict::Replay => None,
        }
    }

    /// Retune to a different `band`/`channel` at runtime (both ends must agree).
    /// Rewrites the synthesizer registers (VCO recalibrates on the next TX/RX) and
    /// moves the duty policy to match the band (EU 1 % / US 915 unrestricted).
    /// A single firmware image runs either band — the choice is made here, live.
    pub async fn set_band(&mut self, band: Band, channel: u8) -> Result<(), RadioError> {
        config::set_band(&mut self.radio, band, channel).await?;
        self.duty = match band {
            Band::Eu868 => DutyGovernor::eu(),
            Band::Us915 => DutyGovernor::us915(),
        };
        Ok(())
    }

    /// Wait up to `ACK_WINDOW` for an ACK from `dest` acknowledging `counter`.
    ///
    /// Loops until the window deadline, **ignoring** any frame that isn't the ACK we're waiting
    /// for, rather than giving up on the first one. The SPIRIT1 runs with no address filtering,
    /// so `rx()` surfaces the first CRC-valid frame from *anyone* — a neighbouring node's uplink
    /// landing in our 200 ms window is common in a star. Treating that foreign (undecodable under
    /// our key) frame as "no ACK" was a false `NotDelivered` that burned a retransmit and could
    /// report a delivery failure the gateway had actually received — the exact star-contention
    /// bug `recv()`'s re-ACK cache fixed on the receive side. This mirrors `recv()` /
    /// `fhss_rx_beacon`: keep listening until the deadline. The accept/ignore decision per
    /// heard frame is the [`AckWait`] kernel's (`tower_net_core::ack`).
    async fn await_ack(&mut self, dest: u32, counter: u32) -> bool {
        let key = self.key_for(dest);
        let mut wait = AckWait::new(self.addr, dest, counter);
        let deadline = Instant::now() + ACK_WINDOW;
        let mut buf = [0u8; MAX_FRAME];
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.as_ticks() == 0 {
                return false; // window elapsed
            }
            let (len, _) = match self.radio.rx(&mut buf, remaining).await {
                Ok(v) => v,
                // A collided/corrupt frame (CRC or FIFO) is exactly what star contention
                // produces — the thing this window exists to ride out. Keep waiting rather
                // than reporting a false NotDelivered (and burning a retransmit) on it.
                Err(RadioError::CrcError | RadioError::FifoError) => continue,
                // Timeout over the remaining window, or a real radio fault: window ends.
                Err(_) => return false,
            };
            // A foreign / undecodable frame is NOT a failed ACK — skip it and keep waiting.
            let Ok((hdr, range)) = frame::open_frame(&mut self.ccm, &key, &mut buf[..len]) else {
                continue;
            };
            let is_ack = hdr.frame_type == FrameType::Ack;
            if wait.offer(is_ack, hdr.src, hdr.dest, &buf[range.clone()]) == AckVerdict::Resolved {
                // Keep the resolving ACK's metadata (receiver RSSI + the wire-v3
                // pending-downlink flag) for `last_ack()` — the sleeping-node cue.
                self.last_ack = Some(ack_meta(&buf[range]));
                return true;
            }
        }
    }

    /// Build, cache and transmit an ACK for a received confirmed frame. The ACK
    /// uses the ACKer's *own* fresh counter (docs/radio.md); the acknowledged counter rides
    /// in the payload.
    async fn send_ack(&mut self, dest: u32, acked: u32, rssi: i16) {
        if self.txc.locked() {
            return; // fail closed: can't safely allocate a counter (nonce safety); sender retries
        }
        // Let the sender finish its TX→RX turnaround before we transmit.
        Timer::after(ACK_TURNAROUND).await;
        let ack_counter = self.txc.counter();
        let mut payload = [0u8; 6];
        payload[..4].copy_from_slice(&acked.to_le_bytes());
        // Clamp to i8 range before packing: rssi is i16 and the SPIRIT1 noise floor reaches
        // below −128 dBm, where a bare `as i8` wraps (−130 → +126) and reports a strong link for
        // the weakest one. Clamp so the reported margin is monotonic at the edges.
        payload[4] = rssi.clamp(i8::MIN as i16, i8::MAX as i16) as i8 as u8;
        // Flags byte (wire v3): advertise a queued downlink so a sleeping sender holds an RX
        // window open after this uplink. Appending is interop-safe — the ACK acceptance rule is
        // `len >= 4` (pinned in tower-net-core), so pre-v3 peers simply ignore the byte.
        payload[5] = match self.peer_slot(dest) {
            Some(i) if self.peers[i].as_ref().unwrap().pending => ack_flags::PENDING,
            _ => 0,
        };
        let hdr = Header {
            frame_type: FrameType::Ack,
            flags: 0,
            src: self.addr,
            dest,
            counter: ack_counter,
            bulk_index: None,
        };
        let key = self.key_for(dest);
        let mut ack = [0u8; MAX_FRAME];
        if let Ok(n) = frame::seal_frame(&mut self.ccm, &key, &hdr, &payload, &mut ack) {
            self.advance_tx_counter(); // ACK consumes a counter (its own, docs/radio.md)
            // ACK airtime is charged to the EU 1 % governor ONLY in Duty mode. AFA/FHSS manage
            // spectrum access on their own path and are not under the 1 % cap, so metering the
            // ACK against `self.duty` there (which stays the EU governor) let a drained bucket
            // silently kill confirmed delivery in a mode that has no duty limit. In Duty mode we
            // still gate + consume budget; if over budget we skip the ACK and the sender
            // retransmits (recv re-ACKs by (src, counter)), so no ACK cache is needed.
            let may_tx = match self.access {
                Access::Duty => self.duty.try_tx(duty::frame_toa_ms(n)),
                _ => true,
            };
            if may_tx {
                let _ = self.radio.tx(&ack[..n], false, TX_TIMEOUT).await;
            }
        }
    }

    /// Put the radio into its SLEEP state between transfers (registers retained; wake
    /// with [`wake`](Self::wake) before the next TX/RX). The battery-node pattern —
    /// see `examples/radio_sleep.rs`; `Net` never sleeps the radio on its own.
    pub async fn sleep(&mut self) -> Result<(), RadioError> {
        self.radio.to_sleep().await
    }

    /// Wake the radio from SLEEP back to READY (no re-configuration needed).
    pub async fn wake(&mut self) -> Result<(), RadioError> {
        self.radio.to_ready().await
    }

    /// Sample the ambient RSSI of the current receive channel, in dBm — the
    /// gateway's channel-diagnostics primitive (`RadioStat::Channel`).
    pub async fn rssi_sample(&mut self) -> Result<i16, RadioError> {
        Ok(config::rssi_to_dbm(self.radio.rssi_sample().await?))
    }

    /// Advance the internal xorshift32 PRNG and return the new 32-bit state. Backs the
    /// retransmit backoff and the pairing challenge nonce. **Not cryptographic** (seeded
    /// deterministically from `addr`): enough to de-correlate collided retransmits and to make
    /// a pairing confirm from a *prior* session fail to validate (the challenge advances each
    /// session), but not a source of secret randomness.
    pub(crate) fn rand_u32(&mut self) -> u32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        x
    }

    /// xorshift32 backoff in [0, MAX_BACKOFF_MS).
    fn backoff_ms(&mut self) -> u32 {
        self.rand_u32() % MAX_BACKOFF_MS
    }
}

/// Read a little-endian u32 from a Kv key, if present and exactly 4 bytes.
fn read_u32(kv: Scoped, local: u8) -> Option<u32> {
    let mut b = [0u8; 4];
    match kv.get_bytes(local, &mut b) {
        Ok(Some(4)) => Some(u32::from_le_bytes(b)),
        _ => None,
    }
}

/// A per-peer replay-lane record: the peer `addr` it belongs to, then its `last_seen`
/// counter (both little-endian). Persisting the addr lets [`add_peer`](Net::add_peer)
/// restore a lane only for the peer that owns it, not whichever peer inherits the slot.
fn lane_record(addr: u32, last_seen: u32) -> [u8; 8] {
    let mut b = [0u8; 8];
    b[..4].copy_from_slice(&addr.to_le_bytes());
    b[4..].copy_from_slice(&last_seen.to_le_bytes());
    b
}

/// Read a `(addr, last_seen)` replay-lane record, if present and exactly 8 bytes.
fn read_u32_pair(kv: Scoped, local: u8) -> Option<(u32, u32)> {
    let mut b = [0u8; 8];
    match kv.get_bytes(local, &mut b) {
        Ok(Some(8)) => Some((
            u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            u32::from_le_bytes([b[4], b[5], b[6], b[7]]),
        )),
        _ => None,
    }
}
