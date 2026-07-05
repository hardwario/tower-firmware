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
//! Keys are per-peer: [`Net::add_peer`] binds an `id` to its own AES key and
//! replay lane (star ≤32 / P2P ≤8, docs/radio.md); any unregistered peer falls back to the
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

/// TX-counter reserve block: persist the watermark only once per `RESERVE`
/// transfers, and on boot resume *at* the watermark (> any value actually sent,
/// so a counter is never reused; ≤ one block is skipped per reboot, docs/radio.md).
const RESERVE: u32 = 1024;
/// Receiver last-seen lazy-persist period: the replay window across a reboot is
/// ≤ `P` transfers (docs/radio.md).
const P: u32 = 32;
/// EEPROM local keys (within `NS_NET`) for the persisted counter state.
const KEY_WATERMARK: u8 = 0x00;
const KEY_LASTSEEN: u8 = 0x01;
/// FHSS gateway epoch (boot-id): a master bumps + persists it each session so its beacons are
/// strictly newer than any previous session's, and a node refuses to anchor to an older epoch —
/// defeating a replayed beacon capture. `0x02` is free (peer lanes start at `KEY_LASTSEEN_BASE`).
const KEY_FHSS_EPOCH: u8 = 0x02;

/// Peer-table capacity. A gateway in a star holds up to 32 nodes; a P2P device
/// holds up to 8 peers (docs/radio.md). One table size covers both — the topology
/// is a usage policy, not a different type. Sized at 32 (not 64) so the table
/// (`[Option<Peer>; MAX_PEERS]`, ~32 B/slot) costs ~1 KB, not ~2 KB, of the Net
/// future in `.bss` — RAM that (with flip-link) is stack headroom on this 20 KB part.
pub const MAX_PEERS: usize = 32;
/// Local base (within `NS_NET`) for per-peer last-seen lanes (slot `i` → `KEY_LASTSEEN_BASE + i`,
/// `i < MAX_PEERS = 32`, so lanes occupy locals `0x10..=0x2F`).
const KEY_LASTSEEN_BASE: u8 = 0x10;

/// A registered peer: its ID, per-peer AES key, and replay state (docs/radio.md).
#[derive(Clone, Copy)]
struct Peer {
    id: u32,
    key: [u8; 16],
    last_seen: u32,
    accepts: u32,
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
    pub rssi_dbm: i16,
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
    /// This device's 32-bit ID (rides in the clear header).
    pub my_id: u32,
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
    my_id: u32,
    /// Default key for unregistered peers (see [`NetConfig::key`]).
    default_key: [u8; 16],
    /// Per-peer (id, key, last-seen) table; a registered peer overrides the
    /// default key and gets its own replay lane (docs/radio.md).
    peers: [Option<Peer>; MAX_PEERS],
    /// Replay last-seen for senders not in the peer table (the single-link lane).
    default_last_seen: u32,
    /// Accepted-transfer count on the default lane since its last persist.
    default_accepts: u32,
    /// Monotonic TX counter, advanced by one per transfer (docs/radio.md).
    tx_counter: u32,
    /// Highest reserved (persisted) counter value; `tx_counter < reserve_limit`
    /// holds as an invariant only while [`tx_locked`](Self::tx_locked) is false —
    /// the counter is never used at or past the last *durably* persisted watermark.
    reserve_limit: u32,
    /// Set once a reserve-watermark persist fails: TX is refused (fail closed) so a
    /// counter past the last durable watermark can never go on air (which after a
    /// reboot resuming at the stale watermark would reuse a CCM nonce). See
    /// [`advance_tx_counter`](Self::advance_tx_counter) and `RadioError::NonceLocked`.
    tx_locked: bool,
    /// EEPROM-backed counter persistence (shared handle).
    kv: Nv,
    /// EU duty-cycle governor (airtime budget for all TX).
    duty: DutyGovernor,
    /// Simple LCG state for the retransmit backoff (seeded from my_id).
    rng: u32,
    /// Active spectrum-access mode (Duty default; AFA/FHSS switch at runtime).
    access: Access,
    /// EU LBT+AFA state (inert unless `access == Afa`).
    afa: afa::Afa,
    /// US FHSS state (inert unless `access == Fhss`).
    fhss: fhss::Fhss,
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

        // Reserve-ahead TX counter: resume *at* the persisted watermark (1 on the
        // very first boot, since 0 = "never sent"), then reserve the next block.
        let nkv = kv.scope(NS_NET); // our namespaced view; the raw `kv` is kept for `Net::kv()`
        let stored_watermark = read_u32(nkv, KEY_WATERMARK);
        let stored_last_seen = read_u32(nkv, KEY_LASTSEEN);
        let last_seen = stored_last_seen.unwrap_or(0);
        let resume = stored_watermark.unwrap_or(1).max(1);
        let reserve_limit = resume.wrapping_add(RESERVE);

        // A store that shows prior use (a persisted last-seen) but whose watermark is now
        // unreadable has almost certainly lost the watermark record to EEPROM corruption
        // (`scan_half` stops at the first bad record, orphaning everything after it — and the
        // watermark, rewritten every RESERVE transfers, tends to sit late in the log). Resuming
        // the TX counter at 1 under the unchanged key would then reuse every CCM nonce 1..old.
        // Fail **closed** WITHOUT rewriting a fresh low watermark (that would poison a later boot
        // into resuming low and reusing nonces). The device can still receive; sending needs a
        // re-key / factory reset. A genuinely virgin store (no watermark AND no last-seen)
        // legitimately starts at 1.
        let watermark_lost = stored_watermark.is_none() && stored_last_seen.is_some();
        let tx_locked = if watermark_lost {
            true
        } else {
            // Persist the reserve watermark AND verify it read back. TX is only nonce-safe while a
            // watermark strictly greater than any counter we will ever send is durably stored. If
            // the write fails or does not read back (EEPROM full/faulted), start TX **locked**:
            // the device can still receive, but sending would risk resuming at a stale watermark
            // after a reboot and reusing a CCM nonce. (Fail closed — see the `tx_locked` field.)
            let persisted = nkv.set_bytes(KEY_WATERMARK, &reserve_limit.to_le_bytes()).is_ok();
            !(persisted && read_u32(nkv, KEY_WATERMARK) == Some(reserve_limit))
        };

        // Duty policy follows the band: EU 1 %, US 915 unrestricted (docs/radio.md).
        let duty = match cfg.band {
            Band::Eu868 => DutyGovernor::eu(),
            Band::Us915 => DutyGovernor::us915(),
        };

        Ok(Self {
            radio,
            ccm: Ccm::new(),
            my_id: cfg.my_id,
            default_key: cfg.key,
            peers: [None; MAX_PEERS],
            default_last_seen: last_seen,
            default_accepts: 0,
            tx_counter: resume,
            reserve_limit,
            tx_locked,
            kv,
            duty,
            rng: cfg.my_id | 1,
            access: Access::Duty,
            afa: afa::Afa::disabled(),
            fhss: fhss::Fhss::disabled(),
        })
    }

    /// The active spectrum-access mode ([`Access::Duty`] unless AFA/FHSS was enabled).
    #[must_use]
    pub fn access(&self) -> Access {
        self.access
    }

    /// This device's ID.
    #[must_use]
    pub fn id(&self) -> u32 {
        self.my_id
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
        self.tx_counter
    }

    /// Current persisted reserve watermark.
    #[must_use]
    pub fn reserve_watermark(&self) -> u32 {
        self.reserve_limit
    }

    /// Current last-seen counter on the default lane (single-link diagnostics).
    #[must_use]
    pub fn last_seen(&self) -> u32 {
        self.default_last_seen
    }

    /// Register (or re-key) a peer: an explicit `id` → per-peer `key` binding with
    /// its own replay lane. The peer's persisted last-seen is restored. Returns
    /// `false` only if the table is full (and the id is new) — check the return in
    /// production code. Up to [`MAX_PEERS`] peers (star ≤32 / P2P ≤8 by policy, docs/radio.md).
    pub fn add_peer(&mut self, id: u32, key: &[u8; 16]) -> bool {
        if let Some(i) = self.peer_slot(id) {
            self.peers[i].as_mut().unwrap().key = *key; // re-key in place
            return true;
        }
        for i in 0..MAX_PEERS {
            if self.peers[i].is_none() {
                let nkv = self.kv.scope(NS_NET);
                let local = KEY_LASTSEEN_BASE + i as u8;
                // Replay lanes are persisted per-peer-*id*, not per-slot. A slot is assigned by
                // registration order, so a freed slot can be inherited by a different peer; only
                // restore the stored last-seen if the record was written for THIS id. Restoring a
                // stale value across peers would either reopen the replay window (inherited value
                // lower than the peer's real counter) or dead-lock the link (higher). On id
                // mismatch or no record, start the lane fresh at 0 and immediately bind (id, 0)
                // so the slot now belongs to this peer.
                let last_seen = match read_u32_pair(nkv, local) {
                    Some((stored_id, seen)) if stored_id == id => seen,
                    _ => {
                        let _ = nkv.set_bytes(local, &lane_record(id, 0));
                        0
                    }
                };
                self.peers[i] = Some(Peer {
                    id,
                    key: *key,
                    last_seen,
                    accepts: 0,
                });
                return true;
            }
        }
        false
    }

    /// Remove a peer. Returns whether it was present. (Its persisted last-seen record is
    /// left in EEPROM tagged with its id; re-adding the *same* peer — even into a different
    /// slot — resumes its replay window, while a different peer inheriting the slot starts
    /// fresh at 0, since restore is now id-matched. See [`add_peer`](Self::add_peer).)
    pub fn remove_peer(&mut self, id: u32) -> bool {
        if let Some(i) = self.peer_slot(id) {
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

    /// Last-seen counter for a registered peer (`None` if not registered).
    #[must_use]
    pub fn peer_last_seen(&self, id: u32) -> Option<u32> {
        self.peer_slot(id)
            .map(|i| self.peers[i].as_ref().unwrap().last_seen)
    }

    /// Table slot holding `id`, if registered.
    fn peer_slot(&self, id: u32) -> Option<usize> {
        self.peers
            .iter()
            .position(|p| matches!(p, Some(pe) if pe.id == id))
    }

    /// AES key for `id`: the peer's key if registered, else the default key.
    fn key_for(&self, id: u32) -> [u8; 16] {
        match self.peer_slot(id) {
            Some(i) => self.peers[i].as_ref().unwrap().key,
            None => self.default_key,
        }
    }

    /// Last-seen for `src`'s replay lane (peer lane if registered, else default).
    fn lane_last_seen(&self, src: u32) -> u32 {
        match self.peer_slot(src) {
            Some(i) => self.peers[i].as_ref().unwrap().last_seen,
            None => self.default_last_seen,
        }
    }

    /// Record acceptance of `counter` from `src`: advance that lane's last-seen
    /// and lazy-persist every `P` accepts (replay window ≤ P across a reboot, docs/radio.md).
    fn lane_accept(&mut self, src: u32, counter: u32) {
        match self.peer_slot(src) {
            Some(i) => {
                let p = self.peers[i].as_mut().unwrap();
                p.last_seen = counter;
                p.accepts = p.accepts.wrapping_add(1);
                if p.accepts.is_multiple_of(P) {
                    let id = p.id;
                    let _ = self
                        .kv
                        .scope(NS_NET)
                        .set_bytes(KEY_LASTSEEN_BASE + i as u8, &lane_record(id, counter));
                }
            }
            None => {
                self.default_last_seen = counter;
                self.default_accepts = self.default_accepts.wrapping_add(1);
                if self.default_accepts.is_multiple_of(P) {
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
    /// **Saturating, not wrapping** — the counter is the CCM nonce input, so it must never
    /// wrap back to a reused value. At the 2³²−1 ceiling (≈136 yr at 1 Hz — practically
    /// unreachable) it **locks TX** (`tx_locked = true`): every subsequent send fails closed
    /// rather than transmitting another frame under the pinned `u32::MAX` nonce. A reused
    /// `(key, nonce)` is a CCM confidentiality *and* integrity break (keystream + MAC reuse),
    /// not merely a delivery failure the replay rule would reject — so we must stop emitting,
    /// not rely on the peer to drop it. Re-key well before then.
    fn advance_tx_counter(&mut self) {
        self.tx_counter = self.tx_counter.saturating_add(1);
        if self.tx_counter == u32::MAX {
            // Ceiling reached: fail closed. Do not emit any further frame (the next send would
            // reuse the pinned MAX nonce) and stop churning the reserve watermark.
            self.tx_locked = true;
            return;
        }
        if self.tx_counter >= self.reserve_limit {
            // Extend the reservation, but only trust it once the write lands. If the persist
            // fails we must NOT advance `reserve_limit`: doing so would let us keep emitting
            // counters past the last *durable* watermark, and a reboot that then resumes at the
            // stale watermark would reuse those CCM nonces. Instead lock TX (fail closed) — the
            // guard on every send path refuses to transmit while locked.
            let next = self.reserve_limit.saturating_add(RESERVE);
            match self
                .kv
                .scope(NS_NET)
                .set_bytes(KEY_WATERMARK, &next.to_le_bytes())
            {
                Ok(()) => self.reserve_limit = next,
                Err(_) => self.tx_locked = true,
            }
        }
    }

    /// Send `data` to `dest`. Confirmed sends open an ACK window and retransmit
    /// the byte-identical frame up to `reps` times; unconfirmed sends transmit
    /// once. The transfer consumes exactly one TX counter value (docs/radio.md).
    pub async fn send(&mut self, dest: u32, data: &[u8], confirmed: bool, reps: u8) -> SendResult {
        if self.tx_locked {
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
        let counter = self.tx_counter;
        let hdr = Header {
            frame_type: FrameType::Data,
            flags: if confirmed { flags::CONFIRMED } else { 0 },
            src: self.my_id,
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
        // The counter is consumed whether or not delivery succeeded (the frames
        // went out under this nonce); retransmits reused it intentionally.
        self.advance_tx_counter();
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
        if peek.dest != self.my_id {
            return None; // not for us
        }
        let key = self.key_for(peek.src);

        // CCM-verify first (authenticates the header incl. counter), then decide.
        let (hdr, range) = frame::open_frame(&mut self.ccm, &key, &mut buf[..len]).ok()?;

        let last_seen = self.lane_last_seen(hdr.src);
        if hdr.counter > last_seen {
            // Fresh — accept, advance the sender's lane, ACK if requested.
            self.lane_accept(hdr.src, hdr.counter);
            let confirmed = hdr.flags & flags::CONFIRMED != 0;
            if confirmed {
                self.send_ack(hdr.src, hdr.counter, q.rssi_dbm).await;
            }
            let plen = range.end - range.start;
            let mut out = [0u8; MAX_PAYLOAD];
            out[..plen].copy_from_slice(&buf[range]);
            Some(Received {
                src: hdr.src,
                counter: hdr.counter,
                rssi_dbm: q.rssi_dbm,
                confirmed,
                len: plen,
                buf: out,
            })
        } else if hdr.counter == last_seen {
            // Benign retransmit: this peer's most-recently-accepted counter is being resent
            // because its ACK was lost. Re-ACK deterministically (identified by this src's own
            // lane last-seen), do NOT re-deliver. Keying the re-ACK by (src, counter) rather than
            // a single global cache means interleaved senders in a star can't evict each other's
            // pending ACK — the old single-slot cache reported false NotDelivered + app-level
            // duplicates under that race (see docs/radio.md). Only confirmed frames were ACKed,
            // so only re-ACK confirmed ones.
            if hdr.flags & flags::CONFIRMED != 0 {
                self.send_ack(hdr.src, hdr.counter, q.rssi_dbm).await;
            }
            None
        } else {
            // counter < last-seen → replay; drop silently (replay state untouched).
            None
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
    /// `fhss_rx_beacon`: keep listening until the deadline.
    async fn await_ack(&mut self, dest: u32, counter: u32) -> bool {
        let key = self.key_for(dest);
        let deadline = Instant::now() + ACK_WINDOW;
        let mut buf = [0u8; MAX_FRAME];
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.as_ticks() == 0 {
                return false; // window elapsed
            }
            let Ok((len, _)) = self.radio.rx(&mut buf, remaining).await else {
                return false; // rx timed out over the remaining window (or a radio error)
            };
            // A foreign / undecodable frame is NOT a failed ACK — skip it and keep waiting.
            let Ok((hdr, range)) = frame::open_frame(&mut self.ccm, &key, &mut buf[..len]) else {
                continue;
            };
            if hdr.frame_type != FrameType::Ack || hdr.src != dest || hdr.dest != self.my_id {
                continue;
            }
            // ACK payload: acked counter (4 LE) + rssi (1). A valid ACK for a *different* counter
            // (a stale/duplicate ACK) also isn't ours — keep waiting for the right one.
            let pl = &buf[range];
            if pl.len() >= 4 && u32::from_le_bytes([pl[0], pl[1], pl[2], pl[3]]) == counter {
                return true;
            }
        }
    }

    /// Build, cache and transmit an ACK for a received confirmed frame. The ACK
    /// uses the ACKer's *own* fresh counter (docs/radio.md); the acknowledged counter rides
    /// in the payload.
    async fn send_ack(&mut self, dest: u32, acked: u32, rssi_dbm: i16) {
        if self.tx_locked {
            return; // fail closed: can't safely allocate a counter (nonce safety); sender retries
        }
        // Let the sender finish its TX→RX turnaround before we transmit.
        Timer::after(ACK_TURNAROUND).await;
        let ack_counter = self.tx_counter;
        let mut payload = [0u8; 5];
        payload[..4].copy_from_slice(&acked.to_le_bytes());
        // Clamp to i8 range before packing: rssi_dbm is i16 and the SPIRIT1 noise floor reaches
        // below −128 dBm, where a bare `as i8` wraps (−130 → +126) and reports a strong link for
        // the weakest one. Clamp so the reported margin is monotonic at the edges.
        payload[4] = rssi_dbm.clamp(i8::MIN as i16, i8::MAX as i16) as i8 as u8;
        let hdr = Header {
            frame_type: FrameType::Ack,
            flags: 0,
            src: self.my_id,
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

    /// Advance the internal xorshift32 PRNG and return the new 32-bit state. Backs the
    /// retransmit backoff and the pairing challenge nonce. **Not cryptographic** (seeded
    /// deterministically from `my_id`): enough to de-correlate collided retransmits and to make
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

/// A per-peer replay-lane record: the peer `id` it belongs to, then its `last_seen`
/// counter (both little-endian). Persisting the id lets [`add_peer`](Net::add_peer)
/// restore a lane only for the peer that owns it, not whichever peer inherits the slot.
fn lane_record(id: u32, last_seen: u32) -> [u8; 8] {
    let mut b = [0u8; 8];
    b[..4].copy_from_slice(&id.to_le_bytes());
    b[4..].copy_from_slice(&last_seen.to_le_bytes());
    b
}

/// Read a `(id, last_seen)` replay-lane record, if present and exactly 8 bytes.
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
