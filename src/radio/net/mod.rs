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
//! replay lane (star ≤64 / P2P ≤8, docs/radio.md); any unregistered peer falls back to the
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

use embassy_time::{Duration, Timer};

use super::ccm::Ccm;
use super::config::{self, Band, RfConfig};
use super::device::{RadioError, Spirit1};
use super::duty::{self, DutyGovernor};
use super::frame::{self, FrameType, Header, MAX_FRAME, MAX_PAYLOAD, flags};
use crate::storage::Kv;

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
/// EEPROM key-value keys for the persisted counter state.
const KEY_WATERMARK: u16 = 0x5201;
const KEY_LASTSEEN: u16 = 0x5202;

/// Peer-table capacity. A gateway in a star holds up to 64 nodes; a P2P device
/// holds up to 8 peers (docs/radio.md). One table size covers both — the topology
/// is a usage policy, not a different type.
pub const MAX_PEERS: usize = 64;
/// Base Kv key for per-peer last-seen persistence (slot `i` → `KEY_LASTSEEN_BASE + i`).
const KEY_LASTSEEN_BASE: u16 = 0x5300;

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
    /// Highest reserved (persisted) counter value; `tx_counter < reserve_limit`.
    reserve_limit: u32,
    /// EEPROM-backed counter persistence.
    kv: Kv<'static>,
    /// EU duty-cycle governor (airtime budget for all TX).
    duty: DutyGovernor,
    /// Cached ACK bytes for the most recent confirmed RX, to re-send on a
    /// byte-identical retransmit (docs/radio.md).
    cached_ack: [u8; MAX_FRAME],
    cached_ack_len: usize,
    /// The (src, acked counter) the cached ACK corresponds to (0 = none cached).
    cached_ack_for: u32,
    cached_ack_src: u32,
    /// Simple LCG state for the retransmit backoff (seeded from my_id).
    rng: u32,
    /// Active spectrum-access mode (Duty default; AFA/FHSS switch at runtime).
    access: Access,
    /// EU LBT+AFA state (inert unless `access == Afa`).
    afa: afa::Afa,
    /// US FHSS state (inert unless `access == Fhss`).
    fhss: fhss::Fhss,
    /// FOTA advertise, gateway side: when set, our auto-ACKs carry `DOWNLINK_PENDING`
    /// to tell the node an update is waiting (docs/fota.md). Rides existing ACKs, so
    /// it costs no extra airtime. Set via [`set_downlink_pending`](Net::set_downlink_pending).
    downlink_pending: bool,
    /// FOTA advertise, node side: set when an ACK we received carried `DOWNLINK_PENDING`;
    /// read-and-cleared via [`take_downlink_pending`](Net::take_downlink_pending).
    downlink_pending_rx: bool,
}

impl Net {
    /// Bring the radio up, apply the RF config, and initialise counters from
    /// EEPROM (`kv`): resume the TX counter at the persisted reserve watermark and
    /// reserve the next block, and restore the default-lane last-seen (per-peer
    /// lanes are restored when their peer is registered via [`add_peer`](Self::add_peer)).
    pub async fn new(mut radio: Spirit1, mut kv: Kv<'static>, cfg: NetConfig) -> Result<Self, RadioError> {
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
        let resume = read_u32(&kv, KEY_WATERMARK).unwrap_or(1).max(1);
        let reserve_limit = resume.wrapping_add(RESERVE);
        let _ = kv.set_bytes(KEY_WATERMARK, &reserve_limit.to_le_bytes());
        let last_seen = read_u32(&kv, KEY_LASTSEEN).unwrap_or(0);

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
            kv,
            duty,
            cached_ack: [0; MAX_FRAME],
            cached_ack_len: 0,
            cached_ack_for: 0,
            cached_ack_src: 0,
            rng: cfg.my_id | 1,
            access: Access::Duty,
            afa: afa::Afa::disabled(),
            fhss: fhss::Fhss::disabled(),
            downlink_pending: false,
            downlink_pending_rx: false,
        })
    }

    /// FOTA advertise (gateway): make subsequent auto-ACKs carry `DOWNLINK_PENDING`, the
    /// "an update is waiting for you" signal (docs/fota.md). Clear it once the node has
    /// pulled. The bit rides existing ACKs, so advertising costs no extra airtime.
    pub fn set_downlink_pending(&mut self, pending: bool) {
        self.downlink_pending = pending;
    }

    /// FOTA advertise (node): whether the most recent ACK we received carried
    /// `DOWNLINK_PENDING` (read-and-cleared). Poll it after a confirmed
    /// [`send`](Self::send): if `true`, the gateway has an update — pull it when
    /// idle/scheduled (the node controls *when*, good for battery/duty).
    pub fn take_downlink_pending(&mut self) -> bool {
        core::mem::take(&mut self.downlink_pending_rx)
    }

    /// Consume the network layer and reclaim its [`Kv`] (and thus the `Flash` it owns), for an
    /// app that needs program-flash access once the radio is done. Drops the radio + peer table
    /// — call only when finished transceiving.
    ///
    /// The as-built FOTA flow does **not** need this: [`pull_update`](crate::fota::pull_update)
    /// stages the image through `Net`'s own flash, and the **bootloader** (not the app) arms the
    /// swap after a reset — so the node never reclaims the flash to drive an updater itself.
    /// Kept as a general resource-reclaim accessor.
    pub fn into_kv(self) -> Kv<'static> {
        self.kv
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

    /// Mutable access to the EEPROM key-value store the network layer owns, for
    /// application-level persistence (e.g. soak tallies). The network layer uses
    /// keys `0x5201`, `0x5202` and `0x5300+slot`; pick others to avoid clashes.
    pub fn kv(&mut self) -> &mut Kv<'static> {
        &mut self.kv
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
    /// production code. Up to [`MAX_PEERS`] peers (star ≤64 / P2P ≤8 by policy, docs/radio.md).
    pub fn add_peer(&mut self, id: u32, key: &[u8; 16]) -> bool {
        if let Some(i) = self.peer_slot(id) {
            self.peers[i].as_mut().unwrap().key = *key; // re-key in place
            return true;
        }
        for i in 0..MAX_PEERS {
            if self.peers[i].is_none() {
                let last_seen = read_u32(&self.kv, KEY_LASTSEEN_BASE + i as u16).unwrap_or(0);
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

    /// Remove a peer. Returns whether it was present. (Its persisted last-seen is
    /// left in EEPROM; re-adding the peer resumes the replay window.)
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
                    let _ = self
                        .kv
                        .set_bytes(KEY_LASTSEEN_BASE + i as u16, &counter.to_le_bytes());
                }
            }
            None => {
                self.default_last_seen = counter;
                self.default_accepts = self.default_accepts.wrapping_add(1);
                if self.default_accepts.is_multiple_of(P) {
                    let _ = self.kv.set_bytes(KEY_LASTSEEN, &counter.to_le_bytes());
                }
            }
        }
    }

    /// Advance the TX counter, re-reserving + persisting the next block when the
    /// current reserve is exhausted (the only TX-counter persistence path).
    ///
    /// **Saturating, not wrapping** — the counter is the CCM nonce input, so it must never
    /// wrap back to a reused value. At the 2³²−1 ceiling (≈136 yr at 1 Hz — practically
    /// unreachable) it sticks at `u32::MAX`; the strict `counter > last_seen` replay rule then
    /// makes a peer reject every further frame as a replay, so the link fails **closed**
    /// rather than silently reusing a low nonce. Re-key well before then.
    fn advance_tx_counter(&mut self) {
        self.tx_counter = self.tx_counter.saturating_add(1);
        if self.tx_counter == u32::MAX {
            return; // ceiling reached: stop churning the reserve watermark (see above)
        }
        if self.tx_counter >= self.reserve_limit {
            self.reserve_limit = self.reserve_limit.saturating_add(RESERVE);
            let _ = self
                .kv
                .set_bytes(KEY_WATERMARK, &self.reserve_limit.to_le_bytes());
        }
    }

    /// Send `data` to `dest`. Confirmed sends open an ACK window and retransmit
    /// the byte-identical frame up to `reps` times; unconfirmed sends transmit
    /// once. The transfer consumes exactly one TX counter value (docs/radio.md).
    pub async fn send(&mut self, dest: u32, data: &[u8], confirmed: bool, reps: u8) -> SendResult {
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
        } else if hdr.counter == last_seen
            && self.cached_ack_for == hdr.counter
            && self.cached_ack_src == hdr.src
        {
            // Benign retransmit — re-send the cached identical ACK, do not re-deliver.
            let n = self.cached_ack_len;
            if n > 0 {
                let mut ack = [0u8; MAX_FRAME];
                ack[..n].copy_from_slice(&self.cached_ack[..n]);
                let _ = self.radio.tx(&ack[..n], false, TX_TIMEOUT).await;
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

    /// Wait `ACK_WINDOW` for an ACK from `dest` acknowledging `counter`.
    async fn await_ack(&mut self, dest: u32, counter: u32) -> bool {
        let key = self.key_for(dest);
        let mut buf = [0u8; MAX_FRAME];
        let Ok((len, _)) = self.radio.rx(&mut buf, ACK_WINDOW).await else {
            return false;
        };
        let Ok((hdr, range)) = frame::open_frame(&mut self.ccm, &key, &mut buf[..len]) else {
            return false;
        };
        if hdr.frame_type != FrameType::Ack || hdr.src != dest || hdr.dest != self.my_id {
            return false;
        }
        // A valid ACK may advertise a pending downlink (FOTA "update available", docs/fota.md).
        if hdr.flags & flags::DOWNLINK_PENDING != 0 {
            self.downlink_pending_rx = true;
        }
        // ACK payload: acked counter (4 LE) + rssi (1).
        let pl = &buf[range];
        pl.len() >= 4 && u32::from_le_bytes([pl[0], pl[1], pl[2], pl[3]]) == counter
    }

    /// Build, cache and transmit an ACK for a received confirmed frame. The ACK
    /// uses the ACKer's *own* fresh counter (docs/radio.md); the acknowledged counter rides
    /// in the payload.
    async fn send_ack(&mut self, dest: u32, acked: u32, rssi_dbm: i16) {
        // Let the sender finish its TX→RX turnaround before we transmit.
        Timer::after(ACK_TURNAROUND).await;
        let ack_counter = self.tx_counter;
        let mut payload = [0u8; 5];
        payload[..4].copy_from_slice(&acked.to_le_bytes());
        payload[4] = rssi_dbm as i8 as u8;
        let hdr = Header {
            frame_type: FrameType::Ack,
            // Advertise a pending FOTA downlink if the gateway has one (docs/fota.md).
            flags: if self.downlink_pending {
                flags::DOWNLINK_PENDING
            } else {
                0
            },
            src: self.my_id,
            dest,
            counter: ack_counter,
            bulk_index: None,
        };
        let key = self.key_for(dest);
        let mut ack = [0u8; MAX_FRAME];
        if let Ok(n) = frame::seal_frame(&mut self.ccm, &key, &hdr, &payload, &mut ack) {
            // ACK airtime is governed too (docs/radio.md); skip it if over budget — the
            // sender will retransmit. Cache it regardless for retransmit dedup.
            self.advance_tx_counter(); // ACK consumes a counter (its own, docs/radio.md)
            self.cached_ack[..n].copy_from_slice(&ack[..n]);
            self.cached_ack_len = n;
            self.cached_ack_for = acked;
            self.cached_ack_src = dest;
            if self.duty.try_tx(duty::frame_toa_ms(n)) {
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
fn read_u32(kv: &Kv<'static>, key: u16) -> Option<u32> {
    let mut b = [0u8; 4];
    match kv.get_bytes(key, &mut b) {
        Ok(Some(4)) => Some(u32::from_le_bytes(b)),
        _ => None,
    }
}
