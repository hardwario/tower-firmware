//! Network layer: confirmed delivery, ACK, retransmit and replay protection
//! over the secured frame codec (RADIO.md §7).
//!
//! `Net` owns the radio + CCM and serializes one transfer at a time (§4). A
//! *node* `send(confirmed)` transmits a DATA frame then opens a 200 ms ACK
//! window, retransmitting the byte-identical frame on timeout (random 0–100 ms
//! backoff, 1–10 reps). A *receiver* `recv()` authenticates the frame, applies
//! the counter/replay rule, and auto-ACKs a confirmed frame — caching the ACK so
//! a retransmit re-sends the identical bytes without re-delivering.
//!
//! Keys are per-peer: [`Net::add_peer`] binds an `id` to its own AES key and
//! replay lane (star ≤64 / P2P ≤8, §7.2); any unregistered peer falls back to the
//! [`NetConfig::key`] default lane (the single-link case). The TX counter and each
//! lane's last-seen are EEPROM-persisted (reserve-ahead watermark / lazy-persist, §7.4).

#![allow(dead_code)]

use embassy_time::{Duration, Instant, Timer};

use super::ccm::Ccm;
use super::config::{self, Band, RfConfig};
use super::device::{RadioError, Spirit1};
use super::duty::{self, DutyGovernor};
use super::frame::{self, FrameType, Header, MAX_FRAME, MAX_PAYLOAD, flags};
use crate::storage::Kv;

/// ACK window the sender waits for an acknowledgement (§7.3). The measured ACK
/// round-trip is ~35 ms (turnaround + ACK ToA + RX set-up), so 200 ms is ample.
const ACK_WINDOW: Duration = Duration::from_millis(200);
/// Turnaround the receiver waits before sending the ACK, so the sender has
/// finished switching TX→RX and is listening (the ACK window is 200 ms, so
/// there's ample room). Without this the ACK preamble races the sender's RX
/// set-up (to_ready + flush + mask + strobe, several SPI ops) and is missed.
const ACK_TURNAROUND: Duration = Duration::from_millis(20);
/// Per-TX timeout (CSMA + ToA budget); generous for a ≤96 B frame at 19.2 kbps.
const TX_TIMEOUT: Duration = Duration::from_millis(120);
/// Default confirmed-delivery repetitions.
pub const DEFAULT_REPS: u8 = 3;
/// Max inter-rep backoff (ms), randomised to de-sync collided senders.
const MAX_BACKOFF_MS: u32 = 100;

/// TX-counter reserve block: persist the watermark only once per `RESERVE`
/// transfers, and on boot resume *at* the watermark (> any value actually sent,
/// so a counter is never reused; ≤ one block is skipped per reboot, §7.4).
const RESERVE: u32 = 1024;
/// Receiver last-seen lazy-persist period: the replay window across a reboot is
/// ≤ `P` transfers (§7.4).
const P: u32 = 32;
/// EEPROM key-value keys for the persisted counter state.
const KEY_WATERMARK: u16 = 0x5201;
const KEY_LASTSEEN: u16 = 0x5202;

/// Bulk chunk size (RADIO.md §7.5).
pub const BULK_CHUNK: usize = 64;
/// Bulk session idle timeout: the sender frees the transfer after this with no progress.
const BULK_IDLE: Duration = Duration::from_secs(30);
/// How long the requester waits for a BULK_DATA after a BULK_REQ.
const BULK_RESP_WINDOW: Duration = Duration::from_millis(250);
/// Requester BULK_REQ repetitions per chunk before giving up.
const BULK_REQ_REPS: u8 = 6;
/// One serve-loop receive slice on the sender side.
const BULK_SERVE_SLICE: Duration = Duration::from_millis(500);

/// Fixed, **publicly-known** OTA-pairing key (RADIO.md §7.6). It gives the JOIN
/// frames a uniform CCM format with integrity + in-session replay protection, but
/// NO confidentiality (a sniffer in range during the window recovers the
/// delivered per-node key) and NO mutual authentication. Mitigate with a short
/// window, proximity, reduced power, user-initiated pairing.
pub const PAIRING_KEY: [u8; 16] = *b"TOWER-PAIR-KEY!\0";
/// How long the joiner waits for a JOIN_RESP after a JOIN_REQ.
const JOIN_RESP_WINDOW: Duration = Duration::from_millis(300);
/// How long the host waits for a JOIN_CONFIRM after a JOIN_RESP.
const JOIN_CONFIRM_WINDOW: Duration = Duration::from_millis(300);

/// Peer-table capacity. A gateway in a star holds up to 64 nodes; a P2P device
/// holds up to 8 peers (RADIO.md §7.2). One table size covers both — the topology
/// is a usage policy, not a different type.
pub const MAX_PEERS: usize = 64;
/// Base Kv key for per-peer last-seen persistence (slot `i` → `KEY_LASTSEEN_BASE + i`).
const KEY_LASTSEEN_BASE: u16 = 0x5300;

/// A registered peer: its ID, per-peer AES key, and replay state (§7.2/§7.4).
#[derive(Clone, Copy)]
struct Peer {
    id: u32,
    key: [u8; 16],
    last_seen: u32,
    accepts: u32,
}

/// Outcome of a [`Net::send`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SendResult {
    /// Confirmed and ACKed (or unconfirmed and transmitted).
    Delivered,
    /// Confirmed but no ACK after all repetitions.
    NotDelivered,
    /// CSMA reported the channel busy.
    Busy,
    /// The duty governor refused the TX (would exceed the airtime budget).
    DutyLimited,
    /// A local error (bad length, radio fault).
    Error(RadioError),
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
    /// default key and gets its own replay lane (§7.2/§7.4).
    peers: [Option<Peer>; MAX_PEERS],
    /// Replay last-seen for senders not in the peer table (the single-link lane).
    default_last_seen: u32,
    /// Accepted-transfer count on the default lane since its last persist.
    default_accepts: u32,
    /// Monotonic TX counter, advanced by one per transfer (§6).
    tx_counter: u32,
    /// Highest reserved (persisted) counter value; `tx_counter < reserve_limit`.
    reserve_limit: u32,
    /// EEPROM-backed counter persistence.
    kv: Kv<'static>,
    /// EU duty-cycle governor (airtime budget for all TX).
    duty: DutyGovernor,
    /// Cached ACK bytes for the most recent confirmed RX, to re-send on a
    /// byte-identical retransmit (§7.3).
    cached_ack: [u8; MAX_FRAME],
    cached_ack_len: usize,
    /// The (src, acked counter) the cached ACK corresponds to (0 = none cached).
    cached_ack_for: u32,
    cached_ack_src: u32,
    /// Simple LCG state for the retransmit backoff (seeded from my_id).
    rng: u32,
}

impl Net {
    /// Bring the radio up, apply the RF config, and initialise counters from
    /// EEPROM (`kv`): resume the TX counter at the persisted reserve watermark
    /// and reserve the next block, and restore the per-peer last-seen.
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
            duty: DutyGovernor::eu(),
            cached_ack: [0; MAX_FRAME],
            cached_ack_len: 0,
            cached_ack_for: 0,
            cached_ack_src: 0,
            rng: cfg.my_id | 1,
        })
    }

    /// This device's ID.
    pub fn id(&self) -> u32 {
        self.my_id
    }

    /// Current live TX counter (for diagnostics / persistence demos).
    pub fn tx_counter(&self) -> u32 {
        self.tx_counter
    }

    /// Current persisted reserve watermark.
    pub fn reserve_watermark(&self) -> u32 {
        self.reserve_limit
    }

    /// Current last-seen counter on the default lane (single-link diagnostics).
    pub fn last_seen(&self) -> u32 {
        self.default_last_seen
    }

    /// Register (or re-key) a peer: an explicit `id` → per-peer `key` binding with
    /// its own replay lane. The peer's persisted last-seen is restored. Returns
    /// `false` only if the table is full (and the id is new). Up to
    /// [`MAX_PEERS`] peers (star ≤64 / P2P ≤8 by policy, §7.2).
    pub fn add_peer(&mut self, id: u32, key: &[u8; 16]) -> bool {
        if let Some(i) = self.peer_slot(id) {
            self.peers[i].as_mut().unwrap().key = *key; // re-key in place
            return true;
        }
        for i in 0..MAX_PEERS {
            if self.peers[i].is_none() {
                let last_seen = read_u32(&self.kv, KEY_LASTSEEN_BASE + i as u16).unwrap_or(0);
                self.peers[i] = Some(Peer { id, key: *key, last_seen, accepts: 0 });
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
    pub fn peer_count(&self) -> usize {
        self.peers.iter().filter(|p| p.is_some()).count()
    }

    /// Last-seen counter for a registered peer (`None` if not registered).
    pub fn peer_last_seen(&self, id: u32) -> Option<u32> {
        self.peer_slot(id).map(|i| self.peers[i].as_ref().unwrap().last_seen)
    }

    /// Table slot holding `id`, if registered.
    fn peer_slot(&self, id: u32) -> Option<usize> {
        self.peers.iter().position(|p| matches!(p, Some(pe) if pe.id == id))
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
    /// and lazy-persist every `P` accepts (replay window ≤ P across a reboot, §7.4).
    fn lane_accept(&mut self, src: u32, counter: u32) {
        match self.peer_slot(src) {
            Some(i) => {
                let p = self.peers[i].as_mut().unwrap();
                p.last_seen = counter;
                p.accepts = p.accepts.wrapping_add(1);
                if p.accepts % P == 0 {
                    let _ = self.kv.set_bytes(KEY_LASTSEEN_BASE + i as u16, &counter.to_le_bytes());
                }
            }
            None => {
                self.default_last_seen = counter;
                self.default_accepts = self.default_accepts.wrapping_add(1);
                if self.default_accepts % P == 0 {
                    let _ = self.kv.set_bytes(KEY_LASTSEEN, &counter.to_le_bytes());
                }
            }
        }
    }

    /// Advance the TX counter, re-reserving + persisting the next block when the
    /// current reserve is exhausted (the only TX-counter persistence path).
    fn advance_tx_counter(&mut self) {
        self.tx_counter = self.tx_counter.wrapping_add(1);
        if self.tx_counter >= self.reserve_limit {
            self.reserve_limit = self.reserve_limit.wrapping_add(RESERVE);
            let _ = self.kv.set_bytes(KEY_WATERMARK, &self.reserve_limit.to_le_bytes());
        }
    }

    /// Send `data` to `dest`. Confirmed sends open an ACK window and retransmit
    /// the byte-identical frame up to `reps` times; unconfirmed sends transmit
    /// once. The transfer consumes exactly one TX counter value (§6).
    pub async fn send(
        &mut self,
        dest: u32,
        data: &[u8],
        confirmed: bool,
        reps: u8,
    ) -> SendResult {
        if data.len() > MAX_PAYLOAD {
            return SendResult::Error(RadioError::TooLong); // MTU: use bulk for >74 B (§3)
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
                // Random 0–100 ms backoff before a retransmit (§7.3).
                Timer::after(Duration::from_millis(self.backoff_ms() as u64)).await;
            }
            // Duty governor: every TX (incl. retransmits) counts (§2.2).
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

    // --- Bulk transfer / downlink pull (§7.5): the requester pulls, the sender
    // serves. The session id is a counter reserved by the sender (distinct from
    // the announce frame's counter, so chunk-0's nonce never collides with the
    // announce). All BULK_DATA chunks share that session counter with a distinct
    // bulk_index, keeping their nonces unique. ---

    /// Serve `data` as a bulk transfer to `dest`: announce the length + session,
    /// then answer BULK_REQ(index) with BULK_DATA(index, ≤64 B) until the last
    /// chunk is pulled or the session idles out (30 s). Returns whether the last
    /// chunk was served.
    pub async fn bulk_serve(&mut self, dest: u32, data: &[u8]) -> bool {
        let key = self.key_for(dest);
        let n_chunks = data.len().div_ceil(BULK_CHUNK).max(1);
        let announce_counter = self.tx_counter;
        self.advance_tx_counter();
        let session = self.tx_counter; // reserved for all chunks; consumed at the end

        let mut ann = [0u8; 8];
        ann[..4].copy_from_slice(&(data.len() as u32).to_le_bytes());
        ann[4..8].copy_from_slice(&session.to_le_bytes());
        let ann_hdr = Header {
            frame_type: FrameType::Data,
            flags: flags::BULK_ANNOUNCE,
            src: self.my_id,
            dest,
            counter: announce_counter,
            bulk_index: None,
        };

        let mut got_req = false;
        let mut served_last = false;
        let mut last_progress = Instant::now();
        let mut rxbuf = [0u8; BULK_CHUNK];
        loop {
            if Instant::now().saturating_duration_since(last_progress)
                >= if served_last { Duration::from_secs(2) } else { BULK_IDLE }
            {
                break;
            }
            // Re-announce until the first request arrives (handles a missed announce).
            if !got_req {
                self.tx_frame(&key, &ann_hdr, &ann).await;
            }
            let Some((hdr, plen)) = self.rx_frame(&key, BULK_SERVE_SLICE, &mut rxbuf).await else {
                continue;
            };
            if hdr.frame_type != FrameType::BulkReq || hdr.src != dest || hdr.dest != self.my_id {
                continue;
            }
            if plen < 4 || u32::from_le_bytes([rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]]) != session {
                continue;
            }
            let k = hdr.bulk_index.unwrap_or(0) as usize;
            if k >= n_chunks {
                continue;
            }
            got_req = true;
            last_progress = Instant::now();
            let start = k * BULK_CHUNK;
            let end = (start + BULK_CHUNK).min(data.len());
            let last = k == n_chunks - 1;
            let dhdr = Header {
                frame_type: FrameType::BulkData,
                flags: if last { flags::LAST_CHUNK } else { 0 },
                src: self.my_id,
                dest,
                counter: session,
                bulk_index: Some(k as u32),
            };
            Timer::after(ACK_TURNAROUND).await; // let the requester switch to RX
            self.tx_frame(&key, &dhdr, &data[start..end]).await;
            if last {
                served_last = true;
            }
        }
        self.advance_tx_counter(); // consume the session counter
        served_last
    }

    /// Fetch a bulk transfer from `src` into `out`: receive the announcement,
    /// then pull each chunk with BULK_REQ (retransmitting on loss). Returns the
    /// total length received, or `None` on announce/chunk failure or `out` too small.
    pub async fn bulk_fetch(&mut self, src: u32, out: &mut [u8]) -> Option<usize> {
        let key = self.key_for(src);
        // Wait for the bulk-announce.
        let mut abuf = [0u8; 8];
        let (total_len, session) = loop {
            let (hdr, plen) = self.rx_frame(&key, Duration::from_secs(5), &mut abuf).await?;
            if hdr.frame_type == FrameType::Data
                && hdr.flags & flags::BULK_ANNOUNCE != 0
                && hdr.src == src
                && hdr.dest == self.my_id
                && plen >= 8
            {
                let len = u32::from_le_bytes([abuf[0], abuf[1], abuf[2], abuf[3]]) as usize;
                let s = u32::from_le_bytes([abuf[4], abuf[5], abuf[6], abuf[7]]);
                break (len, s);
            }
        };
        if total_len > out.len() {
            return None;
        }
        let n_chunks = total_len.div_ceil(BULK_CHUNK).max(1);

        let mut received = 0usize;
        for k in 0..n_chunks {
            let req_counter = self.tx_counter; // one counter per chunk; retransmits reuse it
            let req_hdr = Header {
                frame_type: FrameType::BulkReq,
                flags: 0,
                src: self.my_id,
                dest: src,
                counter: req_counter,
                bulk_index: Some(k as u32),
            };
            let req_payload = session.to_le_bytes();
            let mut got = false;
            for _ in 0..BULK_REQ_REPS {
                if !self.tx_frame(&key, &req_hdr, &req_payload).await {
                    continue;
                }
                let mut dbuf = [0u8; BULK_CHUNK];
                if let Some((dhdr, dlen)) = self.rx_frame(&key, BULK_RESP_WINDOW, &mut dbuf).await {
                    if dhdr.frame_type == FrameType::BulkData
                        && dhdr.src == src
                        && dhdr.dest == self.my_id
                        && dhdr.counter == session
                        && dhdr.bulk_index == Some(k as u32)
                    {
                        let off = k * BULK_CHUNK;
                        out[off..off + dlen].copy_from_slice(&dbuf[..dlen]);
                        received += dlen;
                        got = true;
                        break;
                    }
                }
            }
            self.advance_tx_counter();
            if !got {
                return None;
            }
        }
        Some(received)
    }

    // --- OTA pairing: 3-way JOIN under the fixed public PAIRING_KEY (§7.6).
    // Both sides commit only after the confirm; a lost confirm leaves the host's
    // window to time out and discard the tentative entry, and the joiner retries. ---

    /// Host: open a pairing window for `timeout`. On the first valid JOIN_REQ,
    /// assign `assign_id` + `assign_key`, send JOIN_RESP, and wait for the
    /// JOIN_CONFIRM. Returns the joiner's proposed ID on commit (caller installs
    /// the peer), or `None` on timeout / lost confirm. Pairs the first joiner only.
    pub async fn open_pairing(
        &mut self,
        timeout: Duration,
        assign_id: u32,
        assign_key: &[u8; 16],
    ) -> Option<u32> {
        let deadline = Instant::now().checked_add(timeout)?;
        let mut buf = [0u8; 24];
        while Instant::now() < deadline {
            let Some((hdr, plen)) = self.rx_pair(BULK_SERVE_SLICE, &mut buf).await else {
                continue;
            };
            if hdr.frame_type != FrameType::JoinReq || plen < 4 {
                continue;
            }
            let proposed = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);

            // JOIN_RESP: assigned id (4) + per-node key (16).
            let mut resp = [0u8; 20];
            resp[..4].copy_from_slice(&assign_id.to_le_bytes());
            resp[4..20].copy_from_slice(assign_key);
            let resp_hdr = Header {
                frame_type: FrameType::JoinResp,
                flags: 0,
                src: self.my_id,
                dest: proposed,
                counter: self.tx_counter,
                bulk_index: None,
            };
            Timer::after(ACK_TURNAROUND).await;
            self.tx_pair(&resp_hdr, &resp).await;
            self.advance_tx_counter();

            // Wait for the JOIN_CONFIRM — commit only on receipt.
            let mut cbuf = [0u8; 8];
            if let Some((chdr, cplen)) = self.rx_pair(JOIN_CONFIRM_WINDOW, &mut cbuf).await {
                if chdr.frame_type == FrameType::JoinConfirm
                    && cplen >= 4
                    && u32::from_le_bytes([cbuf[0], cbuf[1], cbuf[2], cbuf[3]]) == assign_id
                {
                    return Some(proposed);
                }
            }
            // Lost confirm: discard this tentative entry, keep the window open.
        }
        None
    }

    /// Joiner: request pairing with `proposed_id` for up to `timeout`. Sends
    /// JOIN_REQ, waits for JOIN_RESP, sends JOIN_CONFIRM, and returns the assigned
    /// ID + per-node key on commit (or `None` on timeout).
    pub async fn join(&mut self, proposed_id: u32, timeout: Duration) -> Option<(u32, [u8; 16])> {
        let deadline = Instant::now().checked_add(timeout)?;
        while Instant::now() < deadline {
            let req_hdr = Header {
                frame_type: FrameType::JoinReq,
                flags: 0,
                src: proposed_id,
                dest: 0, // host ID not yet known (unassigned)
                counter: self.tx_counter,
                bulk_index: None,
            };
            self.tx_pair(&req_hdr, &proposed_id.to_le_bytes()).await;
            self.advance_tx_counter();

            let mut buf = [0u8; 24];
            if let Some((hdr, plen)) = self.rx_pair(JOIN_RESP_WINDOW, &mut buf).await {
                if hdr.frame_type == FrameType::JoinResp && plen >= 20 {
                    let assigned = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                    let mut key = [0u8; 16];
                    key.copy_from_slice(&buf[4..20]);
                    // Confirm, then commit.
                    let conf_hdr = Header {
                        frame_type: FrameType::JoinConfirm,
                        flags: 0,
                        src: assigned,
                        dest: hdr.src,
                        counter: self.tx_counter,
                        bulk_index: None,
                    };
                    Timer::after(ACK_TURNAROUND).await;
                    self.tx_pair(&conf_hdr, &assigned.to_le_bytes()).await;
                    self.advance_tx_counter();
                    return Some((assigned, key));
                }
            }
        }
        None
    }

    /// Seal `hdr`+`payload` under the pairing key and transmit (duty-metered).
    async fn tx_pair(&mut self, hdr: &Header, payload: &[u8]) -> bool {
        let mut buf = [0u8; MAX_FRAME];
        let Ok(n) = frame::seal_frame(&mut self.ccm, &PAIRING_KEY, hdr, payload, &mut buf) else {
            return false;
        };
        if !self.duty.try_tx(duty::frame_toa_ms(n)) {
            return false;
        }
        self.radio.tx(&buf[..n], false, TX_TIMEOUT).await.is_ok()
    }

    /// Receive + CCM-open a frame under the pairing key; copy plaintext to `out`.
    async fn rx_pair(&mut self, timeout: Duration, out: &mut [u8]) -> Option<(Header, usize)> {
        let mut buf = [0u8; MAX_FRAME];
        let (len, _) = self.radio.rx(&mut buf, timeout).await.ok()?;
        let (hdr, range) = frame::open_frame(&mut self.ccm, &PAIRING_KEY, &mut buf[..len]).ok()?;
        let plen = range.end - range.start;
        if plen > out.len() {
            return None;
        }
        out[..plen].copy_from_slice(&buf[range]);
        Some((hdr, plen))
    }

    /// Seal `hdr`+`payload` under `key` and transmit it (no ACK), metered by the
    /// duty governor. Returns whether it was actually sent.
    async fn tx_frame(&mut self, key: &[u8; 16], hdr: &Header, payload: &[u8]) -> bool {
        let mut buf = [0u8; MAX_FRAME];
        let Ok(n) = frame::seal_frame(&mut self.ccm, key, hdr, payload, &mut buf) else {
            return false;
        };
        if !self.duty.try_tx(duty::frame_toa_ms(n)) {
            return false;
        }
        self.radio.tx(&buf[..n], false, TX_TIMEOUT).await.is_ok()
    }

    /// Receive one frame and CCM-open it under `key`; copy the plaintext into
    /// `out`. Returns the header and plaintext length. (Raw — no replay/ACK
    /// logic; for bulk.)
    async fn rx_frame(&mut self, key: &[u8; 16], timeout: Duration, out: &mut [u8]) -> Option<(Header, usize)> {
        let mut buf = [0u8; MAX_FRAME];
        let (len, _) = self.radio.rx(&mut buf, timeout).await.ok()?;
        let (hdr, range) = frame::open_frame(&mut self.ccm, key, &mut buf[..len]).ok()?;
        let plen = range.end - range.start;
        if plen > out.len() {
            return None;
        }
        out[..plen].copy_from_slice(&buf[range]);
        Some((hdr, plen))
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
        // ACK payload: acked counter (4 LE) + rssi (1).
        let pl = &buf[range];
        pl.len() >= 4 && u32::from_le_bytes([pl[0], pl[1], pl[2], pl[3]]) == counter
    }

    /// Build, cache and transmit an ACK for a received confirmed frame. The ACK
    /// uses the ACKer's *own* fresh counter (§6); the acknowledged counter rides
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
            flags: 0, // downlink-pending added with the pull mechanism (Step 13)
            src: self.my_id,
            dest,
            counter: ack_counter,
            bulk_index: None,
        };
        let key = self.key_for(dest);
        let mut ack = [0u8; MAX_FRAME];
        if let Ok(n) = frame::seal_frame(&mut self.ccm, &key, &hdr, &payload, &mut ack) {
            // ACK airtime is governed too (§2.2); skip it if over budget — the
            // sender will retransmit. Cache it regardless for retransmit dedup.
            self.advance_tx_counter(); // ACK consumes a counter (its own, §6)
            self.cached_ack[..n].copy_from_slice(&ack[..n]);
            self.cached_ack_len = n;
            self.cached_ack_for = acked;
            self.cached_ack_src = dest;
            if self.duty.try_tx(duty::frame_toa_ms(n)) {
                let _ = self.radio.tx(&ack[..n], false, TX_TIMEOUT).await;
            }
        }
    }

    /// xorshift32 backoff in [0, MAX_BACKOFF_MS).
    fn backoff_ms(&mut self) -> u32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        x % MAX_BACKOFF_MS
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
