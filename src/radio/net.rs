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
//! This step uses a single shared per-link key and in-RAM counters; the per-peer
//! key table and EEPROM persistence are added in later steps (§7.4).

#![allow(dead_code)]

use embassy_time::{Duration, Timer};

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
    /// Shared per-link AES-128 key (per-peer key table is a later step).
    pub key: [u8; 16],
    pub band: Band,
    pub channel: u8,
}

/// The network layer over one SPIRIT1 radio.
pub struct Net {
    radio: Spirit1,
    ccm: Ccm,
    my_id: u32,
    key: [u8; 16],
    /// Monotonic TX counter, advanced by one per transfer (§6).
    tx_counter: u32,
    /// Highest reserved (persisted) counter value; `tx_counter < reserve_limit`.
    reserve_limit: u32,
    /// Per-peer last-seen counter (single peer for now).
    last_seen: u32,
    /// Accepted-transfer count since the last last-seen persist.
    accepts: u32,
    /// EEPROM-backed counter persistence.
    kv: Kv<'static>,
    /// EU duty-cycle governor (airtime budget for all TX).
    duty: DutyGovernor,
    /// Cached ACK bytes for the most recent confirmed RX, to re-send on a
    /// byte-identical retransmit (§7.3).
    cached_ack: [u8; MAX_FRAME],
    cached_ack_len: usize,
    /// The acked counter the cached ACK corresponds to (0 = none cached).
    cached_ack_for: u32,
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
            key: cfg.key,
            tx_counter: resume,
            reserve_limit,
            last_seen,
            accepts: 0,
            kv,
            duty: DutyGovernor::eu(),
            cached_ack: [0; MAX_FRAME],
            cached_ack_len: 0,
            cached_ack_for: 0,
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

    /// Current per-peer last-seen counter.
    pub fn last_seen(&self) -> u32 {
        self.last_seen
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
        let mut frame_buf = [0u8; MAX_FRAME];
        let n = match frame::seal_frame(&mut self.ccm, &self.key, &hdr, data, &mut frame_buf) {
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

        // CCM-verify first (authenticates the header incl. counter), then decide.
        let (hdr, range) = frame::open_frame(&mut self.ccm, &self.key, &mut buf[..len]).ok()?;
        if hdr.dest != self.my_id {
            return None; // not for us
        }

        if hdr.counter > self.last_seen {
            // Fresh — accept, advance last-seen, ACK if requested.
            self.last_seen = hdr.counter;
            // Lazy-persist last-seen every P accepts → replay window ≤ P across a
            // reboot (§7.4).
            self.accepts = self.accepts.wrapping_add(1);
            if self.accepts % P == 0 {
                let _ = self.kv.set_bytes(KEY_LASTSEEN, &self.last_seen.to_le_bytes());
            }
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
        } else if hdr.counter == self.last_seen && self.cached_ack_for == hdr.counter {
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

    /// Wait `ACK_WINDOW` for an ACK from `dest` acknowledging `counter`.
    async fn await_ack(&mut self, dest: u32, counter: u32) -> bool {
        let mut buf = [0u8; MAX_FRAME];
        let Ok((len, _)) = self.radio.rx(&mut buf, ACK_WINDOW).await else {
            return false;
        };
        let Ok((hdr, range)) = frame::open_frame(&mut self.ccm, &self.key, &mut buf[..len]) else {
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
        let mut ack = [0u8; MAX_FRAME];
        if let Ok(n) = frame::seal_frame(&mut self.ccm, &self.key, &hdr, &payload, &mut ack) {
            // ACK airtime is governed too (§2.2); skip it if over budget — the
            // sender will retransmit. Cache it regardless for retransmit dedup.
            self.advance_tx_counter(); // ACK consumes a counter (its own, §6)
            self.cached_ack[..n].copy_from_slice(&ack[..n]);
            self.cached_ack_len = n;
            self.cached_ack_for = acked;
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
