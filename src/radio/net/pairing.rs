//! OTA pairing: 3-way JOIN under the fixed public PAIRING_KEY (docs/radio.md). `impl Net`
//! block over [`super::Net`].
//!
//! The **joiner chooses its own ID** and keeps it; the host only hands out the
//! per-node key — it does NOT assign or override the ID. JOIN_REQ(node_id) →
//! JOIN_RESP(key ‖ challenge) → JOIN_CONFIRM(node_id ‖ challenge). Both sides commit
//! only after the confirm; a lost confirm leaves the host's window to time out (the
//! entry is never installed) and the joiner retries within its window.
//!
//! The host mints a fresh per-session **challenge** in the JOIN_RESP that the joiner
//! must echo in the JOIN_CONFIRM. This is anti-*replay* within the window (a confirm
//! captured from a prior session carries a stale challenge and is rejected) on top of
//! CCM integrity — it does NOT add confidentiality or mutual auth (the challenge rides
//! the public-key frames in the clear-after-decrypt payload).

use embassy_time::{Duration, Instant, Timer};

use super::{ACK_TURNAROUND, Net, TX_TIMEOUT};
use crate::radio::duty;
use crate::radio::frame::{self, FrameType, Header, MAX_FRAME};

/// Fixed, **publicly-known** OTA-pairing key (docs/radio.md). It gives the JOIN
/// frames a uniform CCM format with integrity + in-session replay protection, but
/// NO confidentiality (a sniffer in range during the window recovers the
/// delivered per-node key) and NO mutual authentication. Mitigate with a short
/// window, proximity, reduced power, user-initiated pairing.
pub const PAIRING_KEY: [u8; 16] = *b"TOWER-PAIR-KEY!\0";
/// Default pairing window: the host listens (and the joiner retries) for this
/// long. One minute gives a person time to put both ends into pairing mode.
pub const PAIRING_WINDOW: Duration = Duration::from_secs(60);
/// How long the joiner waits for a JOIN_RESP after a JOIN_REQ.
const JOIN_RESP_WINDOW: Duration = Duration::from_millis(300);
/// How long the host waits for a JOIN_CONFIRM after a JOIN_RESP.
const JOIN_CONFIRM_WINDOW: Duration = Duration::from_millis(300);
/// One listen slice of the host's pairing window between JOIN_REQ polls.
const PAIR_RX_SLICE: Duration = Duration::from_millis(500);

impl Net {
    /// Host: open a pairing window for `timeout` (use [`PAIRING_WINDOW`] for the
    /// default minute). On the first valid JOIN_REQ, hand out the per-node `key`
    /// (JOIN_RESP) and wait for the JOIN_CONFIRM. Returns the joiner's **own,
    /// self-chosen** ID on commit — the caller installs the peer with
    /// `add_peer(id, key)` — or `None` on timeout / lost confirm. The host does
    /// NOT assign the ID. Pairs the first joiner only.
    pub async fn open_pairing(&mut self, timeout: Duration, key: &[u8; 16]) -> Option<u32> {
        let deadline = Instant::now().checked_add(timeout)?;
        let mut buf = [0u8; 24];
        while Instant::now() < deadline {
            let Some((hdr, plen)) = self.rx_pair(PAIR_RX_SLICE, &mut buf).await else {
                continue;
            };
            if hdr.frame_type != FrameType::JoinReq || plen < 4 {
                continue;
            }
            let node_id = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
            let challenge = self.rand_u32(); // fresh per session; the confirm must echo it

            // JOIN_RESP: the per-node key ‖ a fresh challenge (the joiner keeps its own ID).
            let mut resp = [0u8; 20];
            resp[..16].copy_from_slice(key);
            resp[16..20].copy_from_slice(&challenge.to_le_bytes());
            let resp_hdr = Header {
                frame_type: FrameType::JoinResp,
                flags: 0,
                src: self.my_id,
                dest: node_id,
                counter: self.tx_counter,
                bulk_index: None,
            };
            Timer::after(ACK_TURNAROUND).await;
            self.tx_pair(&resp_hdr, &resp).await;
            self.advance_tx_counter();

            // Wait for the JOIN_CONFIRM (echoes node_id ‖ challenge) — commit only when both
            // match, so a confirm replayed from a prior session (stale challenge) is rejected.
            let mut cbuf = [0u8; 8];
            if let Some((chdr, cplen)) = self.rx_pair(JOIN_CONFIRM_WINDOW, &mut cbuf).await
                && chdr.frame_type == FrameType::JoinConfirm
                && cplen >= 8
                && u32::from_le_bytes([cbuf[0], cbuf[1], cbuf[2], cbuf[3]]) == node_id
                && u32::from_le_bytes([cbuf[4], cbuf[5], cbuf[6], cbuf[7]]) == challenge
            {
                return Some(node_id);
            }
            // Lost confirm: discard, keep the window open for a retry.
        }
        None
    }

    /// Joiner: request pairing using `my_id` (the joiner's **own** ID, which it
    /// keeps) for up to `timeout` ([`PAIRING_WINDOW`] for the default). Sends
    /// JOIN_REQ, waits for the JOIN_RESP (the per-node key), sends JOIN_CONFIRM,
    /// and returns the key on commit (or `None` on timeout).
    pub async fn join(&mut self, my_id: u32, timeout: Duration) -> Option<[u8; 16]> {
        let deadline = Instant::now().checked_add(timeout)?;
        while Instant::now() < deadline {
            let req_hdr = Header {
                frame_type: FrameType::JoinReq,
                flags: 0,
                src: my_id,
                dest: 0, // host ID not yet known
                counter: self.tx_counter,
                bulk_index: None,
            };
            self.tx_pair(&req_hdr, &my_id.to_le_bytes()).await;
            self.advance_tx_counter();

            let mut buf = [0u8; 24];
            if let Some((hdr, plen)) = self.rx_pair(JOIN_RESP_WINDOW, &mut buf).await
                && hdr.frame_type == FrameType::JoinResp
                && plen >= 20
            {
                let mut key = [0u8; 16];
                key.copy_from_slice(&buf[..16]);
                // Confirm: echo my_id ‖ the host's challenge, then commit.
                let mut conf = [0u8; 8];
                conf[..4].copy_from_slice(&my_id.to_le_bytes());
                conf[4..8].copy_from_slice(&buf[16..20]);
                let conf_hdr = Header {
                    frame_type: FrameType::JoinConfirm,
                    flags: 0,
                    src: my_id,
                    dest: hdr.src,
                    counter: self.tx_counter,
                    bulk_index: None,
                };
                Timer::after(ACK_TURNAROUND).await;
                self.tx_pair(&conf_hdr, &conf).await;
                self.advance_tx_counter();
                return Some(key);
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
}
