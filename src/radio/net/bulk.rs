//! Bulk transfer / downlink pull (RADIO.md §7.5): the requester pulls, the sender
//! serves. The session id is a counter reserved by the sender (distinct from the
//! announce frame's counter, so chunk-0's nonce never collides with the announce).
//! All BULK_DATA chunks share that session counter with a distinct bulk_index,
//! keeping their nonces unique. `impl Net` block over [`super::Net`].

use embassy_time::{Duration, Instant, Timer};

use super::{ACK_TURNAROUND, Net, TX_TIMEOUT};
use crate::radio::duty;
use crate::radio::frame::{self, FrameType, Header, MAX_FRAME, flags};

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

impl Net {
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
                if let Some((dhdr, dlen)) = self.rx_frame(&key, BULK_RESP_WINDOW, &mut dbuf).await
                    && dhdr.frame_type == FrameType::BulkData
                    && dhdr.src == src
                    && dhdr.dest == self.my_id
                    && dhdr.counter == session
                    && dhdr.bulk_index == Some(k as u32)
                {
                    let off = k * BULK_CHUNK;
                    // Bound the copy to the announced length: a peer-controlled
                    // (authenticated, but possibly buggy) `dlen` must never write
                    // past `out` — e.g. a full 64 B last chunk for a 65 B total.
                    if off + dlen > total_len {
                        continue; // malformed chunk; retry within reps
                    }
                    out[off..off + dlen].copy_from_slice(&dbuf[..dlen]);
                    received += dlen;
                    got = true;
                    break;
                }
            }
            self.advance_tx_counter();
            if !got {
                return None;
            }
        }
        Some(received)
    }

    /// Seal `hdr`+`payload` under `key` and transmit it (no ACK), metered by the
    /// duty governor. Returns whether it was actually sent. (Bulk's raw send/recv;
    /// no replay/ACK logic.)
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
    /// `out`. Returns the header and plaintext length.
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
}
