//! Bulk transfer / downlink pull (docs/radio.md): the requester pulls, the sender
//! serves. The session id is a counter reserved by the sender (distinct from the
//! announce frame's counter, so chunk-0's nonce never collides with the announce).
//! All BULK_DATA chunks share that session counter with a distinct bulk_index,
//! keeping their nonces unique. `impl Net` block over [`super::Net`].
//!
//! The transfer is **streamed on both ends**: the sender pulls each chunk from a
//! [`BulkSource`] on demand, and the requester hands each chunk to a [`BulkSink`]
//! as it arrives — neither side ever buffers the whole transfer, so RAM is constant
//! regardless of length. That is the path a flash-backed source/sink (e.g. FOTA:
//! serve an image from flash, stream the received image straight to flash) needs.
//! The slice-backed [`Net::bulk_serve`] / [`Net::bulk_fetch`] are thin convenience
//! wrappers over the streaming core [`Net::bulk_serve_from`] / [`Net::bulk_fetch_into`].

use embassy_time::{Duration, Instant, Timer};

use super::{ACK_TURNAROUND, Net, TX_TIMEOUT};
use crate::radio::duty;
use crate::radio::frame::{self, FrameType, Header, MAX_FRAME, flags};

/// Bulk chunk size (docs/radio.md).
pub const BULK_CHUNK: usize = 64;
/// Bulk session idle timeout: the sender frees the transfer after this with no progress.
const BULK_IDLE: Duration = Duration::from_secs(30);
/// How long the requester waits for a BULK_DATA after a BULK_REQ.
const BULK_RESP_WINDOW: Duration = Duration::from_millis(250);
/// Requester BULK_REQ repetitions per chunk before giving up.
const BULK_REQ_REPS: u8 = 6;
/// How often [`Net::bulk_fetch_to_flash`] persists its high-water mark (every N chunks =
/// N·64 B). A power-cut/stall re-pulls at most this much; the value trades EEPROM writes
/// (a same-size in-place rewrite of one cell) against re-pull on resume. 32 ≈ 2 KB.
const HWM_PERSIST_CHUNKS: usize = 32;
/// One serve-loop receive slice on the sender side.
const BULK_SERVE_SLICE: Duration = Duration::from_millis(500);

/// A source of bulk data, pulled chunk-by-chunk by [`Net::bulk_serve_from`] so the
/// sender never holds the whole transfer in RAM. Implement it over a slice (see the
/// [`Net::bulk_serve`] wrapper) or, for FOTA, over a flash reader.
pub trait BulkSource {
    /// Total number of bytes to serve (fixed for the whole transfer; announced once).
    fn total_len(&self) -> usize;
    /// Fill `out` with the bytes at byte `offset` (`offset` is chunk-aligned and
    /// `out.len()` ≤ [`BULK_CHUNK`] — exactly the bytes remaining for the last
    /// chunk). Returns the number of bytes written (normally `out.len()`).
    #[allow(async_fn_in_trait)] // single-threaded embedded executor; no Send bound needed
    async fn read(&mut self, offset: usize, out: &mut [u8]) -> usize;
}

/// A sink for bulk data, fed chunk-by-chunk by [`Net::bulk_fetch_into`] so the
/// requester never holds the whole transfer in RAM. Implement it to verify, hash,
/// or — for FOTA — stream straight to a flash staging slot.
pub trait BulkSink {
    /// Called once after the announce with the total length, before any chunk is
    /// pulled. Return `false` to refuse the transfer (e.g. too large for the
    /// destination, or the flash slot can't be erased), aborting the fetch.
    #[allow(async_fn_in_trait)]
    async fn begin(&mut self, total_len: usize) -> bool;
    /// Called for each chunk in increasing-`offset` order with its plaintext bytes
    /// (`chunk.len()` ≤ [`BULK_CHUNK`]). Return `false` to abort the transfer
    /// (e.g. a flash write failed).
    #[allow(async_fn_in_trait)]
    async fn consume(&mut self, offset: usize, chunk: &[u8]) -> bool;
}

impl Net {
    /// Serve an in-RAM `data` slice as a bulk transfer to `dest` — a convenience
    /// wrapper over [`bulk_serve_from`](Self::bulk_serve_from). Returns whether the
    /// last chunk was served.
    pub async fn bulk_serve(&mut self, dest: u32, data: &[u8]) -> bool {
        let mut src = SliceSource { data };
        self.bulk_serve_from(dest, &mut src).await
    }

    /// Serve `source` as a bulk transfer to `dest`: announce the length + session,
    /// then answer BULK_REQ(index) with BULK_DATA(index, ≤64 B) — pulling each chunk
    /// from `source` on demand — until the last chunk is pulled or the session idles
    /// out (30 s). The source is never fully buffered, so the transfer size is
    /// bounded by neither RAM nor (the 24-bit index aside) the protocol. Returns
    /// whether the last chunk was served.
    pub async fn bulk_serve_from<S: BulkSource>(&mut self, dest: u32, source: &mut S) -> bool {
        let key = self.key_for(dest);
        let total_len = source.total_len();
        let n_chunks = total_len.div_ceil(BULK_CHUNK).max(1);
        let announce_counter = self.tx_counter;
        self.advance_tx_counter();
        let session = self.tx_counter; // reserved for all chunks; consumed at the end
        // Nonce-reuse guard: the announce frame rides `announce_counter` (bulk_index 0) and
        // chunk 0 rides `session` (also bulk_index 0) — their CCM nonces differ only while the
        // TX counter still advances. At the u32::MAX ceiling `advance_tx_counter` is a no-op, so
        // the two would collide into one nonce with different plaintext. Refuse to serve there
        // (the link is already failing closed at saturation — re-key long before; see
        // `advance_tx_counter`).
        if announce_counter == session {
            return false;
        }

        let mut ann = [0u8; 8];
        ann[..4].copy_from_slice(&(total_len as u32).to_le_bytes());
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
        let mut chunk = [0u8; BULK_CHUNK];
        loop {
            if Instant::now().saturating_duration_since(last_progress)
                >= if served_last {
                    Duration::from_secs(2)
                } else {
                    BULK_IDLE
                }
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
            let want = (total_len - start).min(BULK_CHUNK);
            let n = source.read(start, &mut chunk[..want]).await;
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
            self.tx_frame(&key, &dhdr, &chunk[..n]).await;
            if last {
                served_last = true;
            }
        }
        self.advance_tx_counter(); // consume the session counter
        served_last
    }

    /// Fetch a bulk transfer from `src` into the `out` slice — a convenience wrapper
    /// over [`bulk_fetch_into`](Self::bulk_fetch_into). Returns the total length
    /// received, or `None` on announce/chunk failure or `out` too small.
    pub async fn bulk_fetch(&mut self, src: u32, out: &mut [u8]) -> Option<usize> {
        let mut sink = SliceSink { out };
        self.bulk_fetch_into(src, &mut sink).await
    }

    /// Fetch a bulk transfer from `src`, streaming each chunk to `sink`: receive the
    /// announcement (calling [`BulkSink::begin`] with the total length), then pull
    /// each chunk with BULK_REQ (retransmitting on loss) and hand it to
    /// [`BulkSink::consume`] in increasing-offset order. Nothing is buffered beyond
    /// one chunk, so the transfer size is bounded by neither RAM nor (the 24-bit
    /// index aside) the protocol. Returns the total length received, or `None` on
    /// announce/chunk failure or a sink that refused (`begin`/`consume` → `false`).
    pub async fn bulk_fetch_into<S: BulkSink>(&mut self, src: u32, sink: &mut S) -> Option<usize> {
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
        if !sink.begin(total_len).await {
            return None; // sink refused the transfer (e.g. too large / can't stage)
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
                    // Require exactly the expected chunk length (full BULK_CHUNK, or the
                    // remainder for the last chunk): a peer-controlled (authenticated but
                    // possibly buggy) or short (e.g. host-proxy miss) `dlen` must never feed
                    // the sink a partial/over-long chunk — retry instead.
                    if dlen != (total_len - off).min(BULK_CHUNK) {
                        continue; // wrong-size chunk; retry within reps
                    }
                    if !sink.consume(off, &dbuf[..dlen]).await {
                        return None; // sink aborted (e.g. flash write failed)
                    }
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

    /// Fetch a bulk transfer from `src` **straight into a program-flash slot**, with **resume**
    /// — the FOTA staging pull (docs/fota.md). Functionally [`bulk_fetch_into`] with the
    /// sink built in: each chunk is programmed into the slot through the network layer's *own*
    /// [`Flash`](embassy_stm32::flash::Flash) (which `Net` owns via its
    /// [`Kv`](crate::storage::Kv)), so there is no borrow conflict with the `&mut self` radio
    /// calls — the flash is touched only *between* receives, never held across one.
    ///
    /// `base`/`size` are flash offsets/lengths of the destination slot (e.g.
    /// [`DFU_OFFSET`](crate::fota::DFU_OFFSET) / [`DFU_SIZE`](crate::fota::DFU_SIZE)). `start`
    /// is the resume offset (bytes already staged from a prior call): `start == 0` erases the
    /// slot up front and pulls the whole image; `start > 0` **skips the erase** and re-requests
    /// only from chunk `start/BULK_CHUNK`. If `progress_key` is `Some`, the running staged
    /// count is persisted there (u32 LE) every [`HWM_PERSIST_CHUNKS`] chunks and at the end —
    /// the high-water mark a duty stall or power-cut resumes from.
    ///
    /// Returns the total bytes **contiguously staged** (`start` + newly received); it equals
    /// the announced length when the image is complete, or less if a chunk stalled (duty limit
    /// / loss / flash error) — the caller persists that as the HWM and retries to resume.
    ///
    /// No hashing here: the **bootloader** recomputes SHA-256 over the staged DFU and checks it
    /// against the signed manifest before swapping (docs/fota.md).
    pub async fn bulk_fetch_to_flash(
        &mut self,
        src: u32,
        base: u32,
        size: u32,
        start: u32,
        progress_key: Option<u16>,
    ) -> usize {
        use crate::fota::{Stage, WRITE_SIZE};

        let w = WRITE_SIZE as usize;
        let key = self.key_for(src);
        // Wait for the bulk-announce (identical handshake to bulk_fetch_into).
        let mut abuf = [0u8; 8];
        let (total_len, session) = loop {
            let Some((hdr, plen)) = self.rx_frame(&key, Duration::from_secs(5), &mut abuf).await else {
                return start as usize; // no announce — no progress, resume next time
            };
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
        if total_len as u32 > size {
            return start as usize; // image too large for the slot
        }
        let start = (start as usize).min(total_len); // clamp a stale HWM
        if start == 0 {
            // Fresh start: erase the destination for the announced length (one short flash
            // borrow, released before the radio loop). A resume keeps the staged bytes.
            if self
                .kv
                .with_flash(|f| Stage::new(f, base, size).erase(total_len as u32))
                .is_err()
            {
                return 0;
            }
        }

        let n_chunks = total_len.div_ceil(BULK_CHUNK).max(1);
        let mut received = start;
        let mut since_persist = 0usize;
        for k in (start / BULK_CHUNK)..n_chunks {
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
                    // Require exactly the expected chunk length (full BULK_CHUNK, or the
                    // remainder for the last chunk). Rejecting a short/long chunk — e.g. a
                    // host-proxy serve that couldn't fetch the bytes in time — makes the node
                    // retransmit its BULK_REQ rather than program a gap into the image.
                    if dlen != (total_len - off).min(BULK_CHUNK) {
                        continue; // wrong-size chunk; retry within reps
                    }
                    // Program the chunk — padding a partial tail up to a flash word.
                    let mut padded = [0u8; BULK_CHUNK];
                    padded[..dlen].copy_from_slice(&dbuf[..dlen]);
                    let plen = dlen.div_ceil(w) * w;
                    if self
                        .kv
                        .with_flash(|f| Stage::new(f, base, size).program(off as u32, &padded[..plen]))
                        .is_err()
                    {
                        break; // flash write failed — stop; the HWM lets a retry resume
                    }
                    received += dlen;
                    got = true;
                    break;
                }
            }
            self.advance_tx_counter();
            if !got {
                break; // chunk stalled (duty / loss) — stop; resume from the HWM next time
            }
            since_persist += 1;
            if since_persist >= HWM_PERSIST_CHUNKS {
                since_persist = 0;
                if let Some(k) = progress_key {
                    let _ = self.kv.set_bytes(k, &(received as u32).to_le_bytes());
                }
            }
        }
        if let Some(k) = progress_key {
            let _ = self.kv.set_bytes(k, &(received as u32).to_le_bytes());
        }
        received
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
    async fn rx_frame(
        &mut self,
        key: &[u8; 16],
        timeout: Duration,
        out: &mut [u8],
    ) -> Option<(Header, usize)> {
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

/// [`BulkSource`] over an in-RAM slice (backs [`Net::bulk_serve`]).
struct SliceSource<'a> {
    data: &'a [u8],
}

impl BulkSource for SliceSource<'_> {
    fn total_len(&self) -> usize {
        self.data.len()
    }
    async fn read(&mut self, offset: usize, out: &mut [u8]) -> usize {
        let n = out.len();
        out.copy_from_slice(&self.data[offset..offset + n]);
        n
    }
}

/// [`BulkSink`] into an in-RAM slice (backs [`Net::bulk_fetch`]); the up-front
/// length check in `begin` preserves the old "out too small → `None`" behaviour.
struct SliceSink<'a> {
    out: &'a mut [u8],
}

impl BulkSink for SliceSink<'_> {
    async fn begin(&mut self, total_len: usize) -> bool {
        total_len <= self.out.len()
    }
    async fn consume(&mut self, offset: usize, chunk: &[u8]) -> bool {
        if offset + chunk.len() > self.out.len() {
            return false;
        }
        self.out[offset..offset + chunk.len()].copy_from_slice(chunk);
        true
    }
}
