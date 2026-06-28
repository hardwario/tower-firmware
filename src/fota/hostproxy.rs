//! Host-proxy image source (docs/fota.md) — serve a FOTA image to a node by pulling
//! its bytes from the **host** over the console link on demand, so a gateway needs no image
//! storage of its own.
//!
//! A gateway holds only the 116-byte signed manifest + one ≤64 B chunk in RAM. For each
//! radio bulk chunk the node requests, [`HostProxySource`] sends a `FotaReq{offset,len}`
//! frame to the host (via [`console::send_fota_req`]) and reads the host's `FotaData` reply
//! off the console RX half. The host side is `tower fota serve` (reads the signed `.bin` +
//! `.fmanifest` and answers requests). The manifest is fetched once up front (a request with
//! [`FOTA_MANIFEST_OFFSET`]); its `size` field becomes the served image length.
//!
//! Timing: a chunk fetch is one host round-trip (~tens of ms over 115200). It must fit inside
//! the node's per-chunk `BULK_RESP_WINDOW`; if the host is briefly slow the node's `BULK_REQ`
//! retransmit drives another fetch, and the (length-checked) bulk fetcher rejects anything
//! short — so a slow/dead host degrades to a failed pull, never a corrupt image.

use embassy_stm32::usart::BufferedUartRx;
use embassy_time::{Duration, with_timeout};
use embedded_io_async::Read as _;
use tower_protocol::fota::FOTA_MANIFEST_OFFSET;
use tower_protocol::{FrameDecoder, MsgType, decode_frame};

use super::{MANIFEST_LEN, Manifest, SIGNED_LEN};
use crate::console;
use crate::radio::net::BulkSource;

/// Per-attempt wait for a `FotaData` reply.
const RESP_WINDOW: Duration = Duration::from_millis(200);
/// Host-request attempts per image chunk (the node's `BULK_REQ` retransmit also retries).
const CHUNK_REPS: u8 = 2;
/// The manifest fetch isn't under bulk timing, so retry it generously.
const MANIFEST_REPS: u8 = 6;
const MANIFEST_WINDOW: Duration = Duration::from_millis(500);

/// A [`BulkSource`] that streams a FOTA image from the host over the console link. Build it
/// with [`connect`](Self::connect) (which also returns the signed manifest to serve first).
/// It **borrows** the console RX half so the gateway keeps ownership and can serve again.
pub struct HostProxySource<'r> {
    rx: &'r mut BufferedUartRx<'static>,
    dec: FrameDecoder,
    image_len: usize,
}

impl<'r> HostProxySource<'r> {
    /// Borrow the console RX half (from [`console::take_rx`]), fetch the signed manifest from
    /// the host, and build the source — its [`total_len`](BulkSource::total_len) is the
    /// manifest's image `size`. Returns the source **and** the 116-byte signed manifest
    /// (serve that to the node as the first bulk transfer, the image as the second). `None`
    /// if the host doesn't answer or the manifest is malformed.
    pub async fn connect(rx: &'r mut BufferedUartRx<'static>) -> Option<(Self, [u8; SIGNED_LEN])> {
        let mut s = Self {
            rx,
            dec: FrameDecoder::new(),
            image_len: 0,
        };
        let mut manifest = [0u8; SIGNED_LEN];
        let n = s
            .fetch(
                FOTA_MANIFEST_OFFSET,
                &mut manifest,
                MANIFEST_REPS,
                MANIFEST_WINDOW,
            )
            .await?;
        if n < SIGNED_LEN {
            return None;
        }
        s.image_len = Manifest::decode(&manifest[..MANIFEST_LEN])?.size as usize;
        Some((s, manifest))
    }

    /// Send one `FotaReq(offset, out.len())`, then read frames until the matching `FotaData`
    /// arrives (within `window`); retry up to `reps`. Returns the bytes written to `out`.
    async fn fetch(&mut self, offset: u32, out: &mut [u8], reps: u8, window: Duration) -> Option<usize> {
        for _ in 0..reps {
            self.dec.reset(); // drop any partial frame left by a prior timeout
            console::send_fota_req(offset, out.len() as u16).await;
            if let Some(n) = self.await_data(offset, out, window).await {
                return Some(n);
            }
        }
        None
    }

    /// Read + deframe console RX until a `FotaData` whose payload offset matches `offset`.
    /// Returns `None` if no byte arrives within `window` (the host went quiet).
    async fn await_data(&mut self, offset: u32, out: &mut [u8], window: Duration) -> Option<usize> {
        let mut buf = [0u8; 64];
        loop {
            let n = with_timeout(window, self.rx.read(&mut buf)).await.ok()?.ok()?;
            for &b in &buf[..n] {
                let Some(inner) = self.dec.push(b) else { continue };
                let Ok((MsgType::FotaData, _seq, p)) = decode_frame(inner) else {
                    continue;
                };
                if p.len() < 4 {
                    continue;
                }
                if u32::from_le_bytes([p[0], p[1], p[2], p[3]]) != offset {
                    continue; // a stale reply to an earlier request — ignore
                }
                let data = &p[4..];
                let m = data.len().min(out.len());
                out[..m].copy_from_slice(&data[..m]);
                return Some(m);
            }
        }
    }
}

impl BulkSource for HostProxySource<'_> {
    fn total_len(&self) -> usize {
        self.image_len
    }

    async fn read(&mut self, offset: usize, out: &mut [u8]) -> usize {
        // 0 on failure → the bulk fetcher's length check rejects the short chunk and the
        // node retransmits its BULK_REQ, driving another fetch (never a corrupt image).
        self.fetch(offset as u32, out, CHUNK_REPS, RESP_WINDOW)
            .await
            .unwrap_or(0)
    }
}
