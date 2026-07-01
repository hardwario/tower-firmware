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

use embassy_time::{Duration, with_timeout};
use tower_protocol::fota::FOTA_MANIFEST_OFFSET;

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

/// A [`BulkSource`] that streams a FOTA image from the host over the console link. Build
/// it with [`connect`](Self::connect) (which also returns the signed manifest to serve
/// first). The decoded `FotaData` replies are delivered by the console's RX router via
/// [`console::FOTA_DATA`](crate::console) — the dynamic console owns the UART, so the
/// host-proxy no longer borrows the raw RX half.
pub struct HostProxySource {
    image_len: usize,
}

impl HostProxySource {
    /// Fetch the signed manifest from the host and build the source — its
    /// [`total_len`](BulkSource::total_len) is the manifest's image `size`. Returns the
    /// source **and** the 116-byte signed manifest (serve that to the node as the first
    /// bulk transfer, the image as the second). `None` if the host doesn't answer or the
    /// manifest is malformed. (Requires USB present so the console/host link is up.)
    pub async fn connect() -> Option<(Self, [u8; SIGNED_LEN])> {
        let mut s = Self { image_len: 0 };
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

    /// Send one `FotaReq(offset, out.len())`, then await the matching `FotaData` (within
    /// `window`); retry up to `reps`. Returns the bytes written to `out`.
    async fn fetch(&mut self, offset: u32, out: &mut [u8], reps: u8, window: Duration) -> Option<usize> {
        for _ in 0..reps {
            // Drop any stale routed chunk left by a prior timeout.
            while console::FOTA_DATA.try_receive().is_ok() {}
            console::send_fota_req(offset, out.len() as u16).await;
            if let Some(n) = self.await_data(offset, out, window).await {
                return Some(n);
            }
        }
        None
    }

    /// Await routed `FotaData` chunks until one whose payload offset matches `offset`.
    /// Returns `None` if nothing arrives within `window` (the host went quiet).
    async fn await_data(&mut self, offset: u32, out: &mut [u8], window: Duration) -> Option<usize> {
        loop {
            let chunk = with_timeout(window, console::FOTA_DATA.receive()).await.ok()?;
            if chunk.len() < 4 {
                continue;
            }
            if u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]) != offset {
                continue; // a stale reply to an earlier request — ignore
            }
            let data = &chunk[4..];
            let m = data.len().min(out.len());
            out[..m].copy_from_slice(&data[..m]);
            return Some(m);
        }
    }
}

impl BulkSource for HostProxySource {
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
