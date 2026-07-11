//! TOWER network frame codec (docs/radio.md).
//!
//! Wire layout (little-endian, fits the 96-byte SPIRIT1 FIFO):
//!
//! | ver_type | flags | src(4) | dest(4) | counter(4) | [bulk_idx(3)] | payload | tag(8) |
//!
//! The whole cleartext header is the CCM **AAD**; the payload is encrypted; the
//! 8-byte CCM tag authenticates both. The 13-byte nonce is *derived* from the
//! header (`src ‖ counter ‖ bulk_index ‖ 0x0000`) — never transmitted — which is
//! why it's reconstructable on receive and unique per (key, frame). This module
//! is pure (no I/O) and ties the layout to [`ccm`](super::ccm) via
//! [`seal_frame`]/[`open_frame`].

use super::ccm::{Ccm, NONCE_LEN, TAG_LEN};

/// Protocol version (`bits[7:5]` of `ver_type`). Caps at 7 — and value 7 is reserved
/// as the extension escape ("real version elsewhere"), never a normal bump, so fielded
/// firmwares that hard-reject unknown versions can still be reasoned about at v8+.
pub const VERSION: u8 = 1;
const _: () = assert!(
    VERSION < 7,
    "radio frame version field is 3 bits; 7 is the reserved escape"
);
/// Header size without a bulk index (ver_type+flags+src+dest+counter).
pub const HDR_LEN: usize = 14;
/// Header size with the 3-byte bulk index.
pub const HDR_LEN_BULK: usize = 17;
/// Max application payload in a non-bulk frame (96 − 14 − 8).
pub const MAX_PAYLOAD: usize = 74;
// The shared radio-application schema (`tower_protocol::radio`) sizes its NodeMsg/NodeCmd
// envelopes against this exact MTU (its encoders reject anything larger). If either side
// drifts, node apps could build envelopes the net layer refuses — make it a compile error.
const _: () = assert!(
    tower_protocol::radio::MAX_RADIO_PAYLOAD == MAX_PAYLOAD,
    "tower_protocol::radio::MAX_RADIO_PAYLOAD must equal the radio MTU"
);
/// Max payload (chunk) in a bulk frame (96 − 17 − 8 = 71, but the protocol caps
/// bulk chunks at 64).
pub const MAX_BULK_PAYLOAD: usize = 64;
/// Full frame buffer size (the FIFO).
pub const MAX_FRAME: usize = 96;

/// Frame types (docs/radio.md).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FrameType {
    Data = 0,
    Ack = 1,
    BulkReq = 2,
    BulkData = 3,
    JoinReq = 4,
    JoinResp = 5,
    JoinConfirm = 6,
    /// FHSS hop-schedule beacon (broadcast time signal; see `net/fhss.rs`).
    Beacon = 7,
}

impl FrameType {
    #[must_use]
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Data,
            1 => Self::Ack,
            2 => Self::BulkReq,
            3 => Self::BulkData,
            4 => Self::JoinReq,
            5 => Self::JoinResp,
            6 => Self::JoinConfirm,
            7 => Self::Beacon,
            _ => return None,
        })
    }
    /// Whether this frame type carries the 3-byte bulk index field.
    #[must_use]
    pub fn has_bulk_index(self) -> bool {
        matches!(self, Self::BulkReq | Self::BulkData)
    }
}

/// Frame flag bits (docs/radio.md). Bit 1 is unassigned (free for a future flag).
pub mod flags {
    /// bit0: confirmed delivery requested.
    pub const CONFIRMED: u8 = 1 << 0;
    /// bit2: last chunk (BULK_DATA).
    pub const LAST_CHUNK: u8 = 1 << 2;
    /// bit3: bulk-announce (DATA frame announcing a pending bulk).
    pub const BULK_ANNOUNCE: u8 = 1 << 3;
}

/// Parsed/clear frame header (the AAD).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    pub frame_type: FrameType,
    pub flags: u8,
    pub src: u32,
    pub dest: u32,
    pub counter: u32,
    /// 24-bit bulk index for bulk frames; `None` otherwise.
    pub bulk_index: Option<u32>,
}

/// Frame codec / crypto errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Buffer too short for the declared header/tag.
    TooShort,
    /// Protocol version not understood (drop).
    BadVersion,
    /// Unknown frame type (drop).
    BadType,
    /// Payload exceeds the per-frame limit.
    PayloadTooLong,
    /// CCM authentication failed (forged/tampered/wrong key) — drop.
    AuthFail,
}

impl core::fmt::Display for FrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            FrameError::TooShort => "frame too short",
            FrameError::BadVersion => "unsupported protocol version",
            FrameError::BadType => "unknown frame type",
            FrameError::PayloadTooLong => "payload too long",
            FrameError::AuthFail => "authentication failed",
        })
    }
}

impl Header {
    /// Header length on the wire (depends on bulk-ness).
    #[must_use]
    pub fn wire_len(&self) -> usize {
        if self.bulk_index.is_some() {
            HDR_LEN_BULK
        } else {
            HDR_LEN
        }
    }

    /// Largest payload this header type may carry.
    #[must_use]
    pub fn max_payload(&self) -> usize {
        if self.bulk_index.is_some() {
            MAX_BULK_PAYLOAD
        } else {
            MAX_PAYLOAD
        }
    }

    /// Serialize the clear header (AAD) into `out`; returns its length.
    pub fn encode(&self, out: &mut [u8]) -> usize {
        let n = self.wire_len();
        out[0] = (VERSION << 5) | (self.frame_type as u8 & 0x1F);
        out[1] = self.flags;
        out[2..6].copy_from_slice(&self.src.to_le_bytes());
        out[6..10].copy_from_slice(&self.dest.to_le_bytes());
        out[10..14].copy_from_slice(&self.counter.to_le_bytes());
        if let Some(idx) = self.bulk_index {
            let b = idx.to_le_bytes();
            out[14..17].copy_from_slice(&b[..3]);
        }
        n
    }

    /// Parse a clear header from the start of `buf`; returns it + its length.
    /// Rejects unknown versions/types (caller drops the frame).
    pub fn parse(buf: &[u8]) -> Result<(Header, usize), FrameError> {
        if buf.len() < HDR_LEN {
            return Err(FrameError::TooShort);
        }
        let ver = buf[0] >> 5;
        if ver != VERSION {
            return Err(FrameError::BadVersion);
        }
        let frame_type = FrameType::from_u8(buf[0] & 0x1F).ok_or(FrameError::BadType)?;
        let flags = buf[1];
        let src = u32::from_le_bytes([buf[2], buf[3], buf[4], buf[5]]);
        let dest = u32::from_le_bytes([buf[6], buf[7], buf[8], buf[9]]);
        let counter = u32::from_le_bytes([buf[10], buf[11], buf[12], buf[13]]);
        let (bulk_index, len) = if frame_type.has_bulk_index() {
            if buf.len() < HDR_LEN_BULK {
                return Err(FrameError::TooShort);
            }
            let idx = u32::from_le_bytes([buf[14], buf[15], buf[16], 0]);
            (Some(idx), HDR_LEN_BULK)
        } else {
            (None, HDR_LEN)
        };
        Ok((
            Header {
                frame_type,
                flags,
                src,
                dest,
                counter,
                bulk_index,
            },
            len,
        ))
    }
}

// The nonce construction lives in the host-testable `tower_net_core` leaf crate, where the
// exact byte layout is pinned by a golden test and uniqueness across (src, counter, bulk_index)
// is proven by round-trip/property tests. Re-exported so call sites keep saying
// `frame::nonce_for` — it remains the single audited nonce source for the whole stack, and no
// behavioural change (the construction is byte-identical).
pub use tower_net_core::nonce::nonce_for;

// `tower-net-core` is dependency-free, so it states the 13-byte CCM nonce length itself; it
// must agree with the CCM core's, or `nonce_for`'s return type wouldn't fit the `Ccm` API.
const _: () = assert!(tower_net_core::nonce::NONCE_LEN == NONCE_LEN);

/// Build a full on-air frame: clear header ‖ CCM(payload) ‖ tag. Returns the
/// total length. `out` must be at least [`MAX_FRAME`] bytes.
pub fn seal_frame(
    ccm: &mut Ccm,
    key: &[u8; 16],
    header: &Header,
    payload: &[u8],
    out: &mut [u8],
) -> Result<usize, FrameError> {
    if payload.len() > header.max_payload() {
        return Err(FrameError::PayloadTooLong);
    }
    let hlen = header.wire_len();
    let plen = payload.len();
    if out.len() < hlen + plen + TAG_LEN {
        return Err(FrameError::TooShort);
    }
    header.encode(out);
    out[hlen..hlen + plen].copy_from_slice(payload);

    let nonce = nonce_for(header.src, header.counter, header.bulk_index.unwrap_or(0));
    // Split so AAD (header) and the to-be-encrypted payload are disjoint borrows.
    let (aad, rest) = out.split_at_mut(hlen);
    let tag = ccm.seal(key, &nonce, aad, &mut rest[..plen]);
    rest[plen..plen + TAG_LEN].copy_from_slice(&tag);
    Ok(hlen + plen + TAG_LEN)
}

/// Parse + authenticate + decrypt an on-air frame in place. On success returns
/// the header and the byte range of the recovered plaintext within `buf`.
pub fn open_frame(
    ccm: &mut Ccm,
    key: &[u8; 16],
    buf: &mut [u8],
) -> Result<(Header, core::ops::Range<usize>), FrameError> {
    let (header, hlen) = Header::parse(buf)?;
    if buf.len() < hlen + TAG_LEN {
        return Err(FrameError::TooShort);
    }
    let plen = buf.len() - hlen - TAG_LEN;
    let nonce = nonce_for(header.src, header.counter, header.bulk_index.unwrap_or(0));

    // tag is the last TAG_LEN bytes; AAD is the header; ciphertext is between.
    let mut tag = [0u8; TAG_LEN];
    tag.copy_from_slice(&buf[hlen + plen..hlen + plen + TAG_LEN]);
    let (aad, rest) = buf.split_at_mut(hlen);
    if !ccm.open(key, &nonce, aad, &mut rest[..plen], &tag) {
        return Err(FrameError::AuthFail);
    }
    Ok((header, hlen..hlen + plen))
}
