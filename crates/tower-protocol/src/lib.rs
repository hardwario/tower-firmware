//! TOWER console wire protocol — the single source of truth for the host↔target
//! link, shared by the `tower` firmware and the `tower-cli` host.
//!
//! Frame on the wire:
//! ```text
//! wire:   COBS( inner )  0x00
//! inner:  ver_type(1) | seq(2, LE) | payload(postcard) | crc32(4, LE)
//!         ver_type = (PROTOCOL_VERSION << 5) | (msg_type & 0x1F)
//!         crc32    = CRC-32/IEEE over [ver_type, seq, payload...]
//! ```
//!
//! [`encode_frame`] builds the whole wire frame (used by every producer, including
//! the panic path). On receive, feed bytes to a [`FrameDecoder`] until it yields a
//! deframed inner buffer, then [`decode_frame`] checks version + CRC and returns the
//! `(MsgType, seq, payload)`; deserialize the payload with `postcard::from_bytes`.

#![no_std]

pub mod crc;
pub mod fota;
pub mod msg;

pub use msg::*;
use serde::Serialize;

/// Protocol version, carried in the top 3 bits of `ver_type` on every frame.
pub const PROTOCOL_VERSION: u8 = 1;

/// Max inner frame (ver_type + seq + payload + crc), pre-COBS.
pub const MAX_FRAME: usize = 256;
/// Max wire frame: COBS worst-case expansion of [`MAX_FRAME`] plus the `0x00`.
/// COBS adds `ceil(n/254)` overhead bytes; 272 leaves comfortable headroom.
pub const MAX_WIRE: usize = 272;

const HDR: usize = 3; // ver_type(1) + seq(2)
const CRC_LEN: usize = 4;

/// Console message types — the low 5 bits of `ver_type`. Target→host are 0..=15,
/// host→target are 16+. postcard never sees this enum (it's the raw `ver_type`
/// byte), so the discriminants are the wire values.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum MsgType {
    // target -> host
    Hello = 0,
    Log = 1,
    Print = 2,
    Event = 3,
    ShellResponse = 4,
    ShellCompletions = 5,
    Dropped = 6,
    /// FOTA host-proxy: the target (a FOTA gateway) asks the host for image/manifest
    /// bytes. **Raw** payload (not postcard): `offset(4, LE) ‖ len(2, LE)` — `offset ==
    /// u32::MAX` ([`fota::FOTA_MANIFEST_OFFSET`]) requests the signed manifest. See
    /// `docs/fota.md` "host-proxy".
    FotaReq = 7,
    // host -> target
    ShellCommand = 16,
    ShellComplete = 17,
    /// FOTA host-proxy reply: the host returns the requested bytes. **Raw** payload (not
    /// postcard): `offset(4, LE) ‖ bytes…` (offset echoes the request, or `u32::MAX` for
    /// the manifest).
    FotaData = 18,
}

impl MsgType {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Hello,
            1 => Self::Log,
            2 => Self::Print,
            3 => Self::Event,
            4 => Self::ShellResponse,
            5 => Self::ShellCompletions,
            6 => Self::Dropped,
            7 => Self::FotaReq,
            16 => Self::ShellCommand,
            17 => Self::ShellComplete,
            18 => Self::FotaData,
            _ => return None,
        })
    }
}

/// Codec errors. All are "drop the frame" conditions on the receive side.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {
    /// The payload didn't fit / failed to serialize.
    Encode,
    /// The output buffer is too small for the COBS-encoded frame.
    Overflow,
    /// Inner frame shorter than the minimum (`ver_type + seq + crc`).
    TooShort,
    /// Protocol version in `ver_type` not understood.
    BadVersion,
    /// Unknown message type.
    BadType,
    /// CRC mismatch (corruption on the wire).
    BadCrc,
}

/// Build a complete wire frame `COBS(ver_type ‖ seq ‖ postcard(payload) ‖ crc) ‖ 0x00`
/// into `out`; returns the byte count written. Pure computation — safe from any
/// context (IRQ, panic handler).
pub fn encode_frame<T: Serialize>(
    msg_type: MsgType,
    seq: u16,
    payload: &T,
    out: &mut [u8],
) -> Result<usize, Error> {
    let mut inner = [0u8; MAX_FRAME];
    inner[0] = (PROTOCOL_VERSION << 5) | (msg_type as u8 & 0x1F);
    inner[1..3].copy_from_slice(&seq.to_le_bytes());

    let used = postcard::to_slice(payload, &mut inner[HDR..])
        .map_err(|_| Error::Encode)?
        .len();
    let body = HDR + used;
    if body + CRC_LEN > MAX_FRAME {
        return Err(Error::Encode);
    }
    let crc = crc::crc32_ieee(&inner[..body]);
    inner[body..body + CRC_LEN].copy_from_slice(&crc.to_le_bytes());
    let inner_len = body + CRC_LEN;

    if cobs::max_encoding_length(inner_len) + 1 > out.len() {
        return Err(Error::Overflow);
    }
    let enc = cobs::encode(&inner[..inner_len], out);
    out[enc] = 0x00;
    Ok(enc + 1)
}

/// Like [`encode_frame`] but the payload is **raw bytes**, not postcard-serialized — for
/// message types whose payload is a fixed binary layout ([`MsgType::FotaReq`] /
/// [`MsgType::FotaData`]), so a host implementation needs only COBS + CRC, no postcard.
/// The frame is otherwise identical (`ver_type ‖ seq ‖ payload ‖ crc`, COBS-wrapped), so
/// [`decode_frame`] reads it back the same way and returns `payload` as the raw slice.
pub fn encode_frame_raw(
    msg_type: MsgType,
    seq: u16,
    payload: &[u8],
    out: &mut [u8],
) -> Result<usize, Error> {
    let mut inner = [0u8; MAX_FRAME];
    inner[0] = (PROTOCOL_VERSION << 5) | (msg_type as u8 & 0x1F);
    inner[1..3].copy_from_slice(&seq.to_le_bytes());

    let body = HDR + payload.len();
    if body + CRC_LEN > MAX_FRAME {
        return Err(Error::Encode);
    }
    inner[HDR..body].copy_from_slice(payload);
    let crc = crc::crc32_ieee(&inner[..body]);
    inner[body..body + CRC_LEN].copy_from_slice(&crc.to_le_bytes());
    let inner_len = body + CRC_LEN;

    if cobs::max_encoding_length(inner_len) + 1 > out.len() {
        return Err(Error::Overflow);
    }
    let enc = cobs::encode(&inner[..inner_len], out);
    out[enc] = 0x00;
    Ok(enc + 1)
}

/// Validate version + CRC on a deframed inner buffer and split it into
/// `(msg_type, seq, payload)`. Deserialize `payload` with `postcard::from_bytes`.
pub fn decode_frame(inner: &[u8]) -> Result<(MsgType, u16, &[u8]), Error> {
    if inner.len() < HDR + CRC_LEN {
        return Err(Error::TooShort);
    }
    let ver_type = inner[0];
    if (ver_type >> 5) != PROTOCOL_VERSION {
        return Err(Error::BadVersion);
    }
    let msg_type = MsgType::from_u8(ver_type & 0x1F).ok_or(Error::BadType)?;
    let seq = u16::from_le_bytes([inner[1], inner[2]]);
    let body = inner.len() - CRC_LEN;
    let stored = u32::from_le_bytes([inner[body], inner[body + 1], inner[body + 2], inner[body + 3]]);
    if crc::crc32_ieee(&inner[..body]) != stored {
        return Err(Error::BadCrc);
    }
    Ok((msg_type, seq, &inner[HDR..body]))
}

/// Byte-fed COBS deframer. Feed received bytes one at a time; on the `0x00`
/// delimiter it COBS-decodes the accumulated frame **in place** and returns the
/// inner bytes (pass them to [`decode_frame`]). A frame larger than [`MAX_WIRE`] or
/// a COBS error yields `None` (dropped); the next `0x00` resynchronizes.
pub struct FrameDecoder {
    buf: [u8; MAX_WIRE],
    len: usize,
    overflow: bool,
}

impl Default for FrameDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameDecoder {
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_WIRE],
            len: 0,
            overflow: false,
        }
    }

    /// Discard any partial frame (e.g. on reconnect / after a gap).
    pub fn reset(&mut self) {
        self.len = 0;
        self.overflow = false;
    }

    /// Feed one byte. Returns the deframed inner bytes when `b` completes a frame.
    pub fn push(&mut self, b: u8) -> Option<&[u8]> {
        if b != 0 {
            if self.len < self.buf.len() {
                self.buf[self.len] = b;
                self.len += 1;
            } else {
                self.overflow = true;
            }
            return None;
        }
        // Frame boundary.
        let len = self.len;
        let overflow = self.overflow;
        self.len = 0;
        self.overflow = false;
        if len == 0 || overflow {
            return None;
        }
        match cobs::decode_in_place(&mut self.buf[..len]) {
            Ok(n) => Some(&self.buf[..n]),
            Err(_) => None,
        }
    }
}
