//! Host-side codec round-trip + corruption tests. Run on the host target (the
//! workspace default is thumbv6m, which can't run tests):
//!   cargo test -p tower-protocol --target aarch64-apple-darwin

use tower_protocol::msg::*;
use tower_protocol::*;

/// Encode a frame, then feed every wire byte through a fresh decoder and return the
/// single deframed inner buffer.
fn roundtrip<T: serde::Serialize>(mt: MsgType, seq: u16, payload: &T) -> (Vec<u8>, MsgType, u16) {
    let mut out = [0u8; MAX_WIRE];
    let n = encode_frame(mt, seq, payload, &mut out).unwrap();
    // Exactly one 0x00 delimiter, at the end.
    assert_eq!(out[..n].iter().filter(|&&b| b == 0).count(), 1);
    assert_eq!(out[n - 1], 0);

    let mut dec = FrameDecoder::new();
    let mut inner_copy = None;
    let mut got = None;
    for &b in &out[..n] {
        if let Some(inner) = dec.push(b) {
            let (rmt, rseq, payload) = decode_frame(inner).unwrap();
            inner_copy = Some(inner.to_vec());
            got = Some((rmt, rseq, payload.to_vec()));
        }
    }
    let (rmt, rseq, _payload) = got.expect("a frame should complete on the 0x00");
    let _ = inner_copy;
    (out[..n].to_vec(), rmt, rseq)
}

#[test]
fn hello_roundtrip() {
    let h = Hello { protocol_version: PROTOCOL_VERSION, firmware_version: "tower 0.1.0" };
    let mut out = [0u8; MAX_WIRE];
    let n = encode_frame(MsgType::Hello, 7, &h, &mut out).unwrap();
    let mut dec = FrameDecoder::new();
    let mut decoded = false;
    for &b in &out[..n] {
        if let Some(inner) = dec.push(b) {
            let (mt, seq, payload) = decode_frame(inner).unwrap();
            assert_eq!(mt, MsgType::Hello);
            assert_eq!(seq, 7);
            let h2: Hello = postcard::from_bytes(payload).unwrap();
            assert_eq!(h2.protocol_version, PROTOCOL_VERSION);
            assert_eq!(h2.firmware_version, "tower 0.1.0");
            decoded = true;
        }
    }
    assert!(decoded);
}

#[test]
fn log_roundtrip() {
    let l = Log { level: Level::Warn, uptime_us: 1_234_567, module: "power", message: "USB connected" };
    let mut out = [0u8; MAX_WIRE];
    let n = encode_frame(MsgType::Log, 42, &l, &mut out).unwrap();
    let mut dec = FrameDecoder::new();
    let inner: Vec<u8> = (0..n).find_map(|i| dec.push(out[i]).map(|s| s.to_vec())).unwrap();
    let (mt, seq, payload) = decode_frame(&inner).unwrap();
    assert_eq!(mt, MsgType::Log);
    assert_eq!(seq, 42);
    let l2: Log = postcard::from_bytes(payload).unwrap();
    assert_eq!(l2.level, Level::Warn);
    assert_eq!(l2.uptime_us, 1_234_567);
    assert_eq!(l2.module, "power");
    assert_eq!(l2.message, "USB connected");
}

#[test]
fn event_fields_roundtrip() {
    let mut fields = heapless::Vec::new();
    fields.push(("temp", "23.5")).unwrap();
    fields.push(("rh", "41")).unwrap();
    let e = Event { name: "measurement", fields };
    let mut out = [0u8; MAX_WIRE];
    let n = encode_frame(MsgType::Event, 1, &e, &mut out).unwrap();
    let mut dec = FrameDecoder::new();
    let inner: Vec<u8> = (0..n).find_map(|i| dec.push(out[i]).map(|s| s.to_vec())).unwrap();
    let (mt, _seq, payload) = decode_frame(&inner).unwrap();
    assert_eq!(mt, MsgType::Event);
    let e2: Event = postcard::from_bytes(payload).unwrap();
    assert_eq!(e2.name, "measurement");
    assert_eq!(e2.fields.len(), 2);
    assert_eq!(e2.fields[0], ("temp", "23.5"));
    assert_eq!(e2.fields[1], ("rh", "41"));
}

#[test]
fn host_to_target_shell_command() {
    let (_wire, mt, seq) = roundtrip(
        MsgType::ShellCommand,
        9,
        &ShellCommand { cmd_id: 3, line: "/system identity print" },
    );
    assert_eq!(mt, MsgType::ShellCommand);
    assert_eq!(seq, 9);
}

#[test]
fn crc_corruption_is_rejected() {
    let h = Hello { protocol_version: PROTOCOL_VERSION, firmware_version: "x" };
    let mut out = [0u8; MAX_WIRE];
    let n = encode_frame(MsgType::Hello, 1, &h, &mut out).unwrap();
    // Flip a bit in the middle of the encoded frame (not the delimiter).
    out[n / 2] ^= 0x01;
    let mut dec = FrameDecoder::new();
    let mut saw_complete = false;
    for &b in &out[..n] {
        if let Some(inner) = dec.push(b) {
            saw_complete = true;
            // Either COBS structure broke (decode_frame on garbage) or CRC catches it.
            assert!(decode_frame(inner).is_err());
        }
    }
    // The flipped byte might also corrupt COBS structure so no frame completes;
    // either way, no valid frame is produced.
    if !saw_complete {
        // acceptable: corruption prevented frame completion entirely
    }
}

#[test]
fn version_mismatch_is_rejected() {
    let h = Hello { protocol_version: PROTOCOL_VERSION, firmware_version: "x" };
    let mut out = [0u8; MAX_WIRE];
    let n = encode_frame(MsgType::Hello, 1, &h, &mut out).unwrap();
    let mut dec = FrameDecoder::new();
    let inner: Vec<u8> = (0..n).find_map(|i| dec.push(out[i]).map(|s| s.to_vec())).unwrap();
    // Forge a wrong version in ver_type, then fix CRC so only the version check fires.
    let mut bad = inner.clone();
    bad[0] = (2 << 5) | (bad[0] & 0x1F); // version 2
    let body = bad.len() - 4;
    let crc = tower_protocol::crc::crc32_ieee(&bad[..body]);
    bad[body..].copy_from_slice(&crc.to_le_bytes());
    assert_eq!(decode_frame(&bad), Err(Error::BadVersion));
}

#[test]
fn resync_after_garbage() {
    // Leading garbage (no 0x00) then a clean frame must still decode.
    let h = Hello { protocol_version: PROTOCOL_VERSION, firmware_version: "ok" };
    let mut frame = [0u8; MAX_WIRE];
    let n = encode_frame(MsgType::Hello, 5, &h, &mut frame).unwrap();
    let mut dec = FrameDecoder::new();
    // Garbage with an embedded 0x00 to force a resync boundary.
    for &b in &[0x11u8, 0x22, 0x00] {
        let _ = dec.push(b);
    }
    let mut ok = false;
    for &b in &frame[..n] {
        if let Some(inner) = dec.push(b) {
            let (mt, seq, _) = decode_frame(inner).unwrap();
            assert_eq!(mt, MsgType::Hello);
            assert_eq!(seq, 5);
            ok = true;
        }
    }
    assert!(ok);
}

// ---- boundary sizes & the inner-frame budget --------------------------------

/// Bytes a payload may occupy: MAX_FRAME minus ver_type(1) + seq(2) + crc32(4).
/// Every producer's owned buffers must keep their postcard encoding within this.
const PAYLOAD_BUDGET: usize = MAX_FRAME - 3 - 4;

/// The largest `Log` the firmware can produce (192-char message, 24-char module,
/// worst-case uptime) must encode and round-trip — guards the firmware's MAX_MSG.
#[test]
fn max_log_fits_and_roundtrips() {
    let message: String = core::iter::repeat('x').take(192).collect();
    let module: String = core::iter::repeat('m').take(24).collect();
    let l = Log { level: Level::Trace, uptime_us: u64::MAX, module: &module, message: &message };
    let mut out = [0u8; MAX_WIRE];
    let n = encode_frame(MsgType::Log, u16::MAX, &l, &mut out).expect("max log must fit the budget");

    let mut dec = FrameDecoder::new();
    let inner: Vec<u8> = (0..n).find_map(|i| dec.push(out[i]).map(|s| s.to_vec())).unwrap();
    let (mt, seq, payload) = decode_frame(&inner).unwrap();
    assert_eq!(mt, MsgType::Log);
    assert_eq!(seq, u16::MAX);
    let l2: Log = postcard::from_bytes(payload).unwrap();
    assert_eq!(l2.message.len(), 192);
    assert_eq!(l2.module.len(), 24);
}

/// A payload past the budget must return `Err(Encode)` — never a silently
/// truncated (and thus corrupt) frame.
#[test]
fn oversize_payload_is_rejected_not_truncated() {
    let big: String = core::iter::repeat('z').take(PAYLOAD_BUDGET + 50).collect();
    let mut out = [0u8; MAX_WIRE];
    assert_eq!(encode_frame(MsgType::Print, 0, &Print { text: &big }, &mut out), Err(Error::Encode));
}

/// An output buffer too small for the COBS frame must return `Err(Overflow)`.
#[test]
fn small_output_buffer_overflows() {
    let h = Hello { protocol_version: PROTOCOL_VERSION, firmware_version: "tower 0.1.0" };
    let mut tiny = [0u8; 4];
    assert_eq!(encode_frame(MsgType::Hello, 0, &h, &mut tiny), Err(Error::Overflow));
}

/// A too-short inner buffer is rejected before any CRC read (no panic / OOB).
#[test]
fn short_inner_is_rejected() {
    assert_eq!(decode_frame(&[]), Err(Error::TooShort));
    assert_eq!(decode_frame(&[0x20, 0, 0, 0, 0, 0]), Err(Error::TooShort)); // 6 < HDR+CRC
}

// ---- decoder state machine --------------------------------------------------

/// A frame longer than MAX_WIRE is dropped on its delimiter, and the decoder
/// resynchronizes on the very next clean frame (no wedging).
#[test]
fn decoder_drops_oversize_then_resyncs() {
    let mut dec = FrameDecoder::new();
    for _ in 0..(MAX_WIRE + 16) {
        assert!(dec.push(0xAB).is_none());
    }
    assert!(dec.push(0x00).is_none(), "oversize frame must be dropped");

    let h = Hello { protocol_version: PROTOCOL_VERSION, firmware_version: "ok" };
    let mut out = [0u8; MAX_WIRE];
    let n = encode_frame(MsgType::Hello, 3, &h, &mut out).unwrap();
    let inner: Vec<u8> = (0..n).find_map(|i| dec.push(out[i]).map(|s| s.to_vec())).unwrap();
    assert_eq!(decode_frame(&inner).unwrap().0, MsgType::Hello);
}

/// `reset()` discards a partial frame so a reconnect can't splice old + new bytes.
#[test]
fn decoder_reset_discards_partial() {
    let mut dec = FrameDecoder::new();
    assert!(dec.push(0x05).is_none());
    assert!(dec.push(0x06).is_none());
    dec.reset();
    let h = Hello { protocol_version: PROTOCOL_VERSION, firmware_version: "ok" };
    let mut out = [0u8; MAX_WIRE];
    let n = encode_frame(MsgType::Hello, 1, &h, &mut out).unwrap();
    let inner: Vec<u8> = (0..n).find_map(|i| dec.push(out[i]).map(|s| s.to_vec())).unwrap();
    assert_eq!(decode_frame(&inner).unwrap().1, 1);
}

/// Empty payloads (e.g. a zero-field Print) still frame and round-trip.
#[test]
fn empty_payload_roundtrips() {
    let mut out = [0u8; MAX_WIRE];
    let n = encode_frame(MsgType::Print, 0, &Print { text: "" }, &mut out).unwrap();
    let mut dec = FrameDecoder::new();
    let inner: Vec<u8> = (0..n).find_map(|i| dec.push(out[i]).map(|s| s.to_vec())).unwrap();
    let (mt, _, payload) = decode_frame(&inner).unwrap();
    assert_eq!(mt, MsgType::Print);
    assert_eq!(postcard::from_bytes::<Print>(payload).unwrap().text, "");
}

// ---- message-type mapping ---------------------------------------------------

#[test]
fn msg_type_from_u8_is_exhaustive() {
    for v in [0u8, 1, 2, 3, 4, 5, 6, 16, 17] {
        assert!(MsgType::from_u8(v).is_some(), "type {v} should be known");
        assert_eq!(MsgType::from_u8(v).unwrap() as u8, v, "round-trip discriminant");
    }
    for v in [7u8, 8, 14, 15, 18, 19, 31, 100, 255] {
        assert!(MsgType::from_u8(v).is_none(), "type {v} should be unknown");
    }
}

#[test]
fn unknown_type_is_rejected() {
    // Forge a frame with a valid version but type 15 (unused), fixing the CRC.
    let h = Hello { protocol_version: PROTOCOL_VERSION, firmware_version: "x" };
    let mut out = [0u8; MAX_WIRE];
    let n = encode_frame(MsgType::Hello, 1, &h, &mut out).unwrap();
    let mut dec = FrameDecoder::new();
    let inner: Vec<u8> = (0..n).find_map(|i| dec.push(out[i]).map(|s| s.to_vec())).unwrap();
    let mut bad = inner.clone();
    bad[0] = (PROTOCOL_VERSION << 5) | 15;
    let body = bad.len() - 4;
    let crc = tower_protocol::crc::crc32_ieee(&bad[..body]);
    bad[body..].copy_from_slice(&crc.to_le_bytes());
    assert_eq!(decode_frame(&bad), Err(Error::BadType));
}

// ---- COBS invariant: no interior zero ---------------------------------------

/// A payload deliberately full of NUL bytes must produce a wire frame whose only
/// `0x00` is the trailing delimiter (the property the byte-fed decoder relies on).
#[test]
fn cobs_output_has_only_the_trailing_zero() {
    // seq=0 also forces zero bytes into the header.
    let p = Print { text: "\0\0\0\0\0\0\0\0" };
    let mut out = [0u8; MAX_WIRE];
    let n = encode_frame(MsgType::Print, 0, &p, &mut out).unwrap();
    assert_eq!(out[..n - 1].iter().filter(|&&b| b == 0).count(), 0, "no interior zero");
    assert_eq!(out[n - 1], 0, "trailing delimiter");
    let mut dec = FrameDecoder::new();
    let inner: Vec<u8> = (0..n).find_map(|i| dec.push(out[i]).map(|s| s.to_vec())).unwrap();
    let (_, _, payload) = decode_frame(&inner).unwrap();
    assert_eq!(postcard::from_bytes::<Print>(payload).unwrap().text, "\0\0\0\0\0\0\0\0");
}

// ---- deterministic fuzzing --------------------------------------------------

/// A tiny LCG — deterministic across runs, no `rand` dependency.
struct Lcg(u64);
impl Lcg {
    fn next_u32(&mut self) -> u32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 32) as u32
    }
    fn ascii(&mut self, len: usize) -> String {
        (0..len).map(|_| (b'!' + (self.next_u32() % 90) as u8) as char).collect()
    }
}

/// Random payloads of random length must round-trip byte-exact through the full
/// encode → byte-fed decode → decode_frame → deserialize path.
#[test]
fn fuzz_roundtrip_exact() {
    let mut rng = Lcg(0x0BAD_F00D_DEAD_BEEF);
    for _ in 0..4000 {
        let len = (rng.next_u32() as usize) % (PAYLOAD_BUDGET - 8);
        let text = rng.ascii(len);
        let seq = rng.next_u32() as u16;
        let mut out = [0u8; MAX_WIRE];
        let n = encode_frame(MsgType::Print, seq, &Print { text: &text }, &mut out).unwrap();
        assert_eq!(out[..n - 1].iter().filter(|&&b| b == 0).count(), 0);
        let mut dec = FrameDecoder::new();
        let inner: Vec<u8> = (0..n).find_map(|i| dec.push(out[i]).map(|s| s.to_vec())).unwrap();
        let (mt, rseq, payload) = decode_frame(&inner).unwrap();
        assert_eq!(mt, MsgType::Print);
        assert_eq!(rseq, seq);
        assert_eq!(postcard::from_bytes::<Print>(payload).unwrap().text, text);
    }
}

/// Any single-bit corruption (outside the trailing delimiter) must be detected:
/// every frame the decoder completes from a corrupted stream fails `decode_frame`.
/// CRC-32 covers all content bytes, so a flipped bit can never yield a frame that
/// both reframes and matches its stored CRC.
#[test]
fn fuzz_single_bit_flip_is_always_detected() {
    let mut rng = Lcg(0xC0FFEE_1234_5678);
    let (mut completed, mut no_frame) = (0u64, 0u64);
    for _ in 0..5000 {
        let len = (rng.next_u32() as usize) % 160;
        let text = rng.ascii(len);
        let seq = rng.next_u32() as u16;
        let mut out = [0u8; MAX_WIRE];
        let n = encode_frame(MsgType::Print, seq, &Print { text: &text }, &mut out).unwrap();
        if n <= 1 {
            continue;
        }
        let bit = (rng.next_u32() as usize) % ((n - 1) * 8); // never the trailing 0x00
        out[bit / 8] ^= 1u8 << (bit % 8);

        let mut dec = FrameDecoder::new();
        let mut any = false;
        for &b in &out[..n] {
            if let Some(inner) = dec.push(b) {
                any = true;
                assert!(
                    decode_frame(inner).is_err(),
                    "single-bit corruption slipped through: seq={seq} text={text:?}"
                );
            }
        }
        if any {
            completed += 1;
        } else {
            no_frame += 1;
        }
    }
    // Both detection paths (CRC-rejected and COBS-broke-no-frame) get exercised.
    assert!(completed > 0 && no_frame > 0, "fuzz should hit both paths: {completed} vs {no_frame}");
}
