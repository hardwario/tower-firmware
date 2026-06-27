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
