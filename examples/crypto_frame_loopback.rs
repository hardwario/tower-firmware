//! crypto_frame_loopback — verify the frame codec + nonce + CCM end to end on a
//! single board (no radio link): build a secured DATA frame, then parse + open
//! it, checking the header round-trips and the payload is recovered; then tamper
//! and confirm rejection; then a bulk frame (with the 3-byte index → nonce).
//!
//!   just flash example crypto_frame_loopback   (watch with: tower logs)

#![no_std]
#![no_main]

use log::{error, info};
use tower::radio::ccm::Ccm;
use tower::radio::frame::{self, FrameType, Header, flags};
use tower::{app, board::Board};

const KEY: [u8; 16] = [
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
];

async fn run(_b: Board) {
    let mut ccm = Ccm::new();
    let mut pass = true;

    // --- 1) Non-bulk DATA frame round-trip ---
    let hdr = Header {
        frame_type: FrameType::Data,
        flags: flags::CONFIRMED,
        src: 0x1234_5678,
        dest: 0xAABB_CCDD,
        counter: 42,
        bulk_index: None,
    };
    let payload = b"hello tower radio!";
    let mut buf = [0u8; frame::MAX_FRAME];
    let n = match frame::seal_frame(&mut ccm, &KEY, &hdr, payload, &mut buf) {
        Ok(n) => n,
        Err(e) => {
            error!(target: "frame", "seal: {e}");
            return;
        }
    };
    info!(target: "frame", "sealed DATA frame: {} bytes (hdr14 + {} payload + tag8)", n, payload.len());

    match frame::open_frame(&mut ccm, &KEY, &mut buf[..n]) {
        Ok((rh, range)) => {
            let pt = &buf[range];
            let hdr_ok = rh == hdr;
            let pt_ok = pt == payload;
            info!(target: "frame", "open: header {} payload {}",
                if hdr_ok { "MATCH" } else { "MISMATCH" },
                if pt_ok { "MATCH" } else { "MISMATCH" });
            pass &= hdr_ok && pt_ok;
        }
        Err(e) => {
            error!(target: "frame", "open (valid): unexpected {e}");
            pass = false;
        }
    }

    // --- 2) Tamper a payload byte → open must fail ---
    let n2 = frame::seal_frame(&mut ccm, &KEY, &hdr, payload, &mut buf).unwrap();
    buf[frame::HDR_LEN + 2] ^= 0x40; // flip a ciphertext bit
    match frame::open_frame(&mut ccm, &KEY, &mut buf[..n2]) {
        Err(frame::FrameError::AuthFail) => {
            info!(target: "frame", "tampered frame: correctly REJECTED (AuthFail)")
        }
        other => {
            error!(target: "frame", "tampered frame: expected AuthFail, got {:?}", other);
            pass = false;
        }
    }

    // --- 3) Wrong key → open must fail ---
    let n3 = frame::seal_frame(&mut ccm, &KEY, &hdr, payload, &mut buf).unwrap();
    let mut wrong = KEY;
    wrong[0] ^= 0xFF;
    match frame::open_frame(&mut ccm, &wrong, &mut buf[..n3]) {
        Err(frame::FrameError::AuthFail) => info!(target: "frame", "wrong key: correctly REJECTED"),
        other => {
            error!(target: "frame", "wrong key: expected AuthFail, got {:?}", other);
            pass = false;
        }
    }

    // --- 4) Bulk frame: 3-byte index feeds the nonce ---
    let bhdr = Header {
        frame_type: FrameType::BulkData,
        flags: flags::LAST_CHUNK,
        src: 0x1234_5678,
        dest: 0xAABB_CCDD,
        counter: 42, // same transfer counter; index keeps nonces distinct
        bulk_index: Some(0x00_1234),
    };
    let chunk = [0xA5u8; 64];
    let nb = frame::seal_frame(&mut ccm, &KEY, &bhdr, &chunk, &mut buf).unwrap();
    info!(target: "frame", "sealed BULK frame: {} bytes (hdr17 + 64 chunk + tag8)", nb);
    match frame::open_frame(&mut ccm, &KEY, &mut buf[..nb]) {
        Ok((rh, range)) => {
            let ok = rh == bhdr && buf[range] == chunk[..];
            info!(target: "frame", "bulk open: {}", if ok { "MATCH (index in nonce ok)" } else { "MISMATCH" });
            pass &= ok;
        }
        Err(e) => {
            error!(target: "frame", "bulk open: {e}");
            pass = false;
        }
    }

    if pass {
        info!(target: "frame", "frame codec + nonce + CCM loopback: ALL PASS ***");
    } else {
        error!(target: "frame", "frame loopback: FAILURES above");
    }

    loop {
        embassy_time::Timer::after_secs(5).await;
    }
}

app!(run);
