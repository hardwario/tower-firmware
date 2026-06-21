//! edge_frame_limits — MTU + malformed/forged-frame rejection KAT (§3/§6/§9).
//! Single board, no radio link: drives the frame codec + AES-CCM at its
//! boundaries and asserts each accept/reject, printing one PASS/FAIL verdict.
//!
//!   just flash edge_frame_limits
//!
//! Covers: payload MTU (1 B / 74 B accept, 75 B reject; bulk 64 B accept, 65 B
//! reject), and the receive-side drops — bad version, unknown type, truncated,
//! tampered ciphertext, tampered tag, wrong key — each must yield the right error.

#![no_std]
#![no_main]

use log::{error, info};
use tower::radio::ccm::Ccm;
use tower::radio::frame::{self, FrameError, FrameType, Header, MAX_FRAME, flags};
use tower::{app, board::Board};

const KEY: [u8; 16] = [
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
];
const KEY2: [u8; 16] = [0xA5; 16];

fn data_hdr(counter: u32) -> Header {
    Header { frame_type: FrameType::Data, flags: flags::CONFIRMED, src: 0x1111_1111, dest: 0x2222_2222, counter, bulk_index: None }
}
fn bulk_hdr(counter: u32, idx: u32) -> Header {
    Header { frame_type: FrameType::BulkData, flags: 0, src: 0x1111_1111, dest: 0x2222_2222, counter, bulk_index: Some(idx) }
}

async fn run(_b: Board) {
    let mut ccm = Ccm::new();
    let mut pass = true;
    let mut check = |name: &str, ok: bool| {
        if ok {
            info!(target: "edge", "  {} ✓", name);
        } else {
            error!(target: "edge", "  {} ✗ FAIL", name);
        }
        pass &= ok;
    };

    let mut buf = [0u8; MAX_FRAME];

    // --- MTU accept/reject (seal side) ---
    check("seal 1 B accepted", frame::seal_frame(&mut ccm, &KEY, &data_hdr(1), &[0x5A], &mut buf).is_ok());
    let p74 = [0xA5u8; 74];
    check("seal 74 B accepted", frame::seal_frame(&mut ccm, &KEY, &data_hdr(2), &p74, &mut buf).is_ok());
    let p75 = [0xA5u8; 75];
    check(
        "seal 75 B rejected (PayloadTooLong)",
        frame::seal_frame(&mut ccm, &KEY, &data_hdr(3), &p75, &mut buf) == Err(FrameError::PayloadTooLong),
    );
    let c64 = [0x33u8; 64];
    check("bulk seal 64 B accepted", frame::seal_frame(&mut ccm, &KEY, &bulk_hdr(4, 0), &c64, &mut buf).is_ok());
    let c65 = [0x33u8; 65];
    check(
        "bulk seal 65 B rejected (PayloadTooLong)",
        frame::seal_frame(&mut ccm, &KEY, &bulk_hdr(5, 0), &c65, &mut buf) == Err(FrameError::PayloadTooLong),
    );

    // --- Receive-side drops (open side). Build one good frame, then corrupt it. ---
    let payload = *b"edge-case-payload";
    let good_len = frame::seal_frame(&mut ccm, &KEY, &data_hdr(42), &payload, &mut buf).unwrap();

    // Valid frame opens and round-trips.
    {
        let mut b = buf;
        let r = frame::open_frame(&mut ccm, &KEY, &mut b[..good_len]);
        let ok = matches!(&r, Ok((h, range)) if h.counter == 42 && b[range.clone()] == payload[..]);
        check("valid frame opens + round-trips", ok);
    }
    // Bad version: flip a version bit (bits[7:5]).
    {
        let mut b = buf;
        b[0] ^= 0x20;
        check("bad version → BadVersion", frame::open_frame(&mut ccm, &KEY, &mut b[..good_len]) == Err(FrameError::BadVersion));
    }
    // Unknown type: set type field (bits[4:0]) to 7 (JoinConfirm=6 is the max valid).
    {
        let mut b = buf;
        b[0] = (b[0] & 0xE0) | 0x07;
        check("unknown type → BadType", frame::open_frame(&mut ccm, &KEY, &mut b[..good_len]) == Err(FrameError::BadType));
    }
    // Truncated below header+tag.
    {
        let mut b = buf;
        check("truncated → TooShort", frame::open_frame(&mut ccm, &KEY, &mut b[..8]) == Err(FrameError::TooShort));
    }
    // Tampered ciphertext byte (a payload byte after the 14-byte header).
    {
        let mut b = buf;
        b[16] ^= 0x01;
        check("tampered ciphertext → AuthFail", frame::open_frame(&mut ccm, &KEY, &mut b[..good_len]) == Err(FrameError::AuthFail));
    }
    // Tampered tag (last byte).
    {
        let mut b = buf;
        b[good_len - 1] ^= 0x80;
        check("tampered tag → AuthFail", frame::open_frame(&mut ccm, &KEY, &mut b[..good_len]) == Err(FrameError::AuthFail));
    }
    // Wrong key.
    {
        let mut b = buf;
        check("wrong key → AuthFail", frame::open_frame(&mut ccm, &KEY2, &mut b[..good_len]) == Err(FrameError::AuthFail));
    }

    if pass {
        info!(target: "edge", "edge_frame_limits: ALL PASS ***");
    } else {
        error!(target: "edge", "edge_frame_limits: FAIL");
    }
    loop {
        embassy_time::Timer::after_secs(5).await;
    }
}

app!(run);
