//! HIL extended group: the full example-matrix sweep a two-Dongle bench can exercise
//! (docs/test-plan.md scope, beyond the committed smoke/radio groups).
//!
//! Single-board KATs (crypto, FHSS, duty governor, frame-codec edges) run on the Dongle slot;
//! two-board RF scenarios (P2P, secure ping, OTA pairing, bulk pull, rapid-fire ordering) use
//! the `core` slot as the second radio peer — on this bench both slots are Radio Dongles, both
//! flashed over the UART bootloader, so the J-Link/PPK2 wiring the power group needs is not
//! required here.
//!
//! `#[ignore]`d like the other groups: `cargo test` only compiles them; run on the bench with
//! `just hil ext` (`--test-threads=1` — the serial ports are exclusive).

use std::process::Command;
use std::time::{Duration, Instant};

use tower_hil::{bench_or_fail, frame_text, Console, Frame};

/// Build an example (optionally with TOWER_FEATURES) and flash it to `port`. Sequential by
/// design — concurrent FTDI flashing is unreliable (see tools/hwtest/README.md).
fn build_and_flash(port: &str, example: &str, features: &str) -> Result<(), String> {
    let repo = concat!(env!("CARGO_MANIFEST_DIR"), "/../..");
    let mut build = Command::new("just");
    build.current_dir(repo).args(["build", "example", example]);
    if !features.is_empty() {
        build.env("TOWER_FEATURES", features);
    }
    let out = build.output().map_err(|e| format!("HIL: spawn just build {example}: {e}"))?;
    if !out.status.success() {
        return Err(format!(
            "HIL: build {example} ({features}) failed:\n{}",
            String::from_utf8_lossy(&out.stderr)
        ));
    }
    let bin = format!("{repo}/target/firmware.bin");
    let out = Command::new("tower")
        .args(["-d", port, "flash", &bin])
        .output()
        .map_err(|e| format!("HIL: spawn tower flash: {e}"))?;
    if out.status.success() {
        Ok(())
    } else {
        Err(format!(
            "HIL: flash {example} to {port} failed:\n{}\n{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ))
    }
}

/// Flash a one-shot KAT to the Dongle, reset, and await its verdict: pass if a frame contains
/// `pass`, fail fast on any frame containing one of `fails`. Tracks the device's `Dropped`
/// markers so a verdict lost to the console writer's drop-newest queue (TX_DEPTH) is diagnosed
/// as such, not as "did it boot?". Also asserts gap-free transport (seq gaps ≠ writer drops:
/// the writer assigns seq at encode, so its drops surface ONLY via the Dropped marker).
fn run_kat(example: &str, pass: &str, fails: &[&str], secs: u64) {
    let bench = bench_or_fail();
    build_and_flash(&bench.dongle.serial, example, "").expect("build+flash KAT");

    let mut console = Console::open(&bench.dongle.serial).expect("open Dongle console");
    console.resync();
    console.reset_into_app().expect("reset the Dongle into the app");

    let deadline = Instant::now() + Duration::from_secs(secs);
    let mut dropped: u32 = 0;
    let mut verdict: Option<String> = None;
    while Instant::now() < deadline {
        match console.next(Duration::from_millis(500)).expect("console read") {
            Some(Frame::Dropped { count }) => dropped += count,
            Some(f) => {
                if let Some(t) = frame_text(&f)
                    && (t.contains(pass) || fails.iter().any(|n| t.contains(n)))
                {
                    verdict = Some(t.to_string());
                    break;
                }
            }
            None => {}
        }
    }

    let text = verdict.unwrap_or_else(|| {
        if dropped > 0 {
            panic!(
                "{example}: no verdict within {secs} s, and the device reported {dropped} \
                 dropped console frame(s) — the verdict line likely fell to the writer's \
                 drop-newest queue (src/console.rs TX_DEPTH); re-emit the verdict periodically"
            )
        }
        panic!("{example}: no verdict within {secs} s — did it boot?")
    });
    assert!(
        text.contains(pass),
        "{example} reported a failure: {text:?}"
    );
    assert_eq!(console.seq_gaps(), 0, "{example}: transport dropped frames during the KAT");
}

/// Parse the decimal number right after `key` in `text` (e.g. `tx_counter=` -> 1025).
fn field_u64(text: &str, key: &str) -> Option<u64> {
    let rest = &text[text.find(key)? + key.len()..];
    let end = rest.find(|c: char| !c.is_ascii_digit()).unwrap_or(rest.len());
    rest[..end].parse().ok()
}

/// Parse the 8-hex-digit id right after `key` (e.g. `id=` -> 0x1234ABCD).
fn field_hex32(text: &str, key: &str) -> Option<u32> {
    let rest = &text[text.find(key)? + key.len()..];
    u32::from_str_radix(rest.get(..8)?, 16).ok()
}

// ---------------------------------------------------------------------------
// Single-board KATs (Dongle slot)
// ---------------------------------------------------------------------------

/// L0 hardware AES vs the FIPS-197 App. B vector (key load, byte order, block compute).
#[test]
#[ignore = "requires the HIL bench (TOWER Radio Dongle on the fixture port)"]
fn ext_crypto_aes_kat() {
    run_kat("crypto_aes_kat", "MATCH ***", &["MISMATCH"], 8);
}

/// Frame codec + nonce + CCM loopback, incl. tamper + wrong-key rejection.
#[test]
#[ignore = "requires the HIL bench (TOWER Radio Dongle on the fixture port)"]
fn ext_crypto_frame_loopback() {
    // 12 s: the example re-emits its verdict every 5 s (the boot-burst copy can be a casualty
    // of the TX queue), so the window must cover at least one re-emission cycle.
    run_kat("crypto_frame_loopback", "ALL PASS ***", &["MISMATCH", "expected AuthFail"], 12);
}

/// FHSS hop permutation + beacon frame round-trip KAT.
#[test]
#[ignore = "requires the HIL bench (TOWER Radio Dongle on the fixture port)"]
fn ext_fhss_kat() {
    run_kat("fhss_kat", "ALL PASS ***", &["FAIL"], 10);
}

/// Duty-cycle governor KAT (the regulatory token-bucket arithmetic, on target).
#[test]
#[ignore = "requires the HIL bench (TOWER Radio Dongle on the fixture port)"]
fn ext_net_duty_kat() {
    run_kat("net_duty_kat", "ALL PASS ***", &["FAIL"], 10);
}

/// Frame-size limits / boundary conditions of the codec.
#[test]
#[ignore = "requires the HIL bench (TOWER Radio Dongle on the fixture port)"]
fn ext_edge_frame_limits() {
    run_kat("edge_frame_limits", "ALL PASS ***", &["FAIL"], 10);
}

/// Radio state machine recovery — must never wedge.
#[test]
#[ignore = "requires the HIL bench (TOWER Radio Dongle on the fixture port)"]
fn ext_edge_recovery() {
    run_kat("edge_recovery", "ALL PASS ***", &["FAIL"], 25);
}

// ---------------------------------------------------------------------------
// Two-board RF scenarios (core slot = second Dongle as radio peer)
// ---------------------------------------------------------------------------

/// Bidirectional confirmed P2P: A pings (Delivered), B receives + pongs (Delivered),
/// A receives the pong — both directions of the half-duplex link with per-link keys.
#[test]
#[ignore = "requires the HIL bench (two TOWER boards in RF range)"]
fn ext_net_p2p_bidirectional() {
    let bench = bench_or_fail();
    build_and_flash(&bench.dongle.serial, "net_p2p", "role-peer-b").expect("flash peer B");
    build_and_flash(&bench.core.serial, "net_p2p", "role-peer-a").expect("flash peer A");

    let mut a = Console::open(&bench.core.serial).expect("open A console");
    let mut b = Console::open(&bench.dongle.serial).expect("open B console");
    a.resync();
    b.resync();
    b.reset_into_app().expect("reset B");
    a.reset_into_app().expect("reset A");

    // A: confirmed PING delivered, and B's PONG received back.
    let a_ping = a
        .wait_for(Duration::from_secs(30), |f| {
            frame_text(f).is_some_and(|t| t.contains("PING") && t.contains("Delivered"))
        })
        .expect("A console read");
    assert!(a_ping.is_some(), "A never reported a Delivered PING within 30 s");
    assert!(
        a.wait_for_text(Duration::from_secs(15), "A: rx").expect("A console read"),
        "A never received B's PONG"
    );

    // B: received A's PING (auto-ACKed) and its own confirmed PONG delivered.
    assert!(
        b.wait_for_text(Duration::from_secs(15), "B: rx").expect("B console read"),
        "B never logged A's PING"
    );
    let b_pong = b
        .wait_for(Duration::from_secs(15), |f| {
            frame_text(f).is_some_and(|t| t.contains("PONG") && t.contains("Delivered"))
        })
        .expect("B console read");
    assert!(b_pong.is_some(), "B never reported a Delivered PONG");
}

/// Full stack over the air: CCM-sealed DATA sent by the node, authenticated + decrypted by
/// the gateway (AUTH OK), with no auth failures in the capture window.
#[test]
#[ignore = "requires the HIL bench (two TOWER boards in RF range)"]
fn ext_net_secure_ping_auth() {
    let bench = bench_or_fail();
    build_and_flash(&bench.dongle.serial, "net_secure_ping", "").expect("flash gateway");
    build_and_flash(&bench.core.serial, "net_secure_ping", "role-node").expect("flash node");

    let mut node = Console::open(&bench.core.serial).expect("open node console");
    let mut gw = Console::open(&bench.dongle.serial).expect("open gateway console");
    node.resync();
    gw.resync();
    gw.reset_into_app().expect("reset gateway");
    node.reset_into_app().expect("reset node");

    assert!(
        node.wait_for_text(Duration::from_secs(20), "tx cnt=").expect("node console read"),
        "node never sent a sealed frame"
    );
    let frames = gw.collect_for(Duration::from_secs(15)).expect("gateway console read");
    let auth_ok = frames.iter().filter_map(frame_text).filter(|t| t.contains("AUTH OK")).count();
    let auth_fail = frames.iter().filter_map(frame_text).filter(|t| t.contains("auth FAIL")).count();
    assert!(auth_ok > 0, "gateway never authenticated a frame (frames: {frames:?})");
    assert_eq!(auth_fail, 0, "gateway saw CCM auth failures");
}

/// OTA 3-way pairing: host opens a window, joiner joins with its own id; both sides must
/// log the SAME node id and the same handed-out key prefix (a0a1a2a3).
#[test]
#[ignore = "requires the HIL bench (two TOWER boards in RF range)"]
fn ext_net_pairing_handshake() {
    let bench = bench_or_fail();
    build_and_flash(&bench.dongle.serial, "net_pairing", "").expect("flash host (gateway role)");
    build_and_flash(&bench.core.serial, "net_pairing", "role-node").expect("flash joiner");

    let mut host = Console::open(&bench.dongle.serial).expect("open host console");
    let mut joiner = Console::open(&bench.core.serial).expect("open joiner console");
    host.resync();
    joiner.resync();
    // Host first: it opens the pairing window the joiner's JOIN_REQ must land in.
    host.reset_into_app().expect("reset host");
    joiner.reset_into_app().expect("reset joiner");

    let joined = joiner
        .wait_for(Duration::from_secs(70), |f| {
            frame_text(f).is_some_and(|t| t.contains("JOINED ***") || t.contains("join failed"))
        })
        .expect("joiner console read")
        .expect("joiner never concluded the handshake within 70 s");
    let jt = frame_text(&joined).unwrap_or("").to_string();
    assert!(jt.contains("JOINED ***"), "join failed on the joiner: {jt:?}");
    assert!(jt.contains("a0a1a2a3"), "joiner got an unexpected key: {jt:?}");

    let paired = host
        .wait_for(Duration::from_secs(15), |f| {
            frame_text(f).is_some_and(|t| t.contains("PAIRED ***"))
        })
        .expect("host console read")
        .expect("host never logged PAIRED");
    let ht = frame_text(&paired).unwrap_or("").to_string();
    assert!(ht.contains("a0a1a2a3"), "host handed an unexpected key: {ht:?}");

    // The joiner chooses its id; the host must have learned exactly that id.
    let host_id = field_hex32(&ht, "id=").expect("host PAIRED line has no id=");
    let joiner_id = field_hex32(&jt, "id=").expect("joiner JOINED line has no id=");
    assert_eq!(host_id, joiner_id, "host installed a different node id than the joiner chose");
}

/// Bulk pull: requester fetches the 200-byte blob in ≤64 B chunks and verifies the pattern.
#[test]
#[ignore = "requires the HIL bench (two TOWER boards in RF range)"]
fn ext_net_bulk_pull() {
    let bench = bench_or_fail();
    build_and_flash(&bench.dongle.serial, "net_bulk", "").expect("flash sender (gateway role)");
    build_and_flash(&bench.core.serial, "net_bulk", "role-node").expect("flash requester");

    let mut req = Console::open(&bench.core.serial).expect("open requester console");
    let mut snd = Console::open(&bench.dongle.serial).expect("open sender console");
    req.resync();
    snd.resync();
    snd.reset_into_app().expect("reset sender");
    req.reset_into_app().expect("reset requester");

    let verdict = req
        .wait_for(Duration::from_secs(40), |f| {
            frame_text(f).is_some_and(|t| {
                t.contains("verify OK ***") || t.contains("MISMATCH") || t.contains("bulk_fetch failed")
            })
        })
        .expect("requester console read")
        .expect("requester never concluded a fetch within 40 s");
    let t = frame_text(&verdict).unwrap_or("");
    assert!(t.contains("verify OK ***"), "bulk pull failed: {t:?}");
    assert!(t.contains("fetched 200 bytes"), "unexpected blob size: {t:?}");
}

/// Rapid-fire confirmed sends: the gateway must accept counters strictly monotonically —
/// an out-of-order or replayed counter is a latched ORDER VIOLATION.
#[test]
#[ignore = "requires the HIL bench (two TOWER boards in RF range)"]
fn ext_edge_rapid_monotonic() {
    let bench = bench_or_fail();
    build_and_flash(&bench.dongle.serial, "edge_rapid", "").expect("flash gateway");
    build_and_flash(&bench.core.serial, "edge_rapid", "role-node").expect("flash node");

    let mut gw = Console::open(&bench.dongle.serial).expect("open gateway console");
    gw.resync();
    gw.reset_into_app().expect("reset gateway");
    // Reset the node WITHOUT keeping its console open — the gateway capture is the assertion.
    tower_hil::reset_into_app(&bench.core.serial).expect("reset node");

    let frames = gw.collect_for(Duration::from_secs(30)).expect("gateway console read");
    let texts: Vec<&str> = frames.iter().filter_map(frame_text).collect();
    assert!(
        texts.iter().any(|t| t.contains("accepted=") && t.contains("(monotonic OK)")),
        "gateway never reported monotonic progress (saw: {texts:?})"
    );
    assert!(
        !texts.iter().any(|t| t.contains("ORDER VIOLATION") || t.contains("(FAILED)")),
        "gateway accepted an out-of-order/replayed counter: {texts:?}"
    );
}

/// TX-counter persistence: across a reset, the counter must resume AT the persisted reserve
/// watermark (jump ahead, never reuse) — the EEPROM anti-replay contract (docs/radio.md).
#[test]
#[ignore = "requires the HIL bench (TOWER Radio Dongle on the fixture port)"]
fn ext_net_persist_counter_resumes() {
    let bench = bench_or_fail();
    build_and_flash(&bench.dongle.serial, "net_persist", "").expect("flash net_persist");

    let mut console = Console::open(&bench.dongle.serial).expect("open console");
    console.resync();
    console.reset_into_app().expect("reset (boot 1)");

    let boot1 = console
        .wait_for(Duration::from_secs(10), |f| {
            frame_text(f).is_some_and(|t| t.contains("BOOT: resumed tx_counter="))
        })
        .expect("console read")
        .expect("no BOOT banner (boot 1)");
    let t1 = frame_text(&boot1).unwrap_or("").to_string();
    let cnt1 = field_u64(&t1, "tx_counter=").expect("boot 1: no tx_counter");
    let wm1 = field_u64(&t1, "reserve_watermark=").expect("boot 1: no watermark");
    assert!(wm1 > cnt1, "watermark must be reserved ahead of the live counter: {t1:?}");

    // Let it burn a few counter values, then reboot and require the resume-at-watermark jump.
    assert!(
        console.wait_for_text(Duration::from_secs(10), "sent; tx_counter now").expect("read"),
        "net_persist never sent (boot 1)"
    );
    console.resync();
    console.reset_into_app().expect("reset (boot 2)");

    let boot2 = console
        .wait_for(Duration::from_secs(10), |f| {
            frame_text(f).is_some_and(|t| t.contains("BOOT: resumed tx_counter="))
        })
        .expect("console read")
        .expect("no BOOT banner (boot 2)");
    let t2 = frame_text(&boot2).unwrap_or("").to_string();
    let cnt2 = field_u64(&t2, "tx_counter=").expect("boot 2: no tx_counter");
    let wm2 = field_u64(&t2, "reserve_watermark=").expect("boot 2: no watermark");
    assert!(
        cnt2 >= wm1,
        "boot 2 resumed at {cnt2}, BELOW boot 1's watermark {wm1} — counter reuse (replay hazard): {t2:?}"
    );
    assert!(wm2 > cnt2, "boot 2 watermark not re-reserved: {t2:?}");
}
