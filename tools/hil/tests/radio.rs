//! HIL radio group: a two-board RF test. Flash `net_confirmed` to both boards — the Core as
//! `role-node` (sender) and the Dongle as the gateway (default role) — and assert the confirmed
//! exchange over the air (the node logs `Delivered`/ACKed, the gateway logs the received frame).
//!
//! Sequential flash, then concurrent capture (concurrent FTDI flashing is unreliable — see
//! tools/hwtest/README.md). `#[ignore]`d: needs the bench + two boards in RF range; run with
//! `just hil` (`--test-threads=1`, exclusive ports).

use std::process::Command;
use std::time::Duration;

use tower_hil::{bench_or_fail, frame_text, Console};

/// Build `net_confirmed` with the given cargo features to target/firmware.bin, then flash `port`.
fn build_and_flash(port: &str, features: &str) -> Result<(), String> {
    let repo = concat!(env!("CARGO_MANIFEST_DIR"), "/../..");
    let mut build = Command::new("just");
    build.current_dir(repo).args(["build", "example", "net_confirmed"]);
    if !features.is_empty() {
        // `just` passes TOWER_FEATURES through to cargo (see the justfile).
        build.env("TOWER_FEATURES", features);
    }
    let out = build.output().map_err(|e| format!("HIL: spawn just build: {e}"))?;
    if !out.status.success() {
        return Err(format!(
            "HIL: build net_confirmed ({features}) failed:\n{}",
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
            "HIL: flash {port} failed:\n{}\n{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ))
    }
}

/// Core = node (sends confirmed frames), Dongle = gateway (receives + auto-ACKs). Assert the
/// node's confirmed send is Delivered (ACK received) and the gateway logs a decrypted RX.
#[test]
#[ignore = "requires the HIL bench (two TOWER boards in RF range)"]
fn radio_net_confirmed_delivered_and_acked() {
    let bench = bench_or_fail();

    // Sequential flash: gateway first (Dongle, default role), then node (Core, role-node).
    build_and_flash(&bench.dongle.serial, "").expect("flash gateway (Dongle)");
    build_and_flash(&bench.core.serial, "role-node").expect("flash node (Core)");

    // Concurrent capture: open both consoles, reset both into the app, then read a window.
    // Each console loans its own handle to jolt for the pulse (Console::reset_into_app) — a
    // second open of an attached tty fails under serialport 4.9's exclusive flock.
    let mut node = Console::open(&bench.core.serial).expect("open node console");
    let mut gw = Console::open(&bench.dongle.serial).expect("open gateway console");
    node.resync();
    gw.resync();
    gw.reset_into_app().expect("reset gateway");
    node.reset_into_app().expect("reset node");

    // Node must report a confirmed delivery (an ACK came back). net_confirmed logs the
    // SendResult VARIANT NAME per send: `Delivered`, `NotDelivered`, `Busy`, `DutyLimited`,
    // `Error …`. We require at least ONE `Delivered` within the window — NOT the strict first
    // outcome: the very first send after a fresh dual-reset lands in the SPIRIT1 warm-up and
    // reliably reports a one-shot `seq=0 Error timeout` (~120 ms after radio init, before the
    // separately-reset gateway is armed); every subsequent send at 2 s cadence Delivers. A bare
    // `contains("Delivered")` would also match `NotDelivered`, so exclude that token explicitly.
    let mut delivered = false;
    let mut last = String::new();
    while node
        .wait_for(Duration::from_secs(18), |f| {
            frame_text(f).is_some_and(|t| t.contains("seq="))
        })
        .expect("node console read")
        .and_then(|f| frame_text(&f).map(str::to_string))
        .map(|t| {
            last = t.clone();
            delivered = t.contains("Delivered") && !t.contains("NotDelivered");
            !delivered // keep reading until a genuine Delivered (or the window elapses)
        })
        .unwrap_or(false)
    {}
    assert!(
        delivered,
        "node never reported a confirmed delivery within the window (last outcome: {last:?})"
    );

    // Gateway must log at least one received/decrypted frame from the node.
    let got_rx = gw
        .wait_for_text(Duration::from_secs(10), "rx")
        .expect("gateway console read")
        || gw
            .wait_for_text(Duration::from_secs(5), "recv")
            .expect("gateway console read");
    assert!(got_rx, "gateway never logged a received frame from the node");
}
