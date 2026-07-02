//! HIL smoke group: flash a self-checking KAT example to the Dongle, decode its framed boot
//! output natively, and await the `ALL PASS` verdict (docs/test-plan.md §1 — self-checking KATs).
//!
//! These need the bench, so they are `#[ignore]`d: `cargo test` COMPILES them (proving the
//! harness API stays in sync) but runs none without `--ignored`. Run on the bench with
//! `just hil` (which passes `--test-threads=1` — the serial ports are exclusive).

use std::process::Command;
use std::time::Duration;

use tower_hil::{Frame, bench_or_fail, frame_text, reset_into_app, Console};

/// Flash a raw `.bin` to a board over the UART bootloader via `tower flash`. Sequential by design
/// (concurrent FTDI flashing is unreliable — Write-Memory timeouts / re-enumeration, see
/// tools/hwtest/README.md). Returns the CLI's combined output on failure.
fn tower_flash(port: &str, bin: &str) -> Result<(), String> {
    let out = Command::new("tower")
        .args(["-d", port, "flash", bin])
        .output()
        .map_err(|e| format!("HIL: spawn `tower flash`: {e}"))?;
    if out.status.success() {
        Ok(())
    } else {
        Err(format!(
            "HIL: `tower -d {port} flash {bin}` failed:\n{}\n{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ))
    }
}

/// Build a firmware example to a raw `.bin` with `just build example <name>`, returning the bin
/// path. (Kept here rather than re-implementing objcopy — the justfile owns the build.)
fn build_example(name: &str) -> Result<String, String> {
    let out = Command::new("just")
        .current_dir(concat!(env!("CARGO_MANIFEST_DIR"), "/../.."))
        .args(["build", "example", name])
        .output()
        .map_err(|e| format!("HIL: spawn `just build example {name}`: {e}"))?;
    if !out.status.success() {
        return Err(format!(
            "HIL: `just build example {name}` failed:\n{}",
            String::from_utf8_lossy(&out.stderr)
        ));
    }
    // `just build` writes the merged image to target/firmware.bin (see the justfile `bin`).
    Ok(concat!(env!("CARGO_MANIFEST_DIR"), "/../../target/firmware.bin").to_string())
}

/// crypto_ccm_kat is a one-shot KAT: it prints RFC 3610 vector verdicts then `*** ALL PASS ***`
/// (or an `error!` FAIL line) within ms of boot. Flash it to the Dongle, reset, and await PASS.
#[test]
#[ignore = "requires the HIL bench (TOWER Radio Dongle on the fixture port)"]
fn smoke_ccm_kat_all_pass() {
    let bench = bench_or_fail();
    let bin = build_example("crypto_ccm_kat").expect("build crypto_ccm_kat");
    tower_flash(&bench.dongle.serial, &bin).expect("flash the Dongle");

    // Reset into the app so the one-shot verdict is emitted while we're listening, then decode.
    let mut console = Console::open(&bench.dongle.serial).expect("open Dongle console");
    console.resync();
    reset_into_app(&bench.dongle.serial).expect("reset the Dongle into the app");

    // Await a positive verdict; a FAIL/MISMATCH/VIOLATION line fails fast.
    let verdict = console
        .wait_for(Duration::from_secs(8), |f| {
            frame_text(f).is_some_and(|t| {
                t.contains("ALL PASS")
                    || t.contains("PASS ***")
                    || t.contains("FAIL")
                    || t.contains("MISMATCH")
                    || t.contains("VIOLATION")
            })
        })
        .expect("console read")
        .expect("no verdict line within 8 s — did the KAT boot?");

    let text = frame_text(&verdict).unwrap_or("");
    assert!(
        !text.contains("FAIL") && !text.contains("MISMATCH") && !text.contains("VIOLATION"),
        "crypto_ccm_kat reported a failure: {text:?}"
    );

    // Bonus over `strings | grep`: the boot stream must be gap-free (no dropped console frames).
    assert_eq!(console.seq_gaps(), 0, "console dropped frames during the KAT boot");
}

/// The boot banner is a typed `Hello` we can inspect (protocol version must match the wire crate
/// the harness pins — a mismatch is exactly the silent-mis-decode hazard the golden rule guards).
#[test]
#[ignore = "requires the HIL bench (TOWER Radio Dongle on the fixture port)"]
fn smoke_hello_banner_decodes() {
    let bench = bench_or_fail();
    let bin = build_example("blinky").expect("build blinky");
    tower_flash(&bench.dongle.serial, &bin).expect("flash the Dongle");

    let mut console = Console::open(&bench.dongle.serial).expect("open Dongle console");
    console.resync();
    reset_into_app(&bench.dongle.serial).expect("reset the Dongle into the app");

    let hello = console
        .wait_for(Duration::from_secs(5), |f| matches!(f, Frame::Hello { .. }))
        .expect("console read")
        .expect("no Hello banner within 5 s");
    if let Frame::Hello { protocol_version, firmware_version } = hello {
        assert_eq!(protocol_version, tower_protocol::PROTOCOL_VERSION, "protocol version drift");
        assert!(!firmware_version.is_empty(), "empty firmware version in Hello");
    }
}
