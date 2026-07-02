//! HIL power group (feature `power`): measure the Core Module's STOP-mode current floor.
//!
//! Flash the `lowpower` example over SWD with probe-rs (NOT `tower flash` — the UART bootloader
//! needs the FTDI, which must be UNPLUGGED for a real STOP measurement), power-cycle the PPK2 at
//! 1.8 V, and assert the averaged deep-sleep current is under 50 µA.
//!
//! Guard (the whole reason this is subtle): **USB present inhibits STOP by design** — the SDK's
//! console `manager` keeps the low-power executor out of STOP while VBUS is high, so a Core with
//! its FTDI plugged in will never reach the µA floor. If the Core's console still answers, USB is
//! present → we SKIP with an "unplug the FTDI" message rather than record a false ~mA reading.
//!
//! Doubly gated: the whole file is behind `#[cfg(feature = "power")]` AND the test is `#[ignore]`d
//! (it needs the J-Link + PPK2). `cargo test` compiles nothing here unless `--features power`, and
//! runs it only under `just hil-power` (`--features power -- --ignored --test-threads=1`).

#![cfg(feature = "power")]

use std::process::Command;
use std::time::Duration;

use tower_hil::{bench_or_fail, Console, Ppk2};

/// Flash an example over SWD with probe-rs (chip STM32L083CZTx). Uses the release ELF the
/// justfile's `build` produces via cargo; here we build the ELF then `probe-rs download` it (the
/// same sequence as examples/lowpower.rs's header). This path deliberately does NOT touch the
/// FTDI, so the board can be measured with USB unplugged.
fn probe_rs_flash_lowpower() -> Result<(), String> {
    let repo = concat!(env!("CARGO_MANIFEST_DIR"), "/../..");
    // Build the ELF (release) for the example.
    let out = Command::new("cargo")
        .current_dir(repo)
        .args(["build", "--release", "--example", "lowpower"])
        .output()
        .map_err(|e| format!("HIL: spawn cargo build lowpower: {e}"))?;
    if !out.status.success() {
        return Err(format!(
            "HIL: build lowpower failed:\n{}",
            String::from_utf8_lossy(&out.stderr)
        ));
    }
    let elf = format!("{repo}/target/thumbv6m-none-eabi/release/examples/lowpower");
    let out = Command::new("probe-rs")
        .args(["download", "--chip", "STM32L083CZTx", "--binary-format", "elf", &elf])
        .output()
        .map_err(|e| format!("HIL: spawn probe-rs download: {e}"))?;
    if !out.status.success() {
        return Err(format!(
            "HIL: probe-rs download failed:\n{}\n{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ));
    }
    // Detach the probe so the core runs free (the debug domain would otherwise add ~200 µA —
    // confounder #1; the PPK2 power-cycle below also clears it).
    let _ = Command::new("probe-rs")
        .args(["reset", "--chip", "STM32L083CZTx"])
        .output();
    Ok(())
}

/// Does the Core's console answer within a short window? If so, USB/VBUS is present, which inhibits
/// STOP — so a power measurement would be meaningless. Used as the "unplug the FTDI" guard.
fn console_answers(port: &str) -> bool {
    match Console::open(port) {
        Ok(mut c) => c
            .next(Duration::from_millis(1500))
            .ok()
            .flatten()
            .is_some(),
        // Port not present / not openable ⇒ treat as "no console" (the desired unplugged state).
        Err(_) => false,
    }
}

/// Assert the Core's STOP-mode current is under 50 µA at 1.8 V, with the USB-present guard.
#[test]
#[ignore = "requires the HIL bench (Core on J-Link + PPK2, FTDI unplugged)"]
fn power_stop_floor_under_50ua() {
    let bench = bench_or_fail();

    // Flash over SWD (does not need the FTDI).
    probe_rs_flash_lowpower().expect("probe-rs flash lowpower");

    // GUARD: if the console answers, USB is present → STOP is inhibited by design. Skip with a
    // clear instruction rather than assert on a false reading.
    if console_answers(&bench.core.serial) {
        eprintln!(
            "SKIP power_stop_floor_under_50ua: the Core's console is answering, so USB/VBUS is \
             present — the SDK inhibits STOP while plugged in (by design). Unplug the FTDI from \
             the Core and re-run; the PPK2 supplies the board."
        );
        return; // an #[ignore]d, manually-run test: a skip is a soft pass, not a hard failure
    }

    // Power-cycle at 1.8 V (clears the debug-domain residual — confounder #1 — and measures at the
    // knee — confounder #2), let the app settle into STOP, then average. The sidecar / harness
    // reject a >50 mA read as a CDC desync (confounder #3).
    let mut ppk2 = Ppk2::spawn().expect("spawn PPK2 sidecar");
    ppk2.cycle(bench.ppk2.supply_mv).expect("PPK2 power-cycle");
    std::thread::sleep(Duration::from_secs(2)); // let lowpower park into STOP between wakeups
    let ua = ppk2.avg_ua(1000).expect("PPK2 average");

    eprintln!("Core STOP floor: {ua:.1} µA @ {} mV", bench.ppk2.supply_mv);
    assert!(
        ua < 50.0,
        "Core STOP current {ua:.1} µA ≥ 50 µA at {} mV — regression in the USB-gated STOP path?",
        bench.ppk2.supply_mv
    );
}
