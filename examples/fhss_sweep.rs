//! fhss_sweep — F1: sweep all FHSS hop channels, verify the synth locks across the
//! whole 903–926.7 MHz set (especially the band edges), and **measure** the
//! retune+VCO-lock time so the FHSS slot GUARD is set from data, not assumed.
//!
//!   just flash fhss_sweep        # one board
//!
//! For each of the 80 channels: set_freq_hz → strobe RX (triggers VCO auto-cal) →
//! poll until MC_STATE reaches RX, timing it, and check ERROR_LOCK. Prints the lock
//! error count and the max retune+lock time, then the recommended GUARD = max(3×, 10 ms).

#![no_std]
#![no_main]

use embassy_time::{Duration, Instant, Timer};
use log::{error, info, warn};
use tower::radio::config::{FHSS_N, fhss_freq_hz};
use tower::radio::{RfConfig, Spirit1, config, regs};
use tower::{app, board::Board};

async fn run(b: Board) {
    let mut radio = Spirit1::new(
        b.radio_spi,
        b.radio_sck,
        b.radio_mosi,
        b.radio_miso,
        b.radio_cs,
        b.radio_sdn,
        b.radio_irq,
    );
    if let Err(e) = radio.exit_shutdown().await {
        error!(target: "sweep", "exit_shutdown: {:?}", e);
    }
    if let Err(e) = radio.read_device_id() {
        error!(target: "sweep", "device id: {:?}", e);
    }
    // Bring up the band-independent RF config (US 915 base; we override the carrier).
    if let Err(e) = config::apply(
        &mut radio,
        &RfConfig {
            band: config::Band::Us915,
            channel: 0,
        },
    )
    .await
    {
        error!(target: "sweep", "config: {:?}", e);
    }

    info!(target: "sweep", "sweeping {} FHSS channels {}–{} MHz",
        FHSS_N, fhss_freq_hz(0) / 1_000_000, fhss_freq_hz(FHSS_N - 1) / 1_000_000);

    let mut lock_errors = 0u32;
    let mut max_lock_us: u64 = 0;
    for k in 0..FHSS_N {
        let t0 = Instant::now();
        let _ = config::set_freq_hz(&mut radio, fhss_freq_hz(k)).await;
        let _ = radio.enter_rx().await; // strobe RX → VCO auto-calibration on entry
        // Poll until the part reaches RX (= synth locked), up to ~6 ms.
        let mut reached = false;
        for _ in 0..60 {
            if radio.mc_state().unwrap_or(0) == regs::STATE_RX {
                reached = true;
                break;
            }
            Timer::after(Duration::from_micros(100)).await;
        }
        let us = t0.elapsed().as_micros();
        let err = radio.error_lock().unwrap_or(true);
        let _ = radio.to_ready().await;
        if reached && !err {
            if us > max_lock_us {
                max_lock_us = us;
            }
        } else {
            lock_errors += 1;
            warn!(target: "sweep", "ch {} ({} Hz): reached_rx={} error_lock={}", k, fhss_freq_hz(k), reached, err);
        }
    }

    let guard_ms = (((max_lock_us / 1000) as u32) * 3).max(10);
    info!(target: "sweep", "swept {} channels: {} lock errors", FHSS_N, lock_errors);
    info!(target: "sweep", "max retune+lock = {} us → recommended GUARD = max(3x,10ms) = {} ms", max_lock_us, guard_ms);
    if lock_errors == 0 {
        info!(target: "sweep", "F1 PASS *** all FHSS channels lock");
    } else {
        error!(target: "sweep", "F1 FAIL: {} channels failed to lock", lock_errors);
    }
    loop {
        Timer::after_secs(5).await;
    }
}

app!(run);
