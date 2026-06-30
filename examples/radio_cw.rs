//! radio_cw — CW carrier bring-up, verified with two boards (no SDR needed).
//!
//! Build twice with a role feature:
//!   TOWER_FEATURES=role-node    just flash example radio_cw   # TX: keys an unmodulated
//!                                                      #     carrier on/off
//!   TOWER_FEATURES=role-gateway just flash example radio_cw   # RX: sits in RX and logs
//!                                                      #     RSSI on the channel
//! (no feature defaults to the RX role).
//!
//! With both on the same band/channel (EU 868.1 MHz, ch0), the RX board's RSSI
//! jumps from the noise floor (~-110 dBm) to a strong level while the TX keys CW,
//! and drops back when it stops — proving the synthesizer, PA and frequency math
//! without lab instruments.

#![no_std]
#![no_main]

use embassy_time::Timer;
use log::{error, info};
use tower::radio::{RfConfig, Spirit1, config};
use tower::{app, board::Board};

const CHANNEL: u8 = 0;

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
        error!(target: "radio_cw", "exit_shutdown: {e}");
    }
    if let Err(e) = radio.read_device_id() {
        error!(target: "radio_cw", "device id: {e}");
    }

    let cfg = RfConfig {
        band: config::Band::DEFAULT,
        channel: CHANNEL,
    };
    match config::apply(&mut radio, &cfg).await {
        Ok(()) => {
            info!(target: "radio_cw", "RF configured: EU868 ch{} (868.{} MHz)", CHANNEL, 1 + CHANNEL * 2)
        }
        Err(e) => error!(target: "radio_cw", "config: {e}"),
    }

    #[cfg(feature = "role-node")]
    tx_role(&mut radio).await;
    #[cfg(not(feature = "role-node"))]
    rx_role(&mut radio).await;
}

#[cfg(feature = "role-node")]
async fn tx_role(radio: &mut Spirit1) -> ! {
    info!(target: "radio_cw", "TX role: keying CW on ch{} (3 s on / 2 s off)", CHANNEL);
    loop {
        match radio.cw_test(true).await {
            Ok(()) => {
                // Let the synth settle (cal ~54-80 µs), then read the steady state.
                Timer::after_millis(50).await;
                let st = radio.mc_state().unwrap_or(0xFF);
                let lock_err = radio.error_lock().unwrap_or(false);
                let what = match st {
                    0x5F => "TX (emitting)",
                    0x13 => "LOCKWON (synth NOT locked!)",
                    0x4F => "still calibrating",
                    _ => "unexpected",
                };
                info!(target: "radio_cw", "CW ON: state=0x{:02X} {} error_lock={}", st, what, lock_err);
            }
            Err(e) => error!(target: "radio_cw", "cw on: {e}"),
        }
        Timer::after_secs(3).await;
        let _ = radio.cw_test(false).await;
        info!(target: "radio_cw", "CW OFF");
        Timer::after_secs(2).await;
    }
}

#[cfg(not(feature = "role-node"))]
async fn rx_role(radio: &mut Spirit1) -> ! {
    info!(target: "radio_cw", "RX role: sampling RSSI on ch{} (carrier detect)", CHANNEL);
    let mut n = 0u32;
    loop {
        match radio.rssi_sample().await {
            Ok(raw) => {
                let dbm = config::rssi_to_dbm(raw);
                let carrier = dbm > -100;
                // First few samples: also report state/lock to confirm RX works.
                let diag = if n < 3 {
                    let st = radio.mc_state().unwrap_or(0xFF);
                    let lock_err = radio.error_lock().unwrap_or(false);
                    info!(target: "radio_cw", "  (diag state=0x{:02X} error_lock={})", st, lock_err);
                    true
                } else {
                    false
                };
                info!(
                    target: "radio_cw",
                    "RSSI {} dBm (raw=0x{:02X}) {}{}",
                    dbm, raw,
                    if carrier { "<<< CARRIER" } else { "(floor)" },
                    if diag { " [diag]" } else { "" }
                );
            }
            Err(e) => error!(target: "radio_cw", "rssi_sample: {e}"),
        }
        n = n.wrapping_add(1);
        Timer::after_millis(500).await;
    }
}

app!(run);
