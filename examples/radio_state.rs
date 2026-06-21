//! radio_state — exercise the SPIRIT1 MC state machine and the nIRQ line.
//!
//! Cycles READY ↔ STANDBY, logging the read-back STATE[6:0] code after each
//! transition (READY=0x03, STANDBY=0x40). Then it routes the nIRQ to GPIO0 (PA7),
//! enables the READY interrupt, and confirms that reaching READY pulls nIRQ low
//! (asserted) and that reading/clearing the IRQ status releases it — proving the
//! interrupt path before any RF traffic.
//!
//! The SLEEP state (0x36) needs the RC oscillator calibrated and the wake timer
//! running, which is set up in Step 6 — see `radio_sleep`. STANDBY is the
//! always-available config-retaining low-power state used here.
//!
//!   just flash radio_state     (watch with: jolt monitor --reset)

#![no_std]
#![no_main]

use log::{error, info, warn};
use tower::radio::{Spirit1, regs};
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
        error!(target: "radio_state", "exit_shutdown: {:?}", e);
    }
    match radio.read_device_id() {
        Ok(id) => info!(target: "radio_state", "SPIRIT1 ok (part_number={})", id.part_number()),
        Err(e) => error!(target: "radio_state", "device id: {:?}", e),
    }

    // Route nIRQ to GPIO0 and enable the READY event; clear any pending status.
    let _ = radio.configure_irq_gpio();
    let _ = radio.set_irq_mask(regs::IRQ_READY);
    let _ = radio.irq_status();

    let mut cycle = 0u32;
    loop {
        cycle += 1;
        info!(target: "radio_state", "--- cycle {} ---", cycle);

        transition(&mut radio, Transition::Standby).await;

        // Going to READY raises the READY interrupt -> nIRQ should assert (low).
        let _ = radio.irq_status(); // clear before the event
        transition(&mut radio, Transition::Ready).await;

        let asserted = radio.irq_asserted();
        let status = radio.irq_status().unwrap_or(0); // read-and-reset
        if asserted && status & regs::IRQ_READY != 0 {
            info!(target: "radio_state", "nIRQ asserted on READY (IRQ_STATUS=0x{:08X})", status);
        } else {
            warn!(
                target: "radio_state",
                "nIRQ check: asserted={} IRQ_STATUS=0x{:08X} (expected READY bit + asserted)",
                asserted, status
            );
        }
        // After clearing the status, nIRQ should release.
        if !radio.irq_asserted() {
            info!(target: "radio_state", "nIRQ released after status read");
        }

        embassy_time::Timer::after_secs(3).await;
    }
}

enum Transition {
    Ready,
    Standby,
}

async fn transition(radio: &mut Spirit1, t: Transition) {
    let (name, expect, res) = match t {
        Transition::Ready => ("READY", regs::STATE_READY, radio.to_ready().await),
        Transition::Standby => ("STANDBY", regs::STATE_STANDBY, radio.to_standby().await),
    };
    match res {
        Ok(()) => {
            let st = radio.mc_state().unwrap_or(0xFF);
            info!(
                target: "radio_state",
                "-> {} ok (STATE=0x{:02X}, expected 0x{:02X})", name, st, expect
            );
        }
        Err(e) => error!(target: "radio_state", "-> {} failed: {:?}", name, e),
    }
}

app!(run);
