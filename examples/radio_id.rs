//! radio_id — bring the SPIRIT1 out of shutdown and verify its device ID.
//!
//! The make-or-break radio bring-up checkpoint: it proves the SPI bus, the
//! software chip-select timing (PA15), and the SDN line (PB7) are all correct.
//! A genuine SPIRIT1 reports PARTNUM=0x01, VERSION=0x30 (combined part number
//! 304, version 48). All-`0x00` or all-`0xFF` means the bus/CS/SDN wiring or
//! timing is wrong.
//!
//!   just flash example radio_id        (watch with: tower logs)

#![no_std]
#![no_main]

use embassy_time::Timer;
use log::{error, info};
use tower::radio::Spirit1;
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

    info!(target: "radio_id", "exiting SHUTDOWN (driving SDN low)...");
    match radio.exit_shutdown().await {
        Ok(()) => info!(target: "radio_id", "radio reached READY"),
        Err(e) => error!(target: "radio_id", "exit_shutdown: {e} (continuing to probe)"),
    }

    loop {
        match radio.read_device_id_raw() {
            Ok(id) => {
                if id.is_supported() {
                    info!(
                        target: "radio_id",
                        "OK: partnum=0x{:02X} version=0x{:02X} (part_number={}) - SPIRIT1 verified",
                        id.partnum, id.version, id.part_number()
                    );
                } else {
                    error!(
                        target: "radio_id",
                        "MISMATCH: partnum=0x{:02X} version=0x{:02X} (expected 0x01 / 0x30) - check SPI/CS/SDN",
                        id.partnum, id.version
                    );
                }
            }
            Err(e) => error!(target: "radio_id", "SPI read failed: {e}"),
        }
        Timer::after_secs(2).await;
    }
}

app!(run);
