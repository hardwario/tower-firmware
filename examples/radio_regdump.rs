//! radio_regdump — read back the key RF registers after config to confirm the
//! writes actually landed (burst writes, state-gated registers, etc.).

#![no_std]
#![no_main]

use log::{error, info};
use tower::radio::{RfConfig, Spirit1, config};
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
    let _ = radio.exit_shutdown().await;
    let _ = radio.read_device_id();
    let cfg = RfConfig {
        band: config::Band::DEFAULT,
        channel: 0,
    };
    if let Err(e) = config::apply(&mut radio, &cfg).await {
        error!(target: "regdump", "config: {e}");
    }

    // (addr, expected) — expected is what config::apply should have written.
    let checks: &[(u8, u8, &str)] = &[
        (0x05, 0x02, "GPIO0_CONF nIRQ"),
        (0x9E, 0xDB, "SYNTH_CONFIG1 REFDIV"),
        (0x9F, 0xA0, "SYNTH_CONFIG0 TSPLIT"),
        (0x07, 0x36, "IF_OFFSET_ANA"),
        (0x0D, 0xAC, "IF_OFFSET_DIG"),
        (0x08, 0xA0, "SYNT3 (WCP0|synt)"),
        (0x1A, 0x93, "MOD1 datarateM=147"),
        (0x1B, 0x19, "MOD0 GFSK|E9"),
        (0x1C, 0x45, "FDEV0 E4 M5"),
        (0x1D, 0x23, "CHFLT"),
        (0x1E, 0xC8, "AFC2"),
        (0x99, 0x80, "IQC 0x99"),
        (0x9A, 0xE3, "IQC 0x9A"),
        (0xBC, 0x22, "IQC 0xBC"),
        (0xA1, 0x25, "VCO_CONFIG current"),
        (0x31, 0x07, "PCKTCTRL3"),
        (0x32, 0x87, "PCKTCTRL2"),
        (0x33, 0x60, "PCKTCTRL1"),
        (0x36, 0xDB, "SYNC4"),
        (0x37, 0x62, "SYNC3"),
        (0x38, 0x47, "SYNC2"),
        (0x39, 0x15, "SYNC1"),
        (0x50, 0x00, "PROTOCOL2 (auto-cal off, bit1=0)"),
    ];

    let spi = radio.spi();
    info!(target: "regdump", "addr  read  expect  name");
    for &(addr, expect, name) in checks {
        match spi.read_reg(addr) {
            Ok((v, _)) => {
                let mark = if addr == 0x08 {
                    "?" // SYNT3 high bits depend on synt; just informational
                } else if v == expect {
                    "ok"
                } else {
                    "<<< MISMATCH"
                };
                info!(target: "regdump", "0x{:02X}  0x{:02X}  0x{:02X}    {} {}", addr, v, expect, name, mark);
            }
            Err(e) => error!(target: "regdump", "0x{:02X} read err {e}", addr),
        }
    }
    // Also dump full SYNT3..0 + PROTOCOL2..0 + VCO cal IN words.
    let mut synt = [0u8; 4];
    let _ = spi.read_regs(0x08, &mut synt);
    info!(target: "regdump", "SYNT3..0 = {:02X} {:02X} {:02X} {:02X}", synt[0], synt[1], synt[2], synt[3]);
    let mut vcal = [0u8; 3];
    let _ = spi.read_regs(0x6D, &mut vcal);
    info!(target: "regdump", "VCO_CALIBR_IN2..0 = {:02X} {:02X} {:02X}", vcal[0], vcal[1], vcal[2]);

    loop {
        embassy_time::Timer::after_secs(5).await;
    }
}

app!(run);
