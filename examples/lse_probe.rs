//! lse_probe — boot forensics: inherited RTC-domain state + hardware reset cause, per boot.
//!
//! Logs ONE line per boot: the raw `RCC_CSR` the boot *inherited* (sampled by `board::init`
//! before embassy's clock init ran) decoded into the LSE/RTC-domain fields plus this reset's
//! hardware cause flags. Deliberately quiet after that (no heartbeat chatter), so a host
//! probe can time reset→Hello cleanly.
//!
//! Reading the line:
//!   - `lseon/lserdy/rtcsel/rtcen/drv` — the RTC-domain state inherited from before the
//!     reset. A full match with the wanted config means embassy skipped the LSE cold start;
//!     a mismatch forces an RTC-domain reset + crystal restart (seconds).
//!   - `rst=[...]` — which hardware reset happened (pin/por/sft/iwdg/…), per-boot (flags are
//!     cleared after sampling).
//!
//! History: written for the 2026-07-05 bimodal-boot investigation, where it RULED the LSE
//! OUT — every ~5.2 s boot inherited a fully-matched domain (`lseon=1 lserdy=1 rtcsel=1
//! rtcen=1`) with `rst=[pin]`, and a Low/MediumHigh/High drive sweep showed no effect. The
//! proven cause (flip-counter 1:1 with slow boots) is the EEPROM KV **compaction CPU stall**
//! in `init_session`'s session bump — see docs/storage.md.
//!
//!   just build example lse_probe
//!   TOWER_FEATURES=lse-drive-high just build example lse_probe

#![no_std]
#![no_main]

use embassy_stm32::pac::rcc::regs::Csr;
use embassy_time::Timer;
use log::info;
use tower::{app, board::Board};

async fn run(_b: Board) {
    let raw = tower::board::preinit_csr();
    let csr = Csr(raw);
    // Reset-cause flags (per-boot; board::init cleared them after sampling).
    let mut flags = heapless::String::<48>::new();
    for (bit, name) in [
        (csr.fwrstf(), "fw"),
        (csr.oblrstf(), "obl"),
        (csr.pinrstf(), "pin"),
        (csr.porrstf(), "por"),
        (csr.sftrstf(), "sft"),
        (csr.iwdgrstf(), "iwdg"),
        (csr.wwdgrstf(), "wwdg"),
        (csr.lpwrrstf(), "lpwr"),
    ] {
        if bit {
            if !flags.is_empty() {
                let _ = flags.push('+');
            }
            let _ = flags.push_str(name);
        }
    }
    info!(
        target: "lse",
        "csr={:#010x} lseon={} lserdy={} rtcsel={} rtcen={} drv={} rst=[{}]",
        raw,
        csr.lseon() as u8,
        csr.lserdy() as u8,
        csr.rtcsel() as u8,
        csr.rtcen() as u8,
        csr.lsedrv() as u8,
        flags
    );

    loop {
        Timer::after_secs(60).await;
    }
}

app!(run);
