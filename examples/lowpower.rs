//! lowpower — measure the SDK's STOP-mode idle floor.
//!
//! Does the standard board init (clocks, console, TMP112 shutdown, VBUS-gated
//! STOP) and then parks forever. With USB unplugged, VBUS reads low, so the
//! low-power executor drops the core into STOP between the (very long) timer
//! wakeups. Flash over SWD and measure VDD current with the debugger detached:
//!
//!   cargo build --release --example lowpower
//!   probe-rs download --chip STM32L083CZTx --binary-format elf \
//!       target/thumbv6m-none-eabi/release/examples/lowpower
//!   probe-rs reset --chip STM32L083CZTx      # detach so the core runs free
//!
//! Uses `no_shell` (shell RX frames are ignored). Note the console `manager` is still
//! spawned by `Board::take` and, while unplugged, polls VBUS every ~500 ms — that RTC
//! poll is the active wake source here, and it re-applies the STOP power tuning
//! (`PWR_CR.LPSDSR`/`ULP`, which embassy's wake path clears) on each wake. So this measures
//! the realistic unplugged idle floor, not a "nothing ever runs" floor.
//!
//! Bench-measured (2026-07-12, PPK2 source-measure @ 1.8 V, USB unplugged), all medians and
//! repeatable to sub-µA:
//!   * **true DUT STOP floor: 5.1 µA** — PPK2 the *only* thing attached (J-Link removed too).
//!     STM32L0 Stop + LSE-RTC territory.
//!   * with the J-Link attached: **19.8 µA** → the J-Link's SWD debug-domain parasitic adds
//!     ~14.7 µA (it energises the target's debug power even in Stop).
//! Both are well under the 50 µA the HIL `power_stop_floor_under_50ua` test asserts.
//! Verified against a high-current anchor on the SAME board: the `led_on` example (LED held on +
//! core spinning) reads **7.7 mA @ 3.0 V / 4.7 mA @ 1.8 V** — a ~1500× step over the STOP floor,
//! proving the meter reads real current at scale (the floor isn't a stuck/zero artifact).
//! (Median, not mean: the ~500 ms console-poll wakes briefly spike the instantaneous current, so
//! a mean would overstate the quiescent floor.)

#![no_std]
#![no_main]

use embassy_time::Timer;
use tower::{app, board::Board};

async fn run(_b: Board) {
    loop {
        // A very long await: the executor is idle the whole time and (VBUS low)
        // drops into STOP, kept alive only by the RTC/LSE wake timer.
        Timer::after_secs(3600).await;
    }
}

app!(run, no_shell);
