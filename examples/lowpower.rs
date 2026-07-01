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
//! the realistic unplugged idle floor (~32 µA @3 V *including* the attached probe; ~12 µA
//! true DUT), not a "nothing ever runs" floor.

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
