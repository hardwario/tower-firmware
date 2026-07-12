//! led_on — a deliberately HIGH-current firmware: on-board LED held steady on,
//! core spinning at full clock, never sleeping.
//!
//! This is the sanity **anchor** for the STOP-floor power measurement. The LED draws a
//! few mA through its external resistor (and the busy-spin keeps the core out of STOP,
//! adding run-current), so a PPK2 measurement of this firmware must read **mA**. Flashing
//! `lowpower` on the same board then reads **µA**. A clear mA-vs-µA split proves the PPK2
//! is metering real DUT current at the right scale — i.e. the ~20 µA STOP floor is a true
//! reading, not a stuck/uncalibrated artifact. See `hil/tests/power.rs` and the
//! low-power measurement notes.
//!
//!   just build example led_on
//!   probe-rs download --chip STM32L083CZTx --binary-format elf \
//!       target/thumbv6m-none-eabi/release/examples/led_on
//!   probe-rs reset --chip STM32L083CZTx
//!
//! Uses `no_shell` (no console interaction needed). It never awaits, so the console
//! `manager` never runs and the executor never idles into STOP — intentional: the whole
//! point is a firmware that cannot reach the low-power floor.

#![no_std]
#![no_main]

use embassy_stm32::gpio::{Level, Output, Speed};
use tower::{app, board::Board};

async fn run(b: Board) {
    // LED steady ON (blinky pins the LED as ActiveHigh, so Level::High lights it). Held for
    // the lifetime of the task — the external LED keeps drawing mA even if the core were to
    // sleep, which is exactly why it's a robust high-current anchor.
    let _led = Output::new(b.led, Level::High, Speed::Low);
    loop {
        // Busy-spin: never `.await`, so the executor never idles → no STOP. The core runs at
        // full clock (mA of run-current) on top of the LED current.
        cortex_m::asm::nop();
    }
}

app!(run, no_shell);
