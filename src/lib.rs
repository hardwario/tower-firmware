//! HARDWARIO TOWER Firmware SDK — reusable building blocks for the Core Module
//! (STM32L083CZ), built on [Embassy](https://embassy.dev).
//!
//! Each module is an independent block; [`board`] provides the common one-line
//! board init. See `examples/` for per-block samples and ready-made apps —
//! build and flash one with `just flash <name>`.
//!
//! Blocks:
//! - [`board`] — [`Board::take`](board::Board::take) + the [`app!`] macro: the common
//!   one-line entry (clock, console, EXTI, radio pins, USB-aware low power).
//! - [`console`] — serial console + `log` backend (level + uptime timestamp).
//! - [`shell`] — RouterOS-style interactive shell + declarative EEPROM-backed settings
//!   framework; pairs with [`console`] and the `tower` host CLI.
//! - [`led`] — non-blocking single-LED blink dispatcher (background + instant).
//! - [`button`] — debounced button (click/hold), EXTI-gated or polled.
//! - [`power`] — USB-presence-gated STOP via `WakeGuard`.
//! - [`radio`] — SPIRIT1 sub-GHz radio stack (driver + AES-CCM + network layer).
//! - [`tmp112`] — TMP112 temperature sensor driver (HAL-independent).
//! - [`lis2dh12`] — LIS2DH12 accelerometer: orientation/dice + tilt interrupt.
//! - [`storage`] — EEPROM non-volatile storage: raw byte area + a key-value store (raw or postcard values).
//! - [`fota`] — firmware-over-the-air image staging (program-flash sink + layout + state).
//! - [`ws2812`] — WS2812B/SK6812 strip driver (timer PWM + DMA).
//! - [`strip`] — addressable-LED effects (rainbow, chase, …) with brightness+gamma.

#![no_std]

pub mod board;
pub mod button;
pub mod console;
pub mod fota;
pub mod led;
pub mod lis2dh12;
pub mod power;
pub mod radio;
pub mod shell;
pub mod storage;
pub mod strip;
pub mod tmp112;
pub mod ws2812;

pub use embassy_executor::Spawner;

/// Define an application entry point with the common boilerplate handled.
///
/// Wraps your `async fn run(b: Board)` with the STOP-mode executor + reset
/// entry, and the always-on board setup ([`board::Board::take`] — clock,
/// console, and the TMP112 shut into one-shot mode). The whole app is then just:
///
/// ```ignore
/// #![no_std]
/// #![no_main]
/// use tower::{app, board::Board};
///
/// async fn run(mut b: Board) {
///     // use b.spawner, b.tmp112, b.led, b.button, b.accel_int, b.storage, b.strip_* …
/// }
/// app!(run);
/// ```
#[macro_export]
macro_rules! app {
    ($run:path) => {
        #[embassy_executor::main(
            executor = "embassy_stm32::executor::Executor",
            entry = "cortex_m_rt::entry"
        )]
        async fn __tower_app(spawner: $crate::Spawner) {
            let board = $crate::board::Board::take(spawner);
            // Uniform startup banner, naming this example/app (the `just flash
            // <name>` target). `CARGO_BIN_NAME` is the example's name.
            $crate::console::boot_banner(option_env!("CARGO_BIN_NAME").unwrap_or("app"));
            $run(board).await
        }
    };
}
