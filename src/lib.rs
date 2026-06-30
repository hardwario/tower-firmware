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
/// Wraps your `async fn run(b: Board)` with the STOP-mode executor + reset entry and the
/// always-on board setup ([`board::Board::take`] — clock, console, TMP112 one-shot). It also
/// **serves the interactive [`shell`](crate::shell) by default**, over the shared EEPROM
/// [`Nv`](crate::storage::Nv) handle, so the app can drive `Net`/FOTA on the same `b.kv`
/// alongside it. The whole app is then just:
///
/// ```ignore
/// #![no_std]
/// #![no_main]
/// use tower::{app, board::Board};
///
/// async fn run(mut b: Board) {
///     // use b.spawner, b.tmp112, b.led, b.button, b.accel_int, b.kv, b.strip_* …
/// }
/// app!(run);                                    // base shell (/system/resource, settings, …)
/// // app!(run, commands: CMDS, settings: SETS); // + an app command tree / settings
/// // app!(run, no_shell);                       // opt out (app owns console RX, or stays minimal)
/// ```
///
/// The shell is served **before** `run`, so it claims the console RX while still free; an app
/// that reads the console RX itself (e.g. a host-proxy gateway) must use `no_shell`.
#[macro_export]
macro_rules! app {
    // Default: serve the base SDK shell (over the shared EEPROM KV), then run.
    ($run:path) => {
        $crate::app!(@entry $run, |b: &$crate::board::Board| {
            $crate::shell::serve(b.spawner, b.kv);
        });
    };
    // Serve the shell with an app command tree + settings, then run.
    ($run:path, commands: $cmds:expr, settings: $sets:expr) => {
        $crate::app!(@entry $run, |b: &$crate::board::Board| {
            $crate::shell::serve_ext(b.spawner, b.kv, $cmds, $sets);
        });
    };
    // Serve the shell with an app command tree (no extra settings), then run.
    ($run:path, commands: $cmds:expr) => {
        $crate::app!(@entry $run, |b: &$crate::board::Board| {
            $crate::shell::serve_ext(b.spawner, b.kv, $cmds, &[]);
        });
    };
    // Run with NO shell — the app owns the console RX, or stays minimal.
    ($run:path, no_shell) => {
        $crate::app!(@entry $run, |_b: &$crate::board::Board| {});
    };

    // Internal: emit the embassy entry. `$setup` (a closure taking `&Board`) runs after the
    // board + console are up but BEFORE `run`, so the shell claims the console RX while it is
    // still free. The board is then moved into the app's `run`.
    (@entry $run:path, $setup:expr) => {
        #[embassy_executor::main(
            executor = "embassy_stm32::executor::Executor",
            entry = "cortex_m_rt::entry"
        )]
        async fn __tower_app(spawner: $crate::Spawner) {
            let board = $crate::board::Board::take(spawner);
            // Uniform startup banner, naming this example/app (the `just flash <name>` target).
            $crate::console::boot_banner(option_env!("CARGO_BIN_NAME").unwrap_or("app"));
            let __setup = $setup;
            __setup(&board);
            $run(board).await
        }
    };
}
