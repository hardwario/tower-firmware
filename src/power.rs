//! USB-presence-gated low power.
//!
//! `VBUS_SENSE` (PA12) reads logic-high while USB is plugged in. The rule the
//! SDK wants: **USB present → never enter STOP** (so the UART console stays
//! live and responsive), **USB absent → allow STOP** (µA-level idle).
//!
//! Mechanism: while VBUS is high we hold a [`WakeGuard`], which raises the RCC
//! stop refcount. With the refcount non-zero, the low-power executor's
//! `get_stop_mode()` returns `None`, so on idle it falls back to a plain `WFI`
//! (Sleep mode — core clock-gated but all peripheral clocks, including the
//! USART, keep running) instead of STOP. Dropping the guard re-enables STOP.
//!
//! The pin lives on EXTI line 12, and EXTI works in STOP, so a plug-in event
//! wakes the MCU out of STOP to re-arm the guard.

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_stm32::rcc::{StopMode, WakeGuard};
use log::info;

/// Watch `VBUS_SENSE` and gate STOP on USB presence. Spawned automatically by
/// [`Board::take`](crate::board::Board::take) with the `ExtiInput` bound to PA12
/// (interrupt-driven, hence `Async`); apps that build the board manually via
/// [`board::init`](crate::board::init) can spawn it themselves.
#[embassy_executor::task]
pub async fn vbus_task(mut vbus: ExtiInput<'static, Async>) {
    loop {
        if vbus.is_high() {
            // USB present: inhibit STOP for as long as it stays plugged. The
            // guard lives across the `.await` below (stored in the task future),
            // so the stop refcount is held the entire time USB is connected.
            let _guard = WakeGuard::new(StopMode::Stop1);
            info!("USB connected - STOP inhibited (Sleep/WFI only)");

            // `wait_for_low` returns immediately if the line is already low, so a
            // disconnect in the window since `is_high()` isn't missed (unlike
            // `wait_for_falling_edge`, which would block until the *next* unplug).
            vbus.wait_for_low().await;

            info!("USB disconnected - STOP enabled");
            // `_guard` drops at the end of this block, releasing the refcount.
        } else {
            // USB absent: STOP is allowed. Park until the next plug-in; EXTI line
            // 12 wakes the MCU out of STOP (returns at once if already plugged).
            vbus.wait_for_high().await;
        }
    }
}
