//! USB-presence-gated low power.
//!
//! The SDK's rule: **USB present → full console live** (UART + shell), **USB absent →
//! console torn down so the MCU can STOP** (µA-level idle). On the STM32L0 an *enabled*
//! USART holds embassy's STOP refcount, so the console cannot simply be left running —
//! it must be dropped when unplugged for the low-power executor to reach STOP.
//!
//! This is implemented **dynamically** by [`console::manager`](crate::console::manager),
//! which owns the console UART + the `VBUS_SENSE` (PA12) `ExtiInput`: while VBUS is
//! high it builds the `BufferedUart` and runs the writer + RX router; on unplug it drops
//! the UART (releasing the STOP refcount) and waits for USB on the PA12 EXTI edge **plus a
//! ~500 ms RTC poll**. EXTI line 12 works in STOP and brings the console up the instant
//! VBUS rises (no reset); the poll is a fallback for a *missed* edge — the FT231X asserts
//! PA12 via its CBUS3 output only tens of ms after power-up. There is no separate power
//! task; the gating lives entirely in the console manager, spawned by
//! [`board::Board::take`](crate::board::Board::take).
//!
//! **STOP-mode tuning.** [`apply_stop_tuning`](crate::board::apply_stop_tuning) sets
//! `PWR_CR.LPSDSR` (low-power regulator in deep sleep) + `ULP` (VREFINT off in Stop) for
//! the datasheet µA STOP floor. embassy's wake path re-inits RCC and *clears* those bits,
//! so the console manager re-applies them on each idle poll (see that function).
