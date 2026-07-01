//! USB-presence-gated low power.
//!
//! The SDK's rule: **USB present → full console live** (UART + shell), **USB absent →
//! console torn down so the MCU can STOP** (µA-level idle). On the STM32L0 an *enabled*
//! USART holds embassy's STOP refcount, so the console cannot simply be left running —
//! it must be dropped when unplugged for the low-power executor to reach STOP.
//!
//! This is implemented **dynamically** by [`console::manager`](crate::console::manager),
//! which owns the console UART + the `VBUS_SENSE` (PA12) [`ExtiInput`]: while VBUS is
//! high it builds the `BufferedUart` and runs the writer + RX router; on unplug it drops
//! the UART (releasing the STOP refcount) and parks on the PA12 EXTI edge. EXTI line 12
//! works in STOP, so a plug-in wakes the MCU to bring the console back — no reset, no
//! polling. There is no separate power task; the gating lives entirely in the console
//! manager, spawned by [`board::Board::take`](crate::board::Board::take).
