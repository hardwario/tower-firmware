//! SPIRIT1 sub-GHz radio stack for the TOWER Core Module.
//!
//! Built on the SPIRIT1 transceiver (in the SPSGRF module) wired to the
//! STM32L083CZ. The stack is layered, mirroring the design spec (`RADIO.md`):
//!
//! - **Radio layer** — [`regs`] (register map), [`spi`] ([`Spirit1Spi`]
//!   transport, software CS), [`device`] ([`Spirit1`] chip handle: power states,
//!   device-ID check, calibration). RF configuration and the IRQ-driven async
//!   operation driver are added on top in later steps.
//! - **Network layer** — addressing, confirmed delivery, AES-CCM security,
//!   replay protection, bulk transfers and pairing (added in later steps).
//!
//! See `docs/radio.md` for the user-facing guide and `examples/radio_*.rs` /
//! `examples/net_*.rs` for runnable demos. Pins and parameters come from the
//! board (PB7 SDN, PA15 CS, SPI1 on PB3/PB5/PB4, PA7 nIRQ) — see
//! [`board::Board`](crate::board::Board).

pub mod aes;
pub mod ccm;
pub mod config;
pub mod device;
pub mod frame;
pub mod net;
pub mod regs;
pub mod spi;

pub use config::{Band, RfConfig, SignalQuality, rssi_to_dbm};
pub use device::{DeviceId, RadioError, Spirit1};
pub use spi::Spirit1Spi;
