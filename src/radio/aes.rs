//! AES-128 on the STM32L0 hardware AES engine (register-level).
//!
//! embassy-stm32 0.6.0 doesn't wrap the L0 AES, so this drives `pac::AES`
//! directly: enable the clock (`RCC.ahbenr.crypen`), load the key, and run one
//! 16-byte ECB block at a time. The byte/word ordering is handled by the
//! engine's `datatype = BYTE` swap, so callers pass plain big-endian byte
//! blocks (matching the FIPS-197 / RFC 3610 test-vector convention).
//!
//! Only single-block **ECB encryption** is exposed — [`ccm`](super::ccm) builds
//! CBC-MAC and CTR (and thus AES-CCM) on top of it in firmware, which keeps this
//! driver tiny and avoids the engine's chaining-mode state machine.

use embassy_stm32::pac;
use embassy_stm32::pac::aes::vals::{Datatype, Mode};

/// AES-128 block size.
pub const BLOCK: usize = 16;

/// Handle to the L0 AES engine. Construction enables its clock.
pub struct Aes {
    _private: (),
}

impl Aes {
    /// Enable the AES peripheral clock and return a handle.
    pub fn new() -> Self {
        pac::RCC.ahbenr().modify(|w| w.set_crypen(true));
        // Ensure the clock-enable write lands before the first AES access.
        cortex_m::asm::dsb();
        let aes = pac::AES;
        // Start disabled, in encrypt/ECB/byte-swap mode.
        aes.cr().write(|w| {
            w.set_en(false);
            w.set_mode(Mode::MODE1); // encryption
            w.set_chmod10(0b00); // ECB
            w.set_datatype(Datatype::BYTE); // byte-oriented data
        });
        Self { _private: () }
    }

    /// Load the 128-bit key. `key[0]` is the most-significant byte (KEYR3 holds
    /// the top word).
    pub fn set_key(&mut self, key: &[u8; 16]) {
        let aes = pac::AES;
        for i in 0..4 {
            let word = u32::from_be_bytes([key[4 * i], key[4 * i + 1], key[4 * i + 2], key[4 * i + 3]]);
            // KEYR3 = MSB word .. KEYR0 = LSB word.
            aes.keyr(3 - i).write_value(pac::aes::regs::Keyr(word));
        }
    }

    /// Encrypt one 16-byte block in place (ECB). The key must already be set.
    pub fn encrypt_block(&mut self, block: &mut [u8; 16]) {
        let aes = pac::AES;
        // (Re)assert encrypt/ECB/byte mode and enable.
        aes.cr().modify(|w| {
            w.set_mode(Mode::MODE1);
            w.set_chmod10(0b00);
            w.set_datatype(Datatype::BYTE);
            w.set_en(true);
        });
        // Feed the 4 input words. With datatype=BYTE the engine byte-swaps each
        // word, so assemble little-endian here → AES sees block[0] as the MSB.
        for i in 0..4 {
            let word =
                u32::from_le_bytes([block[4 * i], block[4 * i + 1], block[4 * i + 2], block[4 * i + 3]]);
            aes.dinr().write_value(pac::aes::regs::Dinr(word));
        }
        // Wait for computation-complete.
        while !aes.sr().read().ccf() {}
        // Read the 4 output words back (same byte-swap convention as input).
        for i in 0..4 {
            let word = aes.doutr().read().0;
            block[4 * i..4 * i + 4].copy_from_slice(&word.to_le_bytes());
        }
        // Clear CCF and disable until next use.
        aes.cr().modify(|w| w.set_ccfc(true));
        aes.cr().modify(|w| w.set_en(false));
    }
}

impl Default for Aes {
    fn default() -> Self {
        Self::new()
    }
}
