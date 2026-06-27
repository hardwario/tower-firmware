//! CRC-32/IEEE (reflected, poly `0xEDB8_8320`) — the single integrity primitive
//! shared by the console frame and the EEPROM key-value store.
//!
//! [`crc32_update`] is the raw running update (no init, no final XOR): fold one or
//! more slices into a running value. [`crc32_ieee`] is the one-shot (init
//! `0xFFFF_FFFF`, finalize `!`) used for a single contiguous buffer.
//!
//! `storage.rs`'s `entry_crc` folds two slices (`hdr ‖ value`) through
//! [`crc32_update`] with that same init/XOR — so moving the primitive here is
//! **byte-for-byte identical** to the firmware's previous private copy, and
//! existing EEPROM records stay valid.

/// One bitwise CRC-32 pass over `data`, continuing from `crc` (no table).
/// This is the raw update — apply the `0xFFFF_FFFF` init and final `!` yourself
/// (or use [`crc32_ieee`] for a one-shot).
pub fn crc32_update(mut crc: u32, data: &[u8]) -> u32 {
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    crc
}

/// One-shot CRC-32/IEEE over a single contiguous buffer (init `0xFFFF_FFFF`,
/// finalize `!`). Used for the console frame's integrity field.
pub fn crc32_ieee(data: &[u8]) -> u32 {
    !crc32_update(0xFFFF_FFFF, data)
}
