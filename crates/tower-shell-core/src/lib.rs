//! Host-testable pure kernels behind the shell's `addr` setting.
//!
//! Extracted from the no_std firmware `src/` (which can't `cargo test`) so the address
//! value parser and the `addr random` PRNG get host coverage — the same split as
//! `tower-net-core` / `tower-gw-core`. The firmware keeps the EEPROM I/O and the hardware
//! TRNG; the pure parse/transform lives here.

#![no_std]

/// Parse a `Kind::Addr` setting value into the stored 32-bit radio address.
///
/// * `auto` (any case) → `Some(0)` — the sentinel `shell::radio_addr` resolves to the
///   chip-UID-derived address.
/// * a hex literal — `0x1a2b3c4d` / `0X…` / bare `1a2b3c4d`, 1..=8 hex digits → its value.
/// * anything else → `None`.
///
/// The hex digits are validated explicitly, because `u32::from_str_radix(_, 16)` otherwise
/// silently accepts a leading `+` (`"+1"`, even `"0x+1"` → 1) — not a valid address literal.
/// (`random` is resolved by the caller before this ever runs.)
pub fn parse_addr(value: &str) -> Option<u32> {
    if value.eq_ignore_ascii_case("auto") {
        return Some(0);
    }
    let hex = value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
        .unwrap_or(value);
    if hex.is_empty() || hex.len() > 8 || !hex.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    u32::from_str_radix(hex, 16).ok()
}

/// One xorshift32 step over a seed forced non-zero (`| 1`).
///
/// The result is **never 0** — xorshift of a non-zero word stays non-zero — which is the
/// load-bearing property for an `addr` value (`0` is the `auto` sentinel). **Not
/// cryptographic**: the firmware mixes live entropy (chip UID, uptime tick, rolling state)
/// into `seed`; this is only the deterministic transform.
pub fn xorshift32_nonzero(seed: u32) -> u32 {
    let mut x = seed | 1;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_addr_auto_is_the_zero_sentinel() {
        assert_eq!(parse_addr("auto"), Some(0));
        assert_eq!(parse_addr("AUTO"), Some(0));
        assert_eq!(parse_addr("Auto"), Some(0));
    }

    #[test]
    fn parse_addr_hex_forms_agree() {
        assert_eq!(parse_addr("0x1a2b3c4d"), Some(0x1a2b_3c4d));
        assert_eq!(parse_addr("0X1A2B3C4D"), Some(0x1a2b_3c4d));
        assert_eq!(parse_addr("1a2b3c4d"), Some(0x1a2b_3c4d));
        assert_eq!(parse_addr("1"), Some(1));
        assert_eq!(parse_addr("ffffffff"), Some(0xffff_ffff));
    }

    #[test]
    fn parse_addr_length_bounds() {
        assert_eq!(parse_addr(""), None, "empty");
        assert_eq!(parse_addr("0x"), None, "prefix only");
        assert_eq!(parse_addr("123456789"), None, "9 hex digits > 32-bit");
        assert_eq!(parse_addr("0x123456789"), None, "9 digits after the prefix");
    }

    #[test]
    fn parse_addr_rejects_non_hex_and_signs() {
        // `u32::from_str_radix` would silently accept the sign forms — parse_addr must not.
        assert_eq!(parse_addr("+1"), None, "leading + is not an address literal");
        assert_eq!(parse_addr("0x+1"), None);
        assert_eq!(parse_addr("-1"), None);
        assert_eq!(parse_addr("12 34"), None, "embedded whitespace");
        assert_eq!(parse_addr("0xghij"), None, "non-hex letters");
        assert_eq!(
            parse_addr("random"),
            None,
            "resolved by the caller, never reaches here"
        );
    }

    #[test]
    fn xorshift_is_never_zero_even_from_a_zero_seed() {
        for seed in [0u32, 1, 0xffff_ffff, 0x8000_0000, 0x88a4_e90d] {
            assert_ne!(xorshift32_nonzero(seed), 0, "0 collides with the auto sentinel");
        }
    }

    #[test]
    fn xorshift_diverges_across_seeds() {
        // Distinct seeds → distinct outputs (the point of mixing fresh entropy per call).
        assert_ne!(xorshift32_nonzero(1), xorshift32_nonzero(2));
        assert_ne!(xorshift32_nonzero(0x1000), xorshift32_nonzero(0x2000));
    }

    #[test]
    fn xorshift_transform_is_pinned() {
        // Pin the exact transform (seed|1, then <<13 / >>17 / <<5) so an accidental
        // constant tweak is caught — the firmware's rand_u32_sw depends on this word.
        assert_eq!(xorshift32_nonzero(0), 270_369);
    }
}
