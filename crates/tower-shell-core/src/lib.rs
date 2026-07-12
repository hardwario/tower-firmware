//! Host-testable pure kernels behind the shell's `addr` setting.
//!
//! Extracted from the no_std firmware `src/` (which can't `cargo test`) so the address
//! value parser and the `addr random` PRNG get host coverage — the same split as
//! `tower-net-core` / `tower-gw-core`. The firmware keeps the EEPROM I/O and the hardware
//! TRNG; the pure parse/transform lives here.

#![no_std]

// Host test harness only: the `Window` tests use std collections (Vec/String).
#[cfg(test)]
extern crate std;

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

/// One window of a paginated byte stream — the kernel behind the shell's **windowed
/// re-run streaming**.
///
/// The shell dispatcher streams *any* command's response uniformly (no per-command
/// special-casing) by running the sync handler repeatedly and capturing one output
/// window per pass: the handler writes its full response as `&str` pieces; a `Window`
/// keeps only the char-aligned byte range `[skip, skip+cap)` and counts the total. The
/// driver sends the captured window, advances `skip` by [`captured`](Window::captured)
/// and re-runs until [`more`](Window::more) is false. A big dump therefore re-executes a
/// few times (fine — those commands are rare and read-only), while every ordinary
/// response fits one window and runs once.
///
/// `feed` returns the exact sub-slice to append to the caller's `cap`-sized buffer; all
/// slicing lands on UTF-8 boundaries, so a multi-byte char never straddles two windows
/// (which would make a frame's text an invalid `&str`).
#[derive(Debug, Clone, Copy)]
pub struct Window {
    skip: usize,
    cap: usize,
    total: usize,
    captured: usize,
}

impl Window {
    /// Capture the window `[skip, skip+cap)`. `skip` MUST be a char boundary of the
    /// stream (0, or a previous window's `skip + captured` — both are char-aligned).
    /// `cap` MUST be ≥ 4 (the max UTF-8 char length): a window narrower than the next
    /// char would capture nothing and the driver's re-run loop would never advance. The
    /// shell's chunk sizes (≥ 56) satisfy this trivially.
    #[must_use]
    pub fn new(skip: usize, cap: usize) -> Self {
        debug_assert!(cap >= 4, "window cap must fit any UTF-8 char");
        Self {
            skip,
            cap,
            total: 0,
            captured: 0,
        }
    }

    /// Feed the next `&str` piece of the stream; returns the sub-slice of `s` that falls
    /// inside this window (empty if none). Append the returned slice to the out buffer.
    pub fn feed<'a>(&mut self, s: &'a str) -> &'a str {
        let start = self.total; // stream offset of this piece (char-aligned: whole pieces)
        self.total += s.len();
        let room = self.cap - self.captured;
        if room == 0 {
            return "";
        }
        // Intersect the piece [start, start+len) with the window [skip, skip+cap).
        let lo = self.skip.saturating_sub(start).min(s.len());
        let hi_win = (self.skip + self.cap).saturating_sub(start).min(s.len());
        // `lo` is a char boundary: `start` is (piece boundary) and `skip` is (invariant),
        // so `skip - start` lands on one. Clamp the high end to a char boundary AND to the
        // remaining capacity so the out buffer never overflows.
        let mut hi = hi_win.min(lo + room);
        while hi > lo && !s.is_char_boundary(hi) {
            hi -= 1;
        }
        if hi <= lo {
            return "";
        }
        let seg = &s[lo..hi];
        self.captured += seg.len();
        seg
    }

    /// Total bytes the handler wrote this pass (window-independent).
    #[must_use]
    pub fn total(&self) -> usize {
        self.total
    }

    /// Bytes captured into this window (≤ `cap`).
    #[must_use]
    pub fn captured(&self) -> usize {
        self.captured
    }

    /// True if output exists beyond this window — re-run with `skip += captured`.
    #[must_use]
    pub fn more(&self) -> bool {
        self.skip + self.captured < self.total
    }
}

/// Drive a [`Window`] over deterministic `pieces` (a test helper): returns each
/// captured window, so a round-trip proves the pagination loses and duplicates nothing.
#[cfg(test)]
fn paginate(pieces: &[&str], cap: usize) -> std::vec::Vec<std::string::String> {
    use std::string::String;
    use std::vec::Vec;
    let mut windows: Vec<String> = Vec::new();
    let mut skip = 0;
    loop {
        let mut w = Window::new(skip, cap);
        let mut chunk = String::new();
        for p in pieces {
            chunk.push_str(w.feed(p));
        }
        windows.push(chunk);
        if !w.more() {
            break;
        }
        skip += w.captured();
        assert!(skip <= w.total(), "skip overran the stream");
    }
    windows
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

    // ---- Window (shell response pagination) -------------------------------------

    /// The core invariant: paginating and reassembling loses/duplicates nothing, and
    /// no window exceeds `cap`.
    fn assert_roundtrip(pieces: &[&str], cap: usize) {
        let expected: std::string::String = pieces.concat();
        let windows = paginate(pieces, cap);
        for w in &windows {
            assert!(w.len() <= cap, "window {:?} exceeds cap {cap}", w);
        }
        let joined: std::string::String = windows.concat();
        assert_eq!(joined, expected, "pieces={pieces:?} cap={cap}");
    }

    #[test]
    fn window_paginates_ascii_without_loss() {
        // "/export"-shaped output split across many small windows.
        let pieces = &[
            "identity = tower\r\n",
            "addr = 0x88a4e90d\r\n",
            "therm-period = 60\r\n",
            "therm-delta = 50\r\n",
        ];
        for cap in [4, 7, 16, 20, 1000] {
            assert_roundtrip(pieces, cap);
        }
    }

    #[test]
    fn window_single_pass_when_it_fits() {
        let mut w = Window::new(0, 64);
        assert_eq!(w.feed("hello"), "hello");
        assert_eq!(w.feed(" world"), " world");
        assert!(!w.more(), "fits one window → no re-run");
        assert_eq!(w.total(), 11);
        assert_eq!(w.captured(), 11);
    }

    #[test]
    fn window_never_splits_a_multibyte_char() {
        // A 3-byte '✓' (and 2-byte 'αβγ') straddling every window boundary must move
        // whole. `cap` starts at 4 — the documented minimum (a narrower window than the
        // next char captures nothing and the driver can't advance).
        let pieces = &["ok ✓ done ✓✓ αβγ end"];
        for cap in 4..24 {
            let windows = paginate(pieces, cap);
            for w in &windows {
                // Each window is itself valid UTF-8 (would panic on a split char).
                assert!(std::str::from_utf8(w.as_bytes()).is_ok());
            }
            assert_roundtrip(pieces, cap);
        }
    }

    #[test]
    fn window_empty_and_exact_multiple() {
        assert_roundtrip(&[""], 8);
        assert_roundtrip(&["", "", ""], 8);
        // Exactly two full windows, nothing left over.
        assert_roundtrip(&["abcdefgh", "ijklmnop"], 8);
    }
}
