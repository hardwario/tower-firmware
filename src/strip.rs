//! Addressable-LED **strip effects** — a high-level layer over the [`ws2812`](crate::ws2812)
//! driver.
//!
//! A [`Strip`] owns an RGB *intent* framebuffer plus the WS2812 driver. You
//! compose a frame (static colour, compound segments, gradient, or one of the
//! animated effects), then [`show`](Strip::show) renders it — applying the
//! global brightness and gamma correction — over the wire.
//!
//! **Brightness (0–100 %)** is gamma-corrected so the knob is *approximately
//! perceptually* linear: 50 % looks roughly half as bright to the eye. Uses
//! integer gamma 2.0 (`out = (ch·b/100)² / 255`), no float or lookup tables — the
//! eye's response (≈ √light) is the inverse of the squaring, so perceived
//! brightness ∝ the 0–100 value (gamma 2.0 approximates the ~2.2 sRGB curve).
//! The same correction smooths colour gradients.
//!
//! Effects are **frame-based**: each effect method fills the framebuffer for a
//! frame counter `t` that the caller advances in its own loop, e.g.
//!
//! ```ignore
//! let mut t: u16 = 0;
//! loop {
//!     strip.rainbow(t);
//!     strip.show().await;
//!     t = t.wrapping_add(1);
//!     Timer::after_millis(20).await;
//! }
//! ```
//!
//! Common maker effects provided: solid, [compound segments](Strip::segments),
//! [gradient](Strip::gradient), [rainbow](Strip::rainbow),
//! [color wipe](Strip::color_wipe), [theater chase](Strip::theater_chase),
//! [breathe](Strip::breathe), [scanner/Larson](Strip::scanner), and
//! [sparkle](Strip::sparkle).
//!
//! Scope: the framebuffer is RGB; on an RGBW strip the white channel is left
//! off (use the [`ws2812`](crate::ws2812) driver directly for white-channel control).

use embassy_stm32::Peri;
use embassy_stm32::peripherals::{DMA1_CH3, PA1, TIM2};

use crate::ws2812::{Rgb, Rgbw, Ws2812, rgbw_buf_len};

/// Which LED type the strip drives (sets the wire format at [`Strip::show`]).
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum LedKind {
    /// WS2812B — 24-bit RGB.
    Rgb,
    /// SK6812 — 32-bit RGBW (white left off by the effects).
    Rgbw,
}

/// One run of the [compound fill](Strip::segments): `len` pixels of `color`.
#[derive(Clone, Copy)]
pub struct Segment {
    pub len: usize,
    pub color: Rgb,
}

impl Segment {
    pub const fn new(len: usize, color: Rgb) -> Self {
        Self { len, color }
    }
}

// A few named colours (full-intent RGB; brightness/gamma applied at `show`).
pub const OFF: Rgb = Rgb::new(0, 0, 0);
pub const RED: Rgb = Rgb::new(255, 0, 0);
pub const GREEN: Rgb = Rgb::new(0, 255, 0);
pub const BLUE: Rgb = Rgb::new(0, 0, 255);
pub const YELLOW: Rgb = Rgb::new(255, 255, 0);
pub const CYAN: Rgb = Rgb::new(0, 255, 255);
pub const MAGENTA: Rgb = Rgb::new(255, 0, 255);
pub const ORANGE: Rgb = Rgb::new(255, 96, 0);
pub const WHITE: Rgb = Rgb::new(255, 255, 255);

/// Apply brightness (0–100 %) and gamma 2.0 to one channel.
fn correct(channel: u8, brightness: u8) -> u8 {
    let v = channel as u16 * brightness.min(100) as u16 / 100; // 0..=255 intent
    ((v * v + 127) / 255) as u8 // gamma 2.0 (+127 = round to nearest)
}

/// Scale an intent colour by `factor`/255 (used by effects, stays in intent space).
fn scale(c: Rgb, factor: u8) -> Rgb {
    let s = |x: u8| (x as u16 * factor as u16 / 255) as u8;
    Rgb::new(s(c.r), s(c.g), s(c.b))
}

/// Colour wheel: 0..=255 → smooth R→G→B→R spectrum (for rainbow effects).
fn wheel(pos: u8) -> Rgb {
    let p = 255 - pos;
    if p < 85 {
        Rgb::new(255 - p * 3, 0, p * 3)
    } else if p < 170 {
        let p = p - 85;
        Rgb::new(0, p * 3, 255 - p * 3)
    } else {
        let p = p - 170;
        Rgb::new(p * 3, 255 - p * 3, 0)
    }
}

/// Tiny xorshift PRNG for the random effects (sparkle, …). Deterministic per seed.
pub struct Rng(u32);

impl Rng {
    pub const fn new(seed: u32) -> Self {
        // Avoid the all-zero state.
        Self(if seed == 0 { 0x1234_5678 } else { seed })
    }
    fn next_u32(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }
    /// Random value in `0..n`.
    fn below(&mut self, n: usize) -> usize {
        if n == 0 { 0 } else { (self.next_u32() as usize) % n }
    }
}

/// A WS2812/SK6812 strip with an effects framebuffer.
///
/// `NUM` is the pixel count; `BUF` is the underlying driver buffer length —
/// always pass `{ ws2812::rgbw_buf_len(NUM) }` (it fits both RGB and RGBW).
pub struct Strip<'d, const NUM: usize, const BUF: usize> {
    ws: Ws2812<'d, BUF>,
    fb: [Rgb; NUM],
    brightness: u8,
    kind: LedKind,
}

impl<'d, const NUM: usize, const BUF: usize> Strip<'d, NUM, BUF> {
    /// Compile-time guard: `BUF` must hold the full RGBW encoding of `NUM` pixels, so either
    /// wire format fits. A wrong `BUF` (anything but `{ ws2812::rgbw_buf_len(NUM) }`) fails the
    /// **build** — unlike a `debug_assert!`, which would be compiled out in release and only trap
    /// (or silently overflow) on hardware.
    const _BUF_FITS: () = assert!(BUF >= rgbw_buf_len(NUM), "Strip: BUF must be rgbw_buf_len(NUM)");

    /// Create a strip on PA1 (TIM2_CH2 + DMA1_CH3). `brightness` is 0–100 %.
    pub fn new(
        tim: Peri<'d, TIM2>,
        data: Peri<'d, PA1>,
        dma: Peri<'d, DMA1_CH3>,
        kind: LedKind,
        brightness: u8,
    ) -> Self {
        let () = Self::_BUF_FITS; // force the compile-time BUF check for this (NUM, BUF)
        Self {
            ws: Ws2812::new(tim, data, dma),
            fb: [OFF; NUM],
            brightness: brightness.min(100),
            kind,
        }
    }

    /// Set the global brightness (0–100 %, gamma-corrected at [`show`](Self::show)).
    pub fn set_brightness(&mut self, percent: u8) {
        self.brightness = percent.min(100);
    }

    /// Number of pixels.
    pub const fn len(&self) -> usize {
        NUM
    }

    /// Whether the strip has no pixels (`NUM == 0`).
    pub const fn is_empty(&self) -> bool {
        NUM == 0
    }

    // --- Composition (fills the framebuffer; call `show` to display) ---------

    /// All pixels off.
    pub fn clear(&mut self) {
        self.fb = [OFF; NUM];
    }

    /// Solid colour across the whole strip.
    pub fn fill(&mut self, color: Rgb) {
        self.fb = [color; NUM];
    }

    /// Set a single pixel (out-of-range index is ignored).
    pub fn set(&mut self, i: usize, color: Rgb) {
        if let Some(p) = self.fb.get_mut(i) {
            *p = color;
        }
    }

    /// Compound fill: consecutive runs, e.g. `&[Segment::new(20, RED),
    /// Segment::new(80, GREEN)]`. Pixels past the last segment are cleared;
    /// runs past the end of the strip are clamped.
    pub fn segments(&mut self, segments: &[Segment]) {
        let mut i = 0;
        for seg in segments {
            for _ in 0..seg.len {
                if i >= NUM {
                    return;
                }
                self.fb[i] = seg.color;
                i += 1;
            }
        }
        for p in &mut self.fb[i..] {
            *p = OFF;
        }
    }

    /// Linear gradient from `from` (pixel 0) to `to` (last pixel).
    pub fn gradient(&mut self, from: Rgb, to: Rgb) {
        let last = NUM.saturating_sub(1).max(1);
        for (i, p) in self.fb.iter_mut().enumerate() {
            let lerp = |a: u8, b: u8| {
                let a = a as i32;
                let b = b as i32;
                (a + (b - a) * i as i32 / last as i32) as u8
            };
            *p = Rgb::new(lerp(from.r, to.r), lerp(from.g, to.g), lerp(from.b, to.b));
        }
    }

    // --- Effects (fill the framebuffer for frame `t`) ------------------------

    /// Full rainbow spread across the strip, scrolling with `t`.
    pub fn rainbow(&mut self, t: u16) {
        for (i, p) in self.fb.iter_mut().enumerate() {
            let hue = (i as u32 * 256 / NUM.max(1) as u32 + t as u32) as u8;
            *p = wheel(hue);
        }
    }

    /// Fill the strip one pixel at a time (then wrap), in `color`.
    pub fn color_wipe(&mut self, color: Rgb, t: u16) {
        let lit = (t as usize) % (NUM + 1);
        for (i, p) in self.fb.iter_mut().enumerate() {
            *p = if i < lit { color } else { OFF };
        }
    }

    /// Marquee "theater chase": every third pixel lit, advancing with `t`.
    pub fn theater_chase(&mut self, color: Rgb, t: u16) {
        let phase = (t as usize) % 3;
        for (i, p) in self.fb.iter_mut().enumerate() {
            *p = if i % 3 == phase { color } else { OFF };
        }
    }

    /// Whole-strip brightness pulse (triangle wave over `t`).
    pub fn breathe(&mut self, color: Rgb, t: u16) {
        let x = t % 512;
        let level = if x < 256 { x } else { 511 - x } as u8; // 0→255→0
        let c = scale(color, level);
        self.fb = [c; NUM];
    }

    /// Larson/Cylon scanner: a dot that bounces end to end with a fading trail.
    pub fn scanner(&mut self, color: Rgb, t: u16) {
        let span = (2 * NUM).saturating_sub(2).max(1);
        let x = (t as usize) % span;
        let head = if x < NUM { x } else { span - x }; // bounce 0..NUM-1..0
        for (i, p) in self.fb.iter_mut().enumerate() {
            let dist = head.abs_diff(i);
            *p = match dist {
                0 => color,
                1 => scale(color, 80),
                2 => scale(color, 20),
                _ => OFF,
            };
        }
    }

    /// Random twinkles in `color` that fade out; advance `t` and reuse `rng`.
    pub fn sparkle(&mut self, color: Rgb, rng: &mut Rng) {
        // Fade everything, then light a random pixel.
        for p in &mut self.fb {
            *p = scale(*p, 200); // ~78 % each frame → a soft tail
        }
        let i = rng.below(NUM);
        if let Some(p) = self.fb.get_mut(i) {
            *p = color;
        }
    }

    // --- Output --------------------------------------------------------------

    /// Render the framebuffer (brightness + gamma applied) over the wire.
    pub async fn show(&mut self) {
        let b = self.brightness;
        match self.kind {
            LedKind::Rgb => {
                let mut out = [Rgb::default(); NUM];
                for (o, p) in out.iter_mut().zip(&self.fb) {
                    *o = Rgb::new(correct(p.r, b), correct(p.g, b), correct(p.b, b));
                }
                self.ws.write_rgb(&out).await;
            }
            LedKind::Rgbw => {
                let mut out = [Rgbw::default(); NUM];
                for (o, p) in out.iter_mut().zip(&self.fb) {
                    *o = Rgbw::new(correct(p.r, b), correct(p.g, b), correct(p.b, b), 0);
                }
                self.ws.write_rgbw(&out).await;
            }
        }
    }
}
