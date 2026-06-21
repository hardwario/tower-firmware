//! WS2812B / SK6812 addressable-LED driver — timer PWM + DMA.
//!
//! Drives a strip on **PA1** (the TOWER WS2812 data pin) using **TIM2 channel 2**
//! in PWM mode: one timer period is one protocol bit (~1.25 µs at 800 kHz), and
//! each bit's high-time is set by a per-bit compare value that **DMA1 channel 3**
//! (the TIM2_CH2 request) streams from RAM into the capture/compare register on
//! every timer update. A short compare = logic '0' (~0.4 µs high), a long
//! compare = logic '1' (~0.8 µs high). A run of zero-duty slots after the data
//! holds the line low to latch the strip (reset).
//!
//! Supports 24-bit **RGB** (WS2812B, sent G-R-B) and 32-bit **RGBW** (SK6812,
//! sent G-R-B-W), and an arbitrary pixel count up to the buffer the driver is
//! sized for: `Ws2812::<N>` where `N` = [`rgb_buf_len`] / [`rgbw_buf_len`].
//!
//! **Clock requirement:** the timer must resolve the ~0.4/0.8 µs high-times, so
//! it needs a fast clock — comfortable at the firmware's 16 MHz sysclk (20 ticks
//! per bit). It will *not* meet WS2812 timing at the old MSI 4 MHz (~5 ticks).
//! Compare values are derived from the timer's actual period, so it self-adjusts
//! to the clock.
//!
//! ```ignore
//! let mut strip = Ws2812::<{ ws2812::rgb_buf_len(8) }>::new(p.TIM2, p.PA1, p.DMA1_CH3);
//! strip.write_rgb(&[Rgb::new(16, 0, 0); 8]).await;
//! ```

#![allow(dead_code)]

use embassy_stm32::Peri;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::InterruptHandler;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::peripherals::{DMA1_CH3, PA1, TIM2};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};

// DMA1_CH3 (TIM2_CH2 request) shares the L0 DMA1_CHANNEL2_3 vector. Only this
// driver uses that channel, so binding it here is self-contained.
bind_interrupts!(struct Irqs {
    DMA1_CHANNEL2_3 => InterruptHandler<DMA1_CH3>;
});

/// WS2812 bit rate — one timer period per protocol bit.
const BIT_HZ: u32 = 800_000;

/// Reset/latch gap appended after the data, in bit-slots held low.
/// 64 × 1.25 µs ≈ 80 µs — safe for both WS2812B and SK6812.
pub const RESET_SLOTS: usize = 64;

const RGB_BITS: usize = 24;
const RGBW_BITS: usize = 32;

/// 24-bit pixel for WS2812B (transmitted in G, R, B order).
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Rgb {
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }
}

/// 32-bit pixel for SK6812 RGBW (transmitted in G, R, B, W order).
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub struct Rgbw {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub w: u8,
}

impl Rgbw {
    pub const fn new(r: u8, g: u8, b: u8, w: u8) -> Self {
        Self { r, g, b, w }
    }
}

/// Compare-buffer length (u16 slots) needed for `pixels` RGB LEDs.
pub const fn rgb_buf_len(pixels: usize) -> usize {
    pixels * RGB_BITS + RESET_SLOTS
}

/// Compare-buffer length (u16 slots) needed for `pixels` RGBW LEDs.
pub const fn rgbw_buf_len(pixels: usize) -> usize {
    pixels * RGBW_BITS + RESET_SLOTS
}

/// WS2812/SK6812 strip on PA1 (TIM2_CH2 + DMA1_CH3). `N` is the compare-value
/// buffer length — size it with [`rgb_buf_len`] / [`rgbw_buf_len`].
pub struct Ws2812<'d, const N: usize> {
    pwm: SimplePwm<'d, TIM2>,
    dma: Peri<'d, DMA1_CH3>,
    buf: [u16; N],
    ccr0: u16, // compare value for a logic '0' bit
    ccr1: u16, // compare value for a logic '1' bit
}

impl<'d, const N: usize> Ws2812<'d, N> {
    /// Create the driver: `tim` = TIM2, `data` = PA1, `dma` = DMA1_CH3.
    pub fn new(tim: Peri<'d, TIM2>, data: Peri<'d, PA1>, dma: Peri<'d, DMA1_CH3>) -> Self {
        let pwm = SimplePwm::new(
            tim,
            None,
            Some(PwmPin::new(data, OutputType::PushPull)), // CH2 = PA1
            None,
            None,
            Hertz(BIT_HZ),
            CountingMode::EdgeAlignedUp,
        );
        // Period = `max` ticks. High-time as a fraction of the 1.25 µs bit:
        // '0' ≈ 0.40 µs (0.32), '1' ≈ 0.80 µs (0.64); round to the nearest tick.
        let max = pwm.max_duty_cycle();
        let ccr0 = ((max * 32 + 50) / 100) as u16;
        let ccr1 = ((max * 64 + 50) / 100) as u16;
        Self {
            pwm,
            dma,
            buf: [0; N],
            ccr0,
            ccr1,
        }
    }

    /// Send RGB pixels (WS2812B). Awaits the DMA transfer and the reset latch.
    pub async fn write_rgb(&mut self, pixels: &[Rgb]) {
        let used = rgb_buf_len(pixels.len());
        assert!(used <= N, "ws2812: buffer too small for RGB pixels");
        let mut i = 0;
        for px in pixels {
            i = self.encode_byte(i, px.g);
            i = self.encode_byte(i, px.r);
            i = self.encode_byte(i, px.b);
        }
        self.flush(i, used).await;
    }

    /// Send RGBW pixels (SK6812). Awaits the DMA transfer and the reset latch.
    pub async fn write_rgbw(&mut self, pixels: &[Rgbw]) {
        let used = rgbw_buf_len(pixels.len());
        assert!(used <= N, "ws2812: buffer too small for RGBW pixels");
        let mut i = 0;
        for px in pixels {
            i = self.encode_byte(i, px.g);
            i = self.encode_byte(i, px.r);
            i = self.encode_byte(i, px.b);
            i = self.encode_byte(i, px.w);
        }
        self.flush(i, used).await;
    }

    /// Encode one byte MSB-first into `buf` from index `i`; return the next index.
    fn encode_byte(&mut self, mut i: usize, byte: u8) -> usize {
        for bit in (0..8).rev() {
            self.buf[i] = if (byte >> bit) & 1 != 0 {
                self.ccr1
            } else {
                self.ccr0
            };
            i += 1;
        }
        i
    }

    async fn flush(&mut self, data_end: usize, used: usize) {
        // Reset/latch: hold the line low for the trailing slots.
        for slot in &mut self.buf[data_end..used] {
            *slot = 0;
        }
        self.pwm
            .waveform(self.dma.reborrow(), Irqs, Channel::Ch2, &self.buf[..used])
            .await;
    }
}
