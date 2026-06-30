//! strip — scrolling rainbow on the WS2812 strip (PA1).
//!
//! Demonstrates the effects layer ([`strip`](tower::strip)) over the
//! PWM/DMA driver. Edit `NUM` for your strip length.
//!
//!   just flash example strip

#![no_std]
#![no_main]

use embassy_time::Timer;
use tower::strip::{LedKind, Strip};
use tower::{app, board::Board, ws2812};

/// Pixels on the strip.
const NUM: usize = 8;

async fn run(b: Board) {
    let mut strip = Strip::<NUM, { ws2812::rgbw_buf_len(NUM) }>::new(
        b.strip_tim,
        b.strip_data,
        b.strip_dma,
        LedKind::Rgb,
        50, // brightness % (gamma-corrected → perceptually ~half)
    );

    let mut t: u16 = 0;
    loop {
        strip.rainbow(t);
        strip.show().await;
        t = t.wrapping_add(1);
        Timer::after_millis(20).await;
    }
}

app!(run);
