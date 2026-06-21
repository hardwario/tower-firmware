//! blinky — the on-board LED ([`led`](tower::led) block).
//!
//! A slow heartbeat with a double-blink fired every 5 s that preempts it.
//!
//!   just flash blinky

#![no_std]
#![no_main]

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;
use tower::led::{self, LedChannel, Pattern, Polarity, Step};
use tower::{app, board::Board};

static CH: LedChannel = LedChannel::new();
static HEARTBEAT: Pattern = &[Step::on(40), Step::off(1960)];
static DOUBLE: Pattern = &[Step::on(60), Step::off(80), Step::on(60)];

async fn run(b: Board) {
    let led = led::init(
        b.spawner,
        Output::new(b.led, Level::Low, Speed::Low),
        &CH,
        Polarity::ActiveHigh,
    );
    led.set_background(Some(HEARTBEAT));

    loop {
        Timer::after_secs(5).await;
        led.play(DOUBLE);
    }
}

app!(run);
