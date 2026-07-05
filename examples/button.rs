//! button — debounced button events ([`button`](tower::button) block).
//!
//! Logs every event from the on-board button (PA8): `Press`, `Release`,
//! `Click` (short press), `Hold` (long press). A click flashes the LED briefly,
//! a hold flashes it longer — so it's visible with or without `tower logs`.
//!
//! Tune the debounce / click / hold timings via `button::Config`.
//!
//!   just flash example button   (then press/click/hold the button)

#![no_std]
#![no_main]

use embassy_stm32::gpio::{Level, Output, Speed};
use log::info;
use tower::{app, board::Board, button, led};

static BTN_CH: button::ButtonChannel = button::ButtonChannel::new();
static LED_CH: led::LedChannel = led::LedChannel::new();
static SHORT: led::Pattern = &[led::Step::on(60)];
static LONG: led::Pattern = &[led::Step::on(400)];

async fn run(b: Board) {
    let led = led::init(
        b.spawner,
        Output::new(b.led, Level::Low, Speed::Low),
        &LED_CH,
        led::Polarity::ActiveHigh,
    );
    let btn = button::init_exti(
        b.spawner,
        b.button,
        button::Polarity::ActiveHigh,
        button::Config::default(),
        &BTN_CH,
    );

    info!("press / release / click / hold the button...");
    loop {
        let event = btn.next_event().await;
        info!(target: "button", "{event}");
        match event {
            button::Event::Click => led.play(SHORT),
            button::Event::Hold => led.play(LONG),
            _ => {}
        }
    }
}

app!(run);
