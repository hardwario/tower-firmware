//! radio_push_button — TOWER IoT Kit product firmware (SKELETON).
//!
//! A battery push-button node: on each click it (will) send a secured radio message to the
//! gateway, then drop back to STOP low-power between presses. The on-board LED flashes as
//! local feedback.
//!
//! This is a starting skeleton — button events are handled and logged; the radio send is
//! marked TODO. The `net_*` examples and `docs/radio.md` (the `net` node role) show the
//! full pattern (`radio::init` from `b.radio_*`, pair with a gateway, then `send`).
//!
//!   just build app radio_push_button
//!   just run   app radio_push_button   (then press the button)

#![no_std]
#![no_main]

use embassy_stm32::gpio::{Level, Output, Speed};
use log::info;
use tower::{app, board::Board, button, led};

static BTN_CH: button::ButtonChannel = button::ButtonChannel::new();
static LED_CH: led::LedChannel = led::LedChannel::new();
static CLICK: led::Pattern = &[led::Step::on(60)];

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

    // TODO: bring up the SPIRIT1 radio as a node and pair with the gateway. See the
    // `net_*` examples + the `net` node role in `docs/radio.md`.
    info!(target: "button", "radio_push_button skeleton — press the button");

    loop {
        if let button::Event::Click = btn.next_event().await {
            led.play(CLICK);
            // TODO: send a secured radio message to the gateway here, e.g.
            //   net.send(&payload).await;
            info!(target: "button", "click — TODO: send radio message to the gateway");
        }
    }
}

app!(run);
