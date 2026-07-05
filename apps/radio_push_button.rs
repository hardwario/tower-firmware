//! radio_push_button — TOWER IoT Kit product firmware (SKELETON).
//!
//! A battery push-button node: on each click it (will) send a secured radio message to the
//! gateway, then drop back to STOP low-power between presses. The on-board LED flashes as
//! local feedback.
//!
//! This is a starting skeleton — button events are handled and logged; the radio send is
//! marked TODO. The `net_*` examples and `docs/radio.md` (the `net` node role) show the
//! full pattern: build the radio with `Spirit1::new(b.radio_*)`, then `Net::new(radio, b.kv,
//! NetConfig { .. })`, pair with a gateway, and `net.send(..)`.
//!
//!   just build app radio_push_button
//!   just run   app radio_push_button   (then press the button)

#![no_std]
#![no_main]

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Duration;
use log::info;
use tower::{app, board::Board, button, led, watchdog};

static BTN_CH: button::ButtonChannel = button::ButtonChannel::new();
static LED_CH: led::LedChannel = led::LedChannel::new();
static CLICK: led::Pattern = &[led::Step::on(60)];

async fn run(b: Board) {
    // Hardware safety net: a wedged unit resets itself instead of dying in the field. The
    // feeder wakes the low-power executor even from STOP; the L0 hardware ceiling (~26 s)
    // keeps those wakes rare on this battery node.
    watchdog::enable(b.iwdg, b.spawner, Duration::from_secs(26));

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
    // `net_*` examples + the `net` node role in `docs/radio.md`, e.g.:
    //   let radio = tower::radio::Spirit1::new(b.radio_spi, b.radio_sck, b.radio_mosi,
    //                   b.radio_miso, b.radio_cs, b.radio_sdn, b.radio_irq);
    //   let mut net = tower::radio::net::Net::new(radio, b.kv,
    //                   NetConfig { my_id, key: KEY, band: Band::Eu868, channel: 0 }).await?;
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
