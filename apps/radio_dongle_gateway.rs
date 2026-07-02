//! radio_dongle_gateway — TOWER IoT Kit product firmware (SKELETON).
//!
//! The USB Radio Dongle acts as the **gateway**: it receives secured frames from the
//! battery nodes (push button, climate monitor, …) over the SPIRIT1 sub-GHz radio and
//! forwards them to the host over the framed console. Stays awake on USB (VBUS).
//!
//! This is a starting skeleton — it boots, sets up the SDK, and blinks a slow heartbeat
//! so you can see it is alive. Wire up the radio gateway role where marked TODO; the
//! `radio_gateway` / `net_*` examples and `docs/radio.md` (the `net` layer, gateway role)
//! show the full pattern: build the radio with `Spirit1::new(b.radio_*)`, then
//! `Net::new(radio, b.kv, NetConfig { .. })`, then `net.recv(..)` + forward to the console.
//!
//!   just build app radio_dongle_gateway
//!   just run   app radio_dongle_gateway

#![no_std]
#![no_main]

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;
use log::info;
use tower::led::{self, LedChannel, Pattern, Polarity, Step};
use tower::{app, board::Board};

static CH: LedChannel = LedChannel::new();
// Slow "gateway alive" heartbeat on the on-board LED.
static HEARTBEAT: Pattern = &[Step::on(30), Step::off(1970)];

async fn run(b: Board) {
    let led = led::init(
        b.spawner,
        Output::new(b.led, Level::Low, Speed::Low),
        &CH,
        Polarity::ActiveHigh,
    );
    led.set_background(Some(HEARTBEAT));

    // TODO: bring up the SPIRIT1 radio as a gateway and forward received frames to the
    // host. See `examples/radio_gateway.rs` + the `net` gateway role in `docs/radio.md`:
    //   let radio = tower::radio::Spirit1::new(b.radio_spi, b.radio_sck, b.radio_mosi,
    //                   b.radio_miso, b.radio_cs, b.radio_sdn, b.radio_irq);
    //   let mut net = tower::radio::net::Net::new(radio, b.kv,
    //                   NetConfig { my_id, key: KEY, band: Band::Eu868, channel: 0 }).await?;
    //   loop { if let Some(rx) = net.recv(timeout).await { /* forward rx.data() to console */ } }
    info!(target: "gateway", "radio_dongle_gateway skeleton — wire up the radio gateway role");

    loop {
        Timer::after_secs(60).await;
    }
}

app!(run);
