//! radio_push_button — TOWER IoT Kit product firmware: a wireless push button with
//! thermometer + accelerometer, built as a **sleeping node**.
//!
//! Every button press / release / click / hold sends a secured radio message carrying
//! that event's running count; temperature is measured on a settable period and sent
//! on-change (threshold) or at latest every heartbeat; the accelerometer's tilt
//! interrupt wakes the node for motion/orientation events. Between events the MCU
//! STOPs (µA) and the SPIRIT1 sleeps — downlinks reach the node via the gateway's
//! queue: each confirmed uplink's ACK carries a *pending* flag, and when set the node
//! holds a short RX window, executes the delivered remote-shell line in the standard
//! shell dispatcher ([`shell::run_line`]), and streams the response back as chunked
//! radio messages. No tab completion over the air — completion is a per-transport
//! feature and the radio transport simply doesn't offer it.
//!
//! Pairing:
//! * **OTA** — hold the button ≥1 s while unprovisioned: the node runs the 3-way JOIN
//!   against any gateway with an open window and persists `(gw, key)`.
//! * **Cable** — on USB, the node serves the management channel: `Describe` (role
//!   probe), `Provision` (host-minted credentials; key never rides the shell), and
//!   `JoinOpen` (host-initiated OTA join).
//!
//! Each of the four button events has a **master enable**: a disabled event is ignored
//! entirely — neither reported over radio nor shown on the LED. The event *recognition*
//! timing is configurable (debounce, click-timeout, hold-time — what the physical input
//! means); the LED feedback is a fixed indicator (press/release/hold a single pulse, a
//! click a double-blink — distinct shapes because a quick tap fires Press on the down
//! edge and Release+Click together on the up edge, so a shared shape would blend). The
//! default is the coherent **gesture** scheme: click + hold on, press + release off;
//! enable press/release for raw-edge reporting.
//!
//! All boards also play the common power-on signature first (500 ms off → 2 s on →
//! 500 ms off, [`Board::boot_indicator`](tower::board::Board::boot_indicator)) — this
//! app's own behaviour begins after that.
//!
//! Settings (remote-shell reconfigurable, `/system settings set …`): `temp-period`,
//! `temp-delta` (centi-°C, 0 = always send), `accel` (off/low/medium/high — applied at
//! boot), `heartbeat`; per-event `{press,release,click,hold}` (on/off master enable);
//! button timing `debounce-press`, `debounce-release`, `click-timeout`, `hold-time`
//! (ms — applied at boot). The `/button simulate <ms>` command injects a synthetic press
//! of that length through the *real* recognition machine (debounce/click/hold) and on
//! into reporting — the console "finger" for testing without a physical press, also
//! used by the HIL bench.
//!
//!   just build app radio_push_button
//!   just run   app radio_push_button   (then press the button)

#![no_std]
#![no_main]

use embassy_futures::select::{Either4, select4};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Instant, Timer};
use log::{info, warn};
use tower::board::GuardedI2c;
use tower::lis2dh12::{self, Lis2dh12, Sensitivity, TiltConfig};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig, PAIRING_WINDOW, SendResult};
use tower::storage::{NS_APP, NS_SHELL, Nv};
use tower::tmp112::{self, Tmp112};
use tower::{app, board::Board, button, console, led, shell, watchdog};
use tower_protocol::mgmt::{self, DeviceInfo, DeviceRole, Joined, MgmtOp, ProvisionAck};
use tower_protocol::msg::MgmtRequest;
use tower_protocol::radio::{
    AccelKind, ButtonKind, MAX_RADIO_PAYLOAD, NodeCmd, NodeInfo, NodeMsg, NodeShellChunk,
    RADIO_SCHEMA_VERSION, RADIO_SHELL_CHUNK, decode_node_cmd, encode_node_msg,
};
use tower_protocol::{MsgType, decode_frame};

// --- persistence (NS_APP) --------------------------------------------------------

/// Provision record: `(gw_id, key, band, channel)` (postcard). The node's own radio
/// address is the SDK `address` base setting (`shell::radio_address`), not stored here.
const KEY_PROVISION: u8 = 0x00;

#[derive(serde::Serialize, serde::Deserialize, Clone, Copy)]
struct Provisioned {
    gw_id: u32,
    key: [u8; 16],
    band: u8,
    channel: u8,
}

// --- settings (NS_SHELL locals) ----------------------------------------------------

const SET_TEMP_PERIOD: u8 = 0x10;
const SET_TEMP_DELTA: u8 = 0x11;
const SET_ACCEL: u8 = 0x12;
const SET_HEARTBEAT: u8 = 0x13;

// Per-button-event master enable: a disabled event is ignored entirely — NOT reported
// over radio and no LED. Each of the four events is independently switchable.
const SET_PRESS: u8 = 0x20;
const SET_RELEASE: u8 = 0x22;
const SET_CLICK: u8 = 0x24;
const SET_HOLD: u8 = 0x26;

// Button *recognition* timing (ms) → `button::Config`. These shape what the physical
// input means: how long the level must hold to count as a press/release, the longest
// press+release that still reads as a click, and the hold-to-trigger duration. Read
// once at boot (a change needs a reboot, like `accel`).
const SET_DEBOUNCE_PRESS: u8 = 0x30;
const SET_DEBOUNCE_RELEASE: u8 = 0x31;
const SET_CLICK_TIMEOUT: u8 = 0x32;
const SET_HOLD_TIME: u8 = 0x33;

// Fixed LED-feedback shapes/timings (not user-configurable — the LED is just an
// indicator). press/release/hold flash a single pulse; a click is a double-blink of
// two `LED_CLICK_MS` blinks so it reads apart from an adjacent press on the one LED.
const LED_PRESS_MS: u32 = 250;
const LED_RELEASE_MS: u32 = 250;
const LED_CLICK_MS: u32 = 100;
const LED_HOLD_MS: u32 = 1000;

static BTN_SETTINGS: &[shell::Setting] = &[
    shell::Setting {
        key: SET_TEMP_PERIOD,
        name: "temp-period",
        kind: shell::Kind::Uint { min: 5, max: 86_400 },
        default: "60",
    },
    shell::Setting {
        key: SET_TEMP_DELTA,
        name: "temp-delta",
        kind: shell::Kind::Uint { min: 0, max: 10_000 },
        default: "50",
    },
    shell::Setting {
        key: SET_ACCEL,
        name: "accel",
        kind: shell::Kind::Enum(&["off", "low", "medium", "high"]),
        default: "medium",
    },
    shell::Setting {
        key: SET_HEARTBEAT,
        name: "heartbeat",
        kind: shell::Kind::Uint { min: 60, max: 86_400 },
        default: "900",
    },
    // Button events — each a master enable (report over radio + fixed LED feedback); a
    // disabled event is ignored entirely. Default = the coherent **gesture** scheme:
    // click + hold ON, press + release OFF (a click is a quick press+release, so
    // reporting all four is redundant and their LEDs blend on a tap; enable
    // press/release for raw edges).
    shell::Setting {
        key: SET_PRESS,
        name: "press",
        kind: shell::Kind::Bool,
        default: "off",
    },
    shell::Setting {
        key: SET_RELEASE,
        name: "release",
        kind: shell::Kind::Bool,
        default: "off",
    },
    shell::Setting {
        key: SET_CLICK,
        name: "click",
        kind: shell::Kind::Bool,
        default: "on",
    },
    shell::Setting {
        key: SET_HOLD,
        name: "hold",
        kind: shell::Kind::Bool,
        default: "on",
    },
    // Button recognition timing (ms) — applied at boot (reboot to change).
    shell::Setting {
        key: SET_DEBOUNCE_PRESS,
        name: "debounce-press",
        kind: shell::Kind::Uint { min: 1, max: 1000 },
        default: "30",
    },
    shell::Setting {
        key: SET_DEBOUNCE_RELEASE,
        name: "debounce-release",
        kind: shell::Kind::Uint { min: 1, max: 1000 },
        default: "30",
    },
    shell::Setting {
        key: SET_CLICK_TIMEOUT,
        name: "click-timeout",
        kind: shell::Kind::Uint { min: 50, max: 5000 },
        default: "500",
    },
    shell::Setting {
        key: SET_HOLD_TIME,
        name: "hold-time",
        kind: shell::Kind::Uint {
            min: 100,
            max: 10_000,
        },
        default: "1000",
    },
];

// --- synthetic button press (the console "finger" / HIL bench) ---------------------

/// Longest press `/button simulate` will inject (10 s — well past any hold-time).
const SIM_MAX_MS: u32 = 10_000;

/// `/button simulate <ms>` — inject a synthetic press held `<ms>` milliseconds into the
/// button driver, which derives the event(s) through the *real* recognition machine
/// (debounce / click-timeout / hold-time). So a short press debounces away, a tap
/// yields a click, a long press yields a hold — the same path a physical press takes,
/// exercising the configured button timing (which feeding a finished event would skip).
fn cmd_simulate(ctx: &mut shell::Ctx<'_>, args: &[&str]) -> shell::Outcome {
    use core::fmt::Write as _;
    match args.first().and_then(|s| s.parse::<u32>().ok()) {
        Some(ms) if (1..=SIM_MAX_MS).contains(&ms) => {
            button::simulate(ms);
            let _ = write!(ctx, "simulating a {ms} ms press");
            shell::Outcome::ok()
        }
        _ => {
            let _ = write!(ctx, "usage: /button simulate <ms> (1..={SIM_MAX_MS})");
            shell::Outcome::code(shell::R_BAD_ARG)
        }
    }
}

static BTN_COMMANDS: &[shell::Entry] = &[shell::Entry::menu(
    "button",
    &[shell::Entry::cmd("simulate", shell::Args::None, cmd_simulate)],
)];

// --- LED patterns -------------------------------------------------------------------

static LED_CH: led::LedChannel = led::LedChannel::new();
/// Double-blink background: not paired with any gateway yet (hold the button to pair).
static UNPROVISIONED: led::Pattern = &[
    led::Step::on(60),
    led::Step::off(120),
    led::Step::on(60),
    led::Step::off(1760),
];
/// Fast blink while the OTA join runs.
static JOINING: led::Pattern = &[led::Step::on(100), led::Step::off(150)];

/// Post-uplink RX window when the gateway's ACK advertises a queued downlink. The
/// gateway transmits ~20 ms after our uplink with one retry (~450 ms worst) — 1 s
/// covers it with margin.
const DOWNLINK_WINDOW: Duration = Duration::from_secs(1);
/// Shell-response chunks ride confirmed uplinks with extra reps: the gateway's
/// single-owner loop may be busy retrying a queued downlink when a chunk goes out.
const CHUNK_REPS: u8 = 5;

fn read_u32_setting(kv: Nv, local: u8, default: u32) -> u32 {
    let mut b = [0u8; 4];
    match kv.scope(NS_SHELL).get_bytes(local, &mut b) {
        Ok(Some(4)) => u32::from_le_bytes(b),
        _ => default,
    }
}

fn read_bool_setting(kv: Nv, local: u8, default: bool) -> bool {
    let mut b = [0u8; 1];
    match kv.scope(NS_SHELL).get_bytes(local, &mut b) {
        Ok(Some(1)) => b[0] != 0,
        _ => default,
    }
}

/// Whether a button event is enabled (reported over radio + LED). Defaults mirror
/// `BTN_SETTINGS`: the gesture events (click/hold) on, the raw edges (press/release) off.
fn event_enabled(kv: Nv, ev: button::Event) -> bool {
    let (key, default) = match ev {
        button::Event::Press => (SET_PRESS, false),
        button::Event::Release => (SET_RELEASE, false),
        button::Event::Click => (SET_CLICK, true),
        button::Event::Hold => (SET_HOLD, true),
    };
    read_bool_setting(kv, key, default)
}

/// Flash the LED for one (already enabled) button event with its fixed shape:
/// press/release/hold a single pulse, a click a double-blink so it reads apart from an
/// adjacent press even on the one LED. Immediate, fire-and-forget — the LED dispatcher
/// owns the timing; the uplink follows independently.
fn led_feedback(led: &led::Led, ev: button::Event) {
    match ev {
        button::Event::Press => led.flash(LED_PRESS_MS),
        button::Event::Release => led.flash(LED_RELEASE_MS),
        button::Event::Click => led.pulse(LED_CLICK_MS, LED_CLICK_MS, 2),
        button::Event::Hold => led.flash(LED_HOLD_MS),
    }
}

/// Build the button recognition config from the persisted timing settings (read once
/// at boot). `scan_interval` stays a fixed internal poll rate.
fn button_config(kv: Nv) -> button::Config {
    button::Config {
        scan_interval: Duration::from_millis(5),
        debounce_press: Duration::from_millis(read_u32_setting(kv, SET_DEBOUNCE_PRESS, 30) as u64),
        debounce_release: Duration::from_millis(read_u32_setting(kv, SET_DEBOUNCE_RELEASE, 30) as u64),
        click_timeout: Duration::from_millis(read_u32_setting(kv, SET_CLICK_TIMEOUT, 500) as u64),
        hold_time: Duration::from_millis(read_u32_setting(kv, SET_HOLD_TIME, 1000) as u64),
    }
}

fn read_accel_setting(kv: Nv) -> Option<Sensitivity> {
    let mut b = [0u8; 8];
    match kv.scope(NS_SHELL).get_bytes(SET_ACCEL, &mut b) {
        Ok(Some(n)) => match &b[..n] {
            b"off" => None,
            b"low" => Some(Sensitivity::Low),
            b"high" => Some(Sensitivity::High),
            _ => Some(Sensitivity::Medium),
        },
        _ => Some(Sensitivity::Medium),
    }
}

/// The shared I²C2 bus, resident with the accelerometer (its tilt IRQ needs register
/// access most often); `measure_temp` borrows it for each one-shot read. `None` only
/// transiently inside a swap — see `board::Board` for the bus-sharing contract.
type AccelBus = Option<Lis2dh12<GuardedI2c>>;

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
    static BTN_CH: button::ButtonChannel = button::ButtonChannel::new();
    let btn = button::init_exti(
        b.spawner,
        b.button,
        button::Polarity::ActiveHigh,
        button_config(b.kv), // debounce / click-timeout / hold-time from settings
        &BTN_CH,
    );
    let mut accel_int = b.accel_int;

    let kv = b.kv;
    let app_kv = kv.scope(NS_APP);

    // This node's radio address = the `address` base setting (pinned or UID-derived).
    let my_id = shell::radio_address(kv);
    let mut provision: Option<Provisioned> = app_kv.get::<Provisioned>(KEY_PROVISION).ok().flatten();

    // Accelerometer: reclaim the shared I²C2 bus and arm the tilt interrupt (register
    // state persists while powered, so this is a boot-time configuration; the `accel`
    // setting applies on the next reboot).
    let accel_sens = read_accel_setting(kv);
    let mut bus = {
        let mut accel = Lis2dh12::new(b.tmp112.release(), lis2dh12::ADDR_DEFAULT);
        if let Some(sens) = accel_sens {
            let _ = accel.init();
            match accel.enable_tilt(TiltConfig::new(sens)) {
                Ok(()) => info!(target: "button", "accel tilt armed"),
                Err(_) => warn!(target: "button", "accel tilt enable failed"),
            }
        }
        Some(accel)
    };

    let radio = Spirit1::new(
        b.radio_spi,
        b.radio_sck,
        b.radio_mosi,
        b.radio_miso,
        b.radio_cs,
        b.radio_sdn,
        b.radio_irq,
    );
    let (band, channel, key) = match provision {
        Some(p) => (
            if p.band == mgmt::BAND_US915 {
                Band::Us915
            } else {
                Band::Eu868
            },
            p.channel,
            p.key,
        ),
        None => (Band::DEFAULT, 0, [0u8; 16]),
    };
    let mut net = match Net::new(
        radio,
        kv,
        NetConfig {
            my_id,
            key,
            band,
            channel,
        },
    )
    .await
    {
        Ok(n) => n,
        Err(e) => {
            log::error!(target: "button", "net init: {e} — radio dead, local mode only");
            return;
        }
    };
    if let Some(p) = provision {
        net.add_peer(p.gw_id, &p.key);
    }
    let _ = net.sleep().await; // radio sleeps between events from the start

    info!(
        target: "button",
        "PUSH-BUTTON {:08X}: {}",
        my_id,
        if provision.is_some() { "paired" } else { "UNPAIRED — hold the button to pair" }
    );
    if provision.is_none() {
        led.set_background(Some(UNPROVISIONED));
    }

    // Per-kind running counts since boot (RAM — a reset is a reboot, disambiguated
    // host-side by the heartbeat's session_id).
    let mut counts = [0u32; 4];
    let mut last_temp_sent: Option<i32> = None;
    let mut last_temp_tx = Instant::now();
    let mut next_temp = Instant::now(); // measure immediately on boot
    let mut next_heartbeat = Instant::now(); // boot Info doubles as the first heartbeat

    loop {
        let timer_due = next_temp.min(next_heartbeat);
        match select4(
            btn.next_event(),
            Timer::at(timer_due),
            accel_int.wait_for_high(),
            console::mgmt_next(),
        )
        .await
        {
            // --- button (real or simulated — same recognition path) ---
            Either4::First(ev) => {
                let kind = match ev {
                    button::Event::Press => ButtonKind::Press,
                    button::Event::Release => ButtonKind::Release,
                    button::Event::Click => ButtonKind::Click,
                    button::Event::Hold => ButtonKind::Hold,
                };
                let idx = kind as usize;
                counts[idx] = counts[idx].wrapping_add(1);

                // Pairing is a fixed gesture, not a reportable event: while unpaired a
                // long hold always starts the OTA join, regardless of the per-event
                // enables (which govern normal reporting once paired).
                if provision.is_none() {
                    if matches!(ev, button::Event::Hold) {
                        provision = ota_join(&mut net, my_id, band, channel, kv, &led).await;
                        if provision.is_some() {
                            led.set_background(None);
                            send_info(&mut net, provision, &counts).await;
                        }
                    }
                    continue; // unpaired: no gateway to report events to
                }

                // The per-event enable is the master switch: a disabled event is
                // ignored entirely — no LED, no radio.
                if !event_enabled(kv, ev) {
                    continue;
                }
                led_feedback(&led, ev); // immediate local feedback (fixed shape)
                // Click/Hold are the semantic events — worth more repetitions than the
                // raw press/release edges around them.
                let reps = match kind {
                    ButtonKind::Click | ButtonKind::Hold => 5,
                    _ => 3,
                };
                let msg = NodeMsg::Button {
                    kind,
                    count: counts[idx],
                };
                uplink(&mut net, provision, &msg, reps).await;
            }

            // --- periodic: temperature and/or heartbeat due ---
            Either4::Second(()) => {
                let now = Instant::now();
                if now >= next_temp {
                    let period = read_u32_setting(kv, SET_TEMP_PERIOD, 60) as u64;
                    next_temp = now + Duration::from_secs(period.max(5));
                    if let Some(millic) = measure_temp(&mut bus).await {
                        let delta_mc = read_u32_setting(kv, SET_TEMP_DELTA, 50) as i32 * 10;
                        let heartbeat = read_u32_setting(kv, SET_HEARTBEAT, 900) as u64;
                        let changed = last_temp_sent.is_none_or(|last| (millic - last).abs() >= delta_mc);
                        let stale = last_temp_tx.elapsed() >= Duration::from_secs(heartbeat);
                        if provision.is_some() && (changed || stale) {
                            let msg = NodeMsg::Temperature { millic };
                            if uplink(&mut net, provision, &msg, 3).await {
                                last_temp_sent = Some(millic);
                                last_temp_tx = Instant::now();
                            }
                        }
                    }
                }
                if now >= next_heartbeat {
                    let heartbeat = read_u32_setting(kv, SET_HEARTBEAT, 900) as u64;
                    next_heartbeat = now + Duration::from_secs(heartbeat.max(60));
                    if provision.is_some() {
                        send_info(&mut net, provision, &counts).await;
                    }
                }
            }

            // --- accelerometer tilt interrupt ---
            Either4::Third(()) => {
                if let Some(face) = handle_tilt(&mut bus).await
                    && provision.is_some()
                {
                    let msg = NodeMsg::Accel {
                        kind: AccelKind::Motion,
                        face,
                    };
                    uplink(&mut net, provision, &msg, 3).await;
                }
            }

            // --- management over the cable (USB present by definition) ---
            Either4::Fourth(frame) => {
                let Ok((MsgType::MgmtRequest, _seq, payload)) = decode_frame(&frame) else {
                    continue;
                };
                let Ok(req) = postcard::from_bytes::<MgmtRequest>(payload) else {
                    continue;
                };
                handle_mgmt(req, &mut net, &mut provision, my_id, band, channel, kv, &led).await;
                if provision.is_some() {
                    led.set_background(None);
                }
            }
        }
    }
}

/// OTA join (button held): run the 3-way JOIN for the standard window, persist the
/// credentials on success.
async fn ota_join(
    net: &mut Net,
    my_id: u32,
    band: Band,
    channel: u8,
    kv: Nv,
    led: &led::Led,
) -> Option<Provisioned> {
    info!(target: "button", "JOIN: looking for a pairing window ({} s)…", PAIRING_WINDOW.as_secs());
    led.set_background(Some(JOINING));
    let _ = net.wake().await;
    let joined = net.join(my_id, PAIRING_WINDOW).await;
    let _ = net.sleep().await;
    match joined {
        Some((gw_id, key)) => {
            let p = Provisioned {
                gw_id,
                key,
                band: if band == Band::Us915 {
                    mgmt::BAND_US915
                } else {
                    mgmt::BAND_EU868
                },
                channel,
            };
            if kv.scope(NS_APP).set(KEY_PROVISION, &p).is_err() {
                warn!(target: "button", "JOINED {:08X} but persist failed — will not survive reboot", gw_id);
            } else {
                info!(target: "button", "JOINED gateway {:08X}", gw_id);
            }
            net.add_peer(gw_id, &key);
            Some(p)
        }
        None => {
            warn!(target: "button", "join failed (no gateway window in range)");
            led.set_background(Some(UNPROVISIONED));
            None
        }
    }
}

/// One-shot temperature read: take the bus from the accelerometer, measure, hand it
/// back (chip register state — the tilt config — persists across the swap).
async fn measure_temp(bus: &mut AccelBus) -> Option<i32> {
    let mut t = Tmp112::new(bus.take()?.release(), tmp112::ADDR_VPLUS);
    let raw = t.oneshot().await;
    let millic = raw.ok().map(tmp112::raw_to_millicelsius);
    *bus = Some(Lis2dh12::new(t.release(), lis2dh12::ADDR_DEFAULT));
    millic
}

/// Tilt IRQ: clear/validate the latched interrupt, settle, and report the face up
/// (0 = unknown/moving).
async fn handle_tilt(bus: &mut AccelBus) -> Option<u8> {
    let Some(accel) = bus else {
        return None; // bus mid-swap (can't happen: swaps are scoped to measure_temp)
    };
    if !accel.tilt_triggered().unwrap_or(false) {
        return None; // spurious edge / rate-limited by the driver's min_interval
    }
    // Let the unit settle, then read which face is up for the event's context.
    Timer::after_millis(300).await;
    let face = accel.read().ok().and_then(|s| s.dice()).unwrap_or(0);
    Some(face)
}

/// Send the identity/heartbeat `Info` (also the delivery opportunity for queued
/// downlinks on an otherwise idle node).
async fn send_info(net: &mut Net, provision: Option<Provisioned>, _counts: &[u32; 4]) {
    let name = console::firmware_name();
    let msg = NodeMsg::Info(NodeInfo {
        firmware_name: name.as_str(),
        firmware_version: console::firmware_version(),
        session_id: console::session_id(),
        sleeping: true,
        battery_mv: None, // reserved until the SDK grows an ADC block
    });
    uplink(net, provision, &msg, 3).await;
}

/// One uplink cycle: wake the radio, send confirmed, honour the ACK's pending flag
/// (downlink window + remote shell), then sleep the radio again.
async fn uplink(net: &mut Net, provision: Option<Provisioned>, msg: &NodeMsg<'_>, reps: u8) -> bool {
    let Some(p) = provision else {
        return false;
    };
    let mut buf = [0u8; MAX_RADIO_PAYLOAD];
    let Ok(n) = encode_node_msg(msg, &mut buf) else {
        warn!(target: "button", "uplink encode failed");
        return false;
    };
    let _ = net.wake().await;
    let result = net.send(p.gw_id, &buf[..n], true, reps).await;
    let delivered = matches!(result, SendResult::Delivered);
    if !delivered {
        warn!(target: "button", "uplink: {result}");
    }
    while net.last_ack().is_some_and(|m| m.pending) {
        if !downlink_window(net, p.gw_id).await {
            break;
        }
        // The last response chunk's ACK re-advertises pending, chaining queued
        // commands one per window without any polling.
    }
    let _ = net.sleep().await;
    delivered
}

/// Hold RX open for one queued downlink; execute it and stream the response. Returns
/// whether a downlink actually arrived (false = window expired, stop chaining).
async fn downlink_window(net: &mut Net, gw_id: u32) -> bool {
    let deadline = Instant::now() + DOWNLINK_WINDOW;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.as_ticks() == 0 {
            return false;
        }
        let Some(rx) = net.recv(remaining).await else {
            return false;
        };
        if rx.src != gw_id {
            continue; // another node's traffic — not our downlink
        }
        let (cmd_id, result, reboot, out) = match decode_node_cmd(rx.data()) {
            Ok(NodeCmd::Shell { cmd_id, line }) => {
                info!(target: "button", "remote shell: {}", line);
                let mut out: heapless::String<{ shell::RESP_CAP }> = heapless::String::new();
                let (result, reboot) = shell::run_line(line, &mut out);
                (cmd_id, result, reboot, out)
            }
            Err(tower_protocol::Error::BadVersion { got }) => {
                warn!(
                    target: "button",
                    "downlink schema v{} unsupported (this build: v{})", got, RADIO_SCHEMA_VERSION
                );
                return true;
            }
            Err(_) => {
                warn!(target: "button", "downlink malformed — dropped");
                return true;
            }
        };
        send_shell_chunks(net, gw_id, cmd_id, result, out.as_str()).await;
        if reboot {
            // The response is on the air; the reboot request came over radio, so there
            // is no console to flush — reset now.
            cortex_m::peripheral::SCB::sys_reset();
        }
        return true;
    }
}

/// Stream a remote-shell response as `NodeShellChunk` uplinks (≤ RADIO_SHELL_CHUNK
/// text bytes each, split at char boundaries; empty responses still send one final
/// chunk so the host always completes the command).
async fn send_shell_chunks(net: &mut Net, gw_id: u32, cmd_id: u16, result: u8, text: &str) {
    let mut rest = text;
    let mut chunk: u16 = 0;
    loop {
        let mut take = rest.len().min(RADIO_SHELL_CHUNK);
        while take > 0 && !rest.is_char_boundary(take) {
            take -= 1;
        }
        let (head, tail) = rest.split_at(take);
        let last = tail.is_empty();
        let msg = NodeMsg::Shell(NodeShellChunk {
            cmd_id,
            result,
            chunk,
            last,
            text: head,
        });
        let mut buf = [0u8; MAX_RADIO_PAYLOAD];
        let Ok(n) = encode_node_msg(&msg, &mut buf) else {
            return;
        };
        if !matches!(
            net.send(gw_id, &buf[..n], true, CHUNK_REPS).await,
            SendResult::Delivered
        ) {
            // A lost chunk shows up host-side as a chunk gap (incomplete response);
            // aborting beats streaming the tail of a response the host can't anchor.
            warn!(target: "button", "response chunk {} undelivered — aborting", chunk);
            return;
        }
        if last {
            return;
        }
        rest = tail;
        chunk = chunk.wrapping_add(1);
    }
}

/// Cable-side management: `Describe` / `Provision` / `JoinOpen` (USB present, since
/// the console is USB-gated — exactly the cable-pairing situation).
#[allow(clippy::too_many_arguments)]
async fn handle_mgmt(
    req: MgmtRequest<'_>,
    net: &mut Net,
    provision: &mut Option<Provisioned>,
    my_id: u32,
    band: Band,
    channel: u8,
    kv: Nv,
    led: &led::Led,
) {
    let req_id = req.req_id;
    match req.op {
        MgmtOp::Describe => {
            let name = console::firmware_name();
            respond_record(
                req_id,
                &DeviceInfo {
                    role: DeviceRole::Node,
                    radio_schema_version: RADIO_SCHEMA_VERSION,
                    net_id: my_id,
                    band: match band {
                        Band::Eu868 => mgmt::BAND_EU868,
                        Band::Us915 => mgmt::BAND_US915,
                    },
                    channel,
                    node_capacity: 0,
                    node_count: 0,
                    provisioned: provision.is_some(),
                    gw_id: provision.map(|p| p.gw_id).unwrap_or(0),
                    firmware_name: name.as_str(),
                },
            )
            .await;
        }
        MgmtOp::Provision(p) => {
            if p.gw_id == 0 || p.band > mgmt::BAND_US915 {
                respond_empty(req_id, mgmt::MGMT_BAD_ARG).await;
                return;
            }
            let app_kv = kv.scope(NS_APP);
            let rec = Provisioned {
                gw_id: p.gw_id,
                key: p.key,
                band: p.band,
                channel: p.channel,
            };
            if app_kv.set(KEY_PROVISION, &rec).is_err() {
                respond_empty(req_id, mgmt::MGMT_STORAGE).await;
                return;
            }
            // An optional address override pins the `address` base setting (the same
            // one `system address` edits), so the node comes up under it after reboot.
            let effective_id = match p.my_id {
                Some(id) if id != 0 => {
                    if kv
                        .scope(NS_SHELL)
                        .set_bytes(shell::ADDRESS_KEY, &id.to_le_bytes())
                        .is_err()
                    {
                        respond_empty(req_id, mgmt::MGMT_STORAGE).await;
                        return;
                    }
                    id
                }
                _ => my_id,
            };
            respond_record(req_id, &ProvisionAck { id: effective_id }).await;
            info!(target: "button", "provisioned for gateway {:08X} — rebooting", p.gw_id);
            // Band/channel/id take effect at Net::new — reboot into the new identity
            // (also bumps session_id, so the host sees the reprovision).
            console::flush().await;
            cortex_m::peripheral::SCB::sys_reset();
        }
        MgmtOp::JoinOpen { window_s } => {
            if window_s == 0 {
                respond_empty(req_id, mgmt::MGMT_BAD_ARG).await;
                return;
            }
            led.set_background(Some(JOINING));
            let _ = net.wake().await;
            let joined = net.join(my_id, Duration::from_secs(window_s as u64)).await;
            let _ = net.sleep().await;
            match joined {
                Some((gw_id, key)) => {
                    let rec = Provisioned {
                        gw_id,
                        key,
                        band: if band == Band::Us915 {
                            mgmt::BAND_US915
                        } else {
                            mgmt::BAND_EU868
                        },
                        channel,
                    };
                    if kv.scope(NS_APP).set(KEY_PROVISION, &rec).is_err() {
                        respond_empty(req_id, mgmt::MGMT_STORAGE).await;
                        return;
                    }
                    net.add_peer(gw_id, &key);
                    *provision = Some(rec);
                    respond_record(req_id, &Joined { gw_id }).await;
                }
                None => {
                    led.set_background(if provision.is_some() {
                        None
                    } else {
                        Some(UNPROVISIONED)
                    });
                    respond_empty(req_id, mgmt::MGMT_TIMEOUT).await;
                }
            }
        }
        // Gateway-side ops: this device is a node.
        _ => respond_empty(req_id, mgmt::MGMT_UNSUPPORTED).await,
    }
}

async fn respond_empty(req_id: u16, result: u8) {
    console::mgmt_chunk(req_id, result, 0, true, &[]).await;
}

async fn respond_record<T: serde::Serialize>(req_id: u16, record: &T) {
    let mut buf = [0u8; 192];
    match postcard::to_slice(record, &mut buf) {
        Ok(bytes) => {
            let n = bytes.len();
            console::mgmt_chunk(req_id, mgmt::MGMT_OK, 0, true, &buf[..n]).await;
        }
        Err(_) => respond_empty(req_id, mgmt::MGMT_STORAGE).await,
    }
}

app!(run, commands: BTN_COMMANDS, settings: BTN_SETTINGS);
