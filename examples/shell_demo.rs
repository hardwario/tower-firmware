//! shell_demo — the RouterOS-style shell: SDK built-ins + settings, plus app commands
//! merged into the tree at every level and app settings of every kind.
//!
//! Drive with `tower shell` (TAB completes, incl. enum/bool values after `=`) or
//! `tower exec "<line>"`:
//!   /system settings print
//!   /system settings set identity=node-7         (Str)
//!   /system settings set interval=60             (Uint, range 1..=3600)
//!   /system settings set mode=mesh               (Enum: p2p|star|mesh)
//!   /system settings set tx_power=-10            (Int, range -30..=20)
//!   /system settings get tx_power                (shows value + constraints)
//!   /system hello                                (app command merged into /system)
//!   /radio status        /radio test ping        (app's own nested subtree)
//!   /export

#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_time::{Instant, Timer};
use log::info;
use tower::shell::{self, Args, Ctx, Entry, Kind, Outcome, Setting};
use tower::{app, board::Board};

/// App command `/uptime` (top-level).
fn cmd_uptime(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    let s = Instant::now().as_micros() / 1_000_000;
    let _ = write!(ctx, "up {} h {} m {} s", s / 3600, (s % 3600) / 60, s % 60);
    Outcome::ok()
}

/// App command merged **into the SDK's `/system` menu** → `/system hello`.
fn cmd_hello(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    let _ = write!(ctx, "hello — an app command living under the SDK's /system menu");
    Outcome::ok()
}

/// App command in the app's **own nested subtree** → `/radio status`.
fn cmd_radio_status(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    let _ = write!(ctx, "radio: idle, last RSSI -71 dBm");
    Outcome::ok()
}

/// Deeper still → `/radio test ping`.
fn cmd_radio_ping(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    let _ = write!(ctx, "ping → pong (loopback)");
    Outcome::ok()
}

/// App commands. They deep-merge with the SDK tree: `system` joins the SDK's `/system`
/// menu; `radio` is a brand-new top-level subtree.
static APP_COMMANDS: &[Entry] = &[
    Entry::cmd("uptime", Args::None, cmd_uptime),
    Entry::menu("system", &[Entry::cmd("hello", Args::None, cmd_hello)]),
    Entry::menu(
        "radio",
        &[
            Entry::cmd("status", Args::None, cmd_radio_status),
            Entry::menu("test", &[Entry::cmd("ping", Args::None, cmd_radio_ping)]),
        ],
    ),
];

/// App settings — one of each kind. Keys are above the console base (`0x5500`);
/// `identity` (SDK) stays at `0x5500`.
static APP_SETTINGS: &[Setting] = &[
    Setting {
        key: 0x5510,
        name: "interval",
        kind: Kind::Uint { min: 1, max: 3600 },
        default: "30",
    },
    Setting {
        key: 0x5511,
        name: "verbose",
        kind: Kind::Bool,
        default: "false",
    },
    Setting {
        key: 0x5512,
        name: "mode",
        kind: Kind::Enum(&["p2p", "star", "mesh"]),
        default: "star",
    },
    Setting {
        key: 0x5513,
        name: "tx_power",
        kind: Kind::Int { min: -30, max: 20 },
        default: "14",
    },
];

async fn run(b: Board) {
    // Hand the EEPROM to the shell with the app's commands + settings, then spawn it.
    shell::serve_ext(b.spawner, b.storage, APP_COMMANDS, APP_SETTINGS);
    info!("shell ready — try `/system settings print` via `tower shell`");

    // Logs keep flowing alongside the shell on the same framed link.
    let mut n: u32 = 0;
    loop {
        info!("heartbeat {}", n);
        n = n.wrapping_add(1);
        Timer::after_secs(5).await;
    }
}

app!(run);
