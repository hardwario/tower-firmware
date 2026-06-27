//! shell_demo — the RouterOS-style shell: SDK built-ins + settings, plus an
//! app-defined command and app-defined settings (the two extensibility points).
//!
//! Drive it with `tower shell` (TAB completes) or `tower exec "<line>"`:
//!   /system settings print
//!   /system settings set identity=tower-01     (Str)
//!   /system settings set interval=60           (U32, app setting)
//!   /system settings set verbose=on            (Bool, app setting)
//!   /system settings get interval
//!   /uptime                                    (app command)
//!   /export
//!   /system reboot

#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_time::{Instant, Timer};
use log::info;
use tower::shell::{self, Args, Ctx, Entry, Kind, Outcome, Setting};
use tower::{app, board::Board};

/// App command `/uptime` — shows how an app adds its own command + handler.
fn cmd_uptime(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    let s = Instant::now().as_micros() / 1_000_000;
    let _ = write!(ctx, "up {} h {} m {} s", s / 3600, (s % 3600) / 60, s % 60);
    Outcome::ok()
}

/// App top-level commands, merged into the SDK tree (completes at the root).
static APP_COMMANDS: &[Entry] = &[Entry::cmd("uptime", Args::None, cmd_uptime)];

/// App settings, reachable via `/system settings` — one of each kind. Keys are above
/// the console base (`0x5500`); `identity` (SDK) stays at `0x5500`.
static APP_SETTINGS: &[Setting] = &[
    Setting { key: 0x5510, name: "interval", kind: Kind::U32, default: "30" },
    Setting { key: 0x5511, name: "verbose", kind: Kind::Bool, default: "false" },
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
