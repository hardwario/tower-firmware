//! RouterOS-style shell over the framed console, with **target-authoritative TAB
//! completion**, a **declarative settings framework**, and an **app-extensible
//! command tree**.
//!
//! Opt-in: an app calls [`serve`] (base only) or [`serve_ext`] (with its own commands +
//! settings), which registers the command tree + settings + KV in a static and spawns a small
//! **shell drain task**. The console [`manager`](crate::console::manager) owns the RX half and
//! copies each decoded shell frame into its [`SHELL_RX`](crate::console::SHELL_RX) channel; the
//! drain task dispatches from there (so the console depends only on `tower-protocol`, never
//! calling up into the shell). It handles two request types against one command tree:
//!   * `ShellCommand` → walk the tree → run the command's handler → `ShellResponse`;
//!   * `ShellComplete` → walk the tree **to the cursor** → `ShellCompletions`.
//!
//! Because dispatch runs on its own task (not inline on the RX loop), a slow handler no longer
//! stalls RX draining — but handlers should still be short, and their
//! `ShellResponse`s go through the bounded `TX_CHANNEL`. Not a place for long-running work.
//!
//! Both use the same tokenizer + [`resolve`] walk, so completion can never suggest
//! something execution won't accept.
//!
//! ## Extending it (apps)
//! App commands merge into the SDK tree **at every level** — drop a command into an
//! existing menu (`/system/hello`) or grow your own nested subtree (`/radio/test/ping`).
//! App settings join the same `/system settings` table and complete their values.
//! ```ignore
//! fn cmd_hi(ctx: &mut shell::Ctx, _args: &[&str]) -> shell::Outcome {
//!     let _ = write!(ctx, "hello");        // Ctx: core::fmt::Write
//!     shell::Outcome::ok()
//! }
//! use shell::{Args, Entry, Kind, Setting};
//! static CMDS: &[Entry] = &[
//!     Entry::menu("system", &[Entry::cmd("hello", Args::None, cmd_hi)]), // into /system
//!     Entry::menu("radio", &[Entry::cmd("status", Args::None, cmd_hi)]), // new subtree
//! ];
//! static SETS: &[Setting] = &[
//!     Setting { key: 0x10, name: "interval", kind: Kind::Uint { min: 1, max: 3600 }, default: "30" },
//!     Setting { key: 0x11, name: "mode", kind: Kind::Enum(&["p2p", "star", "mesh"]), default: "star" },
//! ];
//! app!(run, commands: CMDS, settings: SETS); // or: shell::serve_ext(b.spawner, b.kv, CMDS, SETS)
//! ```

use core::fmt::{self, Write};

use embassy_executor::Spawner;
use embassy_time::Instant;
use heapless::{String, Vec};
use tower_protocol::msg::{Candidate, CandidateKind, ShellCommand, ShellComplete, ShellCompletions};
use tower_protocol::{MsgType, PROTOCOL_VERSION, decode_frame};

use crate::console;
use crate::storage::{FLIP_BUDGET, NS_SHELL, Nv, Scoped};

const MAX_LINE: usize = 96;
/// Max whitespace/`/`-separated tokens a command line may hold. An over-long line is rejected
/// rather than having its tail silently dropped (see [`tokenize`] and the `ShellCommand` handler).
const MAX_TOKENS: usize = 8;
/// Shell-response build buffer. Must stay equal to `console::MAX_RESP` (the transport's
/// per-message cap): `console::shell_response` re-clips to that, so if this grew larger the
/// excess would be silently dropped there.
const RESP_CAP: usize = 256;
/// Largest value (bytes) a setting can hold (and a `Str` setting's `max`).
pub const MAX_SETTING: usize = 64;

/// Result codes (0 = success).
pub const R_OK: u8 = 0;
pub const R_NOT_FOUND: u8 = 1;
pub const R_BAD_ARG: u8 = 2;
pub const R_STORAGE: u8 = 3;
/// The response exceeded the buffer and was truncated — the body is incomplete. Set by the
/// dispatcher (not a handler) when [`Ctx`] overflowed, so a caller doesn't mistake a cut-off
/// `/export` / `settings print` for a complete, successful one.
pub const R_TRUNCATED: u8 = 4;

// ---- declarative settings ---------------------------------------------------

/// How a [`Setting`]'s value is validated, encoded in EEPROM, and parsed / printed.
#[derive(Clone, Copy)]
pub enum Kind {
    /// UTF-8 text, 1..=`max` bytes (`max` is clamped to [`MAX_SETTING`]).
    Str { max: u16 },
    /// Unsigned integer constrained to `min..=max` (decimal in; 4 LE bytes stored).
    /// Use `0..=u32::MAX` for "unbounded". For intervals, ports, counts, thresholds.
    Uint { min: u32, max: u32 },
    /// Signed integer constrained to `min..=max`. For offsets, tx-power dBm, calibration.
    Int { min: i32, max: i32 },
    /// Boolean: accepts `true`/`false`, `on`/`off`, `1`/`0`; stored as one byte.
    Bool,
    /// One of a fixed set of string values (stored verbatim; the choices complete after
    /// `=`). For modes, regions, roles.
    Enum(&'static [&'static str]),
}

/// A persisted, named setting. The shell derives `/system settings print|set|get`
/// and `/export` from the table — no per-setting code.
pub struct Setting {
    /// Local key within the shell namespace (`NS_SHELL`); the shell prefixes it, so a setting can
    /// never collide with another subsystem's keys. App settings pick any free local (the base
    /// `identity` setting uses `0x00`).
    pub key: u8,
    /// Name used on the command line and in `print`/`export`.
    pub name: &'static str,
    /// Value type (drives validation + formatting).
    pub kind: Kind,
    /// Shown by `get`/`print` when the key has never been set.
    pub default: &'static str,
}

/// SDK base settings; apps add their own via [`serve_ext`].
static BASE_SETTINGS: &[Setting] = &[Setting {
    key: 0x00,
    name: "identity",
    kind: Kind::Str { max: 32 },
    default: "tower",
}];

/// The merged settings table (SDK base + the app's) handed to handlers + completion.
#[derive(Clone, Copy)]
pub struct SettingsTable {
    base: &'static [Setting],
    app: &'static [Setting],
}

impl SettingsTable {
    /// Iterate all settings (base then app).
    pub fn iter(&self) -> impl Iterator<Item = &'static Setting> {
        self.base.iter().chain(self.app)
    }
    /// Find a setting by name.
    pub fn find(&self, name: &str) -> Option<&'static Setting> {
        self.iter().find(|s| s.name == name)
    }
}

// ---- command tree (extensible; handlers are plain fn pointers) --------------

/// A command handler: write the response with `write!(ctx, …)`, return an [`Outcome`].
pub type Handler = fn(&mut Ctx<'_>, &[&str]) -> Outcome;

/// What completing a command's first argument offers (anti-divergence: the same
/// data the handler will accept).
#[derive(Clone, Copy)]
pub enum Args {
    /// No completable arguments.
    None,
    /// A fixed list of argument names (host appends `=`).
    Names(&'static [&'static str]),
    /// The names of settings (for `set`/`get`); `assign` → host appends `=`.
    Settings { assign: bool },
}

/// A command's result code plus an optional reboot request (the framework flushes
/// the response, then resets).
pub struct Outcome {
    pub result: u8,
    pub reboot: bool,
}

impl Outcome {
    /// Success, no reboot.
    pub fn ok() -> Self {
        Self {
            result: R_OK,
            reboot: false,
        }
    }
    /// A non-zero result code, no reboot.
    pub fn code(result: u8) -> Self {
        Self {
            result,
            reboot: false,
        }
    }
}

/// Execution context for a command handler. Write the response via `write!(ctx, …)`
/// (Ctx is [`core::fmt::Write`]); persistent state is `ctx.kv` / `ctx.settings`.
pub struct Ctx<'a> {
    /// The shell's namespace-scoped EEPROM handle (`NS_SHELL`); settings are keyed by `u8` local.
    pub kv: Scoped,
    /// The merged settings table (SDK base + app).
    pub settings: SettingsTable,
    out: &'a mut String<RESP_CAP>,
    /// Set when a write hit the `RESP_CAP` ceiling and the response body was truncated. The
    /// dispatcher turns this into [`R_TRUNCATED`] so a scripting caller sees the output is
    /// incomplete rather than a silent, R_OK-looking partial (e.g. an `/export` cut mid-setting).
    overflowed: bool,
}

impl Write for Ctx<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        // Truncate at the response cap instead of failing the whole write.
        let mut end = s.len().min(RESP_CAP - self.out.len());
        while end > 0 && !s.is_char_boundary(end) {
            end -= 1;
        }
        if end < s.len() {
            self.overflowed = true; // couldn't fit all of `s` — body is now truncated
        }
        let _ = self.out.push_str(&s[..end]);
        Ok(())
    }
}

/// A node in the command tree: a menu (children) or a command (handler).
pub enum Entry {
    Menu(&'static str, &'static [Entry]),
    Cmd(Command),
}

/// A shell command: its name, what its first arg completes to, and its handler.
pub struct Command {
    pub name: &'static str,
    pub args: Args,
    pub run: Handler,
}

impl Entry {
    fn name(&self) -> &'static str {
        match self {
            Entry::Menu(n, _) => n,
            Entry::Cmd(c) => c.name,
        }
    }
    /// Build a command entry (for app trees).
    pub const fn cmd(name: &'static str, args: Args, run: Handler) -> Self {
        Entry::Cmd(Command { name, args, run })
    }
    /// Build a menu entry (for app trees).
    pub const fn menu(name: &'static str, children: &'static [Entry]) -> Self {
        Entry::Menu(name, children)
    }
}

/// SDK base command tree (apps append their own top-level entries via [`serve_ext`]).
static BASE_ROOT: &[Entry] = &[
    Entry::Menu(
        "system",
        &[
            Entry::cmd("reboot", Args::None, cmd_reboot),
            Entry::Menu("resource", &[Entry::cmd("print", Args::None, cmd_resource)]),
            Entry::Menu("eeprom", &[Entry::cmd("print", Args::None, cmd_eeprom)]),
            Entry::Menu("crash", &[Entry::cmd("print", Args::None, cmd_crash)]),
            Entry::Menu(
                "settings",
                &[
                    Entry::cmd("print", Args::None, settings_print),
                    Entry::cmd("set", Args::Settings { assign: true }, settings_set),
                    Entry::cmd("get", Args::Settings { assign: false }, settings_get),
                ],
            ),
        ],
    ),
    Entry::cmd("export", Args::None, cmd_export),
];

// ---- tree walk (shared by dispatch + completion) ----------------------------

/// Tokenize on `/` and whitespace (both separate path/command tokens). Capped at [`MAX_TOKENS`];
/// the `ShellCommand` handler rejects a line with more tokens rather than reaching this cap.
fn tokenize(line: &str) -> Vec<&str, MAX_TOKENS> {
    let mut v = Vec::new();
    for t in line.split(['/', ' ', '\t']).filter(|s| !s.is_empty()) {
        let _ = v.push(t);
    }
    v
}

/// Outcome of walking the tree with a token slice. `Menu` carries the two slices that
/// make up the current level (SDK base + the app's same-path entries) so completion
/// can list the merged children; both deeper menus and the root merge the same way.
enum Resolved {
    Menu(&'static [Entry], &'static [Entry]),
    /// Reached a command, plus the index of its first argument token.
    Cmd(&'static Command, usize),
    /// A token matched nothing at its level.
    NoMatch,
}

/// Children of the menu named `name` in `level`, if any.
fn menu_children(level: &'static [Entry], name: &str) -> Option<&'static [Entry]> {
    level.iter().find_map(|e| match e {
        Entry::Menu(n, ch) if *n == name => Some(*ch),
        _ => None,
    })
}

/// The command named `name` across both levels (base searched first).
fn find_cmd<'a>(primary: &'a [Entry], secondary: &'a [Entry], name: &str) -> Option<&'a Command> {
    primary.iter().chain(secondary).find_map(|e| match e {
        Entry::Cmd(c) if c.name == name => Some(c),
        _ => None,
    })
}

/// Walk the tree consuming `toks`, **deep-merging** the SDK base tree with the app's at
/// every level: a token names a menu present in either side (descend into the union of
/// their children) or a command. A menu shadows a same-named command (so menus stay
/// descendable); base is searched before app on a command-name collision.
fn resolve(toks: &[&str], app: &'static [Entry]) -> Resolved {
    let mut primary: &'static [Entry] = BASE_ROOT;
    let mut secondary: &'static [Entry] = app;
    let mut i = 0;
    while i < toks.len() {
        let pc = menu_children(primary, toks[i]);
        let sc = menu_children(secondary, toks[i]);
        if pc.is_some() || sc.is_some() {
            primary = pc.unwrap_or(&[]);
            secondary = sc.unwrap_or(&[]);
            i += 1;
            continue;
        }
        return match find_cmd(primary, secondary, toks[i]) {
            Some(c) => Resolved::Cmd(c, i + 1),
            None => Resolved::NoMatch,
        };
    }
    Resolved::Menu(primary, secondary)
}

// ---- task / dispatch --------------------------------------------------------

/// Shell parameters registered by [`serve_ext`] and consumed by the console's RX router
/// (the [`manager`](crate::console::manager)) via [`dispatch_frame`]. `None` until a
/// shell is served — an app using `no_shell` leaves it unset, so incoming shell frames
/// are simply ignored. `Nv` and the slices are `Copy`, so a `Cell` suffices.
// The tuple-in-Cell-in-Mutex type reads as "complex" to clippy but is a plain shared-state cell;
// suppress the lint here (annotation only — no logic change) so the CI clippy `-D warnings` gate
// is green over intentional shared state.
#[allow(clippy::type_complexity)]
static SHELL_PARAMS: critical_section::Mutex<
    core::cell::Cell<Option<(Nv, &'static [Entry], &'static [Setting])>>,
> = critical_section::Mutex::new(core::cell::Cell::new(None));

/// Set once the shell drain task has been spawned, so repeated [`serve`]/[`serve_ext`] calls
/// don't try to spawn it twice (which would exhaust the task's pool of 1). A critical-section
/// cell rather than an atomic swap — Cortex-M0+ has no atomic read-modify-write.
static SHELL_SPAWNED: critical_section::Mutex<core::cell::Cell<bool>> =
    critical_section::Mutex::new(core::cell::Cell::new(false));

/// Drain the console-owned [`SHELL_RX`](crate::console::SHELL_RX) channel and dispatch each
/// shell frame. Running on its own task (rather than inline on the console RX loop) is what lets
/// the console depend only on `tower-protocol` — it copies frames into the channel and never
/// calls into the shell — and it keeps a slow handler from stalling RX draining.
#[embassy_executor::task]
async fn shell_rx_task() {
    loop {
        let frame = crate::console::SHELL_RX.receive().await;
        dispatch_frame(&frame).await;
    }
}

/// Serve the shell with only the SDK base tree + settings.
pub fn serve(spawner: Spawner, kv: Nv) {
    serve_ext(spawner, kv, &[], &[]);
}

/// Register the shell and spawn its drain task: the (dynamic, USB-gated) console routes
/// `ShellCommand`/`ShellComplete` frames into [`SHELL_RX`](crate::console::SHELL_RX), and
/// [`shell_rx_task`] dispatches them here. `app_commands`/`app_settings` add an app command tree
/// / settings (pass `&[]` for none). The task is spawned at most once across repeated calls.
pub fn serve_ext(spawner: Spawner, kv: Nv, app_commands: &'static [Entry], app_settings: &'static [Setting]) {
    critical_section::with(|cs| {
        SHELL_PARAMS
            .borrow(cs)
            .set(Some((kv, app_commands, app_settings)));
    });
    let first = critical_section::with(|cs| {
        let c = SHELL_SPAWNED.borrow(cs);
        let was = c.get();
        c.set(true);
        !was
    });
    if first {
        // Spawn once; the guard above guarantees the pool (size 1) is free, so this can't fail.
        spawner.spawn(shell_rx_task().unwrap());
    }
}

/// Handle one decoded shell RX frame — called by [`shell_rx_task`]. No-op if no shell was
/// served (`no_shell` apps leave [`SHELL_PARAMS`] unset).
async fn dispatch_frame(inner: &[u8]) {
    let Some((kv, app_commands, app_settings)) = critical_section::with(|cs| SHELL_PARAMS.borrow(cs).get())
    else {
        return;
    };
    let settings = SettingsTable {
        base: BASE_SETTINGS,
        app: app_settings,
    };
    handle(kv.scope(NS_SHELL), app_commands, settings, inner).await;
}

/// Decode a complete frame and act on `ShellCommand` / `ShellComplete`.
async fn handle(kv: Scoped, app: &'static [Entry], settings: SettingsTable, inner: &[u8]) {
    let Ok((mt, _seq, payload)) = decode_frame(inner) else {
        return;
    };
    match mt {
        MsgType::ShellCommand => {
            if let Ok(cmd) = postcard::from_bytes::<ShellCommand>(payload) {
                // Reject an over-long line instead of silently executing a truncated prefix. The
                // wire allows ~240-byte lines but the shell buffer is MAX_LINE; clipping mid-value
                // could store a truncated setting and still report success (silent corruption).
                if cmd.line.len() > MAX_LINE {
                    console::shell_response(cmd.cmd_id, R_BAD_ARG, "line too long").await;
                    return;
                }
                // Likewise reject more tokens than `tokenize` holds, rather than dropping the tail.
                if cmd.line.split(['/', ' ', '\t']).filter(|s| !s.is_empty()).count() > MAX_TOKENS {
                    console::shell_response(cmd.cmd_id, R_BAD_ARG, "too many arguments").await;
                    return;
                }
                let mut line = String::<MAX_LINE>::new();
                let _ = line.push_str(cmd.line); // fits: len <= MAX_LINE (checked above)
                dispatch(kv, app, settings, cmd.cmd_id, line.as_str()).await;
            }
        }
        MsgType::ShellComplete => {
            if let Ok(req) = postcard::from_bytes::<ShellComplete>(payload) {
                // `complete` returns only 'static data (tree + settings names), so the
                // frame borrow can end before we await.
                let comp = complete(app, settings, req.req_id, req.line, req.cursor);
                console::shell_completions(comp).await;
            }
        }
        _ => {}
    }
}

async fn dispatch(kv: Scoped, app: &'static [Entry], settings: SettingsTable, cmd_id: u16, line: &str) {
    let toks = tokenize(line);
    match resolve(&toks, app) {
        Resolved::Cmd(cmd, arg_start) => {
            let mut out = String::<RESP_CAP>::new();
            let (outcome, truncated) = {
                let mut ctx = Ctx {
                    kv,
                    settings,
                    out: &mut out,
                    overflowed: false,
                };
                let outcome = (cmd.run)(&mut ctx, &toks[arg_start..]);
                (outcome, ctx.overflowed)
            };
            // A response that overflowed the buffer would otherwise report R_OK on a silently
            // truncated body; surface it as R_TRUNCATED so a scripting caller (`tower exec`) can
            // tell the output is incomplete. Don't mask a handler's own non-zero result.
            let result = if truncated && outcome.result == R_OK {
                R_TRUNCATED
            } else {
                outcome.result
            };
            console::shell_response(cmd_id, result, out.as_str()).await;
            if outcome.reboot {
                // Wait for the response to actually leave the wire before resetting, instead of a
                // fixed sleep shorter than a full TX queue at 115200 baud (which truncated the
                // reboot response). This runs on the console RX task, so a USB unplug within the
                // window cancels it (the manager tears the console down) and the reset is skipped
                // — acceptable: a reboot issued then immediately unplugged is a no-op, and the
                // node reboots on demand only while a host is attached.
                console::flush().await;
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
        _ => console::shell_response(cmd_id, R_NOT_FOUND, "no such command").await,
    }
}

// ---- built-in command handlers ----------------------------------------------

fn cmd_reboot(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    let _ = write!(ctx, "rebooting");
    Outcome {
        result: R_OK,
        reboot: true,
    }
}

fn cmd_resource(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    let us = Instant::now().as_micros();
    // "firmware:" reports the app/example NAME + version and the per-boot session id — the exact
    // fields the boot `Hello` carries and `tower` prints on connect (they must agree). The name
    // is per-app; the version is the SDK crate version; the session bumps once per boot. Multi-line
    // summary (spans more than one wire frame — exercises chunking).
    // Lines kept terse so the whole summary fits the shell response buffer (`RESP_CAP` = 256 B)
    // even at worst case: a 32-char firmware name + 10-digit session + large uptime.
    let _ = write!(
        ctx,
        "firmware: {} {}\r\n\
         session: {}\r\n\
         protocol: v{}\r\n\
         uptime: {}.{:03} s\r\n\
         cpu: STM32L083CZ @ 16 MHz, LSE RTC\r\n\
         memory: 192K flash / 20K RAM / 6K EEPROM\r\n\
         console: USART1 115200 8N1 framed\r\n",
        console::firmware_name(),
        console::firmware_version(),
        console::session_id(),
        PROTOCOL_VERSION,
        us / 1_000_000,
        (us % 1_000_000) / 1000,
    );
    Outcome::ok()
}

fn cmd_eeprom(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    // Wear gauge: the KV compaction-flip count (the persisted superblock generation — a pure read,
    // no added wear) vs the conservative flip budget (FLIP_BUDGET). See docs/storage.md.
    let flips = ctx.kv.raw().flip_generation();
    // Per-mille of budget in integer math (no FPU on the M0+): rendered as X.X%.
    let permille = ((flips as u64) * 1000 / FLIP_BUDGET as u64) as u32;
    let _ = write!(
        ctx,
        "eeprom: 6 KiB data EEPROM\r\n\
         flips: {} / {} ({}.{}%)\r\n\
         resets: {}\r\n",
        flips,
        FLIP_BUDGET,
        permille / 10,
        permille % 10,
        crate::bootguard::consecutive_resets(),
    );
    Outcome::ok()
}

fn cmd_crash(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    // The crash the reset-surviving breadcrumb recovered at THIS boot (cleared by the read, so
    // it reflects the most recent fault since power-on — a brown-out that lost RAM reads clean).
    match crate::crashlog::last() {
        Some(c) => {
            let _ = write!(ctx, "last crash: {}\r\n{}\r\n", c.kind.as_str(), c.message());
        }
        None => {
            let _ = write!(ctx, "no crash recorded since power-on\r\n");
        }
    }
    Outcome::ok()
}

fn cmd_export(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    let table = ctx.settings;
    for s in table.iter() {
        let mut val = String::<MAX_SETTING>::new();
        read_value(ctx.kv, s, &mut val);
        let _ = write!(ctx, "/system settings set {}={}\r\n", s.name, val);
    }
    Outcome::ok()
}

// ---- derived settings commands ----------------------------------------------

fn settings_print(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    let table = ctx.settings;
    for s in table.iter() {
        let mut val = String::<MAX_SETTING>::new();
        read_value(ctx.kv, s, &mut val);
        let _ = write!(ctx, "{} = {}\r\n", s.name, val);
    }
    Outcome::ok()
}

fn settings_get(ctx: &mut Ctx<'_>, args: &[&str]) -> Outcome {
    let Some(&name) = args.first() else {
        let _ = write!(ctx, "usage: get <name>");
        return Outcome::code(R_BAD_ARG);
    };
    let table = ctx.settings;
    let Some(s) = table.find(name) else {
        let _ = write!(ctx, "no such setting: {name}");
        return Outcome::code(R_NOT_FOUND);
    };
    let mut val = String::<MAX_SETTING>::new();
    read_value(ctx.kv, s, &mut val);
    let (kind, default) = (s.kind, s.default);
    let _ = write!(ctx, "{} = {} [", s.name, val);
    write_constraint(ctx, kind);
    let _ = write!(ctx, ", default {default}]");
    Outcome::ok()
}

fn settings_set(ctx: &mut Ctx<'_>, args: &[&str]) -> Outcome {
    let Some((name, value)) = parse_assign(args) else {
        let _ = write!(ctx, "usage: set <name>=<value>");
        return Outcome::code(R_BAD_ARG);
    };
    let table = ctx.settings;
    let Some(s) = table.find(name) else {
        let _ = write!(ctx, "no such setting: {name}");
        return Outcome::code(R_NOT_FOUND);
    };
    let mut buf = [0u8; MAX_SETTING];
    let n = match encode_value(s.kind, value, &mut buf) {
        Ok(n) => n,
        Err(()) => {
            let kind = s.kind;
            let _ = write!(ctx, "invalid value for {name} (");
            write_constraint(ctx, kind);
            let _ = write!(ctx, ")");
            return Outcome::code(R_BAD_ARG);
        }
    };
    match ctx.kv.set_bytes(s.key, &buf[..n]) {
        Ok(()) => {
            let _ = write!(ctx, "ok");
            Outcome::ok()
        }
        Err(_) => {
            let _ = write!(ctx, "storage error");
            Outcome::code(R_STORAGE)
        }
    }
}

/// Parse `name=value` (one token) or `name value` (two tokens).
fn parse_assign<'a>(args: &[&'a str]) -> Option<(&'a str, &'a str)> {
    let first = args.first()?;
    if let Some(eq) = first.find('=') {
        Some((&first[..eq], &first[eq + 1..]))
    } else {
        args.get(1).map(|v| (*first, *v))
    }
}

fn parse_bool(s: &str) -> Option<bool> {
    match s {
        "true" | "on" | "1" => Some(true),
        "false" | "off" | "0" => Some(false),
        _ => None,
    }
}

/// Validate `value` against `kind` and encode it into `buf`; returns the byte count, or
/// `Err(())` if it's out of range / malformed (the caller prints the constraint).
fn encode_value(kind: Kind, value: &str, buf: &mut [u8; MAX_SETTING]) -> Result<usize, ()> {
    match kind {
        Kind::Str { max } => {
            let lim = (max as usize).min(MAX_SETTING);
            let b = value.as_bytes();
            if b.is_empty() || b.len() > lim {
                return Err(());
            }
            buf[..b.len()].copy_from_slice(b);
            Ok(b.len())
        }
        Kind::Uint { min, max } => {
            let v: u32 = value.parse().map_err(|_| ())?;
            if v < min || v > max {
                return Err(());
            }
            buf[..4].copy_from_slice(&v.to_le_bytes());
            Ok(4)
        }
        Kind::Int { min, max } => {
            let v: i32 = value.parse().map_err(|_| ())?;
            if v < min || v > max {
                return Err(());
            }
            buf[..4].copy_from_slice(&v.to_le_bytes());
            Ok(4)
        }
        Kind::Bool => {
            buf[0] = parse_bool(value).ok_or(())? as u8;
            Ok(1)
        }
        Kind::Enum(choices) => {
            let b = value.as_bytes();
            if !choices.contains(&value) || b.len() > MAX_SETTING {
                return Err(());
            }
            buf[..b.len()].copy_from_slice(b);
            Ok(b.len())
        }
    }
}

/// Read a setting's current value (or its default if unset / unreadable) as text.
fn read_value(kv: Scoped, s: &Setting, out: &mut String<MAX_SETTING>) {
    let mut buf = [0u8; MAX_SETTING];
    if let Ok(Some(n)) = kv.get_bytes(s.key, &mut buf) {
        let n = n.min(MAX_SETTING);
        match s.kind {
            Kind::Str { .. } | Kind::Enum(..) => {
                if let Ok(st) = core::str::from_utf8(&buf[..n]) {
                    let _ = out.push_str(st);
                    return;
                }
            }
            Kind::Uint { .. } if n >= 4 => {
                let v = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                let _ = write!(out, "{v}");
                return;
            }
            Kind::Int { .. } if n >= 4 => {
                let v = i32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                let _ = write!(out, "{v}");
                return;
            }
            Kind::Bool if n >= 1 => {
                let _ = out.push_str(if buf[0] != 0 { "true" } else { "false" });
                return;
            }
            _ => {}
        }
    }
    let _ = out.push_str(s.default);
}

/// Write a human-readable constraint for `kind` (e.g. `uint 1..=3600`, `enum p2p|star`).
fn write_constraint(ctx: &mut Ctx<'_>, kind: Kind) {
    match kind {
        Kind::Str { max } => {
            let _ = write!(ctx, "str 1..={max}");
        }
        Kind::Uint { min, max } => {
            let _ = write!(ctx, "uint {min}..={max}");
        }
        Kind::Int { min, max } => {
            let _ = write!(ctx, "int {min}..={max}");
        }
        Kind::Bool => {
            let _ = write!(ctx, "bool");
        }
        Kind::Enum(choices) => {
            let _ = write!(ctx, "enum ");
            for (i, c) in choices.iter().enumerate() {
                let _ = write!(ctx, "{}{c}", if i == 0 { "" } else { "|" });
            }
        }
    }
}

// ---- completion -------------------------------------------------------------

/// Largest byte offset ≤ `max` that is a char boundary of `s` (never splits UTF-8).
fn clip_idx(s: &str, max: usize) -> usize {
    let mut end = s.len().min(max);
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    end
}

/// Walk the tree to the cursor and enumerate candidates. Returns only 'static data
/// (tree names + setting names), so it never borrows the request line.
fn complete(
    app: &'static [Entry],
    settings: SettingsTable,
    req_id: u16,
    line: &str,
    cursor: u16,
) -> ShellCompletions<'static> {
    let cur = clip_idx(line, cursor as usize);
    let upto = &line[..cur];
    // The token being completed = from the last separator to the cursor (empty if the
    // cursor sits right after a separator → "list everything here").
    let partial_start = upto.rfind(['/', ' ', '\t']).map(|i| i + 1).unwrap_or(0);
    let partial = &upto[partial_start..];
    let prefix_toks = tokenize(&upto[..partial_start]);

    let mut candidates: Vec<Candidate<'static>, 16> = Vec::new();
    let mut more = false;
    // The offset the host will replace from — normally the partial's start, but for
    // `set name=<value>` completion it moves to just after the `=`.
    let mut token_start = partial_start;

    match resolve(&prefix_toks, app) {
        // In a menu → complete child menu/command names (base ⧺ app, deduped by name).
        Resolved::Menu(a, b) => {
            for e in a.iter().chain(b) {
                let name = e.name();
                if name.starts_with(partial) && !candidates.iter().any(|c| c.text == name) {
                    let kind = match e {
                        Entry::Menu(..) => CandidateKind::Menu,
                        Entry::Cmd(..) => CandidateKind::Command,
                    };
                    if candidates.push(Candidate { text: name, kind }).is_err() {
                        more = true;
                        break;
                    }
                }
            }
        }
        // Past a command → complete its argument names, or setting names / values.
        Resolved::Cmd(cmd, _) => match cmd.args {
            Args::None => {}
            Args::Names(names) => {
                for a in names {
                    if a.starts_with(partial)
                        && candidates
                            .push(Candidate {
                                text: a,
                                kind: CandidateKind::Arg,
                            })
                            .is_err()
                    {
                        more = true;
                        break;
                    }
                }
            }
            // `set name=<TAB>` → complete the value (enum choices / bool); else the name.
            Args::Settings { assign } if assign && partial.contains('=') => {
                let eq = partial.find('=').unwrap();
                token_start = partial_start + eq + 1;
                let vpart = &partial[eq + 1..];
                if let Some(s) = settings.find(&partial[..eq]) {
                    let vals: &[&'static str] = match s.kind {
                        Kind::Enum(choices) => choices,
                        Kind::Bool => &["true", "false"],
                        _ => &[],
                    };
                    for &v in vals {
                        if v.starts_with(vpart)
                            && candidates
                                .push(Candidate {
                                    text: v,
                                    kind: CandidateKind::Value,
                                })
                                .is_err()
                        {
                            more = true;
                            break;
                        }
                    }
                }
            }
            Args::Settings { assign } => {
                let kind = if assign {
                    CandidateKind::Arg
                } else {
                    CandidateKind::Value
                };
                for s in settings.iter() {
                    if s.name.starts_with(partial)
                        && candidates.push(Candidate { text: s.name, kind }).is_err()
                    {
                        more = true;
                        break;
                    }
                }
            }
        },
        Resolved::NoMatch => {}
    }

    ShellCompletions {
        req_id,
        token_start: token_start as u16,
        common_prefix: common_prefix(&candidates),
        candidates,
        more,
    }
}

/// Longest common prefix of the candidate texts (ASCII tree/setting names → byte-safe).
fn common_prefix(cands: &[Candidate<'static>]) -> &'static str {
    let Some(first) = cands.first().map(|c| c.text) else {
        return "";
    };
    let mut len = first.len();
    for c in &cands[1..] {
        let common = first
            .bytes()
            .zip(c.text.bytes())
            .take_while(|(a, b)| a == b)
            .count();
        len = len.min(common);
    }
    &first[..len]
}
