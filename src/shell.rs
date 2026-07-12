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
use tower_shell_core::Window;

use crate::console;
use crate::storage::{FLIP_BUDGET, NS_SHELL, Nv, Scoped};

const MAX_LINE: usize = 96;
/// Max whitespace/`/`-separated tokens a command line may hold. An over-long line is rejected
/// rather than having its tail silently dropped (see [`tokenize`] and the `ShellCommand` handler).
const MAX_TOKENS: usize = 8;
/// Shell-response build buffer. Must stay equal to `console::MAX_RESP` (the transport's
/// per-message cap): `console::shell_response` re-clips to that, so if this grew larger the
/// excess would be silently dropped there. Public because [`run_line`] callers (the radio
/// remote shell) allocate the response buffer themselves.
pub const RESP_CAP: usize = 256;
/// Largest value (bytes) a setting can hold (and a `Str` setting's `max`).
pub const MAX_SETTING: usize = 64;

/// Result codes (0 = success).
pub const R_OK: u8 = 0;
pub const R_NOT_FOUND: u8 = 1;
pub const R_BAD_ARG: u8 = 2;
pub const R_STORAGE: u8 = 3;
/// The response exceeded the buffer and was truncated — the body is incomplete. Set by the
/// **console** (single-window) dispatcher when the handler wrote past one [`RESP_CAP`] window
/// ([`Ctx`]'s [`Window`] reports [`more`](Window::more)), so a caller doesn't mistake a cut-off
/// `/export` / `settings print` for a complete, successful one. The **streaming** transport
/// ([`stream_line`]) never truncates — it re-runs the handler to capture every window — so it
/// never returns this.
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
    /// A 32-bit network **address**, shown and entered as hex (`0x1a2b3c4d`; a bare
    /// `1a2b3c4d` is accepted too). Stored as 4 LE bytes. The literal `random` mints a
    /// fresh non-zero address; `auto` (the stored `0`) resolves to the chip-UID-derived
    /// default. See [`radio_addr`]. This is the radio device address (`addr`).
    Addr,
}

/// A persisted, named setting. The shell derives `/system settings print|set|get`
/// and `/export` from the table — no per-setting code.
pub struct Setting {
    /// Local key within the shell namespace (`NS_SHELL`); the shell prefixes it, so a setting can
    /// never collide with another subsystem's keys. **`0x00..=0x0F` is reserved for the SDK base
    /// table** (`identity` = `0x00`, `addr` = `0x01`, the rest headroom for base growth) — app
    /// settings start at `0x10`. A collision doesn't error: two settings would silently alias the
    /// same stored bytes, so the partition is load-bearing for persisted config.
    pub key: u8,
    /// Name used on the command line and in `print`/`export`.
    pub name: &'static str,
    /// Value type (drives validation + formatting).
    pub kind: Kind,
    /// Shown by `get`/`print` when the key has never been set.
    pub default: &'static str,
}

/// `NS_SHELL` local key of the base `addr` setting (the radio device address). Public so a
/// provisioning path can pin it (see the cable-`Provision` handler).
pub const ADDR_KEY: u8 = 0x01;

/// SDK base settings; apps add their own via [`serve_ext`].
static BASE_SETTINGS: &[Setting] = &[
    Setting {
        key: 0x00,
        name: "identity",
        kind: Kind::Str { max: 32 },
        default: "tower",
    },
    // The device's 32-bit radio address (`addr`). Default `auto` = the stored 0
    // sentinel, which [`radio_addr`] resolves to the chip-UID-derived address.
    Setting {
        key: ADDR_KEY,
        name: "addr",
        kind: Kind::Addr,
        default: "auto",
    },
];

/// The device's effective 32-bit radio address: the `addr` base setting when pinned
/// to a non-zero value, else the chip-UID-derived default ([`crate::board::unique_id32`]).
/// This is the `addr` / clear-header `src` a radio app should transmit under.
pub fn radio_addr(kv: Nv) -> u32 {
    let mut b = [0u8; 4];
    match kv.scope(NS_SHELL).get_bytes(ADDR_KEY, &mut b) {
        Ok(Some(4)) if u32::from_le_bytes(b) != 0 => u32::from_le_bytes(b),
        _ => crate::board::unique_id32(),
    }
}

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
///
/// A handler always writes its **whole** response, obliviously — the `Ctx` keeps only the one
/// [`Window`] `[skip, skip+RESP_CAP)` its driver asked for and discards the rest (while still
/// counting the total). The console dispatcher uses a single `skip = 0` window (and flags
/// [`R_TRUNCATED`] if the response spilled past it); the streaming transport re-runs the handler
/// with an advancing `skip` to page the whole response out (see [`stream_line`]).
pub struct Ctx<'a> {
    /// The shell's namespace-scoped EEPROM handle (`NS_SHELL`); settings are keyed by `u8` local.
    pub kv: Scoped,
    /// The merged settings table (SDK base + app).
    pub settings: SettingsTable,
    out: &'a mut String<RESP_CAP>,
    /// The captured window of this pass: `feed` keeps only its `[skip, skip+RESP_CAP)` slice into
    /// `out`, and its [`total`](Window::total)/[`more`](Window::more) drive truncation (console)
    /// and re-run paging (streaming).
    window: Window,
}

impl<'a> Ctx<'a> {
    /// A context that captures the window starting at byte `skip` of the handler's output into the
    /// (cleared) `out` buffer. `out` is `RESP_CAP`-sized and the window cap is `RESP_CAP`, so every
    /// captured segment fits — the `push_str`s below never lose bytes.
    fn new(kv: Scoped, settings: SettingsTable, out: &'a mut String<RESP_CAP>, skip: usize) -> Self {
        out.clear();
        Self {
            kv,
            settings,
            out,
            window: Window::new(skip, RESP_CAP),
        }
    }
}

impl Write for Ctx<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        // Keep only the slice of `s` inside this pass's window; the rest is counted (for
        // `total`/`more`) but dropped. Char-aligned by `feed`, so `out` stays valid UTF-8.
        let _ = self.out.push_str(self.window.feed(s));
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
            Entry::Menu("stack", &[Entry::cmd("print", Args::None, cmd_stack)]),
            Entry::Menu(
                "eeprom",
                &[
                    Entry::cmd("print", Args::None, cmd_eeprom),
                    Entry::cmd("wipe", Args::None, cmd_eeprom_wipe),
                ],
            ),
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
                dispatch(kv, app, settings, cmd.cmd_id, cmd.line).await;
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
    let mut out = String::<RESP_CAP>::new();
    let (result, reboot) = run_line_with(kv, app, settings, line, &mut out);
    console::shell_response(cmd_id, result, out.as_str()).await;
    if reboot {
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

/// Run one command line against the shell registered by [`serve`]/[`serve_ext`] — the
/// transport-independent core powering **both** the console dispatcher and the radio
/// remote shell (one dispatcher, two transports). The response body is written into
/// `out`; the return is `(result_code, reboot_requested)`. A reboot request is NOT
/// acted on here — the caller owns its transport and must flush its response (over
/// radio: send the final chunk) before resetting.
///
/// Synchronous: nothing in the resolve/run path awaits (handlers are plain fns).
/// If no shell was served (`no_shell` apps), answers [`R_NOT_FOUND`].
pub fn run_line(line: &str, out: &mut String<RESP_CAP>) -> (u8, bool) {
    let Some((kv, app_commands, app_settings)) = critical_section::with(|cs| SHELL_PARAMS.borrow(cs).get())
    else {
        let _ = out.push_str("no shell served");
        return (R_NOT_FOUND, false);
    };
    let settings = SettingsTable {
        base: BASE_SETTINGS,
        app: app_settings,
    };
    run_line_with(kv.scope(NS_SHELL), app_commands, settings, line, out)
}

/// An async sink for a **streamed** shell response — the unbounded counterpart to the
/// [`RESP_CAP`]-buffered [`run_line`], used by the radio remote-shell transport so a
/// large reply (`/export`, `settings print`) is never truncated at 256 B. The producer
/// calls [`push`](ShellSink::push) for body text of any length and [`done`](ShellSink::done)
/// exactly once; the sink owns chunk framing (numbering + the `last` flag). Either
/// returns `false` on a transport error, and streaming stops.
///
/// `async fn` in a trait is fine here: the executor is single-threaded (embassy), so
/// the returned futures need no `Send` bound — the reason the lint fires.
#[allow(async_fn_in_trait)]
pub trait ShellSink {
    /// Append response text; the sink transmits full frames as they fill.
    async fn push(&mut self, text: &str) -> bool;
    /// Flush the remainder as the final frame, carrying the authoritative `result`.
    async fn done(&mut self, result: u8) -> bool;
}

/// Stream one command line's response through `sink` — like [`run_line`] but with **no
/// `RESP_CAP` ceiling**, and with **no per-command knowledge**. It streams *every* command
/// uniformly by **windowed re-run**: run the handler capturing one `RESP_CAP`-wide output
/// [`Window`], push it, and if the handler wrote more, re-run with `skip` advanced to the next
/// window — until the whole response has been paged out. A terse response (the common case)
/// fits one window and runs exactly once; a big dump (`/export`, `settings print` with many app
/// settings) re-executes a handful of times, which is fine — those commands are rare.
///
/// **Purity contract:** a handler whose output can exceed one window MUST be deterministic and
/// side-effect-free, because it is re-run per window (re-running must reproduce the identical
/// byte stream). All such SDK handlers are read-only (`export`, `settings print`, `resource`,
/// `eeprom print`, `crash print`); the mutating handlers (`set`, `wipe`, `reboot`) emit a few
/// bytes and so never re-run. App handlers that stream long output must follow the same rule.
///
/// Returns `(result, reboot)`; the caller acts on a reboot only after the sink has flushed its
/// final chunk (the response must be on the air before a reset).
pub async fn stream_line<S: ShellSink>(line: &str, sink: &mut S) -> (u8, bool) {
    // Same guards as `run_line_with`: reject rather than silently truncate.
    if line.len() > MAX_LINE {
        sink.push("line too long").await;
        sink.done(R_BAD_ARG).await;
        return (R_BAD_ARG, false);
    }
    if line.split(['/', ' ', '\t']).filter(|s| !s.is_empty()).count() > MAX_TOKENS {
        sink.push("too many arguments").await;
        sink.done(R_BAD_ARG).await;
        return (R_BAD_ARG, false);
    }
    let Some((kv, app_commands, app_settings)) = critical_section::with(|cs| SHELL_PARAMS.borrow(cs).get())
    else {
        sink.push("no shell served").await;
        sink.done(R_NOT_FOUND).await;
        return (R_NOT_FOUND, false);
    };
    let settings = SettingsTable {
        base: BASE_SETTINGS,
        app: app_settings,
    };
    let kv = kv.scope(NS_SHELL);
    let toks = tokenize(line);
    let Resolved::Cmd(cmd, arg_start) = resolve(&toks, app_commands) else {
        sink.push("no such command").await;
        sink.done(R_NOT_FOUND).await;
        return (R_NOT_FOUND, false);
    };
    // Page the response out one RESP_CAP window per handler run. `skip` is char-aligned (a
    // previous window's `skip + captured`), so no re-run ever splits a multi-byte char.
    let mut out = String::<RESP_CAP>::new();
    let mut skip = 0usize;
    loop {
        let (outcome, window) = {
            let mut ctx = Ctx::new(kv, settings, &mut out, skip);
            let outcome = (cmd.run)(&mut ctx, &toks[arg_start..]);
            (outcome, ctx.window)
        };
        if !sink.push(&out).await {
            return (outcome.result, outcome.reboot); // transport aborted mid-stream
        }
        if !window.more() {
            sink.done(outcome.result).await;
            return (outcome.result, outcome.reboot);
        }
        skip += window.captured();
    }
}

/// [`run_line`] against explicit shell state (the console path already holds it).
fn run_line_with(
    kv: Scoped,
    app: &'static [Entry],
    settings: SettingsTable,
    line: &str,
    out: &mut String<RESP_CAP>,
) -> (u8, bool) {
    // Reject an over-long line instead of silently executing a truncated prefix. The
    // wire allows ~240-byte lines but the shell buffer is MAX_LINE; clipping mid-value
    // could store a truncated setting and still report success (silent corruption).
    if line.len() > MAX_LINE {
        let _ = out.push_str("line too long");
        return (R_BAD_ARG, false);
    }
    // Likewise reject more tokens than `tokenize` holds, rather than dropping the tail.
    if line.split(['/', ' ', '\t']).filter(|s| !s.is_empty()).count() > MAX_TOKENS {
        let _ = out.push_str("too many arguments");
        return (R_BAD_ARG, false);
    }
    let toks = tokenize(line);
    match resolve(&toks, app) {
        Resolved::Cmd(cmd, arg_start) => {
            // One `skip = 0` window: the console transport caps a response at RESP_CAP.
            let (outcome, window) = {
                let mut ctx = Ctx::new(kv, settings, out, 0);
                let outcome = (cmd.run)(&mut ctx, &toks[arg_start..]);
                (outcome, ctx.window)
            };
            // A response that spilled past the single window would otherwise report R_OK on a
            // silently truncated body; surface it as R_TRUNCATED so a scripting caller
            // (`tower exec`) can tell the output is incomplete. Don't mask a handler's own
            // non-zero result. (The streaming transport pages the rest out instead — see
            // `stream_line` — so it never hits this.)
            let result = if window.more() && outcome.result == R_OK {
                R_TRUNCATED
            } else {
                outcome.result
            };
            (result, outcome.reboot)
        }
        _ => {
            let _ = out.push_str("no such command");
            (R_NOT_FOUND, false)
        }
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
    // no added wear) vs the conservative flip budget (FLIP_BUDGET), plus the store's occupancy
    // (live/free) and the background-maintenance pre-blank state. All pure reads. See
    // docs/storage.md.
    let nv = ctx.kv.raw();
    let flips = nv.flip_generation();
    // Per-mille of budget in integer math (no FPU on the M0+): rendered as X.X%.
    let permille = ((flips as u64) * 1000 / FLIP_BUDGET as u64) as u32;
    let _ = write!(
        ctx,
        "eeprom: 6 KiB data EEPROM\r\n\
         flips: {} / {} ({}.{}%)\r\n\
         live: {} B\r\n\
         free: {} B\r\n\
         dead-half blanked: {}\r\n\
         resets: {}\r\n",
        flips,
        FLIP_BUDGET,
        permille / 10,
        permille % 10,
        nv.live_bytes(),
        nv.free_bytes(),
        if nv.dead_half_blank() { "yes" } else { "no" },
        crate::bootguard::consecutive_resets(),
    );
    Outcome::ok()
}

fn cmd_stack(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    // Measured stack high-water: the boot painter (`stack::paint`) filled the free stack with a
    // sentinel; this reports how much has since been overwritten — the deepest the call graph +
    // ISR frames ever reached. Weigh `used` against the 8 KB budget floor (docs/gateway.md); a
    // small `free` is a near-overflow warning on this 20 KB part. All pure reads.
    let total = crate::stack::total();
    let used = crate::stack::used();
    let free = crate::stack::free();
    // Per-mille in integer math (no FPU on the M0+), rendered X.X%.
    let permille = if total > 0 {
        (used as u64 * 1000 / total as u64) as u32
    } else {
        0
    };
    let _ = write!(
        ctx,
        "stack: {total} B total (RAM bottom, flip-link)\r\n\
         used: {used} B ({}.{}%) peak high-water\r\n\
         free: {free} B at the deepest reach\r\n",
        permille / 10,
        permille % 10,
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

fn cmd_eeprom_wipe(ctx: &mut Ctx<'_>, args: &[&str]) -> Outcome {
    // Factory reset. Destroys EVERYTHING the store holds — pairing keys, peer tables, replay
    // lanes, the TX watermark, settings, the session counter — so it demands the literal
    // `confirm` argument (which completion never offers) rather than firing on a bare line.
    if args != ["confirm"] {
        let _ = write!(
            ctx,
            "usage: /system/eeprom wipe confirm
erases the WHOLE store (keys, peers, settings) and reboots"
        );
        return Outcome::code(R_BAD_ARG);
    }
    // ~5 s CPU stall while the EEPROM zeroes (docs/storage.md); the response goes out after,
    // then the framework flushes it and reboots into a virgin store.
    match ctx.kv.raw().wipe() {
        Ok(()) => {
            let _ = write!(ctx, "eeprom wiped - rebooting");
            Outcome {
                result: R_OK,
                reboot: true,
            }
        }
        Err(_) => {
            let _ = write!(ctx, "wipe failed");
            Outcome::code(R_STORAGE)
        }
    }
}

/// One setting's line for `print` (`name = value`) or `export` (`/system settings
/// set name=value`), trailing CRLF included. Shared by the sync handlers below and the
/// streaming path ([`stream_line`]) so the two formats can never drift.
fn write_setting(kv: Scoped, s: &Setting, export: bool, out: &mut impl Write) {
    let mut val = String::<MAX_SETTING>::new();
    // `get`/`print` show an unset address as the *effective* UID-derived value — but
    // exporting that literal would pin it on whatever device replays the export (two
    // nodes sharing one radio address share a replay lane: the lower-counter one goes
    // silently dead). Export the stored `auto` sentinel instead.
    if export && matches!(s.kind, Kind::Addr) && stored_addr(kv, s.key).unwrap_or(0) == 0 {
        let _ = val.push_str("auto");
    } else {
        read_value(kv, s, &mut val);
    }
    if export {
        let _ = write!(out, "/system settings set {}={}\r\n", s.name, val);
    } else {
        let _ = write!(out, "{} = {}\r\n", s.name, val);
    }
}

fn cmd_export(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    let table = ctx.settings;
    for s in table.iter() {
        write_setting(ctx.kv, s, true, ctx);
    }
    Outcome::ok()
}

/// The raw stored `Addr` value for `key`, `None` when unset/unreadable (both of which
/// read back as the `auto` sentinel `0` for export purposes).
fn stored_addr(kv: Scoped, key: u8) -> Option<u32> {
    let mut b = [0u8; 4];
    match kv.get_bytes(key, &mut b) {
        Ok(Some(4)) => Some(u32::from_le_bytes(b)),
        _ => None,
    }
}

// ---- derived settings commands ----------------------------------------------

fn settings_print(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    let table = ctx.settings;
    for s in table.iter() {
        write_setting(ctx.kv, s, false, ctx);
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
    // The tokenizer splits on `/`, space, and tab, so a value containing one arrives
    // as EXTRA tokens (`identity=a/b` → ["identity=a", "b"]). Storing just the first
    // piece and answering `ok` would be silent truncation — reject instead.
    let consumed = if args.first().is_some_and(|a| a.contains('=')) {
        1
    } else {
        2
    };
    if args.len() > consumed {
        let _ = write!(ctx, "value must not contain '/', space, or tab");
        return Outcome::code(R_BAD_ARG);
    }
    let table = ctx.settings;
    let Some(s) = table.find(name) else {
        let _ = write!(ctx, "no such setting: {name}");
        return Outcome::code(R_NOT_FOUND);
    };
    let mut buf = [0u8; MAX_SETTING];
    // `addr=random` mints a fresh non-zero address here (the pure encoder has no RNG);
    // everything else goes through the normal validated encode.
    let (n, generated) = if matches!(s.kind, Kind::Addr) && value.eq_ignore_ascii_case("random") {
        let mut a = crate::board::rand_u32();
        if a == 0 {
            a = 1;
        }
        buf[..4].copy_from_slice(&a.to_le_bytes());
        (4, Some(a))
    } else {
        match encode_value(s.kind, value, &mut buf) {
            Ok(n) => (n, None),
            Err(()) => {
                let kind = s.kind;
                let _ = write!(ctx, "invalid value for {name} (");
                write_constraint(ctx, kind);
                let _ = write!(ctx, ")");
                return Outcome::code(R_BAD_ARG);
            }
        }
    };
    match ctx.kv.set_bytes(s.key, &buf[..n]) {
        Ok(()) => {
            match generated {
                Some(a) => {
                    let _ = write!(ctx, "ok (0x{a:08x})");
                }
                None => {
                    let _ = write!(ctx, "ok");
                }
            }
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
        // `random` is resolved to a generated hex value in `settings_set` before this,
        // so here `Addr` only ever sees `auto` (→ 0) or a hex literal.
        Kind::Addr => {
            let v = tower_shell_core::parse_addr(value).ok_or(())?;
            buf[..4].copy_from_slice(&v.to_le_bytes());
            Ok(4)
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
            // Show the *effective* address (hex): a stored 0 = "auto" resolves to the
            // chip-UID-derived value, so `get addr` reports what the radio actually uses.
            Kind::Addr if n >= 4 => {
                let a = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                let eff = if a != 0 { a } else { crate::board::unique_id32() };
                let _ = write!(out, "0x{eff:08x}");
                return;
            }
            _ => {}
        }
    }
    // Unset: an address still has an effective value (the UID-derived default).
    if matches!(s.kind, Kind::Addr) {
        let _ = write!(out, "0x{:08x}", crate::board::unique_id32());
    } else {
        let _ = out.push_str(s.default);
    }
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
        Kind::Addr => {
            let _ = write!(ctx, "hex 32-bit | auto | random");
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
                        Kind::Addr => &["auto", "random"],
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
