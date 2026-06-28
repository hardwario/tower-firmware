//! RouterOS-style shell over the framed console, with **target-authoritative TAB
//! completion**, a **declarative settings framework**, and an **app-extensible
//! command tree**.
//!
//! Opt-in: an app calls [`serve`] (base only) or [`serve_ext`] (with its own
//! commands + settings). A task async-reads the console's `BufferedUartRx`
//! (interrupt-driven; while USB is present `vbus_task` holds STOP off so the RX
//! interrupt fires — see docs/console.md), reassembles frames via
//! [`FrameDecoder`], and handles two request types against one command tree:
//!   * `ShellCommand` → walk the tree → run the command's handler → `ShellResponse`;
//!   * `ShellComplete` → walk the tree **to the cursor** → `ShellCompletions`.
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
//!     Setting { key: 0x5510, name: "interval", kind: Kind::Uint { min: 1, max: 3600 }, default: "30" },
//!     Setting { key: 0x5511, name: "mode", kind: Kind::Enum(&["p2p", "star", "mesh"]), default: "star" },
//! ];
//! shell::serve_ext(b.spawner, b.storage, CMDS, SETS);
//! ```

use core::fmt::{self, Write};

use embassy_executor::Spawner;
use embassy_stm32::usart::BufferedUartRx;
use embassy_time::{Instant, Timer};
use embedded_io_async::Read as _;
use heapless::{String, Vec};
use tower_protocol::msg::{Candidate, CandidateKind, ShellCommand, ShellComplete, ShellCompletions};
use tower_protocol::{FrameDecoder, MsgType, PROTOCOL_VERSION, decode_frame};

use crate::console;
use crate::storage::{CONSOLE_SETTINGS_BASE, Kv, Storage};

const MAX_LINE: usize = 96;
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
    /// EEPROM KV key. SDK settings use `0x5500+`; pick app keys above your base.
    pub key: u16,
    /// Name used on the command line and in `print`/`export`.
    pub name: &'static str,
    /// Value type (drives validation + formatting).
    pub kind: Kind,
    /// Shown by `get`/`print` when the key has never been set.
    pub default: &'static str,
}

/// SDK base settings; apps add their own via [`serve_ext`].
static BASE_SETTINGS: &[Setting] = &[Setting {
    key: CONSOLE_SETTINGS_BASE,
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
        Self { result: R_OK, reboot: false }
    }
    /// A non-zero result code, no reboot.
    pub fn code(result: u8) -> Self {
        Self { result, reboot: false }
    }
}

/// Execution context for a command handler. Write the response via `write!(ctx, …)`
/// (Ctx is [`core::fmt::Write`]); persistent state is `ctx.kv` / `ctx.settings`.
pub struct Ctx<'a> {
    /// The EEPROM key-value store (apps own keys outside the `0x5500` console range).
    pub kv: &'a mut Kv<'static>,
    /// The merged settings table (SDK base + app).
    pub settings: SettingsTable,
    out: &'a mut String<RESP_CAP>,
}

impl Write for Ctx<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        // Truncate at the response cap instead of failing the whole write.
        let mut end = s.len().min(RESP_CAP - self.out.len());
        while end > 0 && !s.is_char_boundary(end) {
            end -= 1;
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

/// Tokenize on `/` and whitespace (both separate path/command tokens).
fn tokenize(line: &str) -> Vec<&str, 8> {
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

/// Serve the shell with only the SDK base tree + settings.
pub fn serve(spawner: Spawner, storage: Storage<'static>) {
    serve_ext(spawner, storage, &[], &[]);
}

/// Serve the shell with app extensions: extra top-level command-tree entries and
/// extra settings (reachable via `/system settings`). Pass `&[]` for none.
pub fn serve_ext(
    spawner: Spawner,
    storage: Storage<'static>,
    app_commands: &'static [Entry],
    app_settings: &'static [Setting],
) {
    let kv = Kv::new(storage);
    let rx = console::take_rx().expect("console RX already taken / not initialised");
    spawner.spawn(shell_task(kv, rx, app_commands, app_settings).unwrap());
}

#[embassy_executor::task]
async fn shell_task(
    mut kv: Kv<'static>,
    mut rx: BufferedUartRx<'static>,
    app_commands: &'static [Entry],
    app_settings: &'static [Setting],
) {
    let settings = SettingsTable { base: BASE_SETTINGS, app: app_settings };
    let mut dec = FrameDecoder::new();
    let mut buf = [0u8; 64];
    loop {
        // Async read — awaits the USART RX interrupt (no busy-poll). When USB is present
        // `vbus_task` holds STOP off, so the interrupt fires; unplugged there's no host.
        let n = match rx.read(&mut buf).await {
            Ok(n) => n,
            Err(_) => continue,
        };
        for &b in &buf[..n] {
            if let Some(inner) = dec.push(b) {
                handle(&mut kv, app_commands, settings, inner).await;
            }
        }
    }
}

/// Decode a complete frame and act on `ShellCommand` / `ShellComplete`.
async fn handle(kv: &mut Kv<'static>, app: &'static [Entry], settings: SettingsTable, inner: &[u8]) {
    let Ok((mt, _seq, payload)) = decode_frame(inner) else {
        return;
    };
    match mt {
        MsgType::ShellCommand => {
            if let Ok(cmd) = postcard::from_bytes::<ShellCommand>(payload) {
                let mut line = String::<MAX_LINE>::new();
                let _ = line.push_str(&cmd.line[..clip_idx(cmd.line, MAX_LINE)]);
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

async fn dispatch(
    kv: &mut Kv<'static>,
    app: &'static [Entry],
    settings: SettingsTable,
    cmd_id: u16,
    line: &str,
) {
    let toks = tokenize(line);
    match resolve(&toks, app) {
        Resolved::Cmd(cmd, arg_start) => {
            let mut out = String::<RESP_CAP>::new();
            let outcome = {
                let mut ctx = Ctx { kv, settings, out: &mut out };
                (cmd.run)(&mut ctx, &toks[arg_start..])
            };
            console::shell_response(cmd_id, outcome.result, out.as_str()).await;
            if outcome.reboot {
                Timer::after_millis(150).await; // let the response flush before reset
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
        _ => console::shell_response(cmd_id, R_NOT_FOUND, "no such command").await,
    }
}

// ---- built-in command handlers ----------------------------------------------

fn cmd_reboot(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    let _ = write!(ctx, "rebooting");
    Outcome { result: R_OK, reboot: true }
}

fn cmd_resource(ctx: &mut Ctx<'_>, _args: &[&str]) -> Outcome {
    let us = Instant::now().as_micros();
    // Multi-line summary (spans more than one wire frame — exercises chunking).
    let _ = write!(
        ctx,
        "firmware:  {}\r\n\
         protocol:  v{}\r\n\
         uptime:    {}.{:03} s\r\n\
         cpu:       STM32L083CZ Cortex-M0+ @ 16 MHz (HSI)\r\n\
         clock:     LSE 32.768 kHz RTC tick\r\n\
         memory:    192 KiB flash / 20 KiB RAM / 6 KiB EEPROM\r\n\
         console:   USART1 PA9/PA10 115200 8N1, framed\r\n",
        env!("CARGO_PKG_VERSION"),
        PROTOCOL_VERSION,
        us / 1_000_000,
        (us % 1_000_000) / 1000,
    );
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
    let _ = write!(ctx, "{} = {}  [", s.name, val);
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
fn read_value(kv: &Kv<'static>, s: &Setting, out: &mut String<MAX_SETTING>) {
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
                        && candidates.push(Candidate { text: a, kind: CandidateKind::Arg }).is_err()
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
                            && candidates.push(Candidate { text: v, kind: CandidateKind::Value }).is_err()
                        {
                            more = true;
                            break;
                        }
                    }
                }
            }
            Args::Settings { assign } => {
                let kind = if assign { CandidateKind::Arg } else { CandidateKind::Value };
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
