//! RouterOS-style shell over the framed console, with **target-authoritative TAB
//! completion**, a **declarative settings framework**, and an **app-extensible
//! command tree**.
//!
//! Opt-in: an app calls [`serve`] (base only) or [`serve_ext`] (with its own
//! commands + settings). A task async-reads the console's `BufferedUartRx`
//! (interrupt-driven; while USB is present `vbus_task` holds STOP off so the RX
//! interrupt fires — see CONSOLE-PLAN.md §5.5), reassembles frames via
//! [`FrameDecoder`], and handles two request types against one command tree:
//!   * `ShellCommand` → walk the tree → run the command's handler → `ShellResponse`;
//!   * `ShellComplete` → walk the tree **to the cursor** → `ShellCompletions`.
//!
//! Both use the same tokenizer + [`resolve`] walk, so completion can never suggest
//! something execution won't accept.
//!
//! ## Extending it (apps)
//! ```ignore
//! fn cmd_hi(ctx: &mut shell::Ctx, _args: &[&str]) -> shell::Outcome {
//!     let _ = write!(ctx, "hello");        // Ctx: core::fmt::Write
//!     shell::Outcome::ok()
//! }
//! static CMDS: &[shell::Entry] = &[shell::Entry::cmd("hi", shell::Args::None, cmd_hi)];
//! static SETS: &[shell::Setting] = &[shell::Setting {
//!     key: 0x5510, name: "interval", kind: shell::Kind::U32, default: "30",
//! }];
//! shell::serve_ext(b.spawner, b.storage, CMDS, SETS); // /hi + /system settings interval
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
const RESP_CAP: usize = 256;
/// Largest value (bytes) a setting can hold (and a `Str` setting's `max`).
pub const MAX_SETTING: usize = 64;

/// Result codes (0 = success).
pub const R_OK: u8 = 0;
pub const R_NOT_FOUND: u8 = 1;
pub const R_BAD_ARG: u8 = 2;
pub const R_STORAGE: u8 = 3;

// ---- declarative settings ---------------------------------------------------

/// How a [`Setting`]'s value is encoded in EEPROM and parsed / printed.
#[derive(Clone, Copy)]
pub enum Kind {
    /// UTF-8 text, 1..=`max` bytes (`max` is clamped to [`MAX_SETTING`]).
    Str { max: u16 },
    /// Unsigned 32-bit integer: decimal on the command line, 4 LE bytes in EEPROM.
    U32,
    /// Boolean: accepts `true`/`false`, `on`/`off`, `1`/`0`; stored as one byte.
    Bool,
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

/// Outcome of walking the tree with a token slice.
enum Resolved {
    /// Landed on a menu — these are its children (primary = SDK/base, secondary =
    /// app entries, non-empty only at the root level).
    Menu(&'static [Entry], &'static [Entry]),
    /// Reached a command, plus the index of its first argument token.
    Cmd(&'static Command, usize),
    /// A token matched nothing at its level.
    NoMatch,
}

/// Walk the tree consuming `toks`. The root level spans the SDK base **and** the
/// app's entries; deeper levels are a single menu's children.
fn resolve(toks: &[&str], app: &'static [Entry]) -> Resolved {
    let mut primary: &'static [Entry] = BASE_ROOT;
    let mut secondary: &'static [Entry] = app;
    let mut i = 0;
    while i < toks.len() {
        match primary.iter().chain(secondary).find(|e| e.name() == toks[i]) {
            Some(Entry::Menu(_, children)) => {
                primary = children;
                secondary = &[];
                i += 1;
            }
            Some(Entry::Cmd(c)) => return Resolved::Cmd(c, i + 1),
            None => return Resolved::NoMatch,
        }
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
    let _ = write!(ctx, "{} = {}", s.name, val);
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
        Err(msg) => {
            let _ = write!(ctx, "{msg}");
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

/// Validate `value` against `kind` and encode it into `buf`; returns the byte count.
fn encode_value(kind: Kind, value: &str, buf: &mut [u8; MAX_SETTING]) -> Result<usize, &'static str> {
    match kind {
        Kind::Str { max } => {
            let lim = (max as usize).min(MAX_SETTING);
            let b = value.as_bytes();
            if b.is_empty() || b.len() > lim {
                return Err("value length out of range");
            }
            buf[..b.len()].copy_from_slice(b);
            Ok(b.len())
        }
        Kind::U32 => {
            let v: u32 = value.parse().map_err(|_| "expected an unsigned integer")?;
            buf[..4].copy_from_slice(&v.to_le_bytes());
            Ok(4)
        }
        Kind::Bool => {
            let v = parse_bool(value).ok_or("expected true/false")?;
            buf[0] = v as u8;
            Ok(1)
        }
    }
}

/// Read a setting's current value (or its default if unset / unreadable) as text.
fn read_value(kv: &Kv<'static>, s: &Setting, out: &mut String<MAX_SETTING>) {
    let mut buf = [0u8; MAX_SETTING];
    if let Ok(Some(n)) = kv.get_bytes(s.key, &mut buf) {
        let n = n.min(MAX_SETTING);
        match s.kind {
            Kind::Str { .. } => {
                if let Ok(st) = core::str::from_utf8(&buf[..n]) {
                    let _ = out.push_str(st);
                    return;
                }
            }
            Kind::U32 if n >= 4 => {
                let v = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
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

    match resolve(&prefix_toks, app) {
        // In a menu → complete child menu/command names (base ⧺ app at the root).
        Resolved::Menu(a, b) => {
            for e in a.iter().chain(b) {
                if e.name().starts_with(partial) {
                    let kind = match e {
                        Entry::Menu(..) => CandidateKind::Menu,
                        Entry::Cmd(..) => CandidateKind::Command,
                    };
                    if candidates.push(Candidate { text: e.name(), kind }).is_err() {
                        more = true;
                        break;
                    }
                }
            }
        }
        // Past a command → complete its argument names, or setting names for set/get.
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
        token_start: partial_start as u16,
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
