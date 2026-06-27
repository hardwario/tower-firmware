//! RouterOS-style shell over the framed console, with **target-authoritative TAB
//! completion**.
//!
//! Opt-in: an app calls [`serve`] with the board's storage. A task polls the USART
//! RX register directly (the low-power executor fights interrupt-driven RX — see
//! CONSOLE-PLAN.md §5.5), reassembles frames via [`FrameDecoder`], and handles two
//! request types against **one static command tree** ([`ROOT`]):
//!   * `ShellCommand` → walk the tree → execute → `ShellResponse`;
//!   * `ShellComplete` → walk the tree **to the cursor** → `ShellCompletions`.
//!
//! Both use the same tokenizer + [`resolve`] walk, so completion can never suggest
//! something execution won't accept. Settings (e.g. identity) live in the EEPROM KV.

use core::fmt::Write;

use embassy_executor::Spawner;
use embassy_stm32::usart::BufferedUartRx;
use embassy_time::{Instant, Timer};
use embedded_io_async::Read as _;
use heapless::{String, Vec};
use tower_protocol::msg::{Candidate, CandidateKind, ShellComplete, ShellCommand, ShellCompletions};
use tower_protocol::{FrameDecoder, MsgType, decode_frame};

use crate::console;
use crate::storage::{CONSOLE_SETTINGS_BASE, Kv, Storage};

const MAX_LINE: usize = 96;
const RESP_CAP: usize = 256;
const IDENTITY_KEY: u16 = CONSOLE_SETTINGS_BASE;
const MAX_IDENTITY: usize = 32;

/// Result codes (0 = success).
const R_OK: u8 = 0;
const R_NOT_FOUND: u8 = 1;
const R_BAD_ARG: u8 = 2;
const R_STORAGE: u8 = 3;

// ---- command tree (the single source of truth for dispatch AND completion) ----

#[derive(Clone, Copy, PartialEq)]
enum CmdId {
    Reboot,
    ResourcePrint,
    IdentityPrint,
    IdentitySet,
    Export,
}

/// A tree node: a menu (with children) or a command (with an id + arg names).
enum Entry {
    Menu(&'static str, &'static [Entry]),
    Cmd(&'static str, CmdId, &'static [&'static str]),
}

impl Entry {
    fn name(&self) -> &'static str {
        match self {
            Entry::Menu(n, _) | Entry::Cmd(n, _, _) => n,
        }
    }
}

static ROOT: &[Entry] = &[
    Entry::Menu(
        "system",
        &[
            Entry::Cmd("reboot", CmdId::Reboot, &[]),
            Entry::Menu("resource", &[Entry::Cmd("print", CmdId::ResourcePrint, &[])]),
            Entry::Menu(
                "identity",
                &[
                    Entry::Cmd("print", CmdId::IdentityPrint, &[]),
                    Entry::Cmd("set", CmdId::IdentitySet, &["name"]),
                ],
            ),
        ],
    ),
    Entry::Cmd("export", CmdId::Export, &[]),
];

/// Outcome of walking the tree with a token slice.
enum Resolved {
    /// All tokens consumed, landed on a menu — these are its children.
    Menu(&'static [Entry]),
    /// Reached a command: its id, its arg names, and the index of the first arg token.
    Cmd(CmdId, &'static [&'static str], usize),
    /// A token matched nothing at its level.
    NoMatch,
}

/// Tokenize on `/` and whitespace (both separate path/command tokens).
fn tokenize(line: &str) -> Vec<&str, 8> {
    let mut v = Vec::new();
    for t in line.split(['/', ' ', '\t']).filter(|s| !s.is_empty()) {
        let _ = v.push(t);
    }
    v
}

/// Walk [`ROOT`] consuming `toks`. Shared by dispatch and completion.
fn resolve(toks: &[&str]) -> Resolved {
    let mut entries: &'static [Entry] = ROOT;
    let mut i = 0;
    while i < toks.len() {
        let next = entries.iter().find(|e| e.name() == toks[i]);
        match next {
            Some(Entry::Menu(_, children)) => {
                entries = children;
                i += 1;
            }
            Some(Entry::Cmd(_, id, args)) => return Resolved::Cmd(*id, args, i + 1),
            None => return Resolved::NoMatch,
        }
    }
    Resolved::Menu(entries)
}

// ---- task / dispatch / execution ----

pub fn serve(spawner: Spawner, storage: Storage<'static>) {
    let kv = Kv::new(storage);
    let rx = console::take_rx().expect("console RX already taken / not initialised");
    spawner.spawn(shell_task(kv, rx).unwrap());
}

#[embassy_executor::task]
async fn shell_task(mut kv: Kv<'static>, mut rx: BufferedUartRx<'static>) {
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
                handle(&mut kv, inner).await;
            }
        }
    }
}

/// Decode a complete frame and act on `ShellCommand` / `ShellComplete`.
async fn handle(kv: &mut Kv<'static>, inner: &[u8]) {
    let Ok((mt, _seq, payload)) = decode_frame(inner) else {
        return;
    };
    match mt {
        MsgType::ShellCommand => {
            if let Ok(cmd) = postcard::from_bytes::<ShellCommand>(payload) {
                let mut line = String::<MAX_LINE>::new();
                let _ = line.push_str(&cmd.line[..cmd.line.len().min(MAX_LINE)]);
                dispatch(kv, cmd.cmd_id, line.as_str()).await;
            }
        }
        MsgType::ShellComplete => {
            if let Ok(req) = postcard::from_bytes::<ShellComplete>(payload) {
                // `complete` returns only 'static data (from the tree), so the frame
                // borrow can end before we await.
                let comp = complete(req.req_id, req.line, req.cursor);
                console::shell_completions(comp).await;
            }
        }
        _ => {}
    }
}

async fn dispatch(kv: &mut Kv<'static>, cmd_id: u16, line: &str) {
    let toks = tokenize(line);
    match resolve(&toks) {
        Resolved::Cmd(id, _args, arg_start) => {
            execute(kv, cmd_id, id, &toks[arg_start..]).await
        }
        _ => console::shell_response(cmd_id, R_NOT_FOUND, "no such command").await,
    }
}

async fn execute(kv: &mut Kv<'static>, cmd_id: u16, id: CmdId, args: &[&str]) {
    match id {
        CmdId::Reboot => {
            console::shell_response(cmd_id, R_OK, "rebooting").await;
            Timer::after_millis(150).await; // let the response flush before reset
            cortex_m::peripheral::SCB::sys_reset();
        }
        CmdId::ResourcePrint => {
            let us = Instant::now().as_micros();
            let mut s = String::<RESP_CAP>::new();
            let _ = write!(
                s,
                "uptime: {}.{:03} s\r\nfirmware: {}\r\ncpu: STM32L083 @ 16 MHz (HSI)\r\n",
                us / 1_000_000,
                (us % 1_000_000) / 1000,
                env!("CARGO_PKG_VERSION"),
            );
            console::shell_response(cmd_id, R_OK, s.as_str()).await;
        }
        CmdId::IdentityPrint => {
            let id = load_identity(kv);
            let mut s = String::<64>::new();
            let _ = write!(s, "name: {}", id.as_str());
            console::shell_response(cmd_id, R_OK, s.as_str()).await;
        }
        CmdId::IdentitySet => match args.first().and_then(|a| a.strip_prefix("name=")) {
            Some(name) if !name.is_empty() && name.len() <= MAX_IDENTITY => {
                match kv.set_bytes(IDENTITY_KEY, name.as_bytes()) {
                    Ok(()) => console::shell_response(cmd_id, R_OK, "ok").await,
                    Err(_) => console::shell_response(cmd_id, R_STORAGE, "storage error").await,
                }
            }
            Some(_) => console::shell_response(cmd_id, R_BAD_ARG, "name must be 1..=32 chars").await,
            None => console::shell_response(cmd_id, R_BAD_ARG, "expected name=<value>").await,
        },
        CmdId::Export => {
            let id = load_identity(kv);
            let mut s = String::<128>::new();
            let _ = write!(s, "/system identity set name={}\r\n", id.as_str());
            console::shell_response(cmd_id, R_OK, s.as_str()).await;
        }
    }
}

fn load_identity(kv: &Kv<'static>) -> String<MAX_IDENTITY> {
    let mut s = String::new();
    let mut buf = [0u8; MAX_IDENTITY];
    match kv.get_bytes(IDENTITY_KEY, &mut buf) {
        Ok(Some(n)) if n <= MAX_IDENTITY => {
            if let Ok(st) = core::str::from_utf8(&buf[..n]) {
                let _ = s.push_str(st);
            }
        }
        _ => {
            let _ = s.push_str("tower");
        }
    }
    s
}

// ---- completion ----

/// Walk the tree to the cursor and enumerate candidates. Returns only 'static data
/// (candidate text comes from the tree), so it never borrows the request line.
fn complete(req_id: u16, line: &str, cursor: u16) -> ShellCompletions<'static> {
    let cur = (cursor as usize).min(line.len());
    let upto = &line[..cur];
    // The token being completed = from the last separator to the cursor (empty if the
    // cursor sits right after a separator → "list everything here").
    let partial_start = upto.rfind(['/', ' ', '\t']).map(|i| i + 1).unwrap_or(0);
    let partial = &upto[partial_start..];
    let prefix_toks = tokenize(&upto[..partial_start]);

    let mut candidates: Vec<Candidate<'static>, 16> = Vec::new();
    let mut more = false;

    match resolve(&prefix_toks) {
        // In a menu → complete child menu/command names.
        Resolved::Menu(children) => {
            for e in children {
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
        // Past a command → complete its arg names (host adds `=` from the kind).
        Resolved::Cmd(_id, arg_names, _) => {
            for a in arg_names {
                if a.starts_with(partial)
                    && candidates.push(Candidate { text: a, kind: CandidateKind::Arg }).is_err()
                {
                    more = true;
                    break;
                }
            }
        }
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

/// Longest common prefix of the candidate texts (ASCII tree names → byte-safe).
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
