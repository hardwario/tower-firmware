//! Message payloads (the postcard schema). Borrowed (`&str`) so they serialize
//! no-alloc on the target and decode zero-copy on the host. Both ends share these
//! exact definitions — postcard is not self-describing, so field order and variant
//! order are load-bearing (postcard encodes enums by variant **index**).

use heapless::Vec;
use serde::{Deserialize, Serialize};

/// Log severity. Order matches the `log` crate (Error highest). postcard encodes
/// this by variant index, so both ends must keep this order.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
pub enum Level {
    Error,
    Warn,
    Info,
    Debug,
    Trace,
}

/// Target → host: one-time announcement on boot / VBUS-present edge. The per-frame
/// `ver_type` is the real version guard; this carries the firmware string for display.
#[derive(Serialize, Deserialize, Debug)]
pub struct Hello<'a> {
    pub protocol_version: u8,
    pub firmware_version: &'a str,
}

/// Target → host: a structured log record. The host prepends local time + colorizes.
#[derive(Serialize, Deserialize, Debug)]
pub struct Log<'a> {
    pub level: Level,
    pub uptime_us: u64,
    pub module: &'a str,
    pub message: &'a str,
}

/// Target → host: raw text from `print!`/`println!`, rendered verbatim.
#[derive(Serialize, Deserialize, Debug)]
pub struct Print<'a> {
    pub text: &'a str,
}

/// Target → host: a self-describing event (key=value pairs) so the host renders any
/// app's event without a shared per-app schema.
#[derive(Serialize, Deserialize, Debug)]
pub struct Event<'a> {
    #[serde(borrow)]
    pub name: &'a str,
    #[serde(borrow)]
    pub fields: Vec<(&'a str, &'a str), 8>,
}

/// Target → host: one chunk of a shell response. `result` is authoritative only on
/// the `last` chunk; `chunk` indexes detect a CRC-dropped chunk mid-response.
#[derive(Serialize, Deserialize, Debug)]
pub struct ShellResponse<'a> {
    pub cmd_id: u16,
    pub result: u8, // 0 = success
    pub chunk: u16,
    pub last: bool,
    pub text: &'a str,
}

/// What a completion candidate is — drives host coloring + the auto-insert separator.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
pub enum CandidateKind {
    Menu,
    Command,
    Arg,
    Value,
}

/// One completion candidate (text from the static command tree).
#[derive(Serialize, Deserialize, Debug)]
pub struct Candidate<'a> {
    pub text: &'a str,
    pub kind: CandidateKind,
}

/// Target → host: completion result for a `ShellComplete`. `token_start` is the byte
/// offset in the request line the host should replace; `more` flags >16 candidates.
#[derive(Serialize, Deserialize, Debug)]
pub struct ShellCompletions<'a> {
    pub req_id: u16,
    pub token_start: u16,
    pub common_prefix: &'a str,
    #[serde(borrow)]
    pub candidates: Vec<Candidate<'a>, 16>,
    pub more: bool,
}

/// Target → host: overflow marker — `count` log/print frames were dropped.
#[derive(Serialize, Deserialize, Debug)]
pub struct Dropped {
    pub count: u32,
}

/// Host → target: a shell command line. `cmd_id` correlates the response.
#[derive(Serialize, Deserialize, Debug)]
pub struct ShellCommand<'a> {
    pub cmd_id: u16,
    pub line: &'a str,
}

/// Host → target: a completion request. `req_id` correlates `ShellCompletions`.
#[derive(Serialize, Deserialize, Debug)]
pub struct ShellComplete<'a> {
    pub req_id: u16,
    pub line: &'a str,
    pub cursor: u16,
}
