//! Framed serial console — the SDK's logging + host-link primitive.
//!
//! The UART is **always framed** (`tower-protocol`): every log record, `print!`,
//! event, and the boot `Hello` is a COBS+CRC+postcard frame on USART1 TX (PA9) at
//! 115200. `jolt monitor` therefore shows binary — use the `tower` host CLI.
//!
//! Architecture (CONSOLE-PLAN.md §5):
//!   * producers (the `log` backend, `print!`, `event`, the boot `Hello`) build an
//!     owned [`Outgoing`] message and `try_send` it into [`TX_CHANNEL`] — non-blocking,
//!     drop-newest on full (a dropped count is reported by the writer);
//!   * one [`writer_task`] **owns** the interrupt-buffered `BufferedUartTx`, assigns the
//!     per-frame `seq` (so a gap means real wire loss, not a queue drop), encodes, and
//!     async-writes — holding a `WakeGuard` across each burst so the low-power executor
//!     uses WFI (USART clocked) instead of STOP, which would gate the TXE interrupt;
//!   * the **panic** path can't use the (dead) executor, so it silences the buffered
//!     ISR and blocking-writes one frame straight to the USART1 registers via the PAC.
//!
//! The full-duplex `BufferedUart` is built and split by the board; TX goes to the
//! writer task, RX is **parked** here ([`take_rx`]) for the shell, which reads it async.

use core::cell::{Cell, RefCell};
use core::fmt::{self, Write};
use core::panic::PanicInfo;

use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_stm32::rcc::{StopMode, WakeGuard};
use embassy_stm32::usart::{BufferedUartRx, BufferedUartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embedded_io_async::Write as _; // brings write_all/flush into scope for BufferedUartTx
use embassy_sync::channel::Channel;
use embassy_time::Instant;
use heapless::{String, Vec};
use log::{LevelFilter, Metadata, Record};
use serde::Serialize;
use tower_protocol::msg::{
    Dropped, Event, Hello, Level, Log, Print, ShellCompletions, ShellResponse,
};
use tower_protocol::{MAX_WIRE, MsgType, PROTOCOL_VERSION, encode_frame};

/// Max log-line / print-text length (clipped past this — fine for a debug console).
const MAX_MSG: usize = 192;
/// Max module-name length carried per log record.
const MOD_LEN: usize = 24;
/// Max shell-response text per frame (single-chunk responses for now).
const MAX_RESP: usize = 256;
/// Max firmware-version string in `Hello`.
const FW_LEN: usize = 32;
/// TX queue depth (frames). Sized to absorb boot-time chatter before the writer runs.
const TX_DEPTH: usize = 8;
/// Event caps: name length, per-field key/value length, and max fields (matches the
/// wire `Vec<(&str,&str), 8>`).
const EV_NAME: usize = 24;
const EV_KEY: usize = 12;
const EV_VAL: usize = 20;
const EV_FIELDS: usize = 8;

/// An owned outgoing message. The writer assigns the `seq` and encodes it, so nothing
/// borrows past the producing call and dropped frames never consume sequence numbers.
enum Outgoing {
    Hello(String<FW_LEN>),
    Log {
        level: Level,
        uptime_us: u64,
        module: String<MOD_LEN>,
        message: String<MAX_MSG>,
    },
    Print(String<MAX_MSG>),
    Event {
        name: String<EV_NAME>,
        fields: Vec<(String<EV_KEY>, String<EV_VAL>), EV_FIELDS>,
    },
    ShellResponse {
        cmd_id: u16,
        result: u8,
        text: String<MAX_RESP>,
    },
    /// Completion result — all fields borrow the `'static` command tree, so no copy.
    Completions(ShellCompletions<'static>),
}

/// Producer → writer queue. Single consumer (the writer task) owns the UART.
static TX_CHANNEL: Channel<CriticalSectionRawMutex, Outgoing, TX_DEPTH> = Channel::new();
/// Count of messages dropped because [`TX_CHANNEL`] was full; reported by the writer
/// as a [`Dropped`] marker before the next real frame.
static DROPPED: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
/// RX half, parked until the shell takes it (`take_rx`).
static PARKED_RX: Mutex<RefCell<Option<BufferedUartRx<'static>>>> = Mutex::new(RefCell::new(None));

fn bump_dropped() {
    critical_section::with(|cs| {
        let c = DROPPED.borrow(cs);
        c.set(c.get().saturating_add(1));
    });
}

fn take_dropped() -> u32 {
    critical_section::with(|cs| {
        let c = DROPPED.borrow(cs);
        let n = c.get();
        c.set(0);
        n
    })
}

/// Enqueue an outgoing message, dropping it (drop-newest) and bumping [`DROPPED`] if
/// the queue is full. Non-blocking — safe from any context (IRQ, the `log` backend).
fn try_enqueue(item: Outgoing) {
    if TX_CHANNEL.try_send(item).is_err() {
        bump_dropped();
    }
}

/// Copy a string into a bounded buffer, clipping at a char boundary (ASCII in practice).
fn clip<const N: usize>(s: &str) -> String<N> {
    let mut out = String::new();
    let mut end = s.len().min(N);
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    let _ = out.push_str(&s[..end]);
    out
}

fn map_level(l: log::Level) -> Level {
    match l {
        log::Level::Error => Level::Error,
        log::Level::Warn => Level::Warn,
        log::Level::Info => Level::Info,
        log::Level::Debug => Level::Debug,
        log::Level::Trace => Level::Trace,
    }
}

/// `log` backend: renders the record into owned buffers and enqueues a [`Log`]; the
/// host does the timestamp / colour / columns.
struct ConsoleLogger;

impl log::Log for ConsoleLogger {
    fn enabled(&self, _: &Metadata) -> bool {
        true // level filtering is handled by `log`'s max-level check
    }

    fn log(&self, record: &Record) {
        let mut message = String::<MAX_MSG>::new();
        let _ = write!(message, "{}", record.args()); // clipped if over MAX_MSG
        let module = record
            .target()
            .rsplit("::")
            .next()
            .unwrap_or(record.target());
        try_enqueue(Outgoing::Log {
            level: map_level(record.level()),
            uptime_us: Instant::now().as_micros(),
            module: clip(module),
            message,
        });
    }

    fn flush(&self) {}
}

static LOGGER: ConsoleLogger = ConsoleLogger;

/// Initialise the framed console: install the `log` backend and spawn the writer
/// task that owns `tx`. `rx` is parked for the shell. Call once at start-up.
pub fn init(
    spawner: Spawner,
    tx: BufferedUartTx<'static>,
    rx: BufferedUartRx<'static>,
    max_level: LevelFilter,
) {
    critical_section::with(|cs| {
        PARKED_RX.borrow(cs).replace(Some(rx));
    });
    // `_racy` variants: no atomic CAS on M0+. Safe — called once, single-threaded,
    // before any task or log call runs.
    unsafe {
        let _ = log::set_logger_racy(&LOGGER);
        log::set_max_level_racy(max_level);
    }
    spawner.spawn(writer_task(tx).unwrap());
}

/// Reclaim the parked RX half for the shell (async reads). Returns `None` if already
/// taken or never initialised.
pub fn take_rx() -> Option<BufferedUartRx<'static>> {
    critical_section::with(|cs| PARKED_RX.borrow(cs).borrow_mut().take())
}

/// The single UART owner. Assigns `seq`, encodes, and drains [`TX_CHANNEL`] over the
/// interrupt-buffered UART. Holds a [`WakeGuard`] across each transmit burst: the async
/// write awaits the USART TXE interrupt, which STOP would gate (the low-power executor
/// enters STOP when idle) — the guard forces a plain WFI so the USART stays clocked and
/// the interrupt fires. At the `receive().await` (truly idle) **no** guard is held, so
/// STOP is still reached when unplugged. (CONSOLE-PLAN.md §5.1.)
#[embassy_executor::task]
async fn writer_task(mut tx: BufferedUartTx<'static>) {
    // Lone 0x00 at start-up: flush any partial frame on the host's decoder.
    {
        let _g = WakeGuard::new(StopMode::Stop1);
        let _ = tx.write_all(&[0u8]).await;
        let _ = embedded_io_async::Write::flush(&mut tx).await;
    }

    let mut seq: u16 = 0;
    loop {
        let item = TX_CHANNEL.receive().await;
        // Hold STOP off across the burst so the interrupt-driven writes complete.
        let _guard = WakeGuard::new(StopMode::Stop1);

        // Report any dropped frames first (one marker before the next real frame).
        let dropped = take_dropped();
        if dropped > 0 {
            send(&mut tx, &mut seq, MsgType::Dropped, &Dropped { count: dropped }).await;
        }

        match &item {
            Outgoing::Hello(fw) => {
                send(
                    &mut tx,
                    &mut seq,
                    MsgType::Hello,
                    &Hello { protocol_version: PROTOCOL_VERSION, firmware_version: fw },
                )
                .await
            }
            Outgoing::Log { level, uptime_us, module, message } => {
                send(
                    &mut tx,
                    &mut seq,
                    MsgType::Log,
                    &Log {
                        level: *level,
                        uptime_us: *uptime_us,
                        module: module.as_str(),
                        message: message.as_str(),
                    },
                )
                .await
            }
            Outgoing::Print(text) => {
                send(&mut tx, &mut seq, MsgType::Print, &Print { text: text.as_str() }).await
            }
            Outgoing::Event { name, fields } => {
                let mut wire: Vec<(&str, &str), EV_FIELDS> = Vec::new();
                for (k, v) in fields {
                    let _ = wire.push((k.as_str(), v.as_str()));
                }
                send(&mut tx, &mut seq, MsgType::Event, &Event { name: name.as_str(), fields: wire })
                    .await
            }
            Outgoing::ShellResponse { cmd_id, result, text } => {
                send(
                    &mut tx,
                    &mut seq,
                    MsgType::ShellResponse,
                    &ShellResponse {
                        cmd_id: *cmd_id,
                        result: *result,
                        chunk: 0,
                        last: true,
                        text: text.as_str(),
                    },
                )
                .await
            }
            Outgoing::Completions(c) => {
                send(&mut tx, &mut seq, MsgType::ShellCompletions, c).await
            }
        }
        // Drain the ring (still under the guard) before idling, so the frames actually
        // leave the wire rather than sitting in the buffer when STOP is next allowed.
        let _ = embedded_io_async::Write::flush(&mut tx).await;
    }
}

/// Encode `payload` with the next `seq` and write it over the buffered UART (the caller
/// holds a `WakeGuard` for the burst).
async fn send<T: Serialize>(
    tx: &mut BufferedUartTx<'static>,
    seq: &mut u16,
    msg_type: MsgType,
    payload: &T,
) {
    let mut buf = [0u8; MAX_WIRE];
    if let Ok(n) = encode_frame(msg_type, *seq, payload, &mut buf) {
        *seq = seq.wrapping_add(1);
        let _ = tx.write_all(&buf[..n]).await;
    }
}

/// Emit a self-describing event (key=value pairs) to the host — rendered by
/// `tower events` without any per-app schema. **Async**: applies backpressure and is
/// never dropped, so call it from an async context. Extra fields beyond [`EV_FIELDS`]
/// and over-long strings are clipped.
///
/// ```ignore
/// console::event("measurement", &[("temp", "23.5"), ("rh", "41")]).await;
/// ```
pub async fn event(name: &str, fields: &[(&str, &str)]) {
    let mut f: Vec<(String<EV_KEY>, String<EV_VAL>), EV_FIELDS> = Vec::new();
    for &(k, v) in fields.iter().take(EV_FIELDS) {
        let _ = f.push((clip(k), clip(v)));
    }
    TX_CHANNEL.send(Outgoing::Event { name: clip(name), fields: f }).await;
}

/// Send a shell command response (single chunk). Async — never dropped. Used by the
/// shell engine ([`crate::shell`]).
pub async fn shell_response(cmd_id: u16, result: u8, text: &str) {
    TX_CHANNEL
        .send(Outgoing::ShellResponse { cmd_id, result, text: clip(text) })
        .await;
}

/// Send a completion result. The candidates borrow the `'static` command tree, so
/// nothing is copied. Async — never dropped. Used by the shell engine.
pub async fn shell_completions(c: tower_protocol::msg::ShellCompletions<'static>) {
    TX_CHANNEL.send(Outgoing::Completions(c)).await;
}

/// Emit the boot `Hello` (firmware string for the host header) and a banner log.
/// Called by the [`app!`](crate::app) macro right after the console comes up.
pub fn boot_banner(name: &str) {
    try_enqueue(Outgoing::Hello(clip(name)));
    log::info!(target: "boot", "booted: {}", name);
}

/// Backing function for [`print!`](crate::print)/[`println!`](crate::println).
pub fn _print(args: fmt::Arguments) {
    let mut s = String::<MAX_MSG>::new();
    let _ = write!(s, "{}", args);
    try_enqueue(Outgoing::Print(s));
}

/// Write formatted text to the console with no trailing newline.
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => { $crate::console::_print(format_args!($($arg)*)) };
}

/// Write formatted text to the console followed by a CRLF (terminal-friendly).
#[macro_export]
macro_rules! println {
    () => { $crate::console::_print(format_args!("\r\n")) };
    ($($arg:tt)*) => {
        $crate::console::_print(format_args!("{}\r\n", format_args!($($arg)*)))
    };
}

/// SDK panic handler: emit one framed error record, then halt. The executor is dead,
/// so this bypasses the channel and blocking-writes the frame straight to the USART1
/// registers via the PAC. If the console isn't up yet (USART1 disabled) it just halts.
#[panic_handler]
fn on_panic(info: &PanicInfo) -> ! {
    use embassy_stm32::pac::USART1;

    if USART1.cr1().read().ue() {
        // Silence the BufferedUart ISR so it can't race our direct register writes.
        USART1.cr1().modify(|w| {
            w.set_txeie(false);
            w.set_rxneie(false);
        });
        let mut msg = String::<MAX_MSG>::new();
        let _ = write!(msg, "{}", info);
        let payload = Log {
            level: Level::Error,
            uptime_us: 0,
            module: "panic",
            message: msg.as_str(),
        };
        let mut buf = [0u8; MAX_WIRE];
        if let Ok(n) = encode_frame(MsgType::Log, 0, &payload, &mut buf) {
            for &b in &buf[..n] {
                while !USART1.isr().read().txe() {}
                USART1.tdr().write(|w| w.set_dr(b as u16));
            }
            while !USART1.isr().read().tc() {}
        }
    }
    loop {
        cortex_m::asm::wfi();
    }
}
