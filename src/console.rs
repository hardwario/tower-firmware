//! Framed serial console — the SDK's logging + host-link primitive.
//!
//! The UART is **always framed** (`tower-protocol`): every log record, `print!`,
//! event, and the boot `Hello` is a COBS+CRC+postcard frame on USART1 TX (PA9) at
//! 115200. A raw serial monitor therefore shows binary — use the `tower` host CLI
//! (`tower logs`), which decodes the frames.
//!
//! Architecture (docs/console.md):
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
use core::ptr::addr_of_mut;

use critical_section::Mutex;
use embassy_futures::select::select3;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{PA9, PA10, USART1};
use embassy_stm32::rcc::{StopMode, WakeGuard};
use embassy_stm32::usart::{
    BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx, Config as UartConfig,
};
use embassy_stm32::{Peri, bind_interrupts};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Read as _; // brings read into scope for BufferedUartRx
use embedded_io_async::Write as _; // brings write_all/flush into scope for BufferedUartTx
use heapless::{String, Vec};
use log::{LevelFilter, Metadata, Record};
use serde::Serialize;
use tower_protocol::msg::{Dropped, Event, Hello, Level, Log, Print, ShellCompletions, ShellResponse};
use tower_protocol::{
    FrameDecoder, MAX_WIRE, MsgType, PROTOCOL_VERSION, decode_frame, encode_frame, encode_frame_raw,
};

// USART1 interrupt for the console UART. Bound here (not in `board`) because the console
// manager owns the UART and rebuilds it on every USB plug-in.
bind_interrupts!(pub struct ConsoleIrqs {
    USART1 => BufferedInterruptHandler<USART1>;
});

/// Max bytes of a routed `FotaData` payload (offset(4) + up to a manifest/chunk read).
const FOTA_CHUNK: usize = 128;
/// `FotaData` frames received on RX are decoded by the manager and routed here for the
/// FOTA host-proxy ([`crate::fota::HostProxySource`]) to consume — so the host-proxy no
/// longer needs to own the raw RX half (the dynamic console manager owns it).
pub(crate) static FOTA_DATA: Channel<CriticalSectionRawMutex, Vec<u8, FOTA_CHUNK>, 1> =
    Channel::new();

/// Max log-line / print-text length (clipped past this — fine for a debug console).
const MAX_MSG: usize = 192;
/// Max module-name length carried per log record.
const MOD_LEN: usize = 24;
/// Max shell-response text buffered per message. Longer than one frame holds — the
/// writer splits it into [`SHELL_CHUNK`]-sized frames (`chunk`/`last`), which the
/// host reassembles by `cmd_id`.
const MAX_RESP: usize = 256;
/// Shell-response text bytes per frame. Kept well under the ~240-byte payload budget
/// (a [`ShellResponse`] header is ~9 bytes) so a frame never fails to encode.
const SHELL_CHUNK: usize = 192;
/// Max firmware-version string in `Hello`.
const FW_LEN: usize = 32;
/// TX queue depth (frames). Sized to absorb boot-time chatter before the writer runs.
const TX_DEPTH: usize = 8;
/// Event caps: name length, per-field key/value length, and max fields. Sized so the
/// **worst case fits one frame**: postcard ≈ name(1+24) + count(1) + EV_FIELDS·(2 +
/// EV_KEY + EV_VAL) = 25 + 1 + 6·34 = 230 ≤ the ~249-byte payload budget. The wire
/// type allows up to 8 fields (`Vec<(&str,&str), 8>`); the firmware emits ≤ EV_FIELDS.
const EV_NAME: usize = 24;
const EV_KEY: usize = 12;
const EV_VAL: usize = 20;
const EV_FIELDS: usize = 6;

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
    /// FOTA host-proxy request (a gateway asks the host for image/manifest bytes). Sent as
    /// a **raw** frame; the reply arrives as a `FotaData` frame on RX. See `docs/fota.md`.
    FotaReq {
        offset: u32,
        len: u16,
    },
}

/// Producer → writer queue. Single consumer (the writer task) owns the UART.
static TX_CHANNEL: Channel<CriticalSectionRawMutex, Outgoing, TX_DEPTH> = Channel::new();
/// Count of messages dropped because [`TX_CHANNEL`] was full; reported by the writer
/// as a [`Dropped`] marker before the next real frame.
static DROPPED: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
/// Firmware name for the boot `Hello` (set by [`boot_banner`]); the dynamic console
/// [`manager`] re-emits `Hello` with it on every USB plug-in so the host resyncs.
static FW_NAME: Mutex<RefCell<String<FW_LEN>>> = Mutex::new(RefCell::new(String::new()));

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

/// A `fmt::Write` sink over a bounded [`String`] that **truncates** rather than
/// failing. A plain `write!` into a `heapless::String` rejects any single piece that
/// would overflow *wholesale* — so `log::info!("{}", over_long)` would log an **empty**
/// line. This sink instead writes what fits (at a char boundary) and silently drops
/// the rest, so over-long messages clip gracefully.
struct Clipper<const N: usize>(String<N>);

impl<const N: usize> Write for Clipper<N> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let room = N - self.0.len();
        if room == 0 {
            return Ok(());
        }
        let mut end = s.len().min(room);
        while end > 0 && !s.is_char_boundary(end) {
            end -= 1;
        }
        let _ = self.0.push_str(&s[..end]);
        Ok(())
    }
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
        let mut message = Clipper::<MAX_MSG>(String::new());
        let _ = write!(message, "{}", record.args()); // truncates past MAX_MSG (never empties)
        let module = record.target().rsplit("::").next().unwrap_or(record.target());
        try_enqueue(Outgoing::Log {
            level: map_level(record.level()),
            uptime_us: Instant::now().as_micros(),
            module: clip(module),
            message: message.0,
        });
    }

    fn flush(&self) {}
}

static LOGGER: ConsoleLogger = ConsoleLogger;

/// Install the `log` backend at `max_level`. Call once at start-up; the console UART
/// itself is brought up/down dynamically by [`manager`] as USB is plugged/unplugged,
/// but the backend (which only enqueues into [`TX_CHANNEL`]) is always present — when
/// no UART exists the queue simply drops (harmless).
pub fn install_logger(max_level: LevelFilter) {
    // `_racy` variants: no atomic CAS on M0+. Safe — called once, single-threaded,
    // before any task or log call runs.
    unsafe {
        let _ = log::set_logger_racy(&LOGGER);
        log::set_max_level_racy(max_level);
    }
}

// Buffers for the console UART, rebuilt on each USB plug-in. A fresh `&'static mut`
// per build is sound because [`manager`] keeps at most one `BufferedUart` alive at a
// time (its loop builds, runs, then drops before rebuilding).
static mut TX_BUF: [u8; 256] = [0; 256];
static mut RX_BUF: [u8; 128] = [0; 128];

/// USB-presence-gated **dynamic** console. Owns USART1 + PA9/PA10 for the whole run.
/// While USB is present (`VBUS_SENSE`/PA12 high) it builds the `BufferedUart` and runs
/// the framed writer + the RX frame-router; on unplug it **drops** the UART — which
/// disables USART1 and releases embassy's STOP refcount, so the low-power executor can
/// enter STOP and idle at µA — then parks on the PA12 EXTI edge (which wakes the MCU
/// out of STOP to bring the console back on plug-in). Spawned by
/// [`board::Board::take`](crate::board::Board::take).
///
/// `usart1`/`tx_pin`/`rx_pin` are held to reserve the peripherals; each build
/// re-acquires them with `steal()` (sound — exactly one instance is alive at a time).
#[embassy_executor::task]
pub async fn manager(
    usart1: Peri<'static, USART1>,
    tx_pin: Peri<'static, PA9>,
    rx_pin: Peri<'static, PA10>,
    mut vbus: ExtiInput<'static, Async>,
) {
    let _reserved = (usart1, tx_pin, rx_pin);
    loop {
        // Wait for USB present (debounced). EXTI on PA12 wakes the MCU out of STOP.
        if !vbus.is_high() {
            vbus.wait_for_high().await;
            Timer::after(Duration::from_millis(50)).await;
            if !vbus.is_high() {
                continue;
            }
        }

        // Build the console UART. SAFETY: at most one BufferedUart is alive at a time.
        let uart = unsafe {
            BufferedUart::new(
                USART1::steal(),
                PA10::steal(),
                PA9::steal(),
                &mut *addr_of_mut!(TX_BUF),
                &mut *addr_of_mut!(RX_BUF),
                ConsoleIrqs,
                UartConfig::default(),
            )
        };
        let Ok(uart) = uart else {
            Timer::after(Duration::from_millis(500)).await; // avoid a tight rebuild loop
            continue;
        };
        let (mut tx, mut rx) = uart.split();

        // Re-announce the link so the host resyncs its per-session `seq` / header.
        let name = critical_section::with(|cs| FW_NAME.borrow(cs).borrow().clone());
        try_enqueue(Outgoing::Hello(name));

        // Run the writer + RX router until USB is unplugged (debounced), then tear the
        // UART down by dropping `tx`/`rx` at the end of this scope.
        let unplug = async {
            vbus.wait_for_low().await;
            Timer::after(Duration::from_millis(50)).await;
        };
        let _ = select3(writer_loop(&mut tx), rx_loop(&mut rx), unplug).await;
        // tx/rx drop → USART1 disabled → STOP re-enabled while unplugged.
    }
}

/// Drain [`TX_CHANNEL`] over `tx` until cancelled. Holds a [`WakeGuard`] across each
/// transmit burst: the async write awaits the USART TXE interrupt, which STOP would
/// gate — the guard forces a plain WFI so the USART stays clocked and the interrupt
/// fires. Between bursts **no** guard is held (docs/console.md).
async fn writer_loop(tx: &mut BufferedUartTx<'static>) {
    // Lone 0x00 at start-up: flush any partial frame on the host's decoder.
    {
        let _g = WakeGuard::new(StopMode::Stop1);
        let _ = tx.write_all(&[0u8]).await;
        let _ = tx.flush().await;
    }

    let mut seq: u16 = 0;
    loop {
        let item = TX_CHANNEL.receive().await;
        // Hold STOP off across the burst so the interrupt-driven writes complete.
        let _guard = WakeGuard::new(StopMode::Stop1);

        // Report any dropped frames first (one marker before the next real frame).
        let dropped = take_dropped();
        if dropped > 0 {
            send(tx, &mut seq, MsgType::Dropped, &Dropped { count: dropped }).await;
        }

        match &item {
            Outgoing::Hello(fw) => {
                send(
                    tx,
                    &mut seq,
                    MsgType::Hello,
                    &Hello {
                        protocol_version: PROTOCOL_VERSION,
                        firmware_version: fw,
                    },
                )
                .await
            }
            Outgoing::Log {
                level,
                uptime_us,
                module,
                message,
            } => {
                send(
                    tx,
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
                send(tx, &mut seq, MsgType::Print, &Print { text: text.as_str() }).await
            }
            Outgoing::Event { name, fields } => {
                // Capacity matches the wire type (`Event.fields: Vec<_, 8>`); the
                // producer already capped the owned `fields` at EV_FIELDS (≤ 8).
                let mut wire: Vec<(&str, &str), 8> = Vec::new();
                for (k, v) in fields {
                    let _ = wire.push((k.as_str(), v.as_str()));
                }
                send(
                    tx,
                    &mut seq,
                    MsgType::Event,
                    &Event {
                        name: name.as_str(),
                        fields: wire,
                    },
                )
                .await
            }
            Outgoing::ShellResponse { cmd_id, result, text } => {
                send_shell_response(tx, &mut seq, *cmd_id, *result, text.as_str()).await
            }
            Outgoing::Completions(c) => send(tx, &mut seq, MsgType::ShellCompletions, c).await,
            Outgoing::FotaReq { offset, len } => {
                let mut payload = [0u8; 6];
                payload[0..4].copy_from_slice(&offset.to_le_bytes());
                payload[4..6].copy_from_slice(&len.to_le_bytes());
                send_raw(tx, &mut seq, MsgType::FotaReq, &payload).await
            }
        }
        // Drain the ring (still under the guard) before idling, so the frames actually
        // leave the wire rather than sitting in the buffer when STOP is next allowed.
        let _ = tx.flush().await;
    }
}

/// Read `rx` and route decoded frames (until cancelled): shell frames go to the
/// [`shell`](crate::shell) dispatcher; `FotaData` frames go to [`FOTA_DATA`] for the
/// FOTA host-proxy. Owning RX here (instead of the shell/host-proxy owning it) is what
/// lets the console be torn down and rebuilt across USB plug/unplug.
async fn rx_loop(rx: &mut BufferedUartRx<'static>) {
    let mut dec = FrameDecoder::new();
    let mut buf = [0u8; 64];
    loop {
        let n = match rx.read(&mut buf).await {
            Ok(n) => n,
            Err(_) => continue,
        };
        for &b in &buf[..n] {
            if let Some(inner) = dec.push(b) {
                route_frame(inner).await;
            }
        }
    }
}

/// Route one decoded inner frame by message type.
async fn route_frame(inner: &[u8]) {
    let Ok((mt, _seq, payload)) = decode_frame(inner) else {
        return;
    };
    match mt {
        MsgType::ShellCommand | MsgType::ShellComplete => {
            crate::shell::dispatch_frame(inner).await;
        }
        MsgType::FotaData => {
            let mut v: Vec<u8, FOTA_CHUNK> = Vec::new();
            let take = payload.len().min(FOTA_CHUNK);
            let _ = v.extend_from_slice(&payload[..take]);
            // Drop-oldest: replace any stale chunk so the host-proxy sees the freshest.
            if FOTA_DATA.try_send(v).is_err() {
                let _ = FOTA_DATA.try_receive();
                let mut v2: Vec<u8, FOTA_CHUNK> = Vec::new();
                let _ = v2.extend_from_slice(&payload[..take]);
                let _ = FOTA_DATA.try_send(v2);
            }
        }
        _ => {}
    }
}

/// Encode `payload` with the next `seq` and write it over the buffered UART (the caller
/// holds a `WakeGuard` for the burst). An encode failure (payload over budget) is
/// counted as a drop rather than silently lost.
async fn send<T: Serialize>(tx: &mut BufferedUartTx<'static>, seq: &mut u16, msg_type: MsgType, payload: &T) {
    let mut buf = [0u8; MAX_WIRE];
    match encode_frame(msg_type, *seq, payload, &mut buf) {
        Ok(n) => {
            *seq = seq.wrapping_add(1);
            let _ = tx.write_all(&buf[..n]).await;
        }
        Err(_) => bump_dropped(),
    }
}

/// Like [`send`] but for a **raw** (non-postcard) payload — used by `FotaReq`, whose
/// payload is a fixed binary layout the host parses without postcard.
async fn send_raw(tx: &mut BufferedUartTx<'static>, seq: &mut u16, msg_type: MsgType, payload: &[u8]) {
    let mut buf = [0u8; MAX_WIRE];
    match encode_frame_raw(msg_type, *seq, payload, &mut buf) {
        Ok(n) => {
            *seq = seq.wrapping_add(1);
            let _ = tx.write_all(&buf[..n]).await;
        }
        Err(_) => bump_dropped(),
    }
}

/// Send a shell response as one or more `chunk`-indexed frames (`last` marks the
/// final one), splitting `text` at char boundaries no larger than [`SHELL_CHUNK`] so
/// each frame fits the wire budget. Empty text still sends one (empty, `last`) frame
/// so the host always completes the response. The host reassembles by `cmd_id`.
async fn send_shell_response(
    tx: &mut BufferedUartTx<'static>,
    seq: &mut u16,
    cmd_id: u16,
    result: u8,
    text: &str,
) {
    let mut rest = text;
    let mut chunk: u16 = 0;
    loop {
        let mut take = rest.len().min(SHELL_CHUNK);
        while take > 0 && !rest.is_char_boundary(take) {
            take -= 1;
        }
        let (head, tail) = rest.split_at(take);
        let last = tail.is_empty();
        send(
            tx,
            seq,
            MsgType::ShellResponse,
            &ShellResponse {
                cmd_id,
                result,
                chunk,
                last,
                text: head,
            },
        )
        .await;
        if last {
            break;
        }
        rest = tail;
        chunk = chunk.wrapping_add(1);
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
    TX_CHANNEL
        .send(Outgoing::Event {
            name: clip(name),
            fields: f,
        })
        .await;
}

/// Send a shell command response. The writer splits `text` into `chunk`/`last` frames
/// the host reassembles, so it may exceed one frame (up to [`MAX_RESP`], clipped past
/// that). Async — never dropped. Used by the shell engine ([`crate::shell`]).
pub async fn shell_response(cmd_id: u16, result: u8, text: &str) {
    TX_CHANNEL
        .send(Outgoing::ShellResponse {
            cmd_id,
            result,
            text: clip(text),
        })
        .await;
}

/// Send a completion result. The candidates borrow the `'static` command tree, so
/// nothing is copied. Async — never dropped. Used by the shell engine.
pub async fn shell_completions(c: tower_protocol::msg::ShellCompletions<'static>) {
    TX_CHANNEL.send(Outgoing::Completions(c)).await;
}

/// Send a FOTA host-proxy request to the host (a gateway asking for image/manifest bytes).
/// Goes through the writer task as a raw `FotaReq` frame; the host replies with a `FotaData`
/// frame the caller reads off the RX half (see `fota::HostProxySource`). Async — applies
/// backpressure and is never dropped (so the request always goes out). `offset ==
/// `[`tower_protocol::fota::FOTA_MANIFEST_OFFSET`] requests the signed manifest.
pub async fn send_fota_req(offset: u32, len: u16) {
    TX_CHANNEL.send(Outgoing::FotaReq { offset, len }).await;
}

/// Record the firmware name and emit the boot `Hello` + a banner log. Called by the
/// [`app!`](crate::app) macro at start-up. The name is stored so the dynamic console
/// [`manager`] can re-emit `Hello` on each USB plug-in.
pub fn boot_banner(name: &str) {
    critical_section::with(|cs| {
        *FW_NAME.borrow(cs).borrow_mut() = clip(name);
    });
    try_enqueue(Outgoing::Hello(clip(name)));
    log::info!(target: "boot", "booted: {}", name);
}

/// Backing function for [`print!`](crate::print)/[`println!`](crate::println).
pub fn _print(args: fmt::Arguments) {
    let mut s = Clipper::<MAX_MSG>(String::new());
    let _ = write!(s, "{}", args);
    try_enqueue(Outgoing::Print(s.0));
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
            // Lead with a 0x00: any byte still in the shift register (or a partial frame
            // the silenced writer left behind) would otherwise prefix and corrupt ours.
            // The delimiter flushes the host decoder so our frame stands alone.
            for &b in core::iter::once(&0u8).chain(&buf[..n]) {
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
