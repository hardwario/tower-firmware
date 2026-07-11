//! Framed serial console — the SDK's logging + host-link primitive.
//!
//! The UART is **always framed** (`tower-protocol`): every log record, `print!`,
//! event, and the boot `Hello` is a COBS+CRC+postcard frame on USART1 TX (PA9) at
//! 115200. A raw serial monitor therefore shows binary — use the `tower` host CLI
//! (`tower logs`), which decodes the frames.
//!
//! Architecture (docs/console.md): the console is **dynamic** — [`manager`] owns USART1
//! (PA9 TX / PA10 RX) and, gated on USB presence (`VBUS_SENSE`/PA12), builds the
//! `BufferedUart` while plugged and **drops** it on unplug — releasing embassy's STOP
//! refcount so an unplugged node idles at µA. While up it splits the UART and runs two
//! halves:
//!
//! * producers (the `log` backend, `print!`, `event`, the boot `Hello`) build an owned
//!   [`Outgoing`] message and `try_send` it into [`TX_CHANNEL`] — non-blocking, drop-newest
//!   on full (a dropped count is reported by the writer); [`writer_loop`] owns the
//!   `BufferedUartTx`, assigns the per-frame `seq` (so a gap means real wire loss, not a
//!   queue drop), encodes, and async-writes — holding a `WakeGuard` across each burst so
//!   the low-power executor uses WFI (USART clocked) instead of STOP, which would gate the
//!   TXE interrupt;
//! * [`rx_loop`] reads the `BufferedUartRx` and [`route_frame`]s decoded frames into the
//!   console-owned RX channels by type — shell frames to [`SHELL_RX`]; the shell drains its
//!   channel on its own task. The console never calls up into that layer, so the transport
//!   depends only on `tower-protocol`;
//! * the **panic** path can't use the (dead) executor, so it silences the buffered ISR
//!   and blocking-writes one frame straight to the USART1 registers via the PAC.

use core::cell::{Cell, RefCell};
use core::fmt::{self, Write};
use core::panic::PanicInfo;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use critical_section::Mutex;
use embassy_futures::select::{select, select3};
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
use tower_protocol::msg::{
    Dropped, Event, Hello, Level, Log, MgmtResponse, Print, RadioStat, ShellCompletions, ShellResponse,
    Uplink,
};
use tower_protocol::{FrameDecoder, MAX_WIRE, MsgType, PROTOCOL_VERSION, decode_frame, encode_frame};

use crate::storage::{NS_SYS, Nv};

// USART1 interrupt for the console UART. Bound here (not in `board`) because the console
// manager owns the UART and rebuilds it on every USB plug-in.
bind_interrupts!(pub struct ConsoleIrqs {
    USART1 => BufferedInterruptHandler<USART1>;
});

/// Max log-line / print-text length (clipped past this — fine for a debug console).
const MAX_MSG: usize = 192;
/// Max module-name length carried per log record.
const MOD_LEN: usize = 24;
/// Max shell-response text buffered per message. Longer than one frame holds — the
/// writer splits it into [`SHELL_CHUNK`]-sized frames (`chunk`/`last`), which the
/// host reassembles by `cmd_id`. Must equal [`crate::shell::RESP_CAP`] (asserted): if
/// this shrank alone, responses would silently clip AFTER the shell already computed
/// its `R_OK`/`R_TRUNCATED` verdict — a "complete" `/export` that isn't.
const MAX_RESP: usize = 256;
const _: () = assert!(MAX_RESP == crate::shell::RESP_CAP);
/// Shell-response text bytes per frame. Kept well under the ~240-byte payload budget
/// (a [`ShellResponse`] header is ~9 bytes) so a frame never fails to encode.
const SHELL_CHUNK: usize = 192;
/// Max firmware-name string in `Hello` (the baked-in app/example name).
const FW_LEN: usize = 32;
/// Firmware version string carried in `Hello` — the SDK crate version with a leading `v`
/// (e.g. `"v0.1.0"`), baked at compile time. Distinct from [`FW_NAME`] (the app name).
const FW_VERSION: &str = concat!("v", env!("CARGO_PKG_VERSION"));
/// `NS_SYS` local key for the persisted boot counter (the `Hello` session_id).
const KEY_BOOT_COUNT: u8 = 0x00;
/// TX queue depth (frames). 4 (was 8): each slot is a ~280 B `Outgoing`, and on the
/// 20 KB part every slot is stack the gateway/push-button products can't spare (the
/// measured `Net`-app stack peak is ~9 KB — see the stack-overflow history). The cost
/// is honest and accounted: a boot burst deeper than 4 frames drops-newest and is
/// reported by the writer's [`Dropped`] marker; the async producers ([`event`],
/// [`uplink`], …) apply backpressure instead of dropping while the console is up.
const TX_DEPTH: usize = 4;
/// Host→device RX-copy capacity for the [`SHELL_RX`]/[`MGMT_RX`] channels. The largest
/// legitimate inner frame is far below `MAX_WIRE`: a `ShellCommand` line caps at the
/// shell's 96 bytes (~110 B framed) and the biggest `MgmtRequest` (a full-MTU
/// `QueuePush`) is ~100 B. A frame larger than this fails the copy wholesale
/// (`heapless` extend rejects, the empty copy is dropped) — same outcome as the
/// shell's own over-long-line rejection, for 112 fewer bytes per queue slot.
/// Public: [`mgmt_next`] hands these buffers to apps.
pub const RX_COPY: usize = 160;
/// Event caps: name length, per-field key/value length, and max fields. Sized so the
/// **worst case fits one frame**: postcard ≈ name(1+24) + count(1) + EV_FIELDS·(2 +
/// EV_KEY + EV_VAL) = 25 + 1 + 6·34 = 230 ≤ the ~249-byte payload budget. The wire
/// type allows up to 8 fields (`Vec<(&str,&str), 8>`); the firmware emits ≤ EV_FIELDS.
const EV_NAME: usize = 24;
const EV_KEY: usize = 12;
const EV_VAL: usize = 20;
const EV_FIELDS: usize = 6;
/// Max forwarded-uplink payload — the radio MTU (`tower_protocol::radio::MAX_RADIO_PAYLOAD`;
/// equality is static-asserted in `radio::frame`).
const UPLINK_MAX: usize = tower_protocol::radio::MAX_RADIO_PAYLOAD;
/// Management-response record bytes per frame. Mirrors [`SHELL_CHUNK`]'s reasoning: a
/// [`MgmtResponse`] header is ~8 bytes, so 192 stays well under the ~249-byte payload
/// budget and a frame never fails to encode.
const MGMT_CHUNK: usize = 192;

/// An owned outgoing message. The writer assigns the `seq` and encodes it, so nothing
/// borrows past the producing call and dropped frames never consume sequence numbers.
enum Outgoing {
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
    /// One decrypted radio uplink, forwarded verbatim by a gateway app (data ≤ the
    /// radio MTU, so it always fits one frame).
    Uplink {
        src: u32,
        counter: u32,
        rssi_dbm: i16,
        lqi: u8,
        data: Vec<u8, UPLINK_MAX>,
    },
    /// One pre-chunked management-response frame. Chunking is **app-driven** (the app
    /// knows its record boundaries and streams them without a big reassembly buffer);
    /// each enqueued item is exactly one wire frame.
    MgmtChunk {
        req_id: u16,
        result: u8,
        chunk: u16,
        last: bool,
        data: Vec<u8, MGMT_CHUNK>,
    },
    /// One radio-diagnostics sample (owned, tiny).
    RadioStat(RadioStat),
}

/// Producer → writer queue. Single consumer (the writer task) owns the UART.
static TX_CHANNEL: Channel<CriticalSectionRawMutex, Outgoing, TX_DEPTH> = Channel::new();
/// Decoded host→device **shell** frames (ShellCommand / ShellComplete), copied whole for the
/// shell to drain on its own task ([`crate::shell::serve_ext`] spawns it). The console owns this
/// RX buffer and never calls into the shell, so the transport layer doesn't depend on the
/// (higher) shell layer. [`RX_COPY`]-sized — every legitimate shell frame fits.
pub(crate) static SHELL_RX: Channel<CriticalSectionRawMutex, Vec<u8, RX_COPY>, 2> = Channel::new();
/// Decoded host→device **management** frames (MgmtRequest), copied whole for the app to drain
/// via [`mgmt_next`]/[`mgmt_try_next`] on its own loop — same ownership story as [`SHELL_RX`]
/// (the console never calls up into the app). Depth 1 (management is strictly
/// request/response — one outstanding op; a drop just makes the host retry, and the
/// 20 KB part wants every spare `MAX_WIRE` buffer back for stack).
static MGMT_RX: Channel<CriticalSectionRawMutex, Vec<u8, RX_COPY>, 1> = Channel::new();
/// Whether the console is up (USB present, UART built, [`writer_loop`] draining). Set by
/// [`manager`] around the writer's lifetime. The "async, never dropped" producers
/// ([`event`], [`shell_response`], …) apply queue backpressure only while this is true; while
/// it is false they fall back to drop-newest, so a task emitting events on a battery node with
/// USB unplugged is never parked forever on a full queue that has no writer to drain it.
static CONSOLE_UP: AtomicBool = AtomicBool::new(false);
/// Count of messages dropped because [`TX_CHANNEL`] was full; reported by the writer
/// as a [`Dropped`] marker before the next real frame.
static DROPPED: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
/// Firmware name for the boot `Hello` (set by [`boot_banner`]); the dynamic console
/// [`manager`] re-emits `Hello` with it on every USB plug-in so the host resyncs.
static FW_NAME: Mutex<RefCell<String<FW_LEN>>> = Mutex::new(RefCell::new(String::new()));
/// Per-boot session id carried in `Hello` — a persisted counter bumped once per boot by
/// [`init_session`], so the host can tell a device reboot from a continuous link.
static SESSION_ID: AtomicU32 = AtomicU32::new(0);
/// Tick timestamp (low 32 bits of `Instant`) of the last decoded host→device frame — the
/// storage maintenance task defers EEPROM slices while the host is actively talking (a CPU
/// stall would eat in-flight RX bytes; see `storage::maintenance`). Wrapping-compared, so the
/// ~36 h wrap at 32 kHz ticks is harmless.
static LAST_HOST_RX: AtomicU32 = AtomicU32::new(0);

/// Whether the console is currently up (USB present, UART built). Cheap atomic read.
pub fn is_up() -> bool {
    CONSOLE_UP.load(Ordering::Relaxed)
}

/// Ticks since the last decoded host→device frame (u32::MAX before the first one).
pub fn host_rx_age_ticks() -> u32 {
    let last = LAST_HOST_RX.load(Ordering::Relaxed);
    if last == 0 {
        return u32::MAX;
    }
    (Instant::now().as_ticks() as u32).wrapping_sub(last)
}

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

/// Enqueue an outgoing message for the "async, never dropped" producers ([`event`],
/// [`shell_response`], [`shell_completions`]).
///
/// While the console is up ([`CONSOLE_UP`]) this applies real backpressure — awaiting a free
/// slot so a burst is never dropped. While it is **down** it falls back to non-blocking
/// drop-newest: with no [`writer_loop`] draining [`TX_CHANNEL`], a plain `send().await` on a
/// full queue would park the calling task until the next USB plug-in, silently stalling e.g. a
/// battery node's measure loop. If USB is unplugged *during* a backpressured send, [`manager`]'s
/// unplugged poll drains the queue (counting drops), which frees a slot and wakes the parked
/// sender within one poll cycle — so the stall is bounded, never indefinite.
async fn enqueue(item: Outgoing) {
    if CONSOLE_UP.load(Ordering::Relaxed) {
        TX_CHANNEL.send(item).await;
    } else {
        try_enqueue(item);
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
        // Wait for USB present. The PA12 EXTI edge wakes the low-power executor out of
        // STOP the instant VBUS rises; the periodic re-check is a fallback for a missed
        // edge — the FT231X (which drives PA12 via CBUS3) only asserts it tens of ms
        // after power-up, i.e. after the executor has already armed the wait, so relying
        // on the edge alone can hang. The ~500 ms RTC wake while unplugged costs sub-µA.
        //
        // Each wake also re-asserts the STOP power tuning: embassy's `exit_stop` re-inits
        // RCC on wake and its full-register PWR_CR write clears LPSDSR/ULP, so re-apply
        // before re-entering STOP. This poll is the idle wake source, so idle STOP always
        // re-enters with the low-power regulator + VREFINT-off bits set.
        while !vbus.is_high() {
            let _ = select(vbus.wait_for_high(), Timer::after(Duration::from_millis(500))).await;
            crate::board::apply_stop_tuning();
            // Drain anything left in the queue while there's no writer (counting it as dropped):
            // frees slots so a producer that raced the unplug (checked CONSOLE_UP just before it
            // cleared) can't stay parked on a full queue — it wakes within one poll cycle.
            while TX_CHANNEL.try_receive().is_ok() {
                bump_dropped();
            }
        }
        Timer::after(Duration::from_millis(50)).await; // debounce
        if !vbus.is_high() {
            continue;
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

        // Console is up: the writer below drains TX_CHANNEL, so producers may now apply
        // backpressure (see `enqueue`). Set this before the Hello so it takes the same path.
        CONSOLE_UP.store(true, Ordering::Relaxed);

        // The writer synthesizes the session `Hello` as its own first frame (see `writer_loop`),
        // so the host resyncs its per-link `seq` on every plug-in WITHOUT the Hello ever riding
        // the shared producer queue — where a burst of logs racing the plug-in could drop it
        // (drop-newest) and strand the host on a stale seq (docs/console.md).

        // Run the writer + RX router until USB is unplugged (debounced), then tear the
        // UART down by dropping `tx`/`rx` at the end of this scope.
        let unplug = async {
            vbus.wait_for_low().await;
            Timer::after(Duration::from_millis(50)).await;
        };
        let _ = select3(writer_loop(&mut tx), rx_loop(&mut rx), unplug).await;
        // Console is down: no writer will drain the queue until the next plug-in, so producers
        // must fall back to drop-newest (see `enqueue`). Clear this before dropping the UART.
        CONSOLE_UP.store(false, Ordering::Relaxed);
        // tx/rx drop → USART1 disabled → STOP re-enabled while unplugged.
    }
}

/// Scope guard that counts one dropped frame if destroyed while still armed. [`writer_loop`] arms
/// it after dequeuing an item so that if the writer is cancelled (USB unplug via `select3`) before
/// the item reaches the UART, the loss is still reflected in the host's Dropped accounting; it is
/// disarmed (`.0 = false`) once the item's bytes are in the UART ring.
struct OnCancelDrop(bool);

impl Drop for OnCancelDrop {
    fn drop(&mut self) {
        if self.0 {
            bump_dropped();
        }
    }
}

/// Drain [`TX_CHANNEL`] over `tx` until cancelled. Holds a [`WakeGuard`] across each
/// transmit burst: the async write awaits the USART TXE interrupt, which STOP would
/// gate — the guard forces a plain WFI so the USART stays clocked and the interrupt
/// fires. Between bursts **no** guard is held (docs/console.md).
async fn writer_loop(tx: &mut BufferedUartTx<'static>) {
    let mut seq: u16 = 0;

    // The writer emits the session's opening frames ITSELF — not via the shared producer queue —
    // so a burst of logs racing a plug-in can never bump the `Hello` out of a full TX_CHANNEL
    // (drop-newest). The host relies on decoding a fresh Hello to reset its per-link `seq`
    // tracking (docs/console.md); dropping it would strand the host on a stale seq and print a
    // spurious wire-loss warning. Lead with a lone 0x00 to flush any partial frame left on the
    // host's decoder, then the Hello as seq 0.
    //
    // KNOWN LIMITATION (measured 2026-07-05, wire probe): while a chatty boot backlog drains
    // right after this Hello (a full TX_CHANNEL ≈ 500 B ≈ 45 ms at 115200), host→device bytes
    // can be lost to receiver overrun — a ShellCommand sent within ~60 ms of the Hello vanishes
    // (CRC-dropped from a partial frame), while a quiet firmware (blinky) accepts at Hello+5 ms.
    // The host CLI guards this with a post-Hello settle (tower-cli session::POST_HELLO_GUARD);
    // a firmware-side fix would need the RX ISR to survive the TX flood (ORE handling/rings).
    {
        let _g = WakeGuard::new(StopMode::Stop1);
        let _ = tx.write_all(&[0u8]).await;
        let _ = tx.flush().await;
        let name = firmware_name();
        send(
            tx,
            &mut seq,
            MsgType::Hello,
            &Hello {
                protocol_version: PROTOCOL_VERSION,
                firmware_name: &name,
                firmware_version: FW_VERSION,
                session_id: session_id(),
            },
        )
        .await;
    }

    loop {
        let item = TX_CHANNEL.receive().await;
        // Hold STOP off across the burst so the interrupt-driven writes complete.
        let _guard = WakeGuard::new(StopMode::Stop1);
        // Count this dequeued item as dropped if the writer is cancelled (USB unplug via the
        // manager's `select3`) before it is handed to the UART — keeps the host's Dropped
        // accounting exact even for the "never dropped" backpressured producers. Disarmed once
        // the item's bytes are written into the UART ring.
        let mut cancel_guard = OnCancelDrop(true);

        // Report any dropped frames first (one marker before the next real frame).
        let dropped = take_dropped();
        if dropped > 0 {
            send(tx, &mut seq, MsgType::Dropped, &Dropped { count: dropped }).await;
        }

        match &item {
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
            Outgoing::Print(text) => send(tx, &mut seq, MsgType::Print, &Print { text: text.as_str() }).await,
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
            Outgoing::Uplink {
                src,
                counter,
                rssi_dbm,
                lqi,
                data,
            } => {
                send(
                    tx,
                    &mut seq,
                    MsgType::Uplink,
                    &Uplink {
                        src: *src,
                        counter: *counter,
                        rssi_dbm: *rssi_dbm,
                        lqi: *lqi,
                        data: data.as_slice(),
                    },
                )
                .await
            }
            Outgoing::MgmtChunk {
                req_id,
                result,
                chunk,
                last,
                data,
            } => {
                send(
                    tx,
                    &mut seq,
                    MsgType::MgmtResponse,
                    &MgmtResponse {
                        req_id: *req_id,
                        result: *result,
                        chunk: *chunk,
                        last: *last,
                        data: data.as_slice(),
                    },
                )
                .await
            }
            Outgoing::RadioStat(stat) => send(tx, &mut seq, MsgType::RadioStat, stat).await,
        }
        cancel_guard.0 = false; // item's bytes are in the UART ring — not a drop
        // Drain the ring (still under the guard) before idling, so the frames actually
        // leave the wire rather than sitting in the buffer when STOP is next allowed.
        let _ = tx.flush().await;
    }
}

/// Read `rx` and route decoded frames (until cancelled): shell frames go to [`SHELL_RX`], a
/// console-owned channel the shell drains on its own task. Owning RX here (instead of the
/// shell owning it) is what lets the console be torn down and rebuilt across USB plug/unplug.
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

/// Route one decoded inner frame by message type. The console (the lowest, transport layer)
/// does **not** call up into the shell: it copies each frame into the console-owned RX
/// channel for that traffic ([`SHELL_RX`]) and the higher layer drains it on its own task.
/// So this module depends only on `tower-protocol`, and adding a new host→device consumer
/// means adding a channel here, not a hard call into another subsystem.
async fn route_frame(inner: &[u8]) {
    let Ok((mt, _seq, _payload)) = decode_frame(inner) else {
        return;
    };
    // Non-zero by construction at boot+1 tick; 0 is reserved as "never" in host_rx_age_ticks.
    let now = Instant::now().as_ticks() as u32;
    LAST_HOST_RX.store(if now == 0 { 1 } else { now }, Ordering::Relaxed);
    match mt {
        MsgType::ShellCommand | MsgType::ShellComplete => {
            // Copy the whole frame for the shell to re-decode on its drain task. Depth-2 queue;
            // a request/response shell sends one command at a time, so it rarely holds >1, and a
            // drop (queue full) just makes the host retry — never stalls the RX loop. An
            // over-RX_COPY frame fails the copy wholesale (empty Vec, dropped below) — no
            // legitimate frame is that big (see RX_COPY).
            let mut v: Vec<u8, RX_COPY> = Vec::new();
            if v.extend_from_slice(inner).is_ok() {
                let _ = SHELL_RX.try_send(v);
            }
        }
        MsgType::MgmtRequest => {
            let mut v: Vec<u8, RX_COPY> = Vec::new();
            if v.extend_from_slice(inner).is_ok() {
                let _ = MGMT_RX.try_send(v);
            }
        }
        _ => {}
    }
}

/// Await the next host→device management frame (a whole inner frame; re-decode with
/// `decode_frame` + `postcard::from_bytes::<MgmtRequest>`). Apps that serve the
/// management channel (the gateway; nodes while on USB for cable pairing) drain this
/// on their own loop.
pub async fn mgmt_next() -> Vec<u8, RX_COPY> {
    MGMT_RX.receive().await
}

/// Non-blocking [`mgmt_next`] — for a main loop that polls between radio slices.
pub fn mgmt_try_next() -> Option<Vec<u8, RX_COPY>> {
    MGMT_RX.try_receive().ok()
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
/// `tower events` without any per-app schema. **Async**: while USB is present it applies
/// backpressure (never dropped); while unplugged — no host, no writer — it drops (counted in
/// the [`Dropped`] marker) rather than parking the caller, so a battery node's measure loop
/// keeps running. Extra fields beyond [`EV_FIELDS`] and over-long strings are clipped.
///
/// ```ignore
/// console::event("measurement", &[("temp", "23.5"), ("rh", "41")]).await;
/// ```
pub async fn event(name: &str, fields: &[(&str, &str)]) {
    let mut f: Vec<(String<EV_KEY>, String<EV_VAL>), EV_FIELDS> = Vec::new();
    for &(k, v) in fields.iter().take(EV_FIELDS) {
        let _ = f.push((clip(k), clip(v)));
    }
    enqueue(Outgoing::Event {
        name: clip(name),
        fields: f,
    })
    .await;
}

/// Send a shell command response. The writer splits `text` into `chunk`/`last` frames
/// the host reassembles, so it may exceed one frame (up to [`MAX_RESP`], clipped past
/// that). Async — backpressured while USB is present (a shell command only arrives over a
/// live console, so the writer is up to drain it). Used by the shell engine ([`crate::shell`]).
pub async fn shell_response(cmd_id: u16, result: u8, text: &str) {
    enqueue(Outgoing::ShellResponse {
        cmd_id,
        result,
        text: clip(text),
    })
    .await;
}

/// Send a completion result. The candidates borrow the `'static` command tree, so
/// nothing is copied. Async — never dropped. Used by the shell engine.
pub async fn shell_completions(c: tower_protocol::msg::ShellCompletions<'static>) {
    enqueue(Outgoing::Completions(c)).await;
}

/// Forward one decrypted, authenticated radio uplink to the host, verbatim — the
/// gateway app's bridge primitive. `data` past the radio MTU is clipped (cannot
/// happen for a frame that came out of the net layer). Async — backpressured while
/// USB is present (a gateway is USB-powered by definition, so this is the normal
/// path); drop-newest while down, counted in the [`Dropped`] marker.
pub async fn uplink(src: u32, counter: u32, rssi_dbm: i16, lqi: u8, data: &[u8]) {
    let mut v: Vec<u8, UPLINK_MAX> = Vec::new();
    let _ = v.extend_from_slice(&data[..data.len().min(UPLINK_MAX)]);
    enqueue(Outgoing::Uplink {
        src,
        counter,
        rssi_dbm,
        lqi,
        data: v,
    })
    .await;
}

/// Send one management-response chunk (exactly one wire frame). Chunking is the
/// caller's job: stream records in ≤ [`MGMT_CHUNK`]-byte (192) `data` pieces with a
/// running `chunk` index and `last` on the final one; `result` is authoritative only
/// on that last chunk (mirror of the shell-response discipline). A result-only reply
/// is a single empty `last` chunk. Async — backpressured while USB is present.
pub async fn mgmt_chunk(req_id: u16, result: u8, chunk: u16, last: bool, data: &[u8]) {
    let mut v: Vec<u8, MGMT_CHUNK> = Vec::new();
    let _ = v.extend_from_slice(&data[..data.len().min(MGMT_CHUNK)]);
    enqueue(Outgoing::MgmtChunk {
        req_id,
        result,
        chunk,
        last,
        data: v,
    })
    .await;
}

/// Send one radio-diagnostics sample ([`RadioStat`]) for the host's running channel
/// graph. Async — backpressured while USB is present.
pub async fn radio_stat(stat: RadioStat) {
    enqueue(Outgoing::RadioStat(stat)).await;
}

/// Record the firmware name and emit the boot banner log. Called by the [`app!`](crate::app)
/// macro at start-up. The name is stored so the console [`writer_loop`] emits the session `Hello`
/// as its own first frame on every plug-in (it does not ride the producer queue).
pub fn boot_banner(name: &str) {
    critical_section::with(|cs| {
        *FW_NAME.borrow(cs).borrow_mut() = clip(name);
    });
    log::info!(target: "boot", "booted: {}", name);
}

/// The firmware name recorded by [`boot_banner`] — the app/example name carried in `Hello` and
/// shown by `tower`'s connect banner. The shell's `resource` command reports this so its
/// "firmware:" line matches the boot `Hello` (they previously disagreed: Hello carried the name
/// while the shell printed the SDK crate version, identical for every app).
pub fn firmware_name() -> String<FW_LEN> {
    critical_section::with(|cs| FW_NAME.borrow(cs).borrow().clone())
}

/// The baked-in firmware version string carried in `Hello` (SDK crate version, `v`-prefixed).
pub fn firmware_version() -> &'static str {
    FW_VERSION
}

/// The per-boot session id carried in `Hello` (0 until [`init_session`] runs).
pub fn session_id() -> u32 {
    SESSION_ID.load(Ordering::Relaxed)
}

/// Read, increment, and persist the per-boot session counter once at start-up (the `Hello`
/// `session_id`). Called by the [`app!`](crate::app) macro after the KV store is installed and
/// before the console emits its first `Hello`. Best-effort: if the store can't be read the
/// counter restarts at 1, and a failed persist just means two boots may share an id — the id is
/// a reboot *hint* for the host, not a security guarantee (that is the radio TX-counter's job).
///
/// **Boot-loop backoff:** the [`bootguard`](crate::bootguard) tracks consecutive fast resets in
/// reset-surviving RAM. Once a unit is judged to be looping, this stops writing the counter to
/// EEPROM (reporting a RAM-derived id instead) so a wedged/brown-out-looping node can't grind the
/// store one record per reset (see `docs/storage.md`).
///
/// This append used to be the classic trigger of the multi-second compaction stall (the "~5 s
/// hung boot" every Nth reboot). With the default [`storage::maintenance`](crate::storage::maintenance)
/// task the store is compacted incrementally *before* it fills, so in steady state this write is
/// just one ~12-byte append (a few word-programs); the synchronous flip remains only as the
/// never-ran-maintenance fallback (docs/storage.md).
pub fn init_session(kv: Nv, spawner: crate::Spawner) {
    let resets = crate::bootguard::on_boot(spawner);
    let sys = kv.scope(NS_SYS);
    let stored = sys.get::<u32>(KEY_BOOT_COUNT).ok().flatten().unwrap_or(0);
    if crate::bootguard::is_looping() {
        // Looping: do NOT touch EEPROM. Report a RAM-derived id so the host still sees it move
        // per reset, without persisting a record each time.
        SESSION_ID.store(stored.wrapping_add(resets), Ordering::Relaxed);
    } else {
        let next = stored.wrapping_add(1);
        let _ = sys.set::<u32>(KEY_BOOT_COUNT, &next);
        SESSION_ID.store(next, Ordering::Relaxed);
    }
}

/// Await the writer draining every queued frame, then a short tail for the last frame to clear
/// the UART. A caller that resets the MCU right after emitting a final response (the shell's
/// `/system reboot`) uses this so the reset can't cut the response off mid-frame — the old fixed
/// 150 ms sleep was shorter than a full `TX_CHANNEL` at 115200 baud (~190 ms). Bounded so an
/// absent/wedged writer (console down) can't hang the caller.
pub async fn flush() {
    for _ in 0..200 {
        if TX_CHANNEL.is_empty() {
            break;
        }
        Timer::after(Duration::from_millis(5)).await;
    }
    // The last dequeued frame may still be in the UART ring/shift register; a full ~270-byte
    // frame is ~24 ms at 115200 8N1.
    Timer::after(Duration::from_millis(30)).await;
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

/// Blocking-write one framed `Error` [`Log`] straight to the USART1 registers via the PAC,
/// bypassing the (dead) executor and the channel. Shared by the [`on_panic`] handler and the
/// `HardFault` exception — both run with the executor stopped and interrupts effectively ours.
/// If the console UART isn't up (USART1 disabled — USB not attached), it's a no-op: there is
/// nowhere to send. `uptime_us` is the real timebase (the time driver is a peripheral, still
/// live here) so the crash time is preserved, not reported as 0.
fn blocking_emit_error(module: &str, message: &str) {
    use embassy_stm32::pac::USART1;

    if !USART1.cr1().read().ue() {
        return;
    }
    // Silence the BufferedUart ISR so it can't race our direct register writes.
    USART1.cr1().modify(|w| {
        w.set_txeie(false);
        w.set_rxneie(false);
    });
    let payload = Log {
        level: Level::Error,
        uptime_us: Instant::now().as_micros(),
        module,
        message,
    };
    let mut buf = [0u8; MAX_WIRE];
    if let Ok(n) = encode_frame(MsgType::Log, 0, &payload, &mut buf) {
        // Lead with a 0x00: any byte still in the shift register (or a partial frame the
        // silenced writer left behind) would otherwise prefix and corrupt ours. The delimiter
        // flushes the host decoder so our frame stands alone.
        for &b in core::iter::once(&0u8).chain(&buf[..n]) {
            while !USART1.isr().read().txe() {}
            USART1.tdr().write(|w| w.set_dr(b as u16));
        }
        while !USART1.isr().read().tc() {}
    }
}

/// SDK panic handler: write the reset-surviving crash breadcrumb, emit one framed error record
/// (if the console is up — the USB-attached dev case), then **reset**. A field unit recovers
/// service instead of hanging until a battery pull; the breadcrumb re-surfaces the crash on the
/// next boot ([`emit_crash_report`]), so nothing is lost by rebooting. A crash loop is bounded
/// by the [`bootguard`](crate::bootguard): after `BOOT_LOOP_THRESHOLD` fast resets the SDK stops
/// grinding per-boot EEPROM state, and the run length is visible in `/system/eeprom print`.
#[panic_handler]
fn on_panic(info: &PanicInfo) -> ! {
    let mut msg = String::<MAX_MSG>::new();
    let _ = write!(msg, "{}", info);
    crate::crashlog::record(crate::crashlog::Kind::Panic, msg.as_str());
    blocking_emit_error("panic", msg.as_str());
    cortex_m::peripheral::SCB::sys_reset()
}

/// HardFault handler: same policy as [`on_panic`] — breadcrumb (with the faulting PC/LR), one
/// blocking framed report when the console is up, then reset. (cortex-m-rt's default handler
/// would spin silently.) The post-reset watchdog question is settled: [`crate::watchdog`] is
/// STOP-aware (its feeder wakes the low-power executor), so apps arm it for the *hang* case,
/// while this path covers the *fault* case.
#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    let mut msg = String::<MAX_MSG>::new();
    let _ = write!(msg, "HardFault pc={:#010x} lr={:#010x}", ef.pc(), ef.lr());
    crate::crashlog::record(crate::crashlog::Kind::HardFault, msg.as_str());
    blocking_emit_error("fault", msg.as_str());
    cortex_m::peripheral::SCB::sys_reset()
}

/// Report the previous boot's crash, if the reset-surviving breadcrumb holds one — one ERROR
/// frame on the `crash` module, emitted through the normal console queue once the manager is
/// up. Called by the [`app!`](crate::app) macro right after the boot banner; also caches the
/// record for `/system/crash print`.
pub fn emit_crash_report() {
    if let Some(c) = crate::crashlog::take() {
        log::error!(
            target: "crash",
            "previous boot crashed ({}): {}",
            c.kind.as_str(),
            c.message()
        );
    }
}
