//! Serial debug console + logging backend — the SDK's logging primitive.
//!
//! Owns the UART transmitter behind a global so any module can emit text with
//! the [`print!`](crate::print)/[`println!`](crate::println) macros, or — the
//! usual way — through the [`log`] facade (`info!`/`warn!`/`error!`/`debug!`/
//! `trace!`), which this module renders as `[<uptime>] <LEVEL> <message>`.
//! On the Core Module this is USART1 TX (PA9) at 115200 8N1, driven in
//! **blocking** mode (poll TXE) — simple and usable from any context.
//!
//! Wiring: call [`init`] once during start-up with the desired max level, then
//! `info!("...")` anywhere. Level filtering is at runtime via
//! [`log::set_max_level`]; for production you can also strip levels at compile
//! time with log's `release_max_level_*` cargo features.
//!
//! Concurrency note: each write takes a critical section for its whole duration,
//! so a long line briefly blocks interrupts (~87 µs/byte at 115200). That is fine
//! for a debug console; a future revision can move to a DMA/async logger task if
//! interrupt latency ever matters.

use core::cell::RefCell;
use core::fmt::{self, Write};
use core::panic::PanicInfo;

use critical_section::Mutex;
use embassy_stm32::mode::Blocking;
use embassy_stm32::usart::UartTx;
use embassy_time::Instant;
use log::{LevelFilter, Metadata, Record};

/// Newtype wrapper so we can implement `core::fmt::Write` for the HAL's blocking
/// `UartTx` (the orphan rule forbids implementing it on the foreign type directly).
struct Console(UartTx<'static, Blocking>);

impl Write for Console {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        // Map the HAL error to `fmt::Error` so a failed line aborts quietly
        // rather than panicking the logger.
        self.0.blocking_write(s.as_bytes()).map_err(|_| fmt::Error)
    }
}

impl Console {
    /// Block until the UART has fully shifted out every byte written so far.
    ///
    /// `blocking_write` returns once the last byte is in the data register, not
    /// once it has left the wire. On this low-power firmware the caller usually
    /// `await`s right after logging, and the executor may enter STOP — which
    /// gates the UART clock and would truncate that final byte (dropping the
    /// trailing newline, corrupting the next line). Flushing to transmission-
    /// complete before we return makes STOP-after-log safe.
    fn flush(&mut self) {
        let _ = self.0.blocking_flush();
    }
}

static CONSOLE: Mutex<RefCell<Option<Console>>> = Mutex::new(RefCell::new(None));

/// `log` backend: renders each record as `[<secs>.<ms>] <LEVEL> <message>`,
/// where the timestamp is the monotonic uptime (survives STOP via the RTC).
struct ConsoleLogger;

impl log::Log for ConsoleLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        // Level filtering is handled by `log`'s max-level check before `log()`.
        true
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        let us = Instant::now().as_micros();
        let secs = us / 1_000_000;
        let ms = (us % 1_000_000) / 1_000;
        // Module tag = last segment of the record's target. The target defaults
        // to the module path (e.g. `tower::power` -> `power`), so logs
        // are auto-prefixed by module; override per call with `info!(target: …)`.
        let module = record
            .target()
            .rsplit("::")
            .next()
            .unwrap_or(record.target());
        _print(format_args!(
            "[{:>5}.{:03}] {:<5} {}: {}\r\n",
            secs,
            ms,
            record.level(),
            module,
            record.args()
        ));
    }

    fn flush(&self) {}
}

static LOGGER: ConsoleLogger = ConsoleLogger;

/// Install `tx` as the global console and the [`log`] backend with `max_level`.
/// Call once during initialisation; any logging before this is a silent no-op.
pub fn init(tx: UartTx<'static, Blocking>, max_level: LevelFilter) {
    critical_section::with(|cs| {
        CONSOLE.borrow(cs).replace(Some(Console(tx)));
    });
    // The atomic `set_logger`/`set_max_level` are unavailable on Cortex-M0+
    // (no atomic CAS), so use the `_racy` variants. Safe here: called once at
    // start-up, single-threaded, before any task or log call runs.
    unsafe {
        let _ = log::set_logger_racy(&LOGGER);
        log::set_max_level_racy(max_level);
    }
}

/// Emit the uniform startup banner naming the running example/app. Called by the
/// [`app!`](crate::app) macro right after the console comes up, so every app logs
/// its start identically: `Example booted: <name>`.
pub fn boot_banner(name: &str) {
    log::info!(target: "boot", "Example booted: {}", name);
}

/// Backing function for the [`print!`](crate::print)/[`println!`](crate::println)
/// macros. Not intended to be called directly.
pub fn _print(args: fmt::Arguments) {
    critical_section::with(|cs| {
        if let Some(console) = CONSOLE.borrow(cs).borrow_mut().as_mut() {
            let _ = console.write_fmt(args);
            // Drain before returning so a STOP entry right after can't cut the
            // last byte (see `Console::flush`).
            console.flush();
        }
    });
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

/// SDK panic handler: print the panic (message + location) to the console, then
/// halt. The whole binary inherits this — examples don't define their own.
///
/// Uses `try_borrow_mut` so a panic that happens *while* a log line is being
/// written degrades to a silent halt instead of double-panicking. If a panic
/// occurs before [`init`], there's no console yet and it just halts.
#[panic_handler]
fn on_panic(info: &PanicInfo) -> ! {
    critical_section::with(|cs| {
        if let Ok(mut slot) = CONSOLE.borrow(cs).try_borrow_mut()
            && let Some(console) = slot.as_mut()
        {
            let _ = writeln!(console, "\r\n*** panic: {} ***", info);
            console.flush();
        }
    });
    loop {
        cortex_m::asm::wfi();
    }
}
