//! Crash breadcrumb — a fault record that survives the reset.
//!
//! The panic / HardFault handlers can emit one framed error over the console — but only when
//! USB is attached. A battery node that faults in the field used to halt forever with no
//! record. Now the handlers write the crash text into a **reset-surviving `.uninit` RAM
//! record** (the [`bootguard`](crate::bootguard) pattern: retained across a warm reset, never
//! persisted — zero EEPROM wear) and reset. The next boot picks the record up
//! ([`take`], called via `console::emit_crash_report` from the [`app!`](crate::app) macro),
//! logs it as an error frame, and caches it for `/system/crash print` — so the crash is
//! visible on the wire after the unit has already recovered, and on demand for the rest of
//! the power-on.
//!
//! A cold boot (random RAM) is told from a real record by a magic marker, exactly like the
//! boot-loop guard; a brown-out that loses RAM retention reads as "no crash", the safe default.

use core::cell::RefCell;
use core::ptr::addr_of_mut;

use critical_section::Mutex;

/// Capacity of the stored crash text — matches the console's `MAX_MSG` so the breadcrumb can
/// hold exactly what the panic handler would have emitted over the wire.
pub const MSG_CAP: usize = 192;

/// Marks [`RECORD`] as a live crash record (vs cold-boot RAM garbage).
const MAGIC: u32 = 0xC7A5_FEED;

/// The faulting path that wrote the record.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// The Rust panic handler (`panic!`, slice OOB, arithmetic checks, …).
    Panic,
    /// The Cortex-M HardFault exception (bad memory access, invalid instruction, …).
    HardFault,
}

impl Kind {
    /// Short human-readable tag (used in the boot report and the shell command).
    pub fn as_str(self) -> &'static str {
        match self {
            Kind::Panic => "panic",
            Kind::HardFault => "hardfault",
        }
    }
}

/// The raw `.uninit` record. Plain `u32`s + a byte array: no invalid bit patterns, so a
/// volatile read of pre-init RAM is well-defined (see `bootguard::STATE` for the argument).
#[repr(C)]
#[derive(Clone, Copy)]
struct Record {
    magic: u32,
    /// 1 = panic, 2 = HardFault (kept as a raw int — an enum would have invalid patterns).
    kind: u32,
    len: u32,
    msg: [u8; MSG_CAP],
}

#[unsafe(link_section = ".uninit.TOWER_CRASHLOG")]
static mut RECORD: Record = Record {
    magic: 0,
    kind: 0,
    len: 0,
    msg: [0; MSG_CAP],
};

/// The record [`take`] recovered on this boot — kept for `/system/crash print`. Written once
/// at start-up (before the shell serves), read-only afterwards.
static LAST: Mutex<RefCell<Option<Crash>>> = Mutex::new(RefCell::new(None));

/// A recovered crash record, copied out of the `.uninit` RAM.
#[derive(Clone)]
pub struct Crash {
    pub kind: Kind,
    len: usize,
    msg: [u8; MSG_CAP],
}

impl Crash {
    /// The crash text (the panic message / fault PC+LR line). Always valid UTF-8 in practice
    /// (the handlers wrote it from a `str`); garbage that survived the magic check by
    /// coincidence degrades to a placeholder rather than a panic in the reporter.
    pub fn message(&self) -> &str {
        core::str::from_utf8(&self.msg[..self.len]).unwrap_or("<corrupt crash record>")
    }
}

/// Store a crash record. Called from the panic / HardFault handlers only — single-threaded,
/// executor stopped, so the volatile write cannot race anything.
pub(crate) fn record(kind: Kind, msg: &str) {
    let bytes = msg.as_bytes();
    let len = bytes.len().min(MSG_CAP);
    let mut rec = Record {
        magic: MAGIC,
        kind: match kind {
            Kind::Panic => 1,
            Kind::HardFault => 2,
        },
        len: len as u32,
        msg: [0; MSG_CAP],
    };
    rec.msg[..len].copy_from_slice(&bytes[..len]);
    // SAFETY: fault context — nothing else runs; volatile write to the retained static.
    unsafe { addr_of_mut!(RECORD).write_volatile(rec) };
}

/// Recover the previous boot's crash record, if any — and clear it, so it is reported once.
/// Call once, early in start-up (the `app!` macro does, via `console::emit_crash_report`);
/// the record is then cached for [`last`].
pub fn take() -> Option<Crash> {
    let p = addr_of_mut!(RECORD);
    // SAFETY: single-threaded start-up; volatile access to RAM retained across a warm reset
    // (or arbitrary-but-valid bytes on a cold boot, which the magic rejects).
    let rec = unsafe { p.read_volatile() };
    if rec.magic != MAGIC {
        return None;
    }
    unsafe {
        p.write_volatile(Record {
            magic: 0,
            kind: 0,
            len: 0,
            msg: [0; MSG_CAP],
        })
    };
    let crash = Crash {
        kind: if rec.kind == 2 {
            Kind::HardFault
        } else {
            Kind::Panic
        },
        len: (rec.len as usize).min(MSG_CAP),
        msg: rec.msg,
    };
    critical_section::with(|cs| {
        *LAST.borrow(cs).borrow_mut() = Some(crash.clone());
    });
    Some(crash)
}

/// The crash recovered at this boot, if any — what `/system/crash print` reports. `None` after
/// a clean boot (or a brown-out that lost RAM retention).
pub fn last() -> Option<Crash> {
    critical_section::with(|cs| LAST.borrow(cs).borrow().clone())
}
