//! Non-volatile storage in the STM32L0 **data EEPROM**.
//!
//! The L0 has true byte-addressable EEPROM (6 KB on the Core Module, ~100k+ write
//! endurance) — no page erase, no wear-leveling needed. Two layers:
//!
//!   * [`Storage`] — the raw area: [`read`](Storage::read) / [`write`](Storage::write)
//!     at any byte offset, [`len`](Storage::len) the total size. You own the layout.
//!   * [`Kv`] — a small **key-value store** over that area, **power-loss-safe**: it survives a
//!     reset mid-write (the codec is double-buffered — see [`tower_kv`]). Each value lives under
//!     a `u16` key as a `{tag, len, crc}`-framed record; updates **append** (a torn write only
//!     ever loses the in-flight record, never committed data) and compaction reclaims superseded
//!     records via an atomic half-flip. Add a key to *evolve* stored data rather than growing a
//!     struct. Values can be stored as **raw bytes** ([`set_bytes`](Kv::set_bytes)) for scalars,
//!     or **postcard** ([`set`](Kv::set)) for any `#[derive(Serialize, Deserialize)]` type.
//!     Double-buffering halves usable space (~3 KB of the 6 KB region; the rest is the flip
//!     buffer) — ample for the SDK's counter/peer/settings state.
//!
//! ```ignore
//! let kv = b.kv.scope(NS_APP); // a namespaced view — apps own NS_APP; locals are u8
//! const BOOTS: u8 = 0x01;
//! const CFG: u8 = 0x02;
//!
//! kv.set_bytes(BOOTS, &count.to_le_bytes())?;       // raw scalar, no codec
//! kv.set(CFG, &Settings { interval_s: 30 })?;       // postcard
//! let cfg: Option<Settings> = kv.get(CFG)?;
//! ```
//!
//! A `get` returns `None` (not garbage) for a missing/blank/corrupt key, so first
//! boot and missing keys fall back to defaults. Reading the EEPROM is just a
//! memory-mapped load; each write programs in a few ms.

// Reusable SDK surface: both layers are fully exposed even if an app uses one.

use core::cell::RefCell;

use embassy_stm32::flash::{Blocking, EEPROM_SIZE, Flash};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use serde::Serialize;
use serde::de::DeserializeOwned;
use tower_kv::{Eeprom, KvError, MaintState};

/// Re-exported from the codec crate ([`tower_kv`]): the largest value a [`Kv`] record holds
/// ([`MAX_VALUE`]) and the most distinct keys [`Kv::compact`] tracks in one pass ([`MAX_KEYS`]).
pub use tower_kv::{MAX_KEYS, MAX_VALUE};

/// STM32L083CZ data-EEPROM endurance: **100,000 erase/write cycles** (datasheet-confirmed).
/// Every wear figure below scales linearly with it.
pub const EEPROM_ENDURANCE_CYCLES: u32 = 100_000;
/// Lifetime compaction-flip budget for the wear gauge, kept **deliberately conservative**. Each
/// flip erases + reprograms the store's most-written cells (the re-packed live-set prefix and the
/// committed superblock); charging one full erase/write cycle of the worst cell per flip caps the
/// store at `endurance` flips. The store wear-levels across two alternating halves, so a given
/// cell is really the flip target only every *other* flip — true life is ~2× this — meaning the
/// gauge errs toward reporting *more* wear, never less. See `docs/storage.md`.
pub const FLIP_BUDGET: u32 = EEPROM_ENDURANCE_CYCLES;

/// A non-volatile storage error.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Error {
    /// The underlying EEPROM read/write failed (e.g. out of bounds).
    Flash(embassy_stm32::flash::Error),
    /// A value exceeded [`MAX_VALUE`] bytes.
    ValueTooLarge,
    /// The store is full even after compaction.
    Full,
    /// Key `0` is reserved (it marks the end of the record log).
    InvalidKey,
    /// More distinct keys than [`MAX_KEYS`] (only hit during compaction).
    TooManyKeys,
    /// A stale incremental-flip plan (handled internally by [`Kv`]; never surfaced in practice).
    Stale,
}

impl From<KvError<embassy_stm32::flash::Error>> for Error {
    fn from(e: KvError<embassy_stm32::flash::Error>) -> Self {
        match e {
            KvError::Backend(f) => Error::Flash(f),
            KvError::ValueTooLarge => Error::ValueTooLarge,
            KvError::Full => Error::Full,
            KvError::InvalidKey => Error::InvalidKey,
            KvError::TooManyKeys => Error::TooManyKeys,
            KvError::Stale => Error::Stale,
        }
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Error::Flash(_) => "EEPROM access failed",
            Error::ValueTooLarge => "value exceeds MAX_VALUE",
            Error::Full => "storage full (even after compaction)",
            Error::InvalidKey => "invalid key (0 is reserved)",
            Error::TooManyKeys => "too many distinct keys",
            Error::Stale => "stale flip state",
        })
    }
}

/// Non-volatile storage over the L0 data EEPROM (the raw byte area).
pub struct Storage<'d> {
    flash: Flash<'d, Blocking>,
}

impl<'d> Storage<'d> {
    /// Wrap the blocking [`Flash`] handle (its EEPROM region) as storage.
    pub fn new(flash: Flash<'d, Blocking>) -> Self {
        Self { flash }
    }

    /// Total EEPROM size in bytes.
    pub const fn len(&self) -> usize {
        EEPROM_SIZE
    }

    /// Whether the EEPROM region is zero-sized (never, on the Core Module).
    pub const fn is_empty(&self) -> bool {
        EEPROM_SIZE == 0
    }

    /// Read `buf.len()` bytes from `offset` (a bounds-checked, memory-mapped copy).
    pub fn read(&self, offset: u32, buf: &mut [u8]) -> Result<(), Error> {
        self.flash.eeprom_read_slice(offset, buf).map_err(Error::Flash)
    }

    /// Write `data` at `offset` (bounds-checked; the HAL handles unlock/relock).
    pub fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Error> {
        self.flash.eeprom_write_slice(offset, data).map_err(Error::Flash)
    }
}

/// The [`Kv`] codec ([`tower_kv`]) drives the EEPROM through this trait. The raw flash error is
/// surfaced verbatim so [`Kv`] maps it back to [`Error::Flash`]. (Distinct from `Storage`'s
/// inherent `read`/`write`, which already wrap the error in [`Error`].)
impl Eeprom for Storage<'_> {
    type Error = embassy_stm32::flash::Error;
    fn read(&self, off: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.flash.eeprom_read_slice(off, buf)
    }
    fn write(&mut self, off: u32, data: &[u8]) -> Result<(), Self::Error> {
        self.flash.eeprom_write_slice(off, data)
    }
}

/// EEPROM key **namespace**. Each subsystem owns one `Ns`; combined with a per-subsystem 8-bit
/// `local` key it forms the 16-bit [`Kv`] key as `(ns << 8) | local`. Two subsystems with
/// different `Ns` can never collide, whatever locals they pick — so the disjoint-key invariant is
/// **structural** (hold a [`Scoped`] view via [`Nv::scope`]), not a hand-maintained convention.
///
/// The high byte is the namespace, the low byte the local key — a flat `u16` space as before, just
/// partitioned by *number*, not by memory (the log-structured [`Kv`] stores by key tag + insertion
/// order, so the key value costs nothing — see the module docs).
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct Ns(pub u8);

/// System/console layer state (the per-boot session counter carried in `Hello`).
pub const NS_SYS: Ns = Ns(0x53);
/// Radio [`net`](crate::radio::net) layer (TX-counter watermark, last-seen lanes).
pub const NS_NET: Ns = Ns(0x52);
/// Console/shell settings (one local per declared [`Setting`](crate::shell)).
pub const NS_SHELL: Ns = Ns(0x55);
/// Application-owned state — apps pick any locals here, clear of every SDK namespace.
pub const NS_APP: Ns = Ns(0x60);

/// Compose a full [`Kv`] key from a namespace + an 8-bit local key. Use it for `const` keys and at
/// API boundaries that take a raw `u16`; for live get/set prefer a [`Scoped`] handle
/// ([`Nv::scope`]), which applies this automatically.
pub const fn key(ns: Ns, local: u8) -> u16 {
    ((ns.0 as u16) << 8) | local as u16
}

/// A key-value store over the EEPROM. Keys are `u16` (1..=65535; `0` is reserved).
///
/// **Power-loss-safe**, double-buffered (see [`tower_kv`]): every `set` appends and compaction
/// commits via a single superblock write, so a reset mid-write never corrupts committed data.
/// [`get`](Self::get) returns the latest valid record for a key. Usable capacity is ~half the
/// EEPROM region (the other half is the compaction buffer).
pub struct Kv<'d> {
    storage: Storage<'d>,
    capacity: u32,
    /// RAM-only maintenance state (pre-blank cursor + at most one pending incremental flip).
    /// Lives here — inside the one [`SharedKv`] mutex — so the write path
    /// ([`set_bytes`](Self::set_bytes)) and the [`maintenance`] task share it: a store that
    /// fills mid-flip *finishes* the pending flip instead of restarting one. Losing it (reboot)
    /// is always safe (see [`tower_kv::FlipState`]). Costs ~540 B of `.bss` — deliberately off
    /// the stack: the flip plan otherwise sat in the deepest call paths (docs/radio.md: no
    /// stack guard on this target).
    maint: MaintState,
}

impl<'d> Kv<'d> {
    /// Build a key-value store over the whole EEPROM region of `storage`, initializing the
    /// double-buffered layout (and migrating legacy single-region data, once) if needed. The
    /// init is best-effort: a flash fault leaves the store empty rather than failing `new`, the
    /// same fallback as a blank/corrupt key.
    pub fn new(mut storage: Storage<'d>) -> Self {
        let capacity = storage.len() as u32;
        let _ = tower_kv::init(&mut storage, capacity);
        Self {
            storage,
            capacity,
            maint: MaintState::new(),
        }
    }

    /// Reclaim the underlying [`Storage`] for raw access.
    pub fn into_storage(self) -> Storage<'d> {
        self.storage
    }

    /// Factory reset: zero the ENTIRE EEPROM — every record, both superblocks — returning the
    /// store to its virgin state ([`tower_kv::init`] re-initializes it on the next boot), and
    /// reset the in-RAM maintenance state. **~5 s of CPU-stalling word writes**
    /// (docs/storage.md); callers must reboot immediately after (the shell's
    /// `/system/eeprom wipe confirm` does) so no subsystem keeps RAM state derived from the
    /// wiped store. Exists because the store has no per-key deletion — the live set only grows.
    pub fn wipe(&mut self) -> Result<(), Error> {
        let zeros = [0u8; 64];
        let mut off = 0u32;
        while off < self.capacity {
            let n = 64u32.min(self.capacity - off) as usize;
            self.storage.write(off, &zeros[..n])?;
            off += n as u32;
        }
        self.maint = MaintState::new();
        Ok(())
    }

    /// Store raw bytes under `key`. Use this for scalars (`&x.to_le_bytes()`) or
    /// any pre-encoded blob — no serializer involved. (Codec in [`tower_kv`].)
    ///
    /// Flip-aware: if the half fills while the [`maintenance`] task has an incremental flip in
    /// flight, the write finishes that flip (bounded: its remaining steps) instead of paying for
    /// a from-scratch synchronous compaction.
    pub fn set_bytes(&mut self, key: u16, value: &[u8]) -> Result<(), Error> {
        tower_kv::set_bytes_with(&mut self.storage, self.capacity, key, value, self.maint.pending())
            .map_err(Error::from)
    }

    /// Read the raw bytes stored under `key` into `out`; returns the value's true
    /// length (which may exceed `out.len()` — only `out.len()` bytes are copied),
    /// or `None` if the key is absent.
    pub fn get_bytes(&self, key: u16, out: &mut [u8]) -> Result<Option<usize>, Error> {
        tower_kv::get_bytes(&self.storage, self.capacity, key, out).map_err(Error::from)
    }

    /// Delete `key` (a tombstone append): it reads back absent, and the next compaction reclaims
    /// its space. A wear-free no-op if already absent. Flip-aware like [`set_bytes`](Self::set_bytes).
    /// The store can now *shrink*, not only grow — the lever against a full-store compaction storm.
    pub fn delete(&mut self, key: u16) -> Result<(), Error> {
        tower_kv::delete_with(&mut self.storage, self.capacity, key, self.maint.pending())
            .map_err(Error::from)
    }

    /// Store any serde value under `key`, serialized with postcard.
    pub fn set<T: Serialize>(&mut self, key: u16, value: &T) -> Result<(), Error> {
        let mut buf = [0u8; MAX_VALUE];
        let bytes = postcard::to_slice(value, &mut buf).map_err(|_| Error::ValueTooLarge)?;
        let n = bytes.len();
        self.set_bytes(key, &buf[..n])
    }

    /// Load a postcard value under `key`, or `None` if absent / it doesn't
    /// deserialize into `T` (e.g. the type's shape changed — add a new key instead).
    pub fn get<T: DeserializeOwned>(&self, key: u16) -> Result<Option<T>, Error> {
        let mut buf = [0u8; MAX_VALUE];
        match self.get_bytes(key, &mut buf)? {
            Some(len) if len <= MAX_VALUE => Ok(postcard::from_bytes::<T>(&buf[..len]).ok()),
            _ => Ok(None),
        }
    }

    /// Reclaim space taken by superseded records: keep only the latest record per
    /// key, packed from the start. Called automatically when an append won't fit.
    /// A pending incremental flip is finished rather than restarted. (Codec in [`tower_kv`].)
    pub fn compact(&mut self) -> Result<(), Error> {
        tower_kv::compact_with(&mut self.storage, self.capacity, self.maint.pending()).map_err(Error::from)
    }

    /// One bounded maintenance slice (≤ `budget_words` EEPROM word-programs ≈ 3.4 ms each):
    /// advances a pending incremental flip, starts one when free space drops below
    /// [`tower_kv::FLIP_THRESHOLD`], or pre-blanks the dead half. Returns whether work remains.
    /// See [`maintenance`] for the task that drives it.
    pub fn maintain(&mut self, budget_words: u32) -> Result<bool, Error> {
        tower_kv::maintain(
            &mut self.storage,
            self.capacity,
            &mut self.maint,
            budget_words,
            tower_kv::FLIP_THRESHOLD,
        )
        .map_err(Error::from)
    }

    /// Lifetime compaction-flip count (the active-half generation) — a pure read for EEPROM-wear
    /// telemetry; see [`tower_kv::generation`] and [`FLIP_BUDGET`]. `0` on a fresh region.
    pub fn flip_generation(&self) -> u32 {
        tower_kv::generation(&self.storage, self.capacity).unwrap_or(0)
    }

    /// Free (appendable) bytes left in the active half — a pure read (`0` on a read fault).
    pub fn free_bytes(&self) -> u32 {
        tower_kv::free_bytes(&self.storage, self.capacity).unwrap_or(0)
    }

    /// Bytes the live set (latest record per key) occupies — a pure read (`0` on a read fault).
    pub fn live_bytes(&self) -> u32 {
        tower_kv::live_bytes(&self.storage, self.capacity).unwrap_or(0)
    }

    /// Whether the dead half is fully pre-blanked (the next flip skips its blank pass) — a pure
    /// read (`false` on a read fault).
    pub fn dead_half_blank(&self) -> bool {
        tower_kv::dead_half_blank(&self.storage, self.capacity).unwrap_or(false)
    }
}

/// The one process-wide [`Kv`] over the EEPROM, behind a blocking mutex + `RefCell`, parked in a
/// `'static` (see [`Nv::install`]). Every subsystem — radio [`net`](crate::radio::net), the
/// [`shell`](crate::shell), and apps — borrows it through an [`Nv`] handle.
///
/// A *blocking* (non-async) mutex suffices: EEPROM access is synchronous and is never held across
/// `.await` on this single-core cooperative executor. The raw mutex is [`ThreadModeRawMutex`], NOT
/// `CriticalSectionRawMutex`: every access is task-context (thread mode), so it never needs to mask
/// interrupts — and it MUST NOT. A compaction flip programs a whole ~3 KB EEPROM half word-by-word
/// (each ~3.4–6.8 ms), and under a critical-section mutex that ran with all interrupts disabled for
/// **2.5–5 s** — long enough to overflow the SPIRIT1 RX FIFO and slip every timer, triggered
/// unpredictably from the radio hot path. With `ThreadModeRawMutex` the flip still monopolizes the
/// *executor* (no other task runs until it returns — no `.await` inside), but the NVIC keeps
/// servicing interrupts, so the radio and timebase stay alive. `ThreadModeRawMutex` also enforces
/// the "thread mode only" contract by construction: a stray EEPROM access from an ISR/exception
/// now panics instead of silently deadlocking.
pub type SharedKv = Mutex<ThreadModeRawMutex, RefCell<Kv<'static>>>;

/// A cheap, `Copy` handle to the one shared [`Kv`]. Hand the same handle to `Net`, the shell, and
/// apps at once (it is [`Board::kv`](crate::board::Board)); each method locks + borrows the store
/// for that one call, so the subsystems coexist over the single EEPROM. They stay non-colliding by
/// each taking a [`Scoped`] view of their own namespace ([`Nv::scope`]).
#[derive(Copy, Clone)]
pub struct Nv(&'static SharedKv);

impl Nv {
    /// Build the one [`Kv`] over `storage`, park it in a process-wide `static`, and return the
    /// handle. Call **exactly once** — the [`cortex_m::singleton!`] enforces it (a second call
    /// panics). The sole caller is [`Board::take`](crate::board::Board::take).
    pub fn install(storage: Storage<'static>) -> Self {
        let cell: &'static SharedKv = cortex_m::singleton!(
            : SharedKv = Mutex::new(RefCell::new(Kv::new(storage)))
        )
        .expect("Nv::install called more than once");
        Nv(cell)
    }

    /// Run `f` against the one [`Kv`] under the lock.
    fn with<R>(&self, f: impl FnOnce(&mut Kv<'static>) -> R) -> R {
        self.0.lock(|c| f(&mut c.borrow_mut()))
    }

    /// Store raw bytes under `key` (see [`Kv::set_bytes`]). Wakes the [`maintenance`] task.
    pub fn set_bytes(&self, key: u16, value: &[u8]) -> Result<(), Error> {
        let r = self.with(|kv| kv.set_bytes(key, value));
        MAINT_WAKE.signal(());
        r
    }

    /// Read the raw bytes stored under `key` into `out` (see [`Kv::get_bytes`]).
    pub fn get_bytes(&self, key: u16, out: &mut [u8]) -> Result<Option<usize>, Error> {
        self.with(|kv| kv.get_bytes(key, out))
    }

    /// Delete `key` (see [`Kv::delete`]). Wakes the [`maintenance`] task so the tombstone (and
    /// the value it hides) get reclaimed by the next flip.
    pub fn delete(&self, key: u16) -> Result<(), Error> {
        let r = self.with(|kv| kv.delete(key));
        MAINT_WAKE.signal(());
        r
    }

    /// Store any serde value under `key`, postcard-serialized (see [`Kv::set`]). Wakes the
    /// [`maintenance`] task.
    pub fn set<T: Serialize>(&self, key: u16, value: &T) -> Result<(), Error> {
        let r = self.with(|kv| kv.set(key, value));
        MAINT_WAKE.signal(());
        r
    }

    /// Load a postcard value under `key`, or `None` if absent (see [`Kv::get`]).
    pub fn get<T: DeserializeOwned>(&self, key: u16) -> Result<Option<T>, Error> {
        self.with(|kv| kv.get(key))
    }

    /// Reclaim space taken by superseded records (see [`Kv::compact`]).
    pub fn compact(&self) -> Result<(), Error> {
        let r = self.with(|kv| kv.compact());
        MAINT_WAKE.signal(()); // the flip left a dead, dirty half — pre-blank it
        r
    }

    /// Operator-controlled full compaction, **now** and synchronously — for products that want
    /// the (bounded) stall at a moment they control (e.g. a provisioning step) rather than
    /// mid-operation. Same operation as [`compact`](Self::compact); the name states the intent.
    pub fn compact_now(&self) -> Result<(), Error> {
        self.compact()
    }

    /// Factory reset — see [`Kv::wipe`]. Deliberately does not wake the maintenance task:
    /// the caller reboots immediately and the next boot starts from a virgin store.
    pub fn wipe(&self) -> Result<(), Error> {
        self.with(|kv| kv.wipe())
    }

    /// One bounded maintenance slice (see [`Kv::maintain`]); `false` on error (best-effort,
    /// like `Kv::new`'s init). Driven by the [`maintenance`] task — call directly only from a
    /// custom maintenance loop.
    pub fn maintain(&self, budget_words: u32) -> bool {
        self.with(|kv| kv.maintain(budget_words)).unwrap_or(false)
    }

    /// Free (appendable) bytes left in the active half — a pure read (see [`Kv::free_bytes`]).
    pub fn free_bytes(&self) -> u32 {
        self.with(|kv| kv.free_bytes())
    }

    /// Live-set size in bytes — a pure read (see [`Kv::live_bytes`]).
    pub fn live_bytes(&self) -> u32 {
        self.with(|kv| kv.live_bytes())
    }

    /// Whether the dead half is fully pre-blanked — a pure read (see [`Kv::dead_half_blank`]).
    pub fn dead_half_blank(&self) -> bool {
        self.with(|kv| kv.dead_half_blank())
    }

    /// Lifetime compaction-flip count for EEPROM-wear telemetry (see [`Kv::flip_generation`]).
    /// A pure read — polling it adds no wear. Compare against [`FLIP_BUDGET`].
    pub fn flip_generation(&self) -> u32 {
        self.with(|kv| kv.flip_generation())
    }

    /// A namespace-scoped view: every get/set is keyed by an 8-bit `local`, silently prefixed with
    /// `ns`. A subsystem holding the returned [`Scoped`] **cannot** address another namespace's
    /// keys, so cross-subsystem collisions are impossible by construction. Hand each subsystem
    /// `b.kv.scope(NS_…)`.
    pub fn scope(self, ns: Ns) -> Scoped {
        Scoped { nv: self, ns: ns.0 }
    }
}

/// A namespace-scoped [`Nv`] (see [`Nv::scope`]). Get/set are keyed by an 8-bit `local` and
/// auto-prefixed with the namespace; [`full_key`](Self::full_key) exposes the composed `u16` for
/// APIs that take a raw key, and [`raw`](Self::raw) drops back to the unscoped handle.
#[derive(Copy, Clone)]
pub struct Scoped {
    nv: Nv,
    ns: u8,
}

impl Scoped {
    /// The full 16-bit [`Kv`] key this `local` maps to — for APIs that take a raw `u16`.
    pub fn full_key(&self, local: u8) -> u16 {
        ((self.ns as u16) << 8) | local as u16
    }

    /// The underlying unscoped handle (e.g. to re-`scope`).
    pub fn raw(&self) -> Nv {
        self.nv
    }

    /// Store raw bytes under `local` in this namespace (see [`Kv::set_bytes`]).
    pub fn set_bytes(&self, local: u8, value: &[u8]) -> Result<(), Error> {
        self.nv.set_bytes(self.full_key(local), value)
    }

    /// Read the raw bytes under `local` in this namespace into `out` (see [`Kv::get_bytes`]).
    pub fn get_bytes(&self, local: u8, out: &mut [u8]) -> Result<Option<usize>, Error> {
        self.nv.get_bytes(self.full_key(local), out)
    }

    /// Delete `local` in this namespace (see [`Kv::delete`]) — reads back absent, space reclaimed
    /// on the next flip. A wear-free no-op if already absent.
    pub fn delete(&self, local: u8) -> Result<(), Error> {
        self.nv.delete(self.full_key(local))
    }

    /// Store a postcard value under `local` in this namespace (see [`Kv::set`]).
    pub fn set<T: Serialize>(&self, local: u8, value: &T) -> Result<(), Error> {
        self.nv.set(self.full_key(local), value)
    }

    /// Load a postcard value under `local` in this namespace (see [`Kv::get`]).
    pub fn get<T: DeserializeOwned>(&self, local: u8) -> Result<Option<T>, Error> {
        self.nv.get(self.full_key(local))
    }
}

// --- background maintenance ----------------------------------------------------------------------

/// Per-slice maintenance budget, in EEPROM word-programs. On the STM32L0 each word program
/// stalls the whole CPU ~3.4 ms (NVM stall — instruction fetch and thus ISRs freeze), so 4 words
/// ≈ **14 ms** of stall per slice — comfortably under the radio's ACK window (200 ms) and the
/// console's tolerance, where the old synchronous flip froze the chip for seconds
/// (docs/storage.md). The slice can overshoot by ~2 words on unaligned copy edges, and the one
/// commit slice per flip adds the tail catch-up + a 12-byte superblock (still tens of ms).
pub const MAINT_BUDGET_WORDS: u32 = 4;

/// Wakes [`maintenance`]: signaled by every [`Nv`] write (and once at boot by the task itself
/// starting). A [`Signal`] — not a timer — so an idle node takes **zero** extra wakeups: the
/// task pends here and the low-power executor can keep reaching STOP (see `console::manager`).
static MAINT_WAKE: Signal<ThreadModeRawMutex, ()> = Signal::new();

/// Background EEPROM-maintenance task — spawned by default by the [`app!`](crate::app) macro
/// ([`spawn_maintenance`]), so every app gets bounded compaction stalls without wiring anything.
///
/// **Event-driven, not polling:** after draining all work it awaits [`MAINT_WAKE`] (raised by KV
/// writes), so an idle battery node sees no extra wakeups and STOP stays reachable. While work
/// remains it runs one [`Nv::maintain`] slice (≤ [`MAINT_BUDGET_WORDS`] word-programs ≈ 14 ms of
/// CPU stall) per executor turn, yielding between slices so the radio/console tasks interleave —
/// the executor simply stays out of STOP until the store is quiescent. The first pass runs at
/// boot (before the first await completes), pre-blanking a dead half left dirty by a previous
/// life's flip.
#[embassy_executor::task]
pub async fn maintenance(kv: Nv) {
    // How long the host link must be quiet before a slice may stall the chip, and how slices
    // are spaced while the console is up. Every EEPROM word-program freezes the CPU (ISRs
    // included), so a slice landing mid-frame eats in-flight host->device bytes: with USB
    // present and a host actively talking, defer entirely; in the gaps, space slices out so
    // an unlucky late command risks only one ~14 ms window (bench 2026-07-05: back-to-back
    // yield_now-paced slices formed a ~2 s stall wall that ate shell commands). With the
    // console DOWN (battery mode — the field case) there is nothing to eat: run full speed.
    const HOST_QUIET: Duration = Duration::from_secs(2);
    const SLICE_GAP: Duration = Duration::from_millis(100);
    loop {
        while {
            if crate::console::is_up() {
                // Timer wake-ups here are free: USB is present, so STOP is inhibited anyway.
                while crate::console::host_rx_age_ticks() < HOST_QUIET.as_ticks() as u32 {
                    Timer::after(Duration::from_millis(250)).await;
                }
            }
            kv.maintain(MAINT_BUDGET_WORDS)
        } {
            if crate::console::is_up() {
                Timer::after(SLICE_GAP).await;
            } else {
                embassy_futures::yield_now().await;
            }
        }
        MAINT_WAKE.wait().await;
    }
}

/// Spawn the [`maintenance`] task. Called by the [`app!`](crate::app) macro; a second spawn
/// attempt (task already running) is ignored, so a custom entry may call it too.
pub fn spawn_maintenance(spawner: crate::Spawner, kv: Nv) {
    if let Ok(token) = maintenance(kv) {
        spawner.spawn(token);
    }
}
