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
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use serde::Serialize;
use serde::de::DeserializeOwned;
use tower_kv::{Eeprom, KvError};

/// Re-exported from the codec crate ([`tower_kv`]): the largest value a [`Kv`] record holds
/// ([`MAX_VALUE`]) and the most distinct keys [`Kv::compact`] tracks in one pass ([`MAX_KEYS`]).
pub use tower_kv::{MAX_KEYS, MAX_VALUE};

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
}

impl From<KvError<embassy_stm32::flash::Error>> for Error {
    fn from(e: KvError<embassy_stm32::flash::Error>) -> Self {
        match e {
            KvError::Backend(f) => Error::Flash(f),
            KvError::ValueTooLarge => Error::ValueTooLarge,
            KvError::Full => Error::Full,
            KvError::InvalidKey => Error::InvalidKey,
            KvError::TooManyKeys => Error::TooManyKeys,
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

    /// Reclaim the underlying blocking [`Flash`] handle.
    ///
    /// The L0 has a single `Flash` peripheral that drives **both** the data EEPROM
    /// (this module) and the **program flash** (where FOTA stages an image). They are
    /// disjoint regions on the same handle, so a program-flash writer (see
    /// [`crate::fota::Stage`]) borrows the very handle `Storage` owns. Use this to hand
    /// it over — e.g. for FOTA staging that does not also need the EEPROM KV store.
    pub fn into_flash(self) -> Flash<'d, Blocking> {
        self.flash
    }

    /// Borrow the underlying blocking [`Flash`] handle (program-flash + EEPROM live on
    /// the same peripheral; see [`into_flash`](Self::into_flash)). Lets a FOTA
    /// [`Stage`](crate::fota::Stage) write program flash while `Storage` retains
    /// ownership of the EEPROM KV state.
    pub fn flash_mut(&mut self) -> &mut Flash<'d, Blocking> {
        &mut self.flash
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

/// Radio [`net`](crate::radio::net) layer (TX-counter watermark, last-seen lanes).
pub const NS_NET: Ns = Ns(0x52);
/// [`fota`](crate::fota) (download high-water mark / ident, installed-version rollback floor).
pub const NS_FOTA: Ns = Ns(0x54);
/// Console/shell settings (one local per declared [`Setting`](crate::shell)).
pub const NS_SHELL: Ns = Ns(0x55);
/// Application-owned state — apps pick any locals here, clear of every SDK namespace.
pub const NS_APP: Ns = Ns(0x60);

/// Compose a full [`Kv`] key from a namespace + an 8-bit local key. Use it for `const` keys and at
/// API boundaries that take a raw `u16` (e.g. `Net::bulk_fetch_to_flash`'s progress key); for live
/// get/set prefer a [`Scoped`] handle ([`Nv::scope`]), which applies this automatically.
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
}

impl<'d> Kv<'d> {
    /// Build a key-value store over the whole EEPROM region of `storage`, initializing the
    /// double-buffered layout (and migrating legacy single-region data, once) if needed. The
    /// init is best-effort: a flash fault leaves the store empty rather than failing `new`, the
    /// same fallback as a blank/corrupt key.
    pub fn new(mut storage: Storage<'d>) -> Self {
        let capacity = storage.len() as u32;
        let _ = tower_kv::init(&mut storage, capacity);
        Self { storage, capacity }
    }

    /// Reclaim the underlying [`Storage`] for raw access.
    pub fn into_storage(self) -> Storage<'d> {
        self.storage
    }

    /// Borrow the underlying [`Storage`] — lets the network layer reach **program flash**
    /// (via [`Storage::flash_mut`]) during a FOTA download while `Net` keeps owning the KV
    /// store for its counter state. The EEPROM (this KV) and program flash are disjoint
    /// regions on the one `Flash` peripheral, so they don't interfere.
    pub fn storage_mut(&mut self) -> &mut Storage<'d> {
        &mut self.storage
    }

    /// Store raw bytes under `key`. Use this for scalars (`&x.to_le_bytes()`) or
    /// any pre-encoded blob — no serializer involved. (Codec in [`tower_kv`].)
    pub fn set_bytes(&mut self, key: u16, value: &[u8]) -> Result<(), Error> {
        tower_kv::set_bytes(&mut self.storage, self.capacity, key, value).map_err(Error::from)
    }

    /// Read the raw bytes stored under `key` into `out`; returns the value's true
    /// length (which may exceed `out.len()` — only `out.len()` bytes are copied),
    /// or `None` if the key is absent.
    pub fn get_bytes(&self, key: u16, out: &mut [u8]) -> Result<Option<usize>, Error> {
        tower_kv::get_bytes(&self.storage, self.capacity, key, out).map_err(Error::from)
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
    /// (Codec in [`tower_kv`].)
    pub fn compact(&mut self) -> Result<(), Error> {
        tower_kv::compact(&mut self.storage, self.capacity).map_err(Error::from)
    }
}

/// The one process-wide [`Kv`] over the EEPROM, behind a blocking mutex + `RefCell`, parked in a
/// `'static` (see [`Nv::install`]). Every subsystem — radio [`net`](crate::radio::net), the
/// [`shell`](crate::shell), [`fota`](crate::fota), and apps — borrows it through an [`Nv`] handle.
///
/// A *blocking* (non-async) mutex suffices: EEPROM access is synchronous and is never held across
/// `.await` on this single-core cooperative executor, and no EEPROM access happens in an interrupt.
/// The `Option` lets a sole-owner FOTA app reclaim the raw `Flash` via [`Nv::into_owned_flash`].
pub type SharedKv = Mutex<CriticalSectionRawMutex, RefCell<Option<Kv<'static>>>>;

/// A cheap, `Copy` handle to the one shared [`Kv`]. Hand the same handle to `Net`, the shell, and
/// FOTA at once (it is [`Board::kv`](crate::board::Board)); each method locks + borrows the store
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
            : SharedKv = Mutex::new(RefCell::new(Some(Kv::new(storage))))
        )
        .expect("Nv::install called more than once");
        Nv(cell)
    }

    /// Run `f` against the one [`Kv`] under the lock. Panics only if the `Kv` was reclaimed by
    /// [`into_owned_flash`](Self::into_owned_flash) — which a sole-owner FOTA app does, and it
    /// serves no shell / builds no `Net`, so nothing else calls this afterwards.
    fn with<R>(&self, f: impl FnOnce(&mut Kv<'static>) -> R) -> R {
        self.0
            .lock(|c| f(c.borrow_mut().as_mut().expect("Nv used after into_owned_flash")))
    }

    /// Store raw bytes under `key` (see [`Kv::set_bytes`]).
    pub fn set_bytes(&self, key: u16, value: &[u8]) -> Result<(), Error> {
        self.with(|kv| kv.set_bytes(key, value))
    }

    /// Read the raw bytes stored under `key` into `out` (see [`Kv::get_bytes`]).
    pub fn get_bytes(&self, key: u16, out: &mut [u8]) -> Result<Option<usize>, Error> {
        self.with(|kv| kv.get_bytes(key, out))
    }

    /// Store any serde value under `key`, postcard-serialized (see [`Kv::set`]).
    pub fn set<T: Serialize>(&self, key: u16, value: &T) -> Result<(), Error> {
        self.with(|kv| kv.set(key, value))
    }

    /// Load a postcard value under `key`, or `None` if absent (see [`Kv::get`]).
    pub fn get<T: DeserializeOwned>(&self, key: u16) -> Result<Option<T>, Error> {
        self.with(|kv| kv.get(key))
    }

    /// Reclaim space taken by superseded records (see [`Kv::compact`]).
    pub fn compact(&self) -> Result<(), Error> {
        self.with(|kv| kv.compact())
    }

    /// Borrow the underlying **program-flash** handle for a *synchronous* op (FOTA staging, or a
    /// boot-state read/confirm), while the EEPROM [`Kv`] stays parked behind the lock. The EEPROM
    /// and program flash are disjoint regions of the one `Flash` peripheral (see
    /// [`Storage::flash_mut`]). The lock is held for the whole closure — do **not** `.await` and do
    /// **not** call other [`Nv`] methods inside `f`.
    pub fn with_flash<R>(&self, f: impl FnOnce(&mut Flash<'static, Blocking>) -> R) -> R {
        self.with(|kv| f(kv.storage_mut().flash_mut()))
    }

    /// Reclaim the owned program `Flash`, consuming the shared [`Kv`]. **Sound only for a
    /// sole-owner FOTA app** that serves no shell and builds no `Net` — e.g. a staging / A-B
    /// updater that must hold `Flash` across `.await` (which the locked
    /// [`with_flash`](Self::with_flash) cannot give). After this, any other [`Nv`] call panics, so
    /// use `app!(run, no_shell)`.
    pub fn into_owned_flash(self) -> Flash<'static, Blocking> {
        self.0
            .lock(|c| c.borrow_mut().take().expect("Nv flash already reclaimed"))
            .into_storage()
            .into_flash()
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

    /// The underlying unscoped handle (e.g. for [`Nv::with_flash`], or to re-`scope`).
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

    /// Store a postcard value under `local` in this namespace (see [`Kv::set`]).
    pub fn set<T: Serialize>(&self, local: u8, value: &T) -> Result<(), Error> {
        self.nv.set(self.full_key(local), value)
    }

    /// Load a postcard value under `local` in this namespace (see [`Kv::get`]).
    pub fn get<T: DeserializeOwned>(&self, local: u8) -> Result<Option<T>, Error> {
        self.nv.get(self.full_key(local))
    }
}
