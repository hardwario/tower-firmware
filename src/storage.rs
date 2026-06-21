//! Non-volatile storage in the STM32L0 **data EEPROM**.
//!
//! The L0 has true byte-addressable EEPROM (6 KB on the Core Module, ~100k+ write
//! endurance) — no page erase, no wear-leveling needed. Two layers:
//!
//!   * [`Storage`] — the raw area: [`read`](Storage::read) / [`write`](Storage::write)
//!     at any byte offset, [`len`](Storage::len) the total size. You own the layout.
//!   * [`Kv`] — a small **key-value store** over that area. Each value lives under a
//!     `u16` key as a `{tag, len, crc}`-framed record. Updates of the same size are
//!     rewritten **in place** (EEPROM is byte-writable, so no log churn); a new key
//!     just appends and never disturbs existing keys — which is how you *evolve*
//!     stored data: add a key rather than growing a struct. Values can be stored as
//!     **raw bytes** ([`set_bytes`](Kv::set_bytes)) for scalars, or **postcard**
//!     ([`set`](Kv::set)) for any `#[derive(Serialize, Deserialize)]` type.
//!
//! ```ignore
//! let mut kv = Kv::new(b.storage);
//! const BOOTS: u16 = 1;
//! const CFG: u16 = 2;
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
#![allow(dead_code)]

use embassy_stm32::flash::{Blocking, EEPROM_SIZE, Flash};
use serde::Serialize;
use serde::de::DeserializeOwned;

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
        self.flash
            .eeprom_read_slice(offset, buf)
            .map_err(Error::Flash)
    }

    /// Write `data` at `offset` (bounds-checked; the HAL handles unlock/relock).
    pub fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Error> {
        self.flash
            .eeprom_write_slice(offset, data)
            .map_err(Error::Flash)
    }
}

/// Largest value (bytes) a [`Kv`] record can hold. Scalars and small configs fit
/// comfortably; for anything larger use the raw [`Storage`] API.
pub const MAX_VALUE: usize = 256;
/// Maximum number of distinct keys [`Kv::compact`] can track in one pass.
pub const MAX_KEYS: usize = 64;
/// Record header: tag(2) + value-len(2) + crc32(4). Value bytes follow.
const KV_HEADER: usize = 8;

/// A key-value store over the EEPROM. Keys are `u16` (1..=65535; `0` is reserved).
///
/// Records are laid out sequentially as `[tag | len | crc | value]`. Setting a key
/// to a same-length value overwrites it in place; a different length (or a brand
/// new key) appends. [`get`](Self::get) returns the latest valid record for a key.
pub struct Kv<'d> {
    storage: Storage<'d>,
    capacity: u32,
}

impl<'d> Kv<'d> {
    /// Build a key-value store over the whole EEPROM region of `storage`.
    pub fn new(storage: Storage<'d>) -> Self {
        let capacity = storage.len() as u32;
        Self { storage, capacity }
    }

    /// Reclaim the underlying [`Storage`] for raw access.
    pub fn into_storage(self) -> Storage<'d> {
        self.storage
    }

    /// Store raw bytes under `key`. Use this for scalars (`&x.to_le_bytes()`) or
    /// any pre-encoded blob — no serializer involved.
    pub fn set_bytes(&mut self, key: u16, value: &[u8]) -> Result<(), Error> {
        if key == 0 {
            return Err(Error::InvalidKey);
        }
        if value.len() > MAX_VALUE {
            return Err(Error::ValueTooLarge);
        }
        let len = value.len() as u16;
        let mut hdr4 = [0u8; 4];
        hdr4[0..2].copy_from_slice(&key.to_le_bytes());
        hdr4[2..4].copy_from_slice(&len.to_le_bytes());
        let crc = entry_crc(&hdr4, value);

        let (free, existing) = self.scan(key)?;
        // Same-size update: rewrite crc+value in place (tag/len unchanged).
        if let Some((off, elen)) = existing
            && elen == len
        {
            let mut tail = [0u8; 4 + MAX_VALUE];
            tail[0..4].copy_from_slice(&crc.to_le_bytes());
            tail[4..4 + value.len()].copy_from_slice(value);
            return self.storage.write(off + 4, &tail[..4 + value.len()]);
        }

        // Otherwise append; compact and retry once if it doesn't fit.
        let needed = KV_HEADER as u32 + len as u32;
        let free = if free + needed > self.capacity {
            self.compact()?;
            let (f, _) = self.scan(key)?;
            if f + needed > self.capacity {
                return Err(Error::Full);
            }
            f
        } else {
            free
        };

        let mut buf = [0u8; KV_HEADER + MAX_VALUE];
        buf[0..2].copy_from_slice(&key.to_le_bytes());
        buf[2..4].copy_from_slice(&len.to_le_bytes());
        buf[4..8].copy_from_slice(&crc.to_le_bytes());
        buf[8..8 + value.len()].copy_from_slice(value);
        self.storage.write(free, &buf[..KV_HEADER + value.len()])
    }

    /// Read the raw bytes stored under `key` into `out`; returns the value's true
    /// length (which may exceed `out.len()` — only `out.len()` bytes are copied),
    /// or `None` if the key is absent.
    pub fn get_bytes(&self, key: u16, out: &mut [u8]) -> Result<Option<usize>, Error> {
        if key == 0 {
            return Ok(None);
        }
        match self.scan(key)?.1 {
            Some((off, len)) => {
                let n = (len as usize).min(out.len());
                self.storage.read(off + KV_HEADER as u32, &mut out[..n])?;
                Ok(Some(len as usize))
            }
            None => Ok(None),
        }
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

    /// Scan the log: returns `(free_offset, latest_valid_record_for_target)`. Stops
    /// at the first blank (`tag == 0`) or corrupt record — which is exactly the end
    /// of a clean log, and also where a power-loss-truncated write would sit.
    fn scan(&self, target: u16) -> Result<(u32, Option<(u32, u16)>), Error> {
        let mut o = 0u32;
        let mut found = None;
        let mut val = [0u8; MAX_VALUE];
        loop {
            if o + KV_HEADER as u32 > self.capacity {
                break;
            }
            let mut hdr = [0u8; KV_HEADER];
            self.storage.read(o, &mut hdr)?;
            let tag = u16::from_le_bytes([hdr[0], hdr[1]]);
            if tag == 0 {
                break; // blank -> end of log
            }
            let len = u16::from_le_bytes([hdr[2], hdr[3]]);
            let crc = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
            if len as usize > MAX_VALUE || o + KV_HEADER as u32 + len as u32 > self.capacity {
                break; // implausible -> treat as end
            }
            self.storage
                .read(o + KV_HEADER as u32, &mut val[..len as usize])?;
            if entry_crc(&hdr[0..4], &val[..len as usize]) != crc {
                break; // corrupt -> end
            }
            if tag == target {
                found = Some((o, len)); // keep the latest occurrence
            }
            o += KV_HEADER as u32 + len as u32;
        }
        Ok((o, found))
    }

    /// Reclaim space taken by superseded records: keep only the latest record per
    /// key, packed from the start. Called automatically when an append won't fit.
    pub fn compact(&mut self) -> Result<(), Error> {
        // Pass 1: latest offset per tag, and the end of the live log.
        let mut latest: [(u16, u32); MAX_KEYS] = [(0, 0); MAX_KEYS];
        let mut nkeys = 0usize;
        let mut val = [0u8; MAX_VALUE];
        let mut o = 0u32;
        loop {
            if o + KV_HEADER as u32 > self.capacity {
                break;
            }
            let mut hdr = [0u8; KV_HEADER];
            self.storage.read(o, &mut hdr)?;
            let tag = u16::from_le_bytes([hdr[0], hdr[1]]);
            if tag == 0 {
                break;
            }
            let len = u16::from_le_bytes([hdr[2], hdr[3]]);
            let crc = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
            if len as usize > MAX_VALUE || o + KV_HEADER as u32 + len as u32 > self.capacity {
                break;
            }
            self.storage
                .read(o + KV_HEADER as u32, &mut val[..len as usize])?;
            if entry_crc(&hdr[0..4], &val[..len as usize]) != crc {
                break;
            }
            match latest[..nkeys].iter().position(|&(t, _)| t == tag) {
                Some(i) => latest[i].1 = o,
                None => {
                    if nkeys >= MAX_KEYS {
                        return Err(Error::TooManyKeys);
                    }
                    latest[nkeys] = (tag, o);
                    nkeys += 1;
                }
            }
            o += KV_HEADER as u32 + len as u32;
        }
        let old_free = o;

        // Pass 2: copy only the latest record of each key forward (dest <= src,
        // so the moves never clobber a not-yet-read source).
        let mut write = 0u32;
        let mut o = 0u32;
        let mut temp = [0u8; KV_HEADER + MAX_VALUE];
        while o < old_free {
            let mut hdr = [0u8; KV_HEADER];
            self.storage.read(o, &mut hdr)?;
            let tag = u16::from_le_bytes([hdr[0], hdr[1]]);
            if tag == 0 {
                break;
            }
            let len = u16::from_le_bytes([hdr[2], hdr[3]]);
            let size = KV_HEADER as u32 + len as u32;
            let is_latest = latest[..nkeys].iter().any(|&(t, off)| t == tag && off == o);
            if is_latest {
                if write != o {
                    self.storage.read(o, &mut temp[..size as usize])?;
                    self.storage.write(write, &temp[..size as usize])?;
                }
                write += size;
            }
            o += size;
        }

        // Pass 3: blank the freed tail so the scan terminates at the new end.
        let zeros = [0u8; 64];
        let mut z = write;
        while z < old_free {
            let chunk = ((old_free - z) as usize).min(zeros.len());
            self.storage.write(z, &zeros[..chunk])?;
            z += chunk as u32;
        }
        Ok(())
    }
}

/// CRC-32 (IEEE 802.3) over the record header bytes followed by the value.
fn entry_crc(hdr4: &[u8], value: &[u8]) -> u32 {
    let crc = crc32_update(0xFFFF_FFFF, hdr4);
    !crc32_update(crc, value)
}

/// One bitwise CRC-32 pass over `data`, continuing from `crc` (no table).
fn crc32_update(mut crc: u32, data: &[u8]) -> u32 {
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    crc
}
