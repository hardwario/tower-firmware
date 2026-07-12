//! Power-loss-safe key-value codec over a byte-addressable EEPROM — the storage engine that
//! backs [`tower::storage::Kv`] in the firmware.
//!
//! It lives in its own crate for one reason: the firmware is `no_std` and targets thumbv6m, so
//! it cannot `cargo test`. The codec, by contrast, is pure logic over a tiny [`Eeprom`] byte-store
//! trait — the firmware implements it over the L0 data EEPROM ([`tower::storage::Storage`]); the
//! tests here implement it over a RAM array — so the layout, the compaction commit, the
//! power-loss edges, and the legacy migration can all be exercised on the host.
//!
//! # Why double-buffered
//!
//! The catastrophic failure for this SDK is losing (or lowering) the radio TX-counter watermark:
//! the counter feeds the AES-CCM nonce, so a reset/rollback of it reuses a nonce. A single-region
//! append-log cannot survive a power-loss mid-write without risking exactly that (a torn in-place
//! update or a torn in-place compaction can orphan or resurface a record). So the region is split
//! into **two halves** plus two **superblocks** (`magic ‖ generation ‖ crc`):
//!
//! ```text
//!   [ half 0 records ][ half 1 records ][ super0 ][ super1 ]
//!    0              H  H             2H   top-24    top-12
//! ```
//!
//! - The **active** half is the one whose superblock is valid and has the highest generation.
//! - **Updates are append-only** within the active half — a torn write only ever corrupts the
//!   *tail* record, so `scan` stops there and every committed record before it survives.
//! - **Compaction is a flip:** blank the inactive half, write the live set packed into it, then
//!   write *its* superblock at `generation + 1`. That single CRC'd superblock write is the atomic
//!   commit — torn before it, the old half is still active (no data lost); torn during it, the
//!   superblock fails its CRC and the old half still wins.
//!
//! The record format inside a half is unchanged from the firmware's original single-region store
//! (`[tag(2) | len(2) | crc(4) | value]`, little-endian, CRC over `tag‖len‖value`), so legacy data
//! migrates without reserialization — see [`init`].
//!
//! **Deletion** rides the same append-only log: [`delete`] writes a **tombstone** — a zero-length
//! record (`len == 0`) — so the key reads back absent ([`get_bytes`]), and the next flip drops any
//! key whose latest record is a tombstone, reclaiming both the value and the tombstone. `len == 0`
//! is therefore reserved as the tombstone marker (no caller stores a genuinely empty value).
//!
//! # Incremental maintenance
//!
//! Because every pre-commit flip write is invisible until the superblock lands, the *same* flip
//! can also run in bounded slices from a background task: [`flip_start`] (RAM plan) →
//! [`flip_step`] (a few word-programs at a time) → [`flip_commit`] (tail catch-up + the atomic
//! superblock write), with [`blank_dead_step`] pre-blanking the dead half between flips and
//! [`maintain`] as the policy driver. All state is RAM-only; a reboot mid-flip restarts from
//! scratch with nothing lost. The synchronous [`compact`]/in-line flip remains as the fallback,
//! semantics unchanged. Motivation: on the STM32L0 each EEPROM word program stalls the CPU
//! ~3.4 ms, so a synchronous full-half flip freezes the chip for seconds (docs/storage.md).

#![cfg_attr(not(test), no_std)]

/// Largest value (bytes) one record can hold.
pub const MAX_VALUE: usize = 256;
/// Maximum number of distinct keys [`compact`] / migration can track in one pass.
pub const MAX_KEYS: usize = 64;
/// Record header: tag(2) + value-len(2) + crc32(4). Value bytes follow.
pub const KV_HEADER: usize = 8;

/// Superblock magic — also distinguishes a double-buffered region from legacy single-region data
/// (a legacy record's first bytes are a `u16` key, never this magic).
const SUPER_MAGIC: [u8; 4] = *b"TKV1";
/// Superblock size: magic(4) + generation(4 LE) + crc(4 LE, over magic‖generation).
const SUPER_LEN: u32 = 12;

/// A byte-addressable store the codec reads and writes. The firmware implements it over the
/// STM32L0 data EEPROM; host tests implement it over a RAM array. All offsets are bounded by the
/// `capacity` passed to the codec functions (the codec never addresses past it).
pub trait Eeprom {
    /// Backend error (e.g. the HAL flash error in the firmware; `()` in tests).
    type Error;
    /// Read `buf.len()` bytes starting at `off`.
    fn read(&self, off: u32, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Write `data` starting at `off`.
    fn write(&mut self, off: u32, data: &[u8]) -> Result<(), Self::Error>;
}

/// A key-value codec error. `Backend` wraps the [`Eeprom`] error so the firmware can preserve its
/// HAL flash error; the rest are codec-level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KvError<E> {
    /// The underlying EEPROM read/write failed (e.g. out of bounds).
    Backend(E),
    /// A value exceeded [`MAX_VALUE`] bytes.
    ValueTooLarge,
    /// The store (one half) is full even after a compaction flip — or the legacy live set being
    /// migrated does not fit one half.
    Full,
    /// Key `0` is reserved (it marks the end of the record log).
    InvalidKey,
    /// More distinct keys than [`MAX_KEYS`] (only hit during compaction / migration).
    TooManyKeys,
    /// A [`FlipState`] no longer matches the store (another flip committed underneath it).
    /// Returned by [`flip_step`]/[`flip_commit`] so a stale RAM plan can never be committed;
    /// the caller drops the state and starts over (see [`maintain`], which does exactly that).
    Stale,
}

impl<E: core::fmt::Display> core::fmt::Display for KvError<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            KvError::Backend(e) => write!(f, "backend error: {e}"),
            KvError::ValueTooLarge => f.write_str("value exceeds MAX_VALUE"),
            KvError::Full => f.write_str("store full"),
            KvError::InvalidKey => f.write_str("invalid key (0 is reserved)"),
            KvError::TooManyKeys => f.write_str("too many distinct keys"),
            KvError::Stale => f.write_str("stale flip state (store changed underneath)"),
        }
    }
}

/// CRC-32 (IEEE 802.3) over the record header bytes (`tag ‖ len`) followed by the value — the
/// `crc` field itself is excluded. Byte-for-byte the firmware's framing, via the shared primitive.
#[must_use]
pub fn entry_crc(hdr4: &[u8], value: &[u8]) -> u32 {
    let crc = tower_protocol::crc::crc32_update(0xFFFF_FFFF, hdr4);
    !tower_protocol::crc::crc32_update(crc, value)
}

// --- layout -------------------------------------------------------------------------------------

/// Record-area size of one half, given the total region `capacity`. The two superblocks sit in the
/// top `2 * SUPER_LEN` bytes; the rest splits evenly into the two halves.
fn half_len(capacity: u32) -> u32 {
    capacity.saturating_sub(2 * SUPER_LEN) / 2
}

/// Record-area base offset of half `h` (0 or 1).
fn half_base(capacity: u32, h: u8) -> u32 {
    h as u32 * half_len(capacity)
}

/// Offset of half `h`'s superblock (top of the region).
fn super_off(capacity: u32, h: u8) -> u32 {
    capacity - 2 * SUPER_LEN + h as u32 * SUPER_LEN
}

/// Read half `h`'s superblock generation, or `None` if it is absent / corrupt (bad magic or CRC).
fn read_gen<S: Eeprom>(store: &S, capacity: u32, h: u8) -> Result<Option<u32>, KvError<S::Error>> {
    let mut b = [0u8; SUPER_LEN as usize];
    store
        .read(super_off(capacity, h), &mut b)
        .map_err(KvError::Backend)?;
    if b[0..4] != SUPER_MAGIC {
        return Ok(None);
    }
    let g = u32::from_le_bytes([b[4], b[5], b[6], b[7]]);
    let crc = u32::from_le_bytes([b[8], b[9], b[10], b[11]]);
    if super_crc(&b[0..8]) != crc {
        return Ok(None);
    }
    Ok(Some(g))
}

/// Write half `h`'s superblock with `generation` — the atomic commit point of a flip. The CRC
/// makes even a torn superblock write fail closed (it won't validate, so the other half wins).
fn write_gen<S: Eeprom>(
    store: &mut S,
    capacity: u32,
    h: u8,
    generation: u32,
) -> Result<(), KvError<S::Error>> {
    let mut b = [0u8; SUPER_LEN as usize];
    b[0..4].copy_from_slice(&SUPER_MAGIC);
    b[4..8].copy_from_slice(&generation.to_le_bytes());
    let crc = super_crc(&b[0..8]);
    b[8..12].copy_from_slice(&crc.to_le_bytes());
    store.write(super_off(capacity, h), &b).map_err(KvError::Backend)
}

fn super_crc(b: &[u8]) -> u32 {
    !tower_protocol::crc::crc32_update(0xFFFF_FFFF, b)
}

/// The active half `(h, generation)`: the valid superblock with the highest generation, or `None`
/// if neither half is initialized yet (a fresh, un-migrated region).
fn active<S: Eeprom>(store: &S, capacity: u32) -> Result<Option<(u8, u32)>, KvError<S::Error>> {
    let g0 = read_gen(store, capacity, 0)?;
    let g1 = read_gen(store, capacity, 1)?;
    Ok(match (g0, g1) {
        (Some(a), Some(b)) => Some(if a >= b { (0, a) } else { (1, b) }),
        (Some(a), None) => Some((0, a)),
        (None, Some(b)) => Some((1, b)),
        (None, None) => None,
    })
}

// --- record scan within a half ------------------------------------------------------------------

/// Result of a [`scan_half`]: `(free_rel, latest_record_for_target)`, offsets **relative** to the
/// half's record base, where a record is `(rel_offset, value_len)`.
type HalfScan = (u32, Option<(u32, u16)>);

/// Scan one half's record area `[base, base + area)`: returns `(free_rel, latest(rel,len))`. Stops
/// at the first blank (`tag == 0`) or corrupt record — the end of a clean log, and where a torn
/// (tail) append would sit.
fn scan_half<S: Eeprom>(store: &S, base: u32, area: u32, target: u16) -> Result<HalfScan, KvError<S::Error>> {
    let mut o = 0u32;
    let mut found = None;
    let mut val = [0u8; MAX_VALUE];
    loop {
        if o + KV_HEADER as u32 > area {
            break;
        }
        let mut hdr = [0u8; KV_HEADER];
        store.read(base + o, &mut hdr).map_err(KvError::Backend)?;
        let tag = u16::from_le_bytes([hdr[0], hdr[1]]);
        if tag == 0 {
            break;
        }
        let len = u16::from_le_bytes([hdr[2], hdr[3]]);
        let crc = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
        if len as usize > MAX_VALUE || o + KV_HEADER as u32 + len as u32 > area {
            break;
        }
        store
            .read(base + o + KV_HEADER as u32, &mut val[..len as usize])
            .map_err(KvError::Backend)?;
        if entry_crc(&hdr[0..4], &val[..len as usize]) != crc {
            break;
        }
        if tag == target {
            found = Some((o, len));
        }
        o += KV_HEADER as u32 + len as u32;
    }
    Ok((o, found))
}

/// Append one `[tag|len|crc|value]` record at absolute offset `at`.
fn append<S: Eeprom>(store: &mut S, at: u32, key: u16, value: &[u8]) -> Result<(), KvError<S::Error>> {
    let len = value.len() as u16;
    let mut buf = [0u8; KV_HEADER + MAX_VALUE];
    buf[0..2].copy_from_slice(&key.to_le_bytes());
    buf[2..4].copy_from_slice(&len.to_le_bytes());
    let crc = entry_crc(&buf[0..4], value);
    buf[4..8].copy_from_slice(&crc.to_le_bytes());
    buf[8..8 + value.len()].copy_from_slice(value);
    store
        .write(at, &buf[..KV_HEADER + value.len()])
        .map_err(KvError::Backend)
}

/// Blank `len` bytes from `at` (so a half's record scan terminates at a clean end).
///
/// **Read-first**: only words that are actually nonzero get programmed, so blanking an
/// already-blank span (e.g. a dead half pre-blanked by [`blank_dead_step`]) costs reads only —
/// no wear, and on the STM32L0 no CPU-stalling word programs. This is what shrinks the in-line
/// fallback flip from "blank + copy" to "copy" once maintenance has pre-blanked (docs/storage.md).
fn blank<S: Eeprom>(store: &mut S, at: u32, len: u32) -> Result<(), KvError<S::Error>> {
    let mut unbounded = u32::MAX;
    blank_step_span(store, at, len, 0, &mut unbounded).map(|_| ())
}

/// Read-first blanking of `[base+from, base+len)`, programming at most `budget` zero-words
/// (already-zero words are skipped with reads only — idempotent, wear-free on re-runs).
/// Returns the new progress offset; `len` means the span is fully blank. The workhorse of
/// [`blank`] (unbounded) and [`blank_dead_step`]/[`flip_step`] (budgeted).
fn blank_step_span<S: Eeprom>(
    store: &mut S,
    base: u32,
    len: u32,
    from: u32,
    budget: &mut u32,
) -> Result<u32, KvError<S::Error>> {
    let zeros = [0u8; 4];
    let mut chunk = [0u8; 64];
    let mut o = from;
    while o < len {
        let n = ((len - o) as usize).min(chunk.len());
        store.read(base + o, &mut chunk[..n]).map_err(KvError::Backend)?;
        let mut w = 0usize;
        while w < n {
            let wn = (n - w).min(4);
            if chunk[w..w + wn].iter().any(|&b| b != 0) {
                if *budget == 0 {
                    return Ok(o + w as u32);
                }
                store
                    .write(base + o + w as u32, &zeros[..wn])
                    .map_err(KvError::Backend)?;
                *budget -= 1;
            }
            w += wn;
        }
        o += n as u32;
    }
    Ok(len)
}

// --- public API ---------------------------------------------------------------------------------

/// Initialize / migrate the region into the double-buffered layout. Idempotent: returns
/// immediately if a valid superblock already exists. Call once before first use (the firmware's
/// `Kv::new` does, best-effort).
///
/// Migration of legacy single-region data is non-destructive in the common case: legacy records
/// already live in half 0's record area, so if they fit one half this just commits half 0's
/// superblock (zero data movement — fully power-safe). A legacy log larger than one half is first
/// packed by a one-time legacy compaction (the only place the old non-atomic pack still runs; if
/// interrupted, no superblock was written, so the next boot re-migrates).
pub fn init<S: Eeprom>(store: &mut S, capacity: u32) -> Result<(), KvError<S::Error>> {
    if active(store, capacity)?.is_some() {
        return Ok(()); // already double-buffered
    }
    let half = half_len(capacity);
    let oldfree = legacy_free(store, capacity)?;
    if oldfree > half {
        // Legacy log spilled past one half — pack its live set to the front so it fits half 0.
        let live = legacy_compact(store, capacity)?;
        if live > half {
            return Err(KvError::Full); // live set genuinely too big for a half
        }
    }
    // Legacy records (if any) now occupy `[0, min(oldfree, half))` ⊂ half 0's record area, in the
    // same record format — so committing half 0's superblock adopts them verbatim. A fresh region
    // (oldfree == 0) commits an empty half 0. Half 1's superblock is left blank → invalid.
    write_gen(store, capacity, 0, 1)
}

/// Store raw `value` bytes under `key`. Append-only within the active half (a torn write only ever
/// corrupts the tail); when the half is full, a compaction flip frees space and the append retries
/// in the freshly-packed other half.
///
/// An **empty** `value` writes a delete tombstone (the key reads back absent) — prefer the named
/// [`delete`], which additionally skips the write when the key is already absent.
pub fn set_bytes<S: Eeprom>(
    store: &mut S,
    capacity: u32,
    key: u16,
    value: &[u8],
) -> Result<(), KvError<S::Error>> {
    set_bytes_with(store, capacity, key, value, &mut None)
}

/// [`set_bytes`], aware of an in-progress incremental flip (`pending`, shared with [`maintain`]).
///
/// Appends keep going to the **source** (active) half while a flip is pending — [`flip_commit`]'s
/// tail catch-up carries them over. If the source fills anyway, the pending flip is **finished
/// synchronously** (bounded: only its remaining steps + tail, not a from-scratch flip) and the
/// append retries in the fresh half — this path never fails where plain [`set_bytes`] would have
/// succeeded. A stale `pending` (a synchronous flip got there first) is simply dropped.
pub fn set_bytes_with<S: Eeprom>(
    store: &mut S,
    capacity: u32,
    key: u16,
    value: &[u8],
    pending: &mut Option<FlipState>,
) -> Result<(), KvError<S::Error>> {
    if key == 0 {
        return Err(KvError::InvalidKey);
    }
    if value.len() > MAX_VALUE {
        return Err(KvError::ValueTooLarge);
    }
    let (h, g) = match active(store, capacity)? {
        Some(x) => x,
        None => {
            init(store, capacity)?;
            (0, 1)
        }
    };
    let half = half_len(capacity);
    let needed = KV_HEADER as u32 + value.len() as u32;

    let base = half_base(capacity, h);
    let (free, _) = scan_half(store, base, half, key)?;
    if free + needed <= half {
        return append(store, base + free, key, value);
    }

    // Active half full → flip-compact into the other half, then append there. Prefer finishing
    // a pending incremental flip (its blank/copy work is already partly done); otherwise the
    // synchronous in-line flip, exactly as before.
    match pending.take() {
        Some(mut st) if (st.src_h, st.src_gen) == (h, g) => flip_commit(store, capacity, &mut st)?,
        _ => flip(store, capacity, h, g)?,
    }
    let (h2, _) = active(store, capacity)?.ok_or(KvError::Full)?;
    let base2 = half_base(capacity, h2);
    let (free2, _) = scan_half(store, base2, half, key)?;
    if free2 + needed > half {
        return Err(KvError::Full);
    }
    append(store, base2 + free2, key, value)
}

/// Read the raw bytes stored under `key` into `out`; returns the value's true length (which may
/// exceed `out.len()` — only `out.len()` bytes are copied), or `None` if the key is absent.
pub fn get_bytes<S: Eeprom>(
    store: &S,
    capacity: u32,
    key: u16,
    out: &mut [u8],
) -> Result<Option<usize>, KvError<S::Error>> {
    if key == 0 {
        return Ok(None);
    }
    let Some((h, _)) = active(store, capacity)? else {
        return Ok(None); // un-initialized region → empty
    };
    let base = half_base(capacity, h);
    match scan_half(store, base, half_len(capacity), key)?.1 {
        // A latest record of length 0 is a delete **tombstone** (see [`delete`]): the key was
        // removed, so it reads back absent exactly like one never set. The tombstone bytes stay
        // on disk until the next flip drops them.
        Some((_, 0)) | None => Ok(None),
        Some((off, len)) => {
            let n = (len as usize).min(out.len());
            store
                .read(base + off + KV_HEADER as u32, &mut out[..n])
                .map_err(KvError::Backend)?;
            Ok(Some(len as usize))
        }
    }
}

/// Delete `key`: append a **tombstone** (a zero-length record) so the key reads back absent.
/// Append-only and power-loss-safe like any [`set_bytes`] — a torn tombstone write only corrupts
/// the tail. The key's old value and the tombstone are both reclaimed by the next compaction
/// [`flip`] (which drops any key whose latest record is a tombstone); until then they occupy
/// space. A no-op (reads only, no write, no wear) if the key is already absent or tombstoned, so
/// deleting a missing key never grows the log.
pub fn delete<S: Eeprom>(store: &mut S, capacity: u32, key: u16) -> Result<(), KvError<S::Error>> {
    delete_with(store, capacity, key, &mut None)
}

/// [`delete`], aware of an in-progress incremental flip (`pending`, shared with [`maintain`]) —
/// the tombstone append follows the same source-half / finish-pending-flip rules as
/// [`set_bytes_with`].
pub fn delete_with<S: Eeprom>(
    store: &mut S,
    capacity: u32,
    key: u16,
    pending: &mut Option<FlipState>,
) -> Result<(), KvError<S::Error>> {
    if key == 0 {
        return Err(KvError::InvalidKey);
    }
    // Skip the write when there is nothing to delete: an un-initialized region, a key never set,
    // or one whose latest record is already a tombstone. Keeps a redundant delete free of wear.
    match active(store, capacity)? {
        None => return Ok(()),
        Some((h, _)) => {
            let base = half_base(capacity, h);
            match scan_half(store, base, half_len(capacity), key)?.1 {
                Some((_, len)) if len > 0 => {} // a live value — fall through to tombstone it
                _ => return Ok(()),             // absent or already tombstoned
            }
        }
    }
    // A zero-length value IS the tombstone; reuse the append path (incl. flip-on-full). If the
    // half is full, the flip first packs the live set (this key's value included), then the
    // tombstone lands in the fresh half and supersedes it — correct, self-healing on the next flip.
    set_bytes_with(store, capacity, key, &[], pending)
}

/// Reclaim space taken by superseded records via a power-safe **flip**: the live set is packed
/// into the inactive half and committed by one superblock write. A no-op on an un-initialized
/// region.
pub fn compact<S: Eeprom>(store: &mut S, capacity: u32) -> Result<(), KvError<S::Error>> {
    if let Some((h, g)) = active(store, capacity)? {
        flip(store, capacity, h, g)?;
    }
    Ok(())
}

/// [`compact`], aware of an in-progress incremental flip (`pending`, shared with [`maintain`]):
/// a fresh pending flip is finished (that *is* the requested compaction — cheaper than starting
/// over); a stale or absent one falls back to the synchronous flip. Consumes `pending` either way.
pub fn compact_with<S: Eeprom>(
    store: &mut S,
    capacity: u32,
    pending: &mut Option<FlipState>,
) -> Result<(), KvError<S::Error>> {
    match pending.take() {
        Some(mut st) => match flip_commit(store, capacity, &mut st) {
            Err(KvError::Stale) => compact(store, capacity),
            r => r,
        },
        None => compact(store, capacity),
    }
}

/// The active half's **generation** — a monotonic counter bumped once per compaction [`flip`]
/// (its atomic commit), so it doubles as the store's **lifetime flip count**: the load-bearing
/// input to an EEPROM-wear estimate, since each flip cycles ~one half's cells. `0` on an
/// un-initialized region. A pure read (two superblock reads, no write), so polling it for
/// telemetry never itself contributes to wear.
pub fn generation<S: Eeprom>(store: &S, capacity: u32) -> Result<u32, KvError<S::Error>> {
    Ok(active(store, capacity)?.map(|(_, g)| g).unwrap_or(0))
}

/// Flip: pack the live set of the active half `src_h` (generation `src_gen`) into the inactive
/// half, then commit by writing the inactive half's superblock at `src_gen + 1`.
fn flip<S: Eeprom>(store: &mut S, capacity: u32, src_h: u8, src_gen: u32) -> Result<(), KvError<S::Error>> {
    let half = half_len(capacity);
    let src_base = half_base(capacity, src_h);
    let dst_h = 1 - src_h;
    let dst_base = half_base(capacity, dst_h);

    // Pass 1: latest offset per tag in the source half (+ the source log's end).
    // `tmp` is reused as pass 1's CRC-check scratch and pass 2's record-copy buffer — the two
    // uses are strictly sequential, so one buffer serves both. This keeps a second MAX_VALUE
    // array off the compaction stack: flip runs synchronously inside the executor poll (via
    // set_bytes ← Nv::with ← net.send), the deepest stack path in a radio app, where peak
    // headroom is tight (see docs/radio.md — no stack guard on this target).
    let mut latest: [(u16, u32); MAX_KEYS] = [(0, 0); MAX_KEYS];
    let mut nkeys = 0usize;
    let mut tmp = [0u8; KV_HEADER + MAX_VALUE];
    let src_free = walk_latest(store, src_base, half, &mut latest, &mut nkeys, &mut tmp)?;

    // Blank the destination half so its scan will terminate cleanly (it may hold stale records
    // from when it was active two generations ago). Its superblock is untouched here, so the
    // source half stays active throughout — the blank/copy are not yet committed.
    blank(store, dst_base, half)?;

    // Pass 2: copy each latest record forward into the destination, packed.
    let mut wr = 0u32;
    let mut o = 0u32;
    while o < src_free {
        let mut hdr = [0u8; KV_HEADER];
        store.read(src_base + o, &mut hdr).map_err(KvError::Backend)?;
        let tag = u16::from_le_bytes([hdr[0], hdr[1]]);
        if tag == 0 {
            break;
        }
        let len = u16::from_le_bytes([hdr[2], hdr[3]]);
        let size = KV_HEADER as u32 + len as u32;
        // Copy a record only if it is the latest for its tag AND not a delete tombstone
        // (`len == 0`): a tombstoned key is dropped here, reclaiming both its value and the
        // tombstone. Superseded records (older than the latest) fall through uncopied as before.
        if len != 0 && latest[..nkeys].iter().any(|&(t, off)| t == tag && off == o) {
            if wr + size > half {
                return Err(KvError::Full); // live set doesn't fit a half
            }
            store
                .read(src_base + o, &mut tmp[..size as usize])
                .map_err(KvError::Backend)?;
            store
                .write(dst_base + wr, &tmp[..size as usize])
                .map_err(KvError::Backend)?;
            wr += size;
        }
        o += size;
    }

    // Commit: the destination becomes active the instant this CRC'd superblock lands.
    write_gen(store, capacity, dst_h, src_gen.wrapping_add(1))
}

/// Walk a half's records recording the latest offset per tag; returns the log's end offset (rel).
fn walk_latest<S: Eeprom>(
    store: &S,
    base: u32,
    area: u32,
    latest: &mut [(u16, u32); MAX_KEYS],
    nkeys: &mut usize,
    val: &mut [u8],
) -> Result<u32, KvError<S::Error>> {
    let mut o = 0u32;
    loop {
        if o + KV_HEADER as u32 > area {
            break;
        }
        let mut hdr = [0u8; KV_HEADER];
        store.read(base + o, &mut hdr).map_err(KvError::Backend)?;
        let tag = u16::from_le_bytes([hdr[0], hdr[1]]);
        if tag == 0 {
            break;
        }
        let len = u16::from_le_bytes([hdr[2], hdr[3]]);
        let crc = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
        if len as usize > MAX_VALUE || o + KV_HEADER as u32 + len as u32 > area {
            break;
        }
        store
            .read(base + o + KV_HEADER as u32, &mut val[..len as usize])
            .map_err(KvError::Backend)?;
        if entry_crc(&hdr[0..4], &val[..len as usize]) != crc {
            break;
        }
        match latest[..*nkeys].iter().position(|&(t, _)| t == tag) {
            Some(i) => latest[i].1 = o,
            None => {
                if *nkeys >= MAX_KEYS {
                    return Err(KvError::TooManyKeys);
                }
                latest[*nkeys] = (tag, o);
                *nkeys += 1;
            }
        }
        o += KV_HEADER as u32 + len as u32;
    }
    Ok(o)
}

// --- incremental maintenance ---------------------------------------------------------------------
//
// A synchronous flip costs *time*, not just wear: on the STM32L0 every EEPROM word program stalls
// the whole CPU ~3.4 ms (instruction fetch halts, ISRs freeze), so blanking + re-packing a full
// half is seconds of chip freeze (docs/storage.md, bench-measured). The pieces below split the
// same work into bounded, RAM-tracked steps a background task can run a few word-programs at a
// time. Power-loss safety is inherited from the two-half + superblock-commit design: nothing
// before [`flip_commit`]'s single superblock write changes the active half, so a reboot anywhere
// mid-maintenance just forgets the RAM state and restarts from scratch — no on-EEPROM state
// machine, no recovery path.

/// Proactive-flip threshold for [`maintain`]: start an incremental flip once the active half's
/// free space drops below this many bytes.
///
/// Rationale: appends during an incremental flip keep landing in the *source* half, so the
/// trigger must leave room for the writes that arrive between crossing the threshold and the
/// commit. Two worst-case records (`2 × (KV_HEADER + MAX_VALUE)` = 528 B) cover the append that
/// crossed the threshold plus a maximal burst record during the drain — generous, given the
/// maintenance task is woken by that very write and drains the flip in back-to-back slices,
/// while real SDK records are 4–8 byte scalars. And the guarantee is soft by design: if a
/// pathological burst fills the source anyway, [`set_bytes_with`] finishes the pending flip
/// synchronously (bounded by its *remaining* steps) — never a failure, just latency.
pub const FLIP_THRESHOLD: u32 = 2 * (KV_HEADER + MAX_VALUE) as u32;

/// Cost of writing `len` bytes, in EEPROM word-programs — the unit of every maintenance budget
/// (each word program stalls the STM32L0 ~3.4 ms). Conservative rounding; unaligned spans may
/// add up to two edge programs, so a slice can overshoot its budget by ~2 words (~7 ms).
fn words(len: u32) -> u32 {
    len.div_ceil(4)
}

/// Free (appendable) bytes remaining in the active half — a pure read (one record scan). On an
/// un-initialized region, reports one full half (what [`init`] would leave). Feeds the
/// [`maintain`] threshold check and `/system/eeprom print`.
pub fn free_bytes<S: Eeprom>(store: &S, capacity: u32) -> Result<u32, KvError<S::Error>> {
    let half = half_len(capacity);
    match active(store, capacity)? {
        Some((h, _)) => {
            let (end, _) = scan_half(store, half_base(capacity, h), half, 0)?;
            Ok(half - end)
        }
        None => Ok(half),
    }
}

/// Bytes the live set (latest record per key) occupies — the packed size a flip would leave.
/// A pure read; `0` on an un-initialized region. `free_bytes + (used − live_bytes)` is the
/// space one flip reclaims.
pub fn live_bytes<S: Eeprom>(store: &S, capacity: u32) -> Result<u32, KvError<S::Error>> {
    let Some((h, _)) = active(store, capacity)? else {
        return Ok(0);
    };
    let mut latest: [(u16, u32); MAX_KEYS] = [(0, 0); MAX_KEYS];
    let mut nkeys = 0usize;
    let mut tmp = [0u8; KV_HEADER + MAX_VALUE];
    let base = half_base(capacity, h);
    walk_latest(store, base, half_len(capacity), &mut latest, &mut nkeys, &mut tmp)?;
    let mut total = 0u32;
    for &(_, off) in &latest[..nkeys] {
        let mut hdr = [0u8; KV_HEADER];
        store.read(base + off, &mut hdr).map_err(KvError::Backend)?;
        let len = u16::from_le_bytes([hdr[2], hdr[3]]);
        // A tombstoned key (latest len 0) leaves nothing behind after a flip — exclude it so
        // this stays an exact "packed size a flip would leave".
        if len != 0 {
            total += KV_HEADER as u32 + len as u32;
        }
    }
    Ok(total)
}

/// Whether the dead (inactive) half is fully blank — a pure read; `true` on an un-initialized
/// region. Exactly the condition under which the next flip's blank pass costs nothing.
pub fn dead_half_blank<S: Eeprom>(store: &S, capacity: u32) -> Result<bool, KvError<S::Error>> {
    let Some((h, _)) = active(store, capacity)? else {
        return Ok(true);
    };
    let base = half_base(capacity, 1 - h);
    let half = half_len(capacity);
    let mut chunk = [0u8; 64];
    let mut o = 0u32;
    while o < half {
        let n = ((half - o) as usize).min(chunk.len());
        store.read(base + o, &mut chunk[..n]).map_err(KvError::Backend)?;
        if chunk[..n].iter().any(|&b| b != 0) {
            return Ok(false);
        }
        o += n as u32;
    }
    Ok(true)
}

/// RAM-only progress cursor for [`blank_dead_step`]. A fresh cursor (or a reboot, which loses
/// it) restarts from the top of the dead half — near-free, because already-blank words are
/// skipped with reads only. The recorded generation makes it self-healing: when a flip changes
/// the active half, the cursor rewinds automatically.
#[derive(Clone, Copy, Default)]
pub struct BlankCursor {
    generation: u32,
    off: u32,
}

impl BlankCursor {
    /// A cursor starting at the top of the (current) dead half.
    pub const fn new() -> Self {
        Self {
            generation: 0,
            off: 0,
        }
    }
}

/// Incrementally blank the **dead** (inactive) half — "pre-blanking", run any time after a flip
/// commits, so the *next* flip (incremental or the in-line fallback) skips its blank pass.
/// Programs at most `budget_words` zero-words per call; already-zero words are skipped with
/// reads only (idempotent, no wear on blank cells). Returns whether blanking work remains.
pub fn blank_dead_step<S: Eeprom>(
    store: &mut S,
    capacity: u32,
    cur: &mut BlankCursor,
    budget_words: u32,
) -> Result<bool, KvError<S::Error>> {
    let Some((h, g)) = active(store, capacity)? else {
        return Ok(false); // un-initialized region — no dead half yet
    };
    if cur.generation != g {
        *cur = BlankCursor {
            generation: g,
            off: 0,
        }; // active half changed → rewind
    }
    let half = half_len(capacity);
    if cur.off >= half {
        return Ok(false);
    }
    let mut budget = budget_words;
    cur.off = blank_step_span(store, half_base(capacity, 1 - h), half, cur.off, &mut budget)?;
    Ok(cur.off < half)
}

/// RAM-only state of an in-progress **incremental** flip ([`flip_start`] → [`flip_step`]* →
/// [`flip_commit`]). Reboot-safe by construction: the source half stays active until the
/// commit's superblock write, so losing this state (power loss, task restart) leaves the store
/// exactly as committed and the flip simply restarts from scratch.
pub struct FlipState {
    src_h: u8,
    src_gen: u32,
    /// Source log end at [`flip_start`]. Appends may land past it (they keep going to the
    /// source half); [`flip_commit`] re-walks that tail.
    src_end: u32,
    /// Pass-1 plan: latest record offset per tag within `[0, src_end)`.
    latest: [(u16, u32); MAX_KEYS],
    nkeys: usize,
    /// Destination blank progress (phase A of [`flip_step`]).
    blank_off: u32,
    /// Source scan progress of the copy (phase B).
    read_off: u32,
    /// Bytes of planned records fully copied — the destination write position.
    write_off: u32,
    /// Bytes of the record at `read_off` already copied (mid-record resume between steps).
    rec_copied: u32,
}

/// Begin an incremental flip: pass-1 scan of the active half into a RAM plan (reads only — no
/// EEPROM writes, no stall). `None` on an un-initialized region.
pub fn flip_start<S: Eeprom>(store: &S, capacity: u32) -> Result<Option<FlipState>, KvError<S::Error>> {
    let Some((h, g)) = active(store, capacity)? else {
        return Ok(None);
    };
    let mut latest: [(u16, u32); MAX_KEYS] = [(0, 0); MAX_KEYS];
    let mut nkeys = 0usize;
    let mut tmp = [0u8; KV_HEADER + MAX_VALUE];
    let src_end = walk_latest(
        store,
        half_base(capacity, h),
        half_len(capacity),
        &mut latest,
        &mut nkeys,
        &mut tmp,
    )?;
    Ok(Some(FlipState {
        src_h: h,
        src_gen: g,
        src_end,
        latest,
        nkeys,
        blank_off: 0,
        read_off: 0,
        write_off: 0,
        rec_copied: 0,
    }))
}

/// Whether `st` still describes the store (no other flip committed since [`flip_start`]).
fn flip_fresh<S: Eeprom>(store: &S, capacity: u32, st: &FlipState) -> Result<bool, KvError<S::Error>> {
    Ok(active(store, capacity)? == Some((st.src_h, st.src_gen)))
}

/// One bounded slice of flip work: blank the destination if still needed (read-first — free when
/// pre-blanked by [`blank_dead_step`]), then copy the planned live set, programming at most
/// `budget_words` words. Returns whether pre-commit work remains (`false` = ready for
/// [`flip_commit`]). [`KvError::Stale`] if another flip committed underneath.
pub fn flip_step<S: Eeprom>(
    store: &mut S,
    capacity: u32,
    st: &mut FlipState,
    budget_words: u32,
) -> Result<bool, KvError<S::Error>> {
    if !flip_fresh(store, capacity, st)? {
        return Err(KvError::Stale);
    }
    let half = half_len(capacity);
    let src_base = half_base(capacity, st.src_h);
    let dst_base = half_base(capacity, 1 - st.src_h);
    let mut budget = budget_words;

    // Phase A: blank the destination so stale records from two generations ago can't survive
    // past the copied set (scan would trust a stale-but-valid record; see `flip`).
    if st.blank_off < half {
        st.blank_off = blank_step_span(store, dst_base, half, st.blank_off, &mut budget)?;
        if st.blank_off < half {
            return Ok(true); // budget exhausted mid-blank
        }
    }

    // Phase B: copy the planned records forward, packed — resumable mid-record, so one step
    // never programs more than the budget even for a MAX_VALUE record. A partially-copied
    // record is harmless: the destination is uncommitted until `flip_commit`.
    let mut tmp = [0u8; KV_HEADER + MAX_VALUE];
    while st.read_off < st.src_end {
        let mut hdr = [0u8; KV_HEADER];
        store
            .read(src_base + st.read_off, &mut hdr)
            .map_err(KvError::Backend)?;
        let tag = u16::from_le_bytes([hdr[0], hdr[1]]);
        if tag == 0 {
            break; // defensive: walk_latest validated [0, src_end)
        }
        let len = u16::from_le_bytes([hdr[2], hdr[3]]);
        let size = KV_HEADER as u32 + len as u32;
        let planned = st.latest[..st.nkeys]
            .iter()
            .any(|&(t, off)| t == tag && off == st.read_off);
        if !planned || len == 0 {
            // superseded, or a delete tombstone (len 0) whose key is dropped — skip either way
            st.read_off += size;
            continue;
        }
        if st.write_off + size > half {
            return Err(KvError::Full); // unreachable (live ⊆ source), kept defensive
        }
        while st.rec_copied < size {
            if budget == 0 {
                return Ok(true);
            }
            let chunk = (size - st.rec_copied).min(budget.saturating_mul(4));
            store
                .read(src_base + st.read_off + st.rec_copied, &mut tmp[..chunk as usize])
                .map_err(KvError::Backend)?;
            store
                .write(dst_base + st.write_off + st.rec_copied, &tmp[..chunk as usize])
                .map_err(KvError::Backend)?;
            budget = budget.saturating_sub(words(chunk));
            st.rec_copied += chunk;
        }
        st.rec_copied = 0;
        st.read_off += size;
        st.write_off += size;
    }
    Ok(false)
}

/// Finish an incremental flip: drive any remaining [`flip_step`] work, **catch up the source
/// tail** (records appended past `src_end` since [`flip_start`], copied verbatim in append order
/// — a tail record superseding a planned or earlier-tail one lands *later* in the destination,
/// so latest-wins scan semantics resolve it), then commit by writing the destination superblock
/// at `generation + 1` — the same single CRC'd atomic commit as the synchronous flip.
/// [`KvError::Stale`] if another flip committed underneath (nothing is written in that case).
pub fn flip_commit<S: Eeprom>(
    store: &mut S,
    capacity: u32,
    st: &mut FlipState,
) -> Result<(), KvError<S::Error>> {
    // Freshness is checked inside flip_step (first call), so a stale plan can't reach the
    // superblock write below.
    while flip_step(store, capacity, st, u32::MAX)? {}

    let half = half_len(capacity);
    let src_base = half_base(capacity, st.src_h);
    let dst_base = half_base(capacity, 1 - st.src_h);

    // Tail catch-up: same validated walk as `scan_half`, copying every intact record whole.
    // Total destination usage = packed live set (≤ src_end) + tail (= src_now − src_end)
    // ≤ src_now ≤ half, so this always fits; the check stays as a defensive invariant.
    let mut tmp = [0u8; KV_HEADER + MAX_VALUE];
    let mut o = st.src_end;
    loop {
        if o + KV_HEADER as u32 > half {
            break;
        }
        store
            .read(src_base + o, &mut tmp[..KV_HEADER])
            .map_err(KvError::Backend)?;
        let tag = u16::from_le_bytes([tmp[0], tmp[1]]);
        if tag == 0 {
            break;
        }
        let len = u16::from_le_bytes([tmp[2], tmp[3]]);
        let crc = u32::from_le_bytes([tmp[4], tmp[5], tmp[6], tmp[7]]);
        if len as usize > MAX_VALUE || o + KV_HEADER as u32 + len as u32 > half {
            break;
        }
        let size = KV_HEADER as u32 + len as u32;
        store
            .read(
                src_base + o + KV_HEADER as u32,
                &mut tmp[KV_HEADER..size as usize],
            )
            .map_err(KvError::Backend)?;
        if entry_crc(&tmp[0..4], &tmp[KV_HEADER..size as usize]) != crc {
            break; // torn tail append — end of the clean log, drop it as scan would
        }
        if st.write_off + size > half {
            return Err(KvError::Full);
        }
        store
            .write(dst_base + st.write_off, &tmp[..size as usize])
            .map_err(KvError::Backend)?;
        st.write_off += size;
        o += size;
    }

    // Commit: the destination becomes active the instant this CRC'd superblock lands.
    write_gen(store, capacity, 1 - st.src_h, st.src_gen.wrapping_add(1))
}

/// RAM-only maintenance state: the pre-blanking cursor plus at most one pending incremental
/// flip. Keep it next to the store handle and share [`pending`](Self::pending) with
/// [`set_bytes_with`]/[`compact_with`], so a store that fills mid-flip *finishes* the flip
/// instead of restarting one. Losing it (reboot) is always safe — see [`FlipState`].
pub struct MaintState {
    blank: BlankCursor,
    flip: Option<FlipState>,
}

impl MaintState {
    /// Fresh state: nothing pending, blanking restarts from the top (near-free if already blank).
    pub const fn new() -> Self {
        Self {
            blank: BlankCursor::new(),
            flip: None,
        }
    }

    /// The pending incremental-flip slot, for [`set_bytes_with`]/[`compact_with`].
    pub fn pending(&mut self) -> &mut Option<FlipState> {
        &mut self.flip
    }
}

impl Default for MaintState {
    fn default() -> Self {
        Self::new()
    }
}

/// One bounded maintenance slice (at most `budget_words` EEPROM word-programs). In priority
/// order: (1) advance/commit a pending incremental flip, (2) start one when the active half's
/// free space is below `threshold` (see [`FLIP_THRESHOLD`]) *and* a flip would restore it,
/// (3) pre-blank the dead half. Returns whether work remains — call again (after yielding to
/// other tasks) until `false`, then sleep until the next store write.
///
/// The commit slice is the one call that can exceed the budget: it also copies the tail of
/// appends that landed during the flip (bounded by `threshold`, typically zero or one small
/// record) plus the 12-byte superblock.
pub fn maintain<S: Eeprom>(
    store: &mut S,
    capacity: u32,
    m: &mut MaintState,
    budget_words: u32,
    threshold: u32,
) -> Result<bool, KvError<S::Error>> {
    let Some((h, g)) = active(store, capacity)? else {
        return Ok(false); // un-initialized: nothing to maintain until the first write
    };

    // (1) Drive a pending flip; drop it if stale (a synchronous flip committed underneath).
    if m.flip.as_ref().is_some_and(|st| (st.src_h, st.src_gen) != (h, g)) {
        m.flip = None;
    }
    if let Some(st) = &mut m.flip {
        if flip_step(store, capacity, st, budget_words)? {
            return Ok(true);
        }
        let mut st = m.flip.take().unwrap();
        flip_commit(store, capacity, &mut st)?;
        return Ok(true); // the old half is now dead + dirty → pre-blanking work remains
    }

    // (2) Start a flip below the threshold — but only if it would lift free space back over it
    // (live + threshold ≤ half). Otherwise the live set is simply too big for proactive help:
    // flipping would burn wear without restoring headroom, so leave it to the in-line fallback
    // (whose blank pass the pre-blanked dead half already halves).
    let half = half_len(capacity);
    let (end, _) = scan_half(store, half_base(capacity, h), half, 0)?;
    if half - end < threshold && live_bytes(store, capacity)? + threshold <= half {
        m.flip = flip_start(store, capacity)?;
        return Ok(true); // pass-1 was reads only; the next slice starts programming
    }

    // (3) Pre-blank the dead half so the next flip skips its blank pass.
    blank_dead_step(store, capacity, &mut m.blank, budget_words)
}

// --- legacy single-region migration -------------------------------------------------------------
//
// The pre-double-buffer store laid records from offset 0 over the whole region with no superblock.
// These two helpers read/pack that layout during a one-time [`init`] migration only.

/// End offset of the legacy single-region log (scanning records from offset 0 over the whole
/// region). `0` means no legacy data (a fresh region).
fn legacy_free<S: Eeprom>(store: &S, capacity: u32) -> Result<u32, KvError<S::Error>> {
    // The whole region is the legacy record area; reuse the half scanner with base 0 / area=cap.
    Ok(scan_half(store, 0, capacity, 0)?.0)
}

/// One-time legacy in-place compaction: pack the legacy live set to `[0, live)` and return `live`.
/// Front-packing keeps `dest <= src`, so the copy never clobbers a not-yet-read source. Used only
/// for a legacy log too large for one half; if interrupted, [`init`] re-runs (no superblock yet).
fn legacy_compact<S: Eeprom>(store: &mut S, capacity: u32) -> Result<u32, KvError<S::Error>> {
    let mut latest: [(u16, u32); MAX_KEYS] = [(0, 0); MAX_KEYS];
    let mut nkeys = 0usize;
    // Reuse one buffer for the CRC-check scan (pass 1) and the record copy (pass 2); see `flip`.
    let mut tmp = [0u8; KV_HEADER + MAX_VALUE];
    let old_free = walk_latest(store, 0, capacity, &mut latest, &mut nkeys, &mut tmp)?;

    let mut wr = 0u32;
    let mut o = 0u32;
    while o < old_free {
        let mut hdr = [0u8; KV_HEADER];
        store.read(o, &mut hdr).map_err(KvError::Backend)?;
        let tag = u16::from_le_bytes([hdr[0], hdr[1]]);
        if tag == 0 {
            break;
        }
        let len = u16::from_le_bytes([hdr[2], hdr[3]]);
        let size = KV_HEADER as u32 + len as u32;
        if latest[..nkeys].iter().any(|&(t, off)| t == tag && off == o) {
            if wr != o {
                store
                    .read(o, &mut tmp[..size as usize])
                    .map_err(KvError::Backend)?;
                store.write(wr, &tmp[..size as usize]).map_err(KvError::Backend)?;
            }
            wr += size;
        }
        o += size;
    }
    // Blank the freed tail through the old end so a later scan/migration sees a clean boundary.
    blank(store, wr, old_free - wr)?;
    Ok(wr)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    const CAP: u32 = 512; // small enough to force flips quickly in tests

    /// RAM-backed [`Eeprom`]; `write` programs bytes verbatim (byte-addressable, like the L0 data
    /// EEPROM). `torn_write` simulates a power-loss mid-program (only a prefix lands).
    /// `words_written` mirrors the codec's own budget accounting (`words()` per write call), so
    /// the maintenance tests can assert budgets and skip-if-zero wear-freedom.
    struct Ram {
        buf: Vec<u8>,
        words_written: usize,
    }
    impl Ram {
        fn new(cap: usize) -> Self {
            Self {
                buf: vec![0u8; cap],
                words_written: 0,
            }
        }
        /// Simulate a torn write: only the first `landed` bytes of `data` are programmed.
        fn torn_write(&mut self, off: u32, data: &[u8], landed: usize) {
            let n = landed.min(data.len());
            self.buf[off as usize..off as usize + n].copy_from_slice(&data[..n]);
        }
        /// Write a legacy single-region record at `off`, returning the next offset (for building
        /// pre-double-buffer fixtures in the migration tests).
        fn legacy_put(&mut self, off: u32, key: u16, value: &[u8]) -> u32 {
            append(self, off, key, value).unwrap();
            off + KV_HEADER as u32 + value.len() as u32
        }
    }
    impl Eeprom for Ram {
        type Error = ();
        fn read(&self, off: u32, buf: &mut [u8]) -> Result<(), ()> {
            buf.copy_from_slice(self.buf.get(off as usize..off as usize + buf.len()).ok_or(())?);
            Ok(())
        }
        fn write(&mut self, off: u32, data: &[u8]) -> Result<(), ()> {
            let s = off as usize;
            self.buf
                .get_mut(s..s + data.len())
                .ok_or(())?
                .copy_from_slice(data);
            self.words_written += data.len().div_ceil(4);
            Ok(())
        }
    }

    fn set(ram: &mut Ram, key: u16, value: &[u8]) -> Result<(), KvError<()>> {
        set_bytes(ram, CAP, key, value)
    }
    fn get_vec(ram: &Ram, key: u16) -> Option<Vec<u8>> {
        let mut out = [0u8; MAX_VALUE];
        get_bytes(ram, CAP, key, &mut out)
            .unwrap()
            .map(|n| out[..n].to_vec())
    }

    #[test]
    fn roundtrip_and_absent() {
        let mut ram = Ram::new(CAP as usize);
        assert_eq!(get_vec(&ram, 1), None);
        set(&mut ram, 1, b"hello").unwrap();
        assert_eq!(get_vec(&ram, 1).as_deref(), Some(&b"hello"[..]));
        assert_eq!(get_vec(&ram, 2), None);
    }

    #[test]
    fn key_zero_reserved_and_value_too_large() {
        let mut ram = Ram::new(CAP as usize);
        assert_eq!(set(&mut ram, 0, b"x"), Err(KvError::InvalidKey));
        assert_eq!(get_bytes(&ram, CAP, 0, &mut [0u8; 4]).unwrap(), None);
        assert_eq!(
            set(&mut ram, 1, &[0u8; MAX_VALUE + 1]),
            Err(KvError::ValueTooLarge)
        );
    }

    #[test]
    fn append_only_latest_wins() {
        let mut ram = Ram::new(CAP as usize);
        set(&mut ram, 1, &[1, 2, 3, 4]).unwrap();
        set(&mut ram, 1, &[9, 9, 9, 9]).unwrap(); // same length still appends (no in-place)
        set(&mut ram, 1, &[5, 6]).unwrap(); // different length appends
        assert_eq!(get_vec(&ram, 1).as_deref(), Some(&[5, 6][..]));
    }

    #[test]
    fn multiple_keys_independent_across_flips() {
        let mut ram = Ram::new(CAP as usize);
        set(&mut ram, 0x5202, b"keepme").unwrap();
        // Churn key 0x5201 enough to force several compaction flips; 0x5202 must survive each.
        for i in 0..400u32 {
            set(&mut ram, 0x5201, &i.to_le_bytes()).unwrap();
            assert_eq!(
                get_vec(&ram, 0x5202).as_deref(),
                Some(&b"keepme"[..]),
                "lost key across flip at i={i}"
            );
        }
        assert_eq!(get_vec(&ram, 0x5201).unwrap(), 399u32.to_le_bytes());
    }

    #[test]
    fn flip_alternates_active_half_and_bumps_generation() {
        let mut ram = Ram::new(CAP as usize);
        set(&mut ram, 1, b"v").unwrap();
        let (h0, g0) = active(&ram, CAP).unwrap().unwrap();
        compact(&mut ram, CAP).unwrap();
        let (h1, g1) = active(&ram, CAP).unwrap().unwrap();
        assert_ne!(h0, h1, "flip must switch the active half");
        assert_eq!(g1, g0 + 1, "flip must bump the generation");
        assert_eq!(get_vec(&ram, 1).as_deref(), Some(&b"v"[..]));
    }

    #[test]
    fn full_when_value_cannot_fit_a_half() {
        let mut ram = Ram::new(CAP as usize);
        let half = half_len(CAP) as usize;
        // A value larger than a half (minus header) can never fit, even after a flip.
        assert_eq!(set(&mut ram, 1, &vec![7u8; half]), Err(KvError::Full));
    }

    #[test]
    fn torn_append_preserves_prior_keys() {
        // A torn append corrupts only the tail record; scan stops there and prior keys survive.
        let mut ram = Ram::new(CAP as usize);
        set(&mut ram, 1, b"alpha").unwrap();
        set(&mut ram, 2, b"bravo").unwrap();
        let (h, _) = active(&ram, CAP).unwrap().unwrap();
        let base = half_base(CAP, h);
        let (free, _) = scan_half(&ram, base, half_len(CAP), 1).unwrap();
        let mut rec = [0u8; KV_HEADER + 4];
        rec[0..2].copy_from_slice(&3u16.to_le_bytes());
        rec[2..4].copy_from_slice(&4u16.to_le_bytes());
        let crc = entry_crc(&rec[0..4], &[7, 7, 7, 7]);
        rec[4..8].copy_from_slice(&crc.to_le_bytes());
        rec[8..12].copy_from_slice(&[7, 7, 7, 7]);
        ram.torn_write(base + free, &rec, 6); // only 6 of 12 bytes land
        assert_eq!(get_vec(&ram, 1).as_deref(), Some(&b"alpha"[..]));
        assert_eq!(get_vec(&ram, 2).as_deref(), Some(&b"bravo"[..]));
        assert_eq!(get_vec(&ram, 3), None, "the torn in-flight record is dropped");
    }

    #[test]
    fn torn_flip_keeps_old_half_active() {
        // Power-safety of compaction: a flip that writes the destination half but loses power
        // before the superblock commit must leave the OLD half active with the last good values —
        // and the next successful set must still see them.
        let mut ram = Ram::new(CAP as usize);
        set(&mut ram, 1, b"committed-1").unwrap();
        set(&mut ram, 2, b"committed-2").unwrap();
        let (src_h, src_gen) = active(&ram, CAP).unwrap().unwrap();

        // Mimic flip()'s body but DROP the final superblock commit (the power-loss point).
        let half = half_len(CAP);
        let dst_h = 1 - src_h;
        let dst_base = half_base(CAP, dst_h);
        blank(&mut ram, dst_base, half).unwrap();
        // (write some plausible-but-uncommitted records into the destination half)
        append(&mut ram, dst_base, 1, b"GARBAGE").unwrap();
        // ... no write_gen() — power lost here.

        // The old half is still active and intact.
        let (now_h, now_gen) = active(&ram, CAP).unwrap().unwrap();
        assert_eq!(
            (now_h, now_gen),
            (src_h, src_gen),
            "old half must remain active after a torn flip"
        );
        assert_eq!(get_vec(&ram, 1).as_deref(), Some(&b"committed-1"[..]));
        assert_eq!(get_vec(&ram, 2).as_deref(), Some(&b"committed-2"[..]));
    }

    #[test]
    fn torn_superblock_commit_fails_closed() {
        // If the superblock write itself tears (CRC won't match), that half is treated as invalid
        // and the other half wins — never a half-committed generation.
        let mut ram = Ram::new(CAP as usize);
        set(&mut ram, 1, b"good").unwrap();
        let (src_h, src_gen) = active(&ram, CAP).unwrap().unwrap();
        let dst_h = 1 - src_h;
        // Build a valid superblock for dst at gen+1, then tear it (only the magic + 2 bytes land).
        let mut sb = [0u8; SUPER_LEN as usize];
        sb[0..4].copy_from_slice(&SUPER_MAGIC);
        sb[4..8].copy_from_slice(&(src_gen + 1).to_le_bytes());
        let crc = super_crc(&sb[0..8]);
        sb[8..12].copy_from_slice(&crc.to_le_bytes());
        ram.torn_write(super_off(CAP, dst_h), &sb, 6);
        assert_eq!(
            active(&ram, CAP).unwrap(),
            Some((src_h, src_gen)),
            "torn superblock must not win"
        );
        assert_eq!(get_vec(&ram, 1).as_deref(), Some(&b"good"[..]));
    }

    #[test]
    fn migrate_legacy_fast_path() {
        // Legacy single-region data that fits one half migrates with zero data movement.
        let mut ram = Ram::new(CAP as usize);
        let mut o = 0u32;
        o = ram.legacy_put(o, 0x5201, &7u32.to_le_bytes()); // e.g. the TX-counter watermark
        o = ram.legacy_put(o, 0x5202, &3u32.to_le_bytes());
        let _ = ram.legacy_put(o, 0x5300, b"peer");
        assert!(
            active(&ram, CAP).unwrap().is_none(),
            "legacy data must look un-initialized"
        );
        init(&mut ram, CAP).unwrap();
        assert!(
            active(&ram, CAP).unwrap().is_some(),
            "init must commit a superblock"
        );
        assert_eq!(get_vec(&ram, 0x5201).unwrap(), 7u32.to_le_bytes());
        assert_eq!(get_vec(&ram, 0x5202).unwrap(), 3u32.to_le_bytes());
        assert_eq!(get_vec(&ram, 0x5300).as_deref(), Some(&b"peer"[..]));
        // And the store keeps working (and stays consistent) after migration.
        set(&mut ram, 0x5201, &8u32.to_le_bytes()).unwrap();
        assert_eq!(get_vec(&ram, 0x5201).unwrap(), 8u32.to_le_bytes());
    }

    #[test]
    fn migrate_fresh_region_is_empty_then_usable() {
        let mut ram = Ram::new(CAP as usize);
        init(&mut ram, CAP).unwrap();
        assert_eq!(get_vec(&ram, 1), None);
        set(&mut ram, 1, b"x").unwrap();
        assert_eq!(get_vec(&ram, 1).as_deref(), Some(&b"x"[..]));
    }

    #[test]
    fn migrate_legacy_slow_path_preserves_live_set() {
        // A legacy log larger than one half: many superseded versions of one key plus a couple of
        // others. After migration only the latest survive, and the watermark is intact.
        let mut ram = Ram::new(CAP as usize);
        let half = half_len(CAP);
        let mut o = 0u32;
        o = ram.legacy_put(o, 0x5201, &1u32.to_le_bytes()); // watermark (early in the log)
        o = ram.legacy_put(o, 0x5202, b"keep");
        let mut last = 0u32;
        while o + (KV_HEADER as u32 + 4) <= CAP - 2 * SUPER_LEN {
            last += 1;
            o = ram.legacy_put(o, 0x5300, &last.to_le_bytes()); // churn key, growing the log
        }
        assert!(o > half, "fixture must exceed one half to exercise the slow path");
        // Update the watermark to a higher value late in the log (its latest must win post-migrate).
        let _ = ram.legacy_put(o, 0x5201, &42u32.to_le_bytes());
        init(&mut ram, CAP).unwrap();
        assert_eq!(
            get_vec(&ram, 0x5201).unwrap(),
            42u32.to_le_bytes(),
            "latest watermark must survive"
        );
        assert_eq!(get_vec(&ram, 0x5202).as_deref(), Some(&b"keep"[..]));
        assert_eq!(get_vec(&ram, 0x5300).unwrap(), last.to_le_bytes());
    }

    // --- incremental maintenance ------------------------------------------------------------

    /// Fixture for the maintenance tests: one committed flip (so the dead half holds stale
    /// records — a dirty blank target), then superseded + live records in the active half.
    fn flip_fixture() -> Ram {
        let mut ram = Ram::new(CAP as usize);
        set(&mut ram, 1, b"one-v1").unwrap();
        set(&mut ram, 2, b"two").unwrap();
        compact(&mut ram, CAP).unwrap(); // gen 2 — the old half is now dead and dirty
        set(&mut ram, 1, b"one-v2").unwrap(); // supersedes the packed copy
        set(&mut ram, 3, &[7u8; 40]).unwrap();
        ram
    }

    fn assert_fixture_live(ram: &Ram) {
        assert_eq!(get_vec(ram, 1).as_deref(), Some(&b"one-v2"[..]));
        assert_eq!(get_vec(ram, 2).as_deref(), Some(&b"two"[..]));
        assert_eq!(get_vec(ram, 3).as_deref(), Some(&[7u8; 40][..]));
    }

    #[test]
    fn preblank_skips_zero_and_is_idempotent() {
        let mut ram = flip_fixture();
        assert!(
            !dead_half_blank(&ram, CAP).unwrap(),
            "fixture's dead half must be dirty"
        );
        // Blank in budget-2 slices; every slice must respect its budget.
        let mut cur = BlankCursor::new();
        loop {
            let before = ram.words_written;
            let more = blank_dead_step(&mut ram, CAP, &mut cur, 2).unwrap();
            assert!(ram.words_written - before <= 2, "slice exceeded its word budget");
            if !more {
                break;
            }
        }
        assert!(dead_half_blank(&ram, CAP).unwrap());
        // Re-run with a fresh cursor (a "reboot"): every word is already zero, so the pass is
        // pure reads — zero programs, zero wear — and completes in one call.
        let w = ram.words_written;
        let mut cur = BlankCursor::new();
        assert!(!blank_dead_step(&mut ram, CAP, &mut cur, 2).unwrap());
        assert_eq!(ram.words_written, w, "re-blank after reboot must be write-free");
        assert_fixture_live(&ram); // the active half was never touched
    }

    #[test]
    fn preblank_restarts_after_mid_blank_reboot() {
        let mut ram = flip_fixture();
        let mut cur = BlankCursor::new();
        assert!(blank_dead_step(&mut ram, CAP, &mut cur, 1).unwrap()); // one word, then "reboot"
        let mut cur = BlankCursor::new(); // the RAM cursor is lost; restart from the top
        while blank_dead_step(&mut ram, CAP, &mut cur, 4).unwrap() {}
        assert!(dead_half_blank(&ram, CAP).unwrap());
        assert_fixture_live(&ram);
        // With the dead half pre-blanked, the in-line fallback flip must skip its blank pass:
        // it programs only the packed live set + the superblock.
        let live = live_bytes(&ram, CAP).unwrap();
        let w = ram.words_written;
        compact(&mut ram, CAP).unwrap();
        assert!(
            (ram.words_written - w) as u32 <= words(live) + 3,
            "pre-blanked in-line flip must not re-blank"
        );
        assert_fixture_live(&ram);
    }

    #[test]
    fn incremental_flip_interruptible_at_every_boundary() {
        // Probe: how many budget-1 slices a full flip of the fixture takes.
        let mut probe = flip_fixture();
        let mut st = flip_start(&probe, CAP).unwrap().unwrap();
        let mut total = 0usize;
        while flip_step(&mut probe, CAP, &mut st, 1).unwrap() {
            total += 1;
        }
        assert!(total > 4, "fixture too small to exercise both phases");

        // Interrupt after flip_start (k = 0), after every step (k = 1..total), and with all
        // steps done but the commit never written (k = total): in every case the old
        // generation stays active with the committed values, and a from-scratch restart
        // converges cleanly.
        for k in 0..=total {
            let mut ram = flip_fixture();
            let (h0, g0) = active(&ram, CAP).unwrap().unwrap();
            let mut st = flip_start(&ram, CAP).unwrap().unwrap();
            for _ in 0..k {
                assert!(flip_step(&mut ram, CAP, &mut st, 1).unwrap());
            }
            drop(st); // ← the "reboot": RAM-only state lost, nothing committed
            assert_eq!(
                active(&ram, CAP).unwrap(),
                Some((h0, g0)),
                "old generation must stay active (interrupted at k={k})"
            );
            assert_fixture_live(&ram);
            // Restart from scratch and complete.
            let mut st = flip_start(&ram, CAP).unwrap().unwrap();
            while flip_step(&mut ram, CAP, &mut st, 1).unwrap() {}
            flip_commit(&mut ram, CAP, &mut st).unwrap();
            let (h1, g1) = active(&ram, CAP).unwrap().unwrap();
            assert_ne!(h0, h1, "flip must switch the active half (k={k})");
            assert_eq!(g1, g0 + 1, "flip must bump the generation (k={k})");
            assert_fixture_live(&ram);
        }
    }

    #[test]
    fn appends_during_flip_are_caught_up_at_commit() {
        let mut ram = flip_fixture();
        let (h0, g0) = active(&ram, CAP).unwrap().unwrap();
        let mut st = flip_start(&ram, CAP).unwrap().unwrap();
        for _ in 0..2 {
            assert!(flip_step(&mut ram, CAP, &mut st, 1).unwrap());
        }
        // Appends land in the SOURCE half while the flip is in progress (active is unchanged),
        // including a key superseded twice in the tail and a brand-new key.
        set(&mut ram, 2, b"tail-1").unwrap();
        set(&mut ram, 2, b"tail-2").unwrap();
        set(&mut ram, 9, b"new").unwrap();
        assert_eq!(
            active(&ram, CAP).unwrap(),
            Some((h0, g0)),
            "appends must not flip"
        );
        assert_eq!(get_vec(&ram, 2).as_deref(), Some(&b"tail-2"[..]));
        while flip_step(&mut ram, CAP, &mut st, 4).unwrap() {}
        flip_commit(&mut ram, CAP, &mut st).unwrap();
        let (h1, g1) = active(&ram, CAP).unwrap().unwrap();
        assert_ne!(h0, h1);
        assert_eq!(g1, g0 + 1);
        // Latest-wins across plan + tail: the twice-superseded key resolves to its last value.
        assert_eq!(get_vec(&ram, 2).as_deref(), Some(&b"tail-2"[..]));
        assert_eq!(get_vec(&ram, 9).as_deref(), Some(&b"new"[..]));
        assert_eq!(get_vec(&ram, 1).as_deref(), Some(&b"one-v2"[..]));
        assert_eq!(get_vec(&ram, 3).as_deref(), Some(&[7u8; 40][..]));
        // And the store keeps working in the new half.
        set(&mut ram, 2, b"post").unwrap();
        assert_eq!(get_vec(&ram, 2).as_deref(), Some(&b"post"[..]));
    }

    #[test]
    fn maintain_flips_below_threshold_only() {
        const T: u32 = 64; // test threshold (FLIP_THRESHOLD exceeds a whole test half)
        const BUDGET: u32 = 4;
        let mut ram = Ram::new(CAP as usize);
        let mut m = MaintState::new();
        set(&mut ram, 1, &[1u8; 20]).unwrap();

        // Plenty of free space: maintenance must quiesce without flipping.
        let g0 = generation(&ram, CAP).unwrap();
        while maintain(&mut ram, CAP, &mut m, BUDGET, T).unwrap() {}
        assert_eq!(generation(&ram, CAP).unwrap(), g0, "no flip above the threshold");

        // Churn until free space drops below the threshold — never filling the half, so no
        // in-line flip fires either.
        while free_bytes(&ram, CAP).unwrap() >= T {
            set(&mut ram, 1, &[9u8; 20]).unwrap();
        }
        assert_eq!(
            generation(&ram, CAP).unwrap(),
            g0,
            "churn must not flip synchronously"
        );

        // Below the threshold: maintenance starts and completes exactly one incremental flip,
        // then pre-blanks the old half and quiesces. Every slice stays within its budget
        // (+3 words for the commit slice's superblock).
        let mut worked = false;
        loop {
            let before = ram.words_written;
            let more = maintain(&mut ram, CAP, &mut m, BUDGET, T).unwrap();
            assert!(
                (ram.words_written - before) as u32 <= BUDGET + 3,
                "slice exceeded budget"
            );
            if !more {
                break;
            }
            worked = true;
        }
        assert!(worked);
        assert_eq!(
            generation(&ram, CAP).unwrap(),
            g0 + 1,
            "exactly one proactive flip"
        );
        assert!(free_bytes(&ram, CAP).unwrap() >= T, "flip must restore headroom");
        assert!(
            dead_half_blank(&ram, CAP).unwrap(),
            "maintenance ends with a blank dead half"
        );
        assert_eq!(get_vec(&ram, 1).unwrap(), [9u8; 20]);
    }

    #[test]
    fn set_bytes_finishes_pending_flip_when_source_fills() {
        let mut ram = Ram::new(CAP as usize);
        for i in 0..5u8 {
            set(&mut ram, 1, &[i; 40]).unwrap(); // 5 × 48 B = 240 of 244 — nearly full
        }
        let (h0, g0) = active(&ram, CAP).unwrap().unwrap();
        let mut pending = flip_start(&ram, CAP).unwrap();
        assert!(flip_step(&mut ram, CAP, pending.as_mut().unwrap(), 1).unwrap()); // barely begun
        // This append cannot fit the source half: set_bytes_with must FINISH the pending flip
        // (not start a synchronous one from scratch) and then succeed — never failing where
        // plain set_bytes would have succeeded.
        set_bytes_with(&mut ram, CAP, 2, &[5u8; 100], &mut pending).unwrap();
        assert!(pending.is_none(), "the pending flip must be consumed");
        let (h1, g1) = active(&ram, CAP).unwrap().unwrap();
        assert_ne!(h0, h1);
        assert_eq!(g1, g0 + 1, "exactly one flip");
        assert_eq!(get_vec(&ram, 1).unwrap(), [4u8; 40]);
        assert_eq!(get_vec(&ram, 2).unwrap(), [5u8; 100]);
    }

    #[test]
    fn free_and_live_bytes_match_hand_layout() {
        let mut ram = Ram::new(CAP as usize);
        init(&mut ram, CAP).unwrap();
        let half = half_len(CAP);
        assert_eq!(free_bytes(&ram, CAP).unwrap(), half);
        assert_eq!(live_bytes(&ram, CAP).unwrap(), 0);
        // Each record is KV_HEADER + len; live counts only the latest per key.
        set(&mut ram, 1, &[1, 2, 3, 4]).unwrap(); // 12 B
        assert_eq!(free_bytes(&ram, CAP).unwrap(), half - 12);
        assert_eq!(live_bytes(&ram, CAP).unwrap(), 12);
        set(&mut ram, 1, &[5, 6, 7, 8]).unwrap(); // supersedes: used 24, live still 12
        assert_eq!(free_bytes(&ram, CAP).unwrap(), half - 24);
        assert_eq!(live_bytes(&ram, CAP).unwrap(), 12);
        set(&mut ram, 2, &[9u8; 6]).unwrap(); // 14 B
        assert_eq!(free_bytes(&ram, CAP).unwrap(), half - 38);
        assert_eq!(live_bytes(&ram, CAP).unwrap(), 26);
        assert!(dead_half_blank(&ram, CAP).unwrap());
    }

    // --- deletion (tombstones) ------------------------------------------------------------------

    fn del(ram: &mut Ram, key: u16) -> Result<(), KvError<()>> {
        delete(ram, CAP, key)
    }

    #[test]
    fn delete_makes_key_absent_then_resurrectable() {
        let mut ram = Ram::new(CAP as usize);
        set(&mut ram, 1, b"hello").unwrap();
        set(&mut ram, 2, b"keep").unwrap();
        del(&mut ram, 1).unwrap();
        assert_eq!(get_vec(&ram, 1), None, "deleted key reads absent");
        assert_eq!(
            get_vec(&ram, 2).as_deref(),
            Some(&b"keep"[..]),
            "sibling untouched"
        );
        // A later set resurrects the key (the new record supersedes the tombstone).
        set(&mut ram, 1, b"back").unwrap();
        assert_eq!(get_vec(&ram, 1).as_deref(), Some(&b"back"[..]));
        del(&mut ram, 1).unwrap();
        assert_eq!(get_vec(&ram, 1), None);
    }

    #[test]
    fn delete_missing_key_is_a_wear_free_noop() {
        let mut ram = Ram::new(CAP as usize);
        assert_eq!(del(&mut ram, 0), Err(KvError::InvalidKey), "key 0 reserved");
        // Never-set key on an un-initialized region: no write at all.
        del(&mut ram, 7).unwrap();
        assert_eq!(
            ram.words_written, 0,
            "deleting a missing key must not program a word"
        );
        set(&mut ram, 7, b"x").unwrap();
        del(&mut ram, 7).unwrap();
        let after_first = ram.words_written;
        del(&mut ram, 7).unwrap(); // already tombstoned → no-op
        assert_eq!(ram.words_written, after_first, "redundant delete is wear-free");
        assert_eq!(get_vec(&ram, 7), None);
    }

    #[test]
    fn flip_reclaims_deleted_keys() {
        let mut ram = Ram::new(CAP as usize);
        set(&mut ram, 1, &[1u8; 40]).unwrap();
        set(&mut ram, 2, b"survivor").unwrap();
        del(&mut ram, 1).unwrap();
        // Before the flip, the tombstone leaves nothing live for key 1…
        assert_eq!(
            live_bytes(&ram, CAP).unwrap(),
            KV_HEADER as u32 + 8,
            "only key 2 is live"
        );
        compact(&mut ram, CAP).unwrap();
        // …and the flip physically drops both key 1's value and its tombstone.
        assert_eq!(get_vec(&ram, 1), None);
        assert_eq!(get_vec(&ram, 2).as_deref(), Some(&b"survivor"[..]));
        assert_eq!(
            free_bytes(&ram, CAP).unwrap(),
            half_len(CAP) - (KV_HEADER as u32 + 8),
            "packed half holds only the survivor"
        );
    }

    #[test]
    fn delete_survives_many_flips() {
        let mut ram = Ram::new(CAP as usize);
        set(&mut ram, 0x10, b"gone").unwrap();
        del(&mut ram, 0x10).unwrap();
        // Churn another key hard enough to force many flips; the deleted key must never reappear.
        for i in 0..400u32 {
            set(&mut ram, 0x11, &i.to_le_bytes()).unwrap();
            assert_eq!(get_vec(&ram, 0x10), None, "deleted key resurfaced at i={i}");
        }
    }

    #[test]
    fn delete_that_triggers_a_flip_still_deletes() {
        // Fill the half so the tombstone append itself must flip; the key must end up absent.
        let mut ram = Ram::new(CAP as usize);
        for i in 0..5u8 {
            set(&mut ram, 1, &[i; 40]).unwrap(); // ~240 of 244 B — nearly full
        }
        let (_, g0) = active(&ram, CAP).unwrap().unwrap();
        del(&mut ram, 1).unwrap(); // no room for the 8-B tombstone → flips, then tombstones
        let (_, g1) = active(&ram, CAP).unwrap().unwrap();
        assert_eq!(g1, g0 + 1, "the delete drove exactly one flip");
        assert_eq!(get_vec(&ram, 1), None);
        // A second flip now reclaims the value+tombstone the first flip carried over.
        compact(&mut ram, CAP).unwrap();
        assert_eq!(get_vec(&ram, 1), None);
        assert_eq!(
            live_bytes(&ram, CAP).unwrap(),
            0,
            "store empty after the delete is reclaimed"
        );
    }

    #[test]
    fn tombstone_in_the_tail_supersedes_a_planned_value() {
        // A delete that lands AFTER flip_start's snapshot (in the source tail) must still win:
        // flip_commit copies the tail tombstone in after the planned value, so latest-wins holds.
        let mut ram = Ram::new(CAP as usize);
        set(&mut ram, 1, b"planned").unwrap();
        set(&mut ram, 2, b"keep").unwrap();
        let mut st = flip_start(&ram, CAP).unwrap().unwrap();
        flip_step(&mut ram, CAP, &mut st, u32::MAX).unwrap(); // copies the planned live set
        del(&mut ram, 1).unwrap(); // tombstone appended to the source tail, past the snapshot
        flip_commit(&mut ram, CAP, &mut st).unwrap(); // tail catch-up carries the tombstone over
        assert_eq!(get_vec(&ram, 1), None, "tail tombstone won");
        assert_eq!(get_vec(&ram, 2).as_deref(), Some(&b"keep"[..]));
        // The carried value+tombstone are dead weight the NEXT flip reclaims.
        compact(&mut ram, CAP).unwrap();
        assert_eq!(get_vec(&ram, 1), None);
        assert_eq!(live_bytes(&ram, CAP).unwrap(), KV_HEADER as u32 + 4);
    }
}
