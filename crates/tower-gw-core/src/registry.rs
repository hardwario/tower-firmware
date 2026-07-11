//! The gateway's persistent node registry: `(id, key, flags, name)` records packed
//! into buckets, one `tower-kv` value per bucket — accessed **bucket-at-a-time**
//! through the [`BucketIo`] trait, never held resident.
//!
//! Two constraints shape this module:
//! * `tower_kv::MAX_KEYS = 64` is a *global* cap on distinct KV keys and `NS_NET`
//!   alone holds 35, so the registry cannot spend one key per node — six records per
//!   bucket × six buckets covers the 32-peer table in 6 keys.
//! * The STM32L083 has 20 KB of RAM and a HW-measured ~9 KB stack peak for any `Net`
//!   app (see the firmware's stack-overflow history): a resident registry copy
//!   (~1.7 KB inside the app's statically-allocated task future) is RAM the stack
//!   cannot afford. Every operation here loads ONE bucket into a ~270 B stack local,
//!   mutates, writes back, and returns owned/loaned data no larger than one record.
//!   All functions are synchronous — their locals live on the stack during the call,
//!   not in the app future.
//!
//! Registry churn (add / remove / rename) is operator-rate, so a bucket rewrite per
//! change is wear-trivial next to the net layer's counter traffic. The encoded worst
//! case is pinned by test against [`MAX_BUCKET_BYTES`] ≤ tower-kv's 256-byte value
//! cap. The record layout is **persisted** — field order is load-bearing; evolve it
//! append-only and bump [`FORMAT_VERSION`] on any change (the app stores the version
//! at its own key and refuses buckets from a newer format).

use heapless::{String, Vec};
use serde::{Deserialize, Serialize};

/// Persisted-format version (the app stores this next to the buckets).
pub const FORMAT_VERSION: u8 = 1;
/// Records per bucket. 6 × ~39 B encoded ≈ 235 B — one tower-kv value.
pub const PER_BUCKET: usize = 6;
/// Bucket count. 3 × [`PER_BUCKET`] = 18 slots ≥ the 16-peer capacity.
pub const BUCKETS: usize = 3;
/// Usable node capacity — matches the net layer's `MAX_PEERS` (the registry must
/// never hold more nodes than the RAM peer table can serve).
pub const CAPACITY: usize = 16;
/// Longest node name, bytes (UTF-8) — mirrors `tower_protocol::mgmt::MAX_NODE_NAME`.
pub const MAX_NAME: usize = 16;
/// Encoded-bucket ceiling, pinned by test. tower-kv's `MAX_VALUE` is 256.
pub const MAX_BUCKET_BYTES: usize = 256;

/// One registered node, as persisted.
#[derive(Serialize, Deserialize, Clone, PartialEq, Eq, Debug)]
pub struct NodeRecord {
    pub id: u32,
    pub key: [u8; 16],
    /// `tower_protocol::mgmt::NODE_FLAG_*` bits.
    pub flags: u8,
    /// Empty while unnamed (the UNNAMED flag is what the host keys auto-naming on).
    pub name: String<MAX_NAME>,
}

/// One bucket of records (a transient stack local — never resident).
pub type Bucket = Vec<NodeRecord, PER_BUCKET>;

/// Why a mutation was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegistryError {
    /// [`CAPACITY`] nodes already registered (or every bucket slot full).
    Full,
    /// No node with that id.
    NotFound,
    /// Name over [`MAX_NAME`] bytes.
    BadName,
    /// The backing store failed to persist; nothing was committed.
    Storage,
}

/// A [`BucketIo`] backend failure (an EEPROM read/write error).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IoError;

/// Bucket persistence the registry operates over — the L0 data EEPROM in the
/// firmware (`NS_APP` locals), a RAM array in host tests. Same seam as
/// `tower_kv::Eeprom`.
pub trait BucketIo {
    /// Read bucket `index` into `out`; `Ok(None)` = never written (empty bucket).
    fn load(&self, index: usize, out: &mut [u8]) -> Result<Option<usize>, IoError>;
    /// Persist bucket `index` (whole-value replace).
    fn store(&mut self, index: usize, bytes: &[u8]) -> Result<(), IoError>;
}

/// Decode bucket `index`. Absent OR undecodable buckets read as empty — a corrupt
/// bucket means those nodes re-pair, which beats refusing to serve the rest of the
/// network (the caller can't do anything smarter with the bytes either way).
pub fn load(io: &impl BucketIo, index: usize) -> Bucket {
    let mut buf = [0u8; MAX_BUCKET_BYTES];
    match io.load(index, &mut buf) {
        Ok(Some(n)) if n <= MAX_BUCKET_BYTES => postcard::from_bytes(&buf[..n]).unwrap_or_default(),
        _ => Bucket::new(),
    }
}

fn store(io: &mut impl BucketIo, index: usize, bucket: &Bucket) -> Result<(), RegistryError> {
    let mut buf = [0u8; MAX_BUCKET_BYTES];
    let bytes = postcard::to_slice(bucket, &mut buf).map_err(|_| RegistryError::Storage)?;
    let n = bytes.len();
    io.store(index, &buf[..n]).map_err(|_| RegistryError::Storage)
}

/// Total registered nodes.
pub fn count(io: &impl BucketIo) -> usize {
    (0..BUCKETS).map(|i| load(io, i).len()).sum()
}

/// Look one node up (returns an owned ~44 B record).
pub fn find(io: &impl BucketIo, id: u32) -> Option<NodeRecord> {
    for i in 0..BUCKETS {
        if let Some(rec) = load(io, i).iter().find(|r| r.id == id) {
            return Some(rec.clone());
        }
    }
    None
}

/// Add a node (or overwrite the record of an already-registered id — the
/// cable-pairing re-provision case).
pub fn add(io: &mut impl BucketIo, rec: &NodeRecord) -> Result<(), RegistryError> {
    // Overwrite in place if the id exists.
    for i in 0..BUCKETS {
        let mut bucket = load(io, i);
        if let Some(slot) = bucket.iter().position(|r| r.id == rec.id) {
            bucket[slot] = rec.clone();
            return store(io, i, &bucket);
        }
    }
    if count(io) >= CAPACITY {
        return Err(RegistryError::Full);
    }
    for i in 0..BUCKETS {
        let mut bucket = load(io, i);
        if bucket.len() < PER_BUCKET {
            let _ = bucket.push(rec.clone()); // fits: len < PER_BUCKET just checked
            return store(io, i, &bucket);
        }
    }
    Err(RegistryError::Full)
}

/// Remove a node.
pub fn remove(io: &mut impl BucketIo, id: u32) -> Result<(), RegistryError> {
    for i in 0..BUCKETS {
        let mut bucket = load(io, i);
        if let Some(slot) = bucket.iter().position(|r| r.id == id) {
            bucket.remove(slot);
            return store(io, i, &bucket);
        }
    }
    Err(RegistryError::NotFound)
}

/// Update mutable metadata (`None` keeps the current value).
pub fn update(
    io: &mut impl BucketIo,
    id: u32,
    name: Option<&str>,
    flags: Option<u8>,
) -> Result<(), RegistryError> {
    for i in 0..BUCKETS {
        let mut bucket = load(io, i);
        if let Some(slot) = bucket.iter().position(|r| r.id == id) {
            if let Some(n) = name {
                let mut s: String<MAX_NAME> = String::new();
                s.push_str(n).map_err(|_| RegistryError::BadName)?;
                bucket[slot].name = s;
            }
            if let Some(f) = flags {
                bucket[slot].flags = f;
            }
            return store(io, i, &bucket);
        }
    }
    Err(RegistryError::NotFound)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// In-memory BucketIo for host tests.
    #[derive(Default)]
    struct Mem {
        buckets: [Option<std::vec::Vec<u8>>; BUCKETS],
        fail_store: bool,
    }

    impl BucketIo for Mem {
        fn load(&self, index: usize, out: &mut [u8]) -> Result<Option<usize>, IoError> {
            match &self.buckets[index] {
                Some(b) => {
                    out[..b.len()].copy_from_slice(b);
                    Ok(Some(b.len()))
                }
                None => Ok(None),
            }
        }
        fn store(&mut self, index: usize, bytes: &[u8]) -> Result<(), IoError> {
            if self.fail_store {
                return Err(IoError);
            }
            self.buckets[index] = Some(bytes.to_vec());
            Ok(())
        }
    }

    fn rec(id: u32, name: &str) -> NodeRecord {
        let mut s: String<MAX_NAME> = String::new();
        s.push_str(name).unwrap();
        NodeRecord {
            id,
            key: [id as u8; 16],
            flags: 0,
            name: s,
        }
    }

    /// A max-size bucket (6 records, 16-byte names) must encode within one tower-kv value.
    #[test]
    fn max_bucket_fits_one_kv_value() {
        let mut io = Mem::default();
        for i in 0..PER_BUCKET as u32 {
            add(&mut io, &rec(0x1000_0000 + i, "sixteen-byte-nam")).unwrap();
        }
        let n = io.buckets[0].as_ref().expect("bucket 0 written").len();
        assert!(n <= MAX_BUCKET_BYTES, "encoded bucket {n} B > {MAX_BUCKET_BYTES}");
    }

    #[test]
    fn roundtrip_through_the_store() {
        let mut io = Mem::default();
        add(&mut io, &rec(1, "kitchen")).unwrap();
        add(&mut io, &rec(2, "")).unwrap();
        assert_eq!(count(&io), 2);
        assert_eq!(find(&io, 1).unwrap().name.as_str(), "kitchen");
        assert_eq!(find(&io, 2).unwrap().key, [2u8; 16]);
        assert!(find(&io, 3).is_none());
    }

    /// Garbage bucket bytes read as empty (those nodes re-pair) — never a panic, and
    /// the healthy buckets keep working.
    #[test]
    fn garbage_bucket_reads_empty() {
        let mut io = Mem::default();
        add(&mut io, &rec(1, "ok")).unwrap();
        io.buckets[0] = Some(std::vec![0xFF; 40]);
        assert_eq!(count(&io), 0);
        add(&mut io, &rec(2, "fresh")).unwrap();
        assert_eq!(find(&io, 2).unwrap().name.as_str(), "fresh");
    }

    /// Adding an existing id overwrites in place (re-provision), never duplicates.
    #[test]
    fn re_add_overwrites() {
        let mut io = Mem::default();
        add(&mut io, &rec(7, "old")).unwrap();
        add(&mut io, &rec(7, "new")).unwrap();
        assert_eq!(count(&io), 1);
        assert_eq!(find(&io, 7).unwrap().name.as_str(), "new");
    }

    /// Slots free up on remove and are reused; capacity is enforced at CAPACITY (32),
    /// not the 36 physical slots.
    #[test]
    fn capacity_and_slot_reuse() {
        let mut io = Mem::default();
        for i in 0..CAPACITY as u32 {
            add(&mut io, &rec(100 + i, "n")).unwrap();
        }
        assert_eq!(add(&mut io, &rec(999, "n")), Err(RegistryError::Full));
        remove(&mut io, 100).unwrap();
        add(&mut io, &rec(999, "n")).unwrap();
        assert_eq!(count(&io), CAPACITY);
        assert!(find(&io, 100).is_none());
        assert!(find(&io, 999).is_some());
    }

    #[test]
    fn update_name_and_flags() {
        let mut io = Mem::default();
        add(&mut io, &rec(5, "")).unwrap();
        update(&mut io, 5, Some("garage-door"), Some(0b11)).unwrap();
        let n = find(&io, 5).unwrap();
        assert_eq!(n.name.as_str(), "garage-door");
        assert_eq!(n.flags, 0b11);
        // None keeps current values.
        update(&mut io, 5, None, None).unwrap();
        assert_eq!(find(&io, 5).unwrap().name.as_str(), "garage-door");
        // Over-long name refused, nothing changed.
        assert_eq!(
            update(&mut io, 5, Some("seventeen-byte-nm"), None),
            Err(RegistryError::BadName)
        );
        assert_eq!(update(&mut io, 6, Some("x"), None), Err(RegistryError::NotFound));
    }

    /// A failed persist reports Storage and commits nothing.
    #[test]
    fn store_failure_is_reported() {
        let mut io = Mem::default();
        add(&mut io, &rec(1, "a")).unwrap();
        io.fail_store = true;
        assert_eq!(add(&mut io, &rec(2, "b")), Err(RegistryError::Storage));
        assert_eq!(remove(&mut io, 1), Err(RegistryError::Storage));
        io.fail_store = false;
        assert_eq!(count(&io), 1, "nothing was committed while failing");
    }

    /// Records spill across buckets and enumerate completely.
    #[test]
    fn spill_across_buckets() {
        let mut io = Mem::default();
        let total = PER_BUCKET * 2 + 1;
        for i in 0..total as u32 {
            add(&mut io, &rec(i + 1, "n")).unwrap();
        }
        assert_eq!(count(&io), total);
        assert!(!load(&io, 2).is_empty(), "third bucket in use");
        let mut seen = 0;
        for i in 0..BUCKETS {
            seen += load(&io, i).len();
        }
        assert_eq!(seen, total);
    }
}
