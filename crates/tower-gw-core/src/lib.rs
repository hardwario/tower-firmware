//! Pure decision core of the TOWER gateway app (`apps/radio_dongle_gateway.rs`).
//!
//! Two kernels, both free of I/O and time sources (callers pass `now_ms` in), so the
//! logic the gateway's correctness rests on — which node records persist where, and
//! which downlink goes out on which uplink — is unit-tested on the host, exactly like
//! `crates/tower-kv` / `crates/tower-net-core` / `crates/tower-radio-core`:
//!
//! * [`registry`] — the persistent node registry: `(id, AES key, flags, name)` records
//!   packed into **buckets** sized for one `tower-kv` value each, accessed
//!   bucket-at-a-time over the [`registry::BucketIo`] seam (EEPROM on target, RAM in
//!   tests) and never held resident — the 20 KB part cannot afford a ~1.7 KB RAM
//!   mirror next to its ~9 KB stack peak. Bucketing itself is not an optimisation:
//!   `tower_kv::MAX_KEYS = 64` is a *global* cap on distinct KV keys and `NS_NET`
//!   alone holds 19 (watermark + last-seen + epoch + 16 replay lanes), so
//!   one-key-per-node would blow the budget; six records per bucket × three buckets
//!   covers the 16-peer table in 3 keys.
//! * [`queue`] — the RAM downlink queue: a small global pool of opaque payloads
//!   (host-built `radio::NodeCmd` envelopes), per-node FIFO, TTL expiry, and stable
//!   u16 item ids for dequeue/TX reporting. RAM-only by design — a gateway reboot
//!   drops it, which the host detects via the `Hello` session_id and re-queues.

#![no_std]

// Host tests use std collections for their in-memory BucketIo fixture.
#[cfg(test)]
extern crate std;

pub mod queue;
pub mod registry;
