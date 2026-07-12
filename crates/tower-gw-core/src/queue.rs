//! The gateway's RAM downlink queue: opaque host-built payloads awaiting a sleeping
//! node's next uplink.
//!
//! Policy (argued in the gateway app):
//! * **Global pool of [`QUEUE_CAP`] items** (~0.4 KB), NOT per-node arrays — a
//!   per-node array for every peer would cost multiples of that on the 20 KB part.
//! * **Per-node FIFO** ([`PER_NODE_CAP`] deep): commands to one node deliver in order,
//!   one per uplink cycle (each delivery's ACK re-advertises the pending flag).
//! * **TTL expiry**: an item that outlives its `ttl` is dropped and reported
//!   (`TX_EXPIRED`), never delivered stale.
//! * **Stable u16 ids** (monotonic from 1 per boot, 0 reserved = "not a queue item"
//!   in `RadioStat::Tx`): the dequeue handle and the TX-report correlator.
//!
//! Time is the caller's (`now_ms` in every method) — kernel style, host-testable.

/// Global item pool size. 4 × ~104 B ≈ 0.4 KB — sized against the 20 KB part's
/// measured ~9 KB stack peak (every byte of app statics is stack the part loses).
/// A full pool answers `MGMT_FULL`; the host retries after deliveries drain it.
pub const QUEUE_CAP: usize = 4;
/// Max queued items per node.
pub const PER_NODE_CAP: usize = 2;
/// Max payload bytes per item — the radio MTU.
pub const MAX_ITEM: usize = 74;

/// Why a push was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PushError {
    /// The global pool is exhausted.
    Full,
    /// This node already holds [`PER_NODE_CAP`] items.
    PerNodeFull,
    /// Payload over [`MAX_ITEM`] bytes (or empty).
    BadPayload,
}

/// One queued downlink.
#[derive(Clone, Copy)]
pub struct Item {
    pub id: u16,
    pub node_addr: u32,
    pub enqueued_at_ms: u64,
    pub ttl: u16,
    /// Pool-insertion order — the per-node FIFO key (ids alone would break on u16 wrap).
    order: u32,
    len: u8,
    buf: [u8; MAX_ITEM],
}

impl Item {
    pub fn data(&self) -> &[u8] {
        &self.buf[..self.len as usize]
    }

    pub fn age(&self, now_ms: u64) -> u16 {
        (now_ms.saturating_sub(self.enqueued_at_ms) / 1000).min(u16::MAX as u64) as u16
    }

    fn expired(&self, now_ms: u64) -> bool {
        now_ms.saturating_sub(self.enqueued_at_ms) >= (self.ttl as u64) * 1000
    }
}

/// The downlink queue.
pub struct Queue {
    slots: [Option<Item>; QUEUE_CAP],
    /// Next item id (wraps past u16::MAX back to 1; 0 is reserved).
    next_id: u16,
    /// Next pool-insertion order stamp.
    next_order: u32,
}

impl Default for Queue {
    fn default() -> Self {
        Self::new()
    }
}

impl Queue {
    pub const fn new() -> Self {
        Self {
            slots: [None; QUEUE_CAP],
            next_id: 1,
            next_order: 0,
        }
    }

    /// Enqueue `data` for `node_addr`; returns the item id (the dequeue / TX-report handle).
    pub fn push(&mut self, node_addr: u32, ttl: u16, data: &[u8], now_ms: u64) -> Result<u16, PushError> {
        if data.is_empty() || data.len() > MAX_ITEM {
            return Err(PushError::BadPayload);
        }
        if self.count_for(node_addr) >= PER_NODE_CAP {
            return Err(PushError::PerNodeFull);
        }
        let slot = self
            .slots
            .iter()
            .position(|s| s.is_none())
            .ok_or(PushError::Full)?;
        let id = self.next_id;
        self.next_id = self.next_id.checked_add(1).filter(|&v| v != 0).unwrap_or(1);
        let mut buf = [0u8; MAX_ITEM];
        buf[..data.len()].copy_from_slice(data);
        self.slots[slot] = Some(Item {
            id,
            node_addr,
            enqueued_at_ms: now_ms,
            ttl,
            order: self.next_order,
            len: data.len() as u8,
            buf,
        });
        self.next_order = self.next_order.wrapping_add(1);
        Ok(id)
    }

    /// The next (oldest, non-expired) item for `node_addr`, if any — what to transmit on
    /// its uplink. Does not remove it; call [`pop`](Self::pop) on delivery.
    pub fn peek_for(&self, node_addr: u32, now_ms: u64) -> Option<&Item> {
        self.slots
            .iter()
            .flatten()
            .filter(|it| it.node_addr == node_addr && !it.expired(now_ms))
            .min_by_key(|it| it.order)
    }

    /// Remove one item by id (delivered, or dequeued by the host).
    pub fn pop(&mut self, id: u16) -> Option<Item> {
        for s in self.slots.iter_mut() {
            if matches!(s, Some(it) if it.id == id) {
                return s.take();
            }
        }
        None
    }

    /// Drop every item queued for `node_addr` (node removed); returns how many.
    pub fn drop_node(&mut self, node_addr: u32) -> usize {
        let mut n = 0;
        for s in self.slots.iter_mut() {
            if matches!(s, Some(it) if it.node_addr == node_addr) {
                *s = None;
                n += 1;
            }
        }
        n
    }

    /// Drop expired items, reporting each as `(id, node_addr)` (→ a `TX_EXPIRED` stat).
    pub fn expire(&mut self, now_ms: u64, mut on_expired: impl FnMut(u16, u32)) {
        for s in self.slots.iter_mut() {
            if matches!(s, Some(it) if it.expired(now_ms)) {
                let it = s.take().unwrap();
                on_expired(it.id, it.node_addr);
            }
        }
    }

    pub fn count_for(&self, node_addr: u32) -> usize {
        self.slots
            .iter()
            .flatten()
            .filter(|it| it.node_addr == node_addr)
            .count()
    }

    pub fn len(&self) -> usize {
        self.slots.iter().flatten().count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// All queued items (arbitrary order; sort by `id`/`order` host-side if needed).
    pub fn iter(&self) -> impl Iterator<Item = &Item> {
        self.slots.iter().flatten()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NODE_A: u32 = 0xAAAA_0001;
    const NODE_B: u32 = 0xBBBB_0002;

    #[test]
    fn fifo_per_node_and_isolation() {
        let mut q = Queue::new();
        let a1 = q.push(NODE_A, 60, b"a1", 0).unwrap();
        let b1 = q.push(NODE_B, 60, b"b1", 1).unwrap();
        let a2 = q.push(NODE_A, 60, b"a2", 2).unwrap();
        assert_eq!(q.peek_for(NODE_A, 10).unwrap().id, a1);
        assert_eq!(q.peek_for(NODE_B, 10).unwrap().id, b1);
        assert!(q.pop(a1).is_some());
        assert_eq!(q.peek_for(NODE_A, 10).unwrap().id, a2);
        assert_eq!(q.peek_for(NODE_A, 10).unwrap().data(), b"a2");
    }

    #[test]
    fn per_node_cap_then_global_cap() {
        let mut q = Queue::new();
        for i in 0..PER_NODE_CAP {
            q.push(NODE_A, 60, &[i as u8], 0).unwrap();
        }
        assert_eq!(q.push(NODE_A, 60, b"x", 0), Err(PushError::PerNodeFull));
        // Fill the rest of the pool with distinct nodes.
        for i in 0..(QUEUE_CAP - PER_NODE_CAP) {
            q.push(1000 + i as u32, 60, b"y", 0).unwrap();
        }
        assert_eq!(q.push(NODE_B, 60, b"z", 0), Err(PushError::Full));
        assert_eq!(q.len(), QUEUE_CAP);
    }

    #[test]
    fn ttl_expiry_reports_and_drops() {
        let mut q = Queue::new();
        let e1 = q.push(NODE_A, 1, b"soon", 0).unwrap(); // expires at 1000 ms
        let keep = q.push(NODE_A, 600, b"later", 0).unwrap();
        // Not yet expired at 999 ms.
        let mut expired = heapless::Vec::<(u16, u32), 4>::new();
        q.expire(999, |id, node| expired.push((id, node)).unwrap());
        assert!(expired.is_empty());
        // peek at 1000 ms skips the expired head even before expire() runs.
        assert_eq!(q.peek_for(NODE_A, 1000).unwrap().id, keep);
        q.expire(1000, |id, node| expired.push((id, node)).unwrap());
        assert_eq!(expired.as_slice(), &[(e1, NODE_A)]);
        assert_eq!(q.len(), 1);
        assert!(q.pop(e1).is_none(), "expired item is gone");
    }

    #[test]
    fn ids_monotonic_skip_zero_on_wrap() {
        let mut q = Queue::new();
        q.next_id = u16::MAX;
        let last = q.push(NODE_A, 60, b"x", 0).unwrap();
        assert_eq!(last, u16::MAX);
        q.pop(last).unwrap();
        let wrapped = q.push(NODE_A, 60, b"y", 0).unwrap();
        assert_eq!(wrapped, 1, "id 0 is reserved (RadioStat::Tx 'not a queue item')");
    }

    #[test]
    fn fifo_survives_id_wrap() {
        let mut q = Queue::new();
        q.next_id = u16::MAX;
        let first = q.push(NODE_A, 60, b"first", 0).unwrap(); // id 65535
        let second = q.push(NODE_A, 60, b"second", 1).unwrap(); // id 1 (wrapped)
        assert!(second < first, "ids wrapped");
        assert_eq!(
            q.peek_for(NODE_A, 2).unwrap().data(),
            b"first",
            "order, not id, keys FIFO"
        );
    }

    #[test]
    fn drop_node_and_payload_validation() {
        let mut q = Queue::new();
        q.push(NODE_A, 60, b"1", 0).unwrap();
        q.push(NODE_A, 60, b"2", 0).unwrap();
        q.push(NODE_B, 60, b"3", 0).unwrap();
        assert_eq!(q.drop_node(NODE_A), 2);
        assert_eq!(q.len(), 1);
        assert_eq!(q.count_for(NODE_A), 0);

        assert_eq!(q.push(NODE_A, 60, &[], 0), Err(PushError::BadPayload));
        assert_eq!(
            q.push(NODE_A, 60, &[0u8; MAX_ITEM + 1], 0),
            Err(PushError::BadPayload)
        );
        assert!(
            q.push(NODE_A, 60, &[0u8; MAX_ITEM], 0).is_ok(),
            "exactly MTU is fine"
        );
    }

    #[test]
    fn age_reporting() {
        let mut q = Queue::new();
        let id = q.push(NODE_A, 600, b"x", 5_000).unwrap();
        let it = q.iter().find(|it| it.id == id).unwrap();
        assert_eq!(it.age(65_000), 60);
        assert_eq!(it.age(4_000), 0, "clock skew clamps to 0, not underflow");
    }

    /// `age` saturates at u16::MAX rather than wrapping for an item older than ~18 h.
    #[test]
    fn age_saturates() {
        let mut q = Queue::new();
        let id = q.push(NODE_A, u16::MAX, b"x", 0).unwrap();
        let it = q.iter().find(|it| it.id == id).unwrap();
        assert_eq!(it.age(100_000_000), u16::MAX, "age clamps, not wraps");
    }

    /// A `ttl == 0` item is expired the instant it is queued (`0 >= 0`): peek skips it and
    /// expire() reports it. Degenerate, but a reachable host input.
    #[test]
    fn zero_ttl_expires_immediately() {
        let mut q = Queue::new();
        let id = q.push(NODE_A, 0, b"x", 1_000).unwrap();
        assert!(
            q.peek_for(NODE_A, 1_000).is_none(),
            "zero-TTL item is never deliverable"
        );
        let mut expired = heapless::Vec::<(u16, u32), 4>::new();
        q.expire(1_000, |i, n| expired.push((i, n)).unwrap());
        assert_eq!(expired.as_slice(), &[(id, NODE_A)]);
        assert!(q.is_empty());
    }

    /// A per-node slot frees on pop and is reusable — the FIFO cap is by live count, not a
    /// high-water mark.
    #[test]
    fn per_node_slot_reuse_after_pop() {
        let mut q = Queue::new();
        let mut ids = heapless::Vec::<u16, 4>::new();
        for i in 0..PER_NODE_CAP {
            ids.push(q.push(NODE_A, 60, &[i as u8], 0).unwrap()).unwrap();
        }
        assert_eq!(q.push(NODE_A, 60, b"x", 0), Err(PushError::PerNodeFull));
        q.pop(ids[0]).unwrap();
        assert!(q.push(NODE_A, 60, b"reused", 0).is_ok(), "freed slot is reusable");
    }
}
