//! radio_dongle_gateway — TOWER IoT Kit product firmware: the network coordinator.
//!
//! The USB Radio Dongle bridges its secured radio network to the host over the framed
//! console — as a **transparent bridge**: every authenticated uplink is forwarded
//! verbatim (`Uplink` frames; the host decodes the radio-application payload), so new
//! node app types never require a gateway firmware change. What the gateway *does* own
//! is coordination:
//!
//! * the **node registry** — paired ids + AES keys + names, persisted in `NS_APP`
//!   EEPROM buckets (`tower-gw-core::registry`), accessed bucket-at-a-time (never
//!   RAM-resident — see the RAM budget note below) and mirrored into the net layer's
//!   peer table at boot;
//! * **pairing** — an OTA window opened on host request (`MgmtOp::PairingOpen`; the
//!   host mints the key) and cable-paired installs (`MgmtOp::NodeAdd`);
//! * the **downlink queue** — opaque host-built payloads (`tower-gw-core::queue`,
//!   RAM-only; a reboot drops it and the host re-queues on the `Hello` session bump)
//!   delivered on a sleeping node's next uplink, advertised via the ACK pending flag;
//! * **radio diagnostics** — ambient channel RSSI at a settable cadence plus per-TX
//!   outcome reports (`RadioStat` frames) feeding the host's running channel graph.
//!
//! Single-owner design: one main loop owns `Net` + queue (no `&mut Net` sharing across
//! tasks) and multiplexes recv slices, management requests, the pairing window, and
//! the stats tick.
//!
//! **RAM budget** (20 KB part, HW-measured ~9 KB stack peak for any `Net` app — see
//! the stack-overflow history): everything this app keeps resident beyond the SDK
//! baseline must fit in ~2 KB, or the stack loses the difference. Hence: registry on
//! EEPROM (transient ~270 B bucket locals on the stack), an 8-item downlink queue,
//! and 12-byte packed link stats. Check `just size app radio_dongle_gateway` after
//! growing anything here.
//!
//!   just build app radio_dongle_gateway
//!   just run   app radio_dongle_gateway

#![no_std]
#![no_main]

use core::fmt::Write as _;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Instant, Timer};
use log::{info, warn};
use tower::led::{self, LedChannel, Pattern, Polarity, Step};
use tower::radio::Spirit1;
use tower::radio::config::Band;
use tower::radio::net::{Net, NetConfig, SendResult};
use tower::storage::{NS_APP, NS_SHELL, Nv, Scoped};
use tower::{app, board::Board, console, shell, watchdog};
use tower_gw_core::queue::{PushError, Queue};
use tower_gw_core::registry::{self, BucketIo, IoError, NodeRecord, RegistryError};
use tower_protocol::mgmt::{self, DeviceInfo, DeviceRole, MgmtOp, NodeEntry, NodeKey, Paired, QueueId};
use tower_protocol::msg::{MgmtRequest, RadioStat};
use tower_protocol::radio::RADIO_SCHEMA_VERSION;
use tower_protocol::{MsgType, decode_frame};

// --- persistence (NS_APP) --------------------------------------------------------

/// Registry format version marker (`tower_gw_core::registry::FORMAT_VERSION`).
const KEY_FORMAT: u8 = 0x00;
/// Operator override of the UID-derived radio id (u32 LE; absent = derived).
const KEY_MY_ID: u8 = 0x01;
/// Registry buckets 0..=5 (one `tower-gw-core` bucket per KV value).
const KEY_BUCKET_BASE: u8 = 0x10;

/// The registry's bucket store: `NS_APP` locals `0x10..=0x15`.
#[derive(Clone, Copy)]
struct EepromBuckets {
    kv: Scoped,
}

impl BucketIo for EepromBuckets {
    fn load(&self, index: usize, out: &mut [u8]) -> Result<Option<usize>, IoError> {
        self.kv
            .get_bytes(KEY_BUCKET_BASE + index as u8, out)
            .map_err(|_| IoError)
    }
    fn store(&mut self, index: usize, bytes: &[u8]) -> Result<(), IoError> {
        self.kv
            .set_bytes(KEY_BUCKET_BASE + index as u8, bytes)
            .map_err(|_| IoError)
    }
}

// --- settings (NS_SHELL locals; base `identity` owns 0x00) ------------------------

const SET_STATS_PERIOD: u8 = 0x10;
const SET_BAND: u8 = 0x11;
const SET_CHANNEL: u8 = 0x12;

static GW_SETTINGS: &[shell::Setting] = &[
    shell::Setting {
        key: SET_STATS_PERIOD,
        name: "stats-period",
        kind: shell::Kind::Uint { min: 0, max: 60_000 },
        default: "1000",
    },
    shell::Setting {
        key: SET_BAND,
        name: "band",
        kind: shell::Kind::Enum(&["eu868", "us915"]),
        default: "eu868",
    },
    shell::Setting {
        key: SET_CHANNEL,
        name: "channel",
        kind: shell::Kind::Uint { min: 0, max: 2 },
        default: "0",
    },
];

static GW_COMMANDS: &[shell::Entry] = &[shell::Entry::menu(
    "gateway",
    &[shell::Entry::cmd("status", shell::Args::None, cmd_status)],
)];

// Live counters for `/gateway status` (shell handlers are plain fns with no app-state
// access; M0+ atomics are load/store only, which is all these need).
static STAT_ID: AtomicU32 = AtomicU32::new(0);
static STAT_NODES: AtomicU32 = AtomicU32::new(0);
static STAT_UPLINKS: AtomicU32 = AtomicU32::new(0);
static STAT_QUEUED: AtomicU32 = AtomicU32::new(0);

fn cmd_status(ctx: &mut shell::Ctx<'_>, _args: &[&str]) -> shell::Outcome {
    let _ = write!(
        ctx,
        "gateway: {:08X}\r\nnodes: {}\r\nuplinks: {}\r\nqueued: {}\r\n",
        STAT_ID.load(Ordering::Relaxed),
        STAT_NODES.load(Ordering::Relaxed),
        STAT_UPLINKS.load(Ordering::Relaxed),
        STAT_QUEUED.load(Ordering::Relaxed),
    );
    shell::Outcome::ok()
}

// --- timing ------------------------------------------------------------------------

/// One receive slice of the main loop — short enough that management requests and the
/// stats tick stay responsive, long enough that the loop isn't spinning.
const RECV_SLICE: Duration = Duration::from_millis(250);
/// One pairing-window slice (management latency ceiling while pairing).
const PAIR_SLICE: Duration = Duration::from_secs(2);
/// Node TX→RX turnaround before a downlink delivery (mirror of the ACK turnaround: the
/// node checks the pending flag on our ACK, then switches to RX).
const DOWNLINK_TURNAROUND: Duration = Duration::from_millis(20);
/// Downlink delivery repetitions. Deliberately low (one retry): the node listens for
/// ~1 s right after its uplink, and a longer retransmit tail would hold the
/// single-owner loop out of `recv` while a mid-response node is uplinking its next
/// shell chunk.
const DOWNLINK_REPS: u8 = 2;
/// Default downlink TTL when the host passes `ttl_s == 0`.
const DEFAULT_TTL_S: u16 = 3600;

static CH: LedChannel = LedChannel::new();
/// Slow "gateway alive" heartbeat on the on-board LED.
static HEARTBEAT: Pattern = &[Step::on(30), Step::off(1970)];
/// Fast blink while the OTA pairing window is open.
static PAIRING: Pattern = &[Step::on(100), Step::off(150)];

/// The default-lane key. Not a secret and not used by any legitimate node (every
/// registered node has its own key; the pairing exchange uses `PAIRING_KEY`): traffic
/// from unregistered peers must simply fail CCM auth, which any value achieves.
/// Derived from the UID so two gateways side by side don't share a lane key.
fn default_lane_key(my_id: u32) -> [u8; 16] {
    let mut k = [0u8; 16];
    for (i, chunk) in k.chunks_mut(4).enumerate() {
        chunk.copy_from_slice(&(my_id ^ (0x9E37_79B9u32.rotate_left(i as u32 * 8))).to_le_bytes());
    }
    k
}

/// RAM-only per-node link stats. Packed to 12 B/entry (32 entries = 384 B of the app
/// future); deliberately NOT persisted — per-uplink EEPROM writes would burn the part
/// (docs/storage.md). `NodeEntry.last_seen_s` is documented as "since gateway boot".
#[derive(Clone, Copy)]
struct LinkStat {
    id: u32,
    /// `Instant::as_secs()` truncated — wraps after 136 years of uptime.
    seen_s: u32,
    rssi_dbm: i8,
    /// Saturating (reported as-is in the u32 wire field).
    uplinks: u16,
}

// 16 tracked nodes, not the full 32-node capacity: entries beyond 16 concurrent
// talkers report "never seen" until a slot frees (NodeRemove) — an accepted v1 limit
// on the RAM-starved part; the registry itself still holds 32.
struct Stats([Option<LinkStat>; 16]);

impl Stats {
    const fn new() -> Self {
        Self([None; 16])
    }
    fn on_uplink(&mut self, id: u32, rssi_dbm: i16) {
        let rssi = rssi_dbm.clamp(i8::MIN as i16, i8::MAX as i16) as i8;
        let now = Instant::now().as_secs() as u32;
        if let Some(s) = self.0.iter_mut().flatten().find(|s| s.id == id) {
            s.seen_s = now;
            s.rssi_dbm = rssi;
            s.uplinks = s.uplinks.saturating_add(1);
            return;
        }
        if let Some(slot) = self.0.iter_mut().find(|s| s.is_none()) {
            *slot = Some(LinkStat {
                id,
                seen_s: now,
                rssi_dbm: rssi,
                uplinks: 1,
            });
        }
    }
    fn get(&self, id: u32) -> Option<&LinkStat> {
        self.0.iter().flatten().find(|s| s.id == id)
    }
    fn remove(&mut self, id: u32) {
        for s in self.0.iter_mut() {
            if matches!(s, Some(st) if st.id == id) {
                *s = None;
            }
        }
    }
}

/// An armed OTA pairing window (delayed `MgmtResponse` — resolves on join/expiry/cancel).
struct PairingSession {
    req_id: u16,
    key: [u8; 16],
    deadline: Instant,
}

/// Chunked management-response sender: accumulates record bytes into ≤192-byte `data`
/// chunks, flushing as they fill (the app-driven-chunking contract of
/// [`console::mgmt_chunk`]). Record boundaries need not align with chunk boundaries —
/// the host concatenates all `data` before decoding the record stream.
struct ChunkStream {
    req_id: u16,
    chunk: u16,
    len: usize,
    // 128, not the full 192-byte chunk budget: this buffer lives across awaits (in
    // the app future); a few more chunks on a 32-node list is cheaper than the RAM.
    buf: [u8; 128],
}

impl ChunkStream {
    fn new(req_id: u16) -> Self {
        Self {
            req_id,
            chunk: 0,
            len: 0,
            buf: [0; 128],
        }
    }
    async fn push(&mut self, mut bytes: &[u8]) {
        while !bytes.is_empty() {
            if self.len == self.buf.len() {
                console::mgmt_chunk(
                    self.req_id,
                    mgmt::MGMT_OK,
                    self.chunk,
                    false,
                    &self.buf[..self.len],
                )
                .await;
                self.chunk = self.chunk.wrapping_add(1);
                self.len = 0;
            }
            let take = bytes.len().min(self.buf.len() - self.len);
            self.buf[self.len..self.len + take].copy_from_slice(&bytes[..take]);
            self.len += take;
            bytes = &bytes[take..];
        }
    }
    async fn finish(self) {
        console::mgmt_chunk(
            self.req_id,
            mgmt::MGMT_OK,
            self.chunk,
            true,
            &self.buf[..self.len],
        )
        .await;
    }
}

/// Reply with a result code and no records.
async fn respond_empty(req_id: u16, result: u8) {
    console::mgmt_chunk(req_id, result, 0, true, &[]).await;
}

/// Reply `MGMT_OK` with a single postcard record (largest single record is a
/// max-name `DeviceInfo` at ~60 B — 96 covers every one-record reply).
async fn respond_record<T: serde::Serialize>(req_id: u16, record: &T) {
    let mut buf = [0u8; 96];
    match postcard::to_slice(record, &mut buf) {
        Ok(bytes) => {
            let n = bytes.len();
            console::mgmt_chunk(req_id, mgmt::MGMT_OK, 0, true, &buf[..n]).await;
        }
        Err(_) => respond_empty(req_id, mgmt::MGMT_STORAGE).await,
    }
}

fn read_u32_setting(kv: Nv, local: u8, default: u32) -> u32 {
    let mut b = [0u8; 4];
    match kv.scope(NS_SHELL).get_bytes(local, &mut b) {
        Ok(Some(4)) => u32::from_le_bytes(b),
        _ => default,
    }
}

fn read_band_setting(kv: Nv) -> Band {
    let mut b = [0u8; 8];
    match kv.scope(NS_SHELL).get_bytes(SET_BAND, &mut b) {
        Ok(Some(n)) if &b[..n] == b"us915" => Band::Us915,
        _ => Band::Eu868,
    }
}

async fn run(b: Board) {
    // Hardware safety net: a hung gateway takes the whole network down — reset it within
    // ~26 s (the L0 hardware ceiling) rather than waiting for someone to notice.
    watchdog::enable(b.iwdg, b.spawner, Duration::from_secs(26));

    let led = led::init(
        b.spawner,
        Output::new(b.led, Level::Low, Speed::Low),
        &CH,
        Polarity::ActiveHigh,
    );
    led.set_background(Some(HEARTBEAT));

    let kv = b.kv;
    let app_kv = kv.scope(NS_APP);
    let mut buckets = EepromBuckets { kv: app_kv };

    // Identity: operator override (mgmt Provision.my_id — reserved) else UID-derived.
    let my_id = {
        let mut idb = [0u8; 4];
        match app_kv.get_bytes(KEY_MY_ID, &mut idb) {
            Ok(Some(4)) => u32::from_le_bytes(idb),
            _ => tower::board::unique_id32(),
        }
    };
    STAT_ID.store(my_id, Ordering::Relaxed);
    let _ = app_kv.set_bytes(KEY_FORMAT, &[registry::FORMAT_VERSION]);

    let band = read_band_setting(kv);
    let channel = read_u32_setting(kv, SET_CHANNEL, 0) as u8;
    let mut stats_period_ms = read_u32_setting(kv, SET_STATS_PERIOD, 1000);

    let radio = Spirit1::new(
        b.radio_spi,
        b.radio_sck,
        b.radio_mosi,
        b.radio_miso,
        b.radio_cs,
        b.radio_sdn,
        b.radio_irq,
    );
    let mut net = match Net::new(
        radio,
        kv,
        NetConfig {
            my_id,
            key: default_lane_key(my_id),
            band,
            channel,
        },
    )
    .await
    {
        Ok(n) => n,
        Err(e) => {
            log::error!(target: "gateway", "net init: {e} — gateway down");
            return;
        }
    };

    // Install every registered node into the peer table, bucket-at-a-time.
    let mut node_count = 0u32;
    for i in 0..registry::BUCKETS {
        for rec in registry::load(&buckets, i).iter() {
            net.add_peer(rec.id, &rec.key);
            node_count += 1;
        }
    }
    STAT_NODES.store(node_count, Ordering::Relaxed);

    info!(
        target: "gateway",
        "GATEWAY {:08X}: {} node(s), band {}, ch {}",
        my_id,
        node_count,
        match band { Band::Eu868 => "eu868", Band::Us915 => "us915" },
        channel
    );

    let mut queue = Queue::new();
    let mut stats = Stats::new();
    let mut pairing: Option<PairingSession> = None;
    let mut last_stat = Instant::now();

    loop {
        // 1. Drain management requests (host → gateway). Non-blocking so the radio
        //    keeps priority; depth-2 channel means a dropped burst is just retried.
        while let Some(frame) = console::mgmt_try_next() {
            let Ok((MsgType::MgmtRequest, _seq, payload)) = decode_frame(&frame) else {
                continue;
            };
            let Ok(req) = postcard::from_bytes::<MgmtRequest>(payload) else {
                continue;
            };
            handle_mgmt(
                req,
                &mut net,
                &mut buckets,
                &mut queue,
                &mut stats,
                &mut pairing,
                &mut stats_period_ms,
                my_id,
                band,
                channel,
                &led,
            )
            .await;
        }

        let now_ms = Instant::now().as_millis();

        // 2. Radio: either a pairing-window slice or a receive slice — the two share
        //    the one radio, and pairing mode is explicit + short-lived.
        if let Some(sess) = pairing.as_ref() {
            if Instant::now() >= sess.deadline {
                respond_empty(sess.req_id, mgmt::MGMT_TIMEOUT).await;
                led.set_background(Some(HEARTBEAT));
                pairing = None;
            } else {
                let slice = PAIR_SLICE.min(sess.deadline.saturating_duration_since(Instant::now()));
                if let Some(node_id) = net.open_pairing(slice, &sess.key).await {
                    let key = sess.key;
                    let req_id = sess.req_id;
                    led.set_background(Some(HEARTBEAT));
                    pairing = None;
                    commit_pairing(node_id, key, req_id, &mut net, &mut buckets).await;
                }
            }
        } else if let Some(rx) = net.recv(RECV_SLICE).await {
            // 3. One authenticated uplink: account, forward verbatim, then deliver a
            //    queued downlink into the node's post-uplink RX window (if any).
            stats.on_uplink(rx.src, rx.rssi_dbm);
            STAT_UPLINKS.store(
                STAT_UPLINKS.load(Ordering::Relaxed).wrapping_add(1),
                Ordering::Relaxed,
            );
            console::uplink(rx.src, rx.counter, rx.rssi_dbm, rx.lqi, rx.data()).await;

            if let Some(item) = queue.peek_for(rx.src, now_ms) {
                let (item_id, mut data, len) = (item.id, [0u8; 74], item.data().len());
                data[..len].copy_from_slice(item.data());
                Timer::after(DOWNLINK_TURNAROUND).await;
                let result = net.send(rx.src, &data[..len], true, DOWNLINK_REPS).await;
                let outcome = match result {
                    SendResult::Delivered => {
                        queue.pop(item_id);
                        mgmt::TX_DELIVERED
                    }
                    SendResult::NotDelivered => mgmt::TX_NOT_DELIVERED,
                    SendResult::Busy => mgmt::TX_BUSY,
                    SendResult::DutyLimited => mgmt::TX_DUTY_LIMITED,
                    _ => mgmt::TX_ERROR,
                };
                console::radio_stat(RadioStat::Tx {
                    dest: rx.src,
                    item: item_id,
                    outcome,
                    ack_rssi_dbm: net.last_ack().and_then(|m| m.rssi_dbm),
                })
                .await;
            }
            net.set_pending(rx.src, queue.count_for(rx.src) > 0);
            STAT_QUEUED.store(queue.len() as u32, Ordering::Relaxed);
        }

        // 4. Housekeeping: TTL expiry (reported as TX_EXPIRED) + the ambient-RSSI tick.
        let mut expired: heapless::Vec<(u16, u32), { tower_gw_core::queue::QUEUE_CAP }> =
            heapless::Vec::new();
        queue.expire(now_ms, |id, node| {
            let _ = expired.push((id, node));
        });
        for &(id, node) in expired.iter() {
            console::radio_stat(RadioStat::Tx {
                dest: node,
                item: id,
                outcome: mgmt::TX_EXPIRED,
                ack_rssi_dbm: None,
            })
            .await;
            net.set_pending(node, queue.count_for(node) > 0);
        }
        if !expired.is_empty() {
            STAT_QUEUED.store(queue.len() as u32, Ordering::Relaxed);
        }

        if stats_period_ms > 0 && last_stat.elapsed() >= Duration::from_millis(stats_period_ms as u64) {
            last_stat = Instant::now();
            if let Ok(rssi_dbm) = net.rssi_sample().await {
                console::radio_stat(RadioStat::Channel { channel, rssi_dbm }).await;
            }
        }
    }
}

/// Commit a joined node: persist it (UNNAMED — the host auto-names from the first
/// `NodeInfo` uplink via `NodeUpdate`, keeping this firmware payload-agnostic), install
/// the peer, and resolve the delayed `PairingOpen` response.
async fn commit_pairing(
    node_id: u32,
    key: [u8; 16],
    req_id: u16,
    net: &mut Net,
    buckets: &mut EepromBuckets,
) {
    let rec = NodeRecord {
        id: node_id,
        key,
        flags: mgmt::NODE_FLAG_UNNAMED,
        name: heapless::String::new(),
    };
    match registry::add(buckets, &rec) {
        Ok(()) => {
            net.add_peer(node_id, &key);
            STAT_NODES.store(registry::count(buckets) as u32, Ordering::Relaxed);
            info!(target: "gateway", "paired node {:08X}", node_id);
            respond_record(req_id, &Paired { node_id }).await;
        }
        Err(RegistryError::Full) => respond_empty(req_id, mgmt::MGMT_FULL).await,
        Err(_) => respond_empty(req_id, mgmt::MGMT_STORAGE).await,
    }
}

#[allow(clippy::too_many_arguments)] // the single-owner loop hands its whole state down once
async fn handle_mgmt(
    req: MgmtRequest<'_>,
    net: &mut Net,
    buckets: &mut EepromBuckets,
    queue: &mut Queue,
    stats: &mut Stats,
    pairing: &mut Option<PairingSession>,
    stats_period_ms: &mut u32,
    my_id: u32,
    band: Band,
    channel: u8,
    led: &led::Led,
) {
    let req_id = req.req_id;
    let now_ms = Instant::now().as_millis();
    match req.op {
        MgmtOp::Describe => {
            let name = console::firmware_name();
            respond_record(
                req_id,
                &DeviceInfo {
                    role: DeviceRole::Gateway,
                    radio_schema_version: RADIO_SCHEMA_VERSION,
                    net_id: my_id,
                    band: match band {
                        Band::Eu868 => mgmt::BAND_EU868,
                        Band::Us915 => mgmt::BAND_US915,
                    },
                    channel,
                    node_capacity: registry::CAPACITY as u8,
                    node_count: registry::count(buckets) as u8,
                    provisioned: true,
                    gw_id: my_id,
                    firmware_name: name.as_str(),
                },
            )
            .await;
        }
        MgmtOp::NodeList => {
            let now_s = Instant::now().as_secs() as u32;
            let mut stream = ChunkStream::new(req_id);
            for i in 0..registry::BUCKETS {
                // Encode one bucket's entries into a staging buffer synchronously, so
                // the ~270 B bucket local lives on the stack, not across the await.
                let mut staged: heapless::Vec<u8, 256> = heapless::Vec::new();
                for rec in registry::load(buckets, i).iter() {
                    let st = stats.get(rec.id);
                    let entry = NodeEntry {
                        id: rec.id,
                        name: rec.name.as_str(),
                        flags: rec.flags,
                        last_seen_s: st
                            .map(|s| now_s.saturating_sub(s.seen_s))
                            .unwrap_or(mgmt::LAST_SEEN_NEVER),
                        rssi_dbm: st.map(|s| s.rssi_dbm).unwrap_or(mgmt::RSSI_NONE),
                        uplinks: st.map(|s| s.uplinks as u32).unwrap_or(0),
                        queued: queue.count_for(rec.id) as u8,
                    };
                    let mut scratch = [0u8; 64];
                    if let Ok(bytes) = postcard::to_slice(&entry, &mut scratch) {
                        let _ = staged.extend_from_slice(bytes); // 6 × ≤42 B fits 256
                    }
                }
                stream.push(&staged).await;
            }
            stream.finish().await;
        }
        MgmtOp::NodeAdd { id, key, name, flags } => {
            if id == 0 || name.len() > mgmt::MAX_NODE_NAME {
                respond_empty(req_id, mgmt::MGMT_BAD_ARG).await;
                return;
            }
            let mut n: heapless::String<{ registry::MAX_NAME }> = heapless::String::new();
            let _ = n.push_str(name); // fits: checked above
            match registry::add(
                buckets,
                &NodeRecord {
                    id,
                    key,
                    flags,
                    name: n,
                },
            ) {
                Ok(()) => {
                    net.add_peer(id, &key);
                    STAT_NODES.store(registry::count(buckets) as u32, Ordering::Relaxed);
                    info!(target: "gateway", "node {:08X} added ({})", id, name);
                    respond_empty(req_id, mgmt::MGMT_OK).await;
                }
                Err(RegistryError::Full) => respond_empty(req_id, mgmt::MGMT_FULL).await,
                Err(_) => respond_empty(req_id, mgmt::MGMT_STORAGE).await,
            }
        }
        MgmtOp::NodeRemove { id } => match registry::remove(buckets, id) {
            Ok(()) => {
                net.remove_peer(id);
                queue.drop_node(id);
                stats.remove(id);
                STAT_NODES.store(registry::count(buckets) as u32, Ordering::Relaxed);
                STAT_QUEUED.store(queue.len() as u32, Ordering::Relaxed);
                info!(target: "gateway", "node {:08X} removed", id);
                respond_empty(req_id, mgmt::MGMT_OK).await;
            }
            Err(RegistryError::NotFound) => respond_empty(req_id, mgmt::MGMT_NOT_FOUND).await,
            Err(_) => respond_empty(req_id, mgmt::MGMT_STORAGE).await,
        },
        MgmtOp::NodeUpdate { id, name, flags } => {
            if name.is_some_and(|n| n.len() > mgmt::MAX_NODE_NAME) {
                respond_empty(req_id, mgmt::MGMT_BAD_ARG).await;
                return;
            }
            match registry::update(buckets, id, name, flags) {
                Ok(()) => respond_empty(req_id, mgmt::MGMT_OK).await,
                Err(RegistryError::NotFound) => respond_empty(req_id, mgmt::MGMT_NOT_FOUND).await,
                Err(RegistryError::BadName) => respond_empty(req_id, mgmt::MGMT_BAD_ARG).await,
                Err(_) => respond_empty(req_id, mgmt::MGMT_STORAGE).await,
            }
        }
        MgmtOp::NodeRevealKey { id } => match registry::find(buckets, id) {
            // The one deliberate key-disclosure path (host asked explicitly; keys are
            // otherwise never emitted on any interface).
            Some(rec) => respond_record(req_id, &NodeKey { id, key: rec.key }).await,
            None => respond_empty(req_id, mgmt::MGMT_NOT_FOUND).await,
        },
        MgmtOp::PairingOpen { window_s, key } => {
            if pairing.is_some() {
                respond_empty(req_id, mgmt::MGMT_BUSY).await;
                return;
            }
            if window_s == 0 {
                respond_empty(req_id, mgmt::MGMT_BAD_ARG).await;
                return;
            }
            if registry::count(buckets) >= registry::CAPACITY {
                respond_empty(req_id, mgmt::MGMT_FULL).await;
                return;
            }
            info!(target: "gateway", "pairing window open ({} s)", window_s);
            led.set_background(Some(PAIRING));
            *pairing = Some(PairingSession {
                req_id,
                key,
                deadline: Instant::now() + Duration::from_secs(window_s as u64),
            });
            // The response is DELAYED — it resolves when the window does.
        }
        MgmtOp::PairingCancel => {
            if let Some(sess) = pairing.take() {
                respond_empty(sess.req_id, mgmt::MGMT_TIMEOUT).await; // the open resolves…
                led.set_background(Some(HEARTBEAT));
            }
            respond_empty(req_id, mgmt::MGMT_OK).await; // …and the cancel acks
        }
        MgmtOp::QueuePush { node, ttl_s, data } => {
            if registry::find(buckets, node).is_none() {
                respond_empty(req_id, mgmt::MGMT_NOT_FOUND).await;
                return;
            }
            let ttl = if ttl_s == 0 { DEFAULT_TTL_S } else { ttl_s };
            match queue.push(node, ttl, data, now_ms) {
                Ok(item) => {
                    // Pre-mark: the pending flag must ride the ACK of the node's NEXT
                    // uplink, which is built inside recv() before we see the frame.
                    net.set_pending(node, true);
                    STAT_QUEUED.store(queue.len() as u32, Ordering::Relaxed);
                    respond_record(req_id, &QueueId { item }).await;
                }
                Err(PushError::BadPayload) => respond_empty(req_id, mgmt::MGMT_BAD_ARG).await,
                Err(_) => respond_empty(req_id, mgmt::MGMT_FULL).await,
            }
        }
        MgmtOp::QueueList { node } => {
            let mut stream = ChunkStream::new(req_id);
            let mut scratch = [0u8; 128];
            for item in queue.iter() {
                if node != 0 && item.node != node {
                    continue;
                }
                let entry = mgmt::QueueEntry {
                    node: item.node,
                    item: item.id,
                    age_s: item.age_s(now_ms),
                    ttl_s: item.ttl_s,
                    data: item.data(),
                };
                if let Ok(bytes) = postcard::to_slice(&entry, &mut scratch) {
                    stream.push(bytes).await;
                }
            }
            stream.finish().await;
        }
        MgmtOp::QueueDrop { node, item } => {
            let dropped = match item {
                Some(id) => match queue.pop(id) {
                    Some(it) if it.node == node => 1,
                    Some(it) => {
                        // Wrong node for that id — refuse without losing the item.
                        let _ = queue.push(it.node, it.ttl_s, it.data(), now_ms);
                        0
                    }
                    None => 0,
                },
                None => queue.drop_node(node),
            };
            net.set_pending(node, queue.count_for(node) > 0);
            STAT_QUEUED.store(queue.len() as u32, Ordering::Relaxed);
            if dropped > 0 {
                respond_empty(req_id, mgmt::MGMT_OK).await;
            } else {
                respond_empty(req_id, mgmt::MGMT_NOT_FOUND).await;
            }
        }
        MgmtOp::StatsConfig { channel_period_ms } => {
            *stats_period_ms = channel_period_ms; // RAM-only override of the setting
            respond_empty(req_id, mgmt::MGMT_OK).await;
        }
        // Node-side ops: this device is a gateway.
        MgmtOp::Provision(_) | MgmtOp::JoinOpen { .. } => {
            warn!(target: "gateway", "unsupported mgmt op (node-side)");
            respond_empty(req_id, mgmt::MGMT_UNSUPPORTED).await;
        }
    }
}

app!(run, commands: GW_COMMANDS, settings: GW_SETTINGS);
