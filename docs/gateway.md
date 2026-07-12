# TOWER Gateway — the wireless product guide

> **Status:** shipped and HW-verified (wire v3 / `tower-protocol` v1.3.0). The two product
> apps — `apps/radio_push_button.rs` (a sleeping sensor node) and
> `apps/radio_dongle_gateway.rs` (the network coordinator) — are complete firmwares, not
> skeletons. The host side (`tower gateway`, `tower nodes`, `tower net`, MQTT) lives in the
> `tower-cli` repo; this guide is the **device** side and the wire contract between them.

This is the standalone reference for the TOWER gateway product: a wireless push-button node
that reports button/temperature/accelerometer events and sleeps between them, a Radio Dongle
that bridges the secured radio network to the host, and the wire-v3 schema they speak. For
the radio stack itself (framing, CCM, pairing handshake, FHSS, duty cycle) see
[`radio.md`](radio.md); for the console framing and the settings framework see
[`console.md`](console.md).

---

## Three pieces

```
   push-button node  ──radio (CCM, NodeMsg schema)──▶  gateway dongle  ──serial v3──▶  tower gateway  ──▶ MQTT
   apps/radio_push_button        transparent bridge       apps/radio_dongle_gateway       (tower-cli)     tower nodes / net
   (STM32L083, sleeps)           forwards Uplink verbatim  registry + queue + pairing      TUI | --service
```

1. **Node** (`radio_push_button`) — a battery product. Button events (with per-kind counts),
   periodic temperature, accelerometer tilt events. STOPs between events (µA); the SPIRIT1
   sleeps. Downlinks reach it through the gateway's queue on its next uplink.
2. **Gateway** (`radio_dongle_gateway`) — the USB Radio Dongle. A **transparent bridge**: it
   authenticates/decrypts each uplink at the net layer and forwards the application payload
   *verbatim* to the host — it never decodes the node's app schema. What it owns is
   *coordination*: the node registry, the downlink queue, pairing, and radio diagnostics.
3. **Host** (`tower gateway`) — decodes the radio application schema, bridges to MQTT, and
   drives the gateway via the management channel. See the `tower-cli` README (Gateway & MQTT).

The transparent-bridge split is the design's backbone: **a new node app type needs no gateway
firmware change**, because the gateway never interprets `NodeMsg`/`NodeCmd` — only the node
firmware and the host CLI agree on that schema (versioned separately; see below).

---

## The wire-v3 gateway link

Wire v3 (`PROTOCOL_VERSION = 3`) added four console message types on top of the log/event/shell
set (full table in [`console.md`](console.md)):

| Type | # | Dir | Payload |
|---|---|---|---|
| `MgmtResponse` | 7 | T→H | `req_id, result, chunk, last, data` (chunked reply, `ShellResponse`-style) |
| `Uplink` | 8 | T→H | `src, counter, rssi_dbm, lqi, data` (a node's radio payload, forwarded verbatim) |
| `RadioStat` | 9 | T→H | ambient channel-RSSI sample, or a per-TX delivery outcome |
| `MgmtRequest` | 18 | H→T | `req_id, op` (a `mgmt::MgmtOp`) |

Two **independently versioned** schemas ride these frames:

- **`tower_protocol::mgmt`** — the console-side management contract (host ↔ device). Bumps with
  `PROTOCOL_VERSION`. `MgmtRequest{req_id, op}` → one or more `MgmtResponse` chunks with the same
  `req_id`; the `data` chunks concatenate into a stream of postcard records typed per op.
- **`tower_protocol::radio`** — the application schema *inside* the encrypted radio frame, carried
  opaquely by `Uplink`/`QueuePush`. Versioned by its own leading byte
  `RADIO_SCHEMA_VERSION = 1`, so it evolves without touching the gateway. Envelope:
  `[RADIO_SCHEMA_VERSION] ‖ postcard(NodeMsg | NodeCmd)`, always ≤ `MAX_RADIO_PAYLOAD` (74 B).

### Management ops (`mgmt::MgmtOp`)

| op | served by | reply record |
|---|---|---|
| `Describe` | both | `DeviceInfo` (role probe — the authoritative "is this a gateway?") |
| `NodeList` | gateway | `NodeEntry` × N (chunked) |
| `NodeAdd {addr,key,name,flags}` | gateway | — (install a cable-paired node) |
| `NodeRemove {addr}` / `NodeUpdate {addr,name?,flags?}` | gateway | — |
| `NodeRevealKey {addr}` | gateway | `NodeKey` (the only path that discloses a stored key) |
| `PairingOpen {window_s,key}` / `PairingCancel` | gateway | `Paired` (delayed) / `MGMT_TIMEOUT` |
| `QueuePush {node_addr,ttl_s,data}` | gateway | `QueueId` |
| `QueueList {node_addr}` / `QueueDrop {node_addr,item?}` | gateway | `QueueEntry` × N / — |
| `StatsConfig {channel_period_ms}` | gateway | — (RAM override of `stats-period`) |
| `Provision {addr?,gw_addr,key,band,channel}` | node | `ProvisionAck` |
| `JoinOpen {window_s}` | node | `Joined` (delayed) |

Result codes: `MGMT_OK`(0), `UNSUPPORTED`(1), `BAD_ARG`(2), `NOT_FOUND`(3), `FULL`(4),
`BUSY`(5), `STORAGE`(6), `TIMEOUT`(7). An op a device does not serve → `UNSUPPORTED`; a
concurrent pairing window → `BUSY`. **The host mints all AES keys** — device PRNGs are
non-cryptographic, so keys travel host→device (in `PairingOpen`/`NodeAdd`/`Provision`), never
the reverse except the deliberate `NodeRevealKey`.

### Radio application schema (`radio::NodeMsg` / `NodeCmd`)

Node → host (`NodeMsg`): `Info(NodeInfo{firmware_name, firmware_version, session_id, sleeping,
battery_mv})` · `Button{kind, count}` · `Temperature{millic}` · `Accel{kind, face}` ·
`Shell(NodeShellChunk{cmd_id, result, chunk, last, text})`. `ButtonKind` = Press/Release/Click/
Hold; `AccelKind` = Motion/Orientation. Per-kind button `count` is RAM-only — a reset means the
node rebooted (watch `NodeInfo::session_id`).

Host → node (`NodeCmd`): `Shell{cmd_id, line}` — queued as a downlink; the node runs it in the
standard shell dispatcher and answers with `NodeMsg::Shell` chunks (≤ `RADIO_SHELL_CHUNK` = 56 B)
correlated by `cmd_id`.

### TX outcomes (`RadioStat::Tx.outcome`)

`TX_DELIVERED`(0, node ACKed), `TX_NOT_DELIVERED`(1), `TX_BUSY`(2), `TX_DUTY_LIMITED`(3),
`TX_ERROR`(4), `TX_EXPIRED`(5, TTL lapsed before delivery — never transmitted).

---

## The push-button node (`radio_push_button`)

A sleeping node. Every enabled button event sends a secured `NodeMsg::Button` with that kind's
running count; temperature is measured on a period and sent on-change or at latest every
heartbeat; the accelerometer's tilt interrupt wakes the node for `NodeMsg::Accel` events. All
of it is remote-shell reconfigurable (`/system settings set …`).

### Settings

| setting | kind | default | range / values | effect |
|---|---|---|---|---|
| `temp-period` | uint s | `60` | 5..86400 | temperature measure/send cadence |
| `temp-delta` | uint c°C | `50` | 0..10000 | send when |ΔT| ≥ this; `0` = every period |
| `accel` | enum | `medium` | off/low/medium/high | tilt sensitivity (**boot-applied**) |
| `heartbeat` | uint s | `900` | 60..86400 | max interval between uplinks (`NodeInfo`) |
| `press` | bool | `off` | on/off | master enable — Press events |
| `release` | bool | `off` | on/off | master enable — Release events |
| `click` | bool | `on` | on/off | master enable — Click events |
| `hold` | bool | `on` | on/off | master enable — Hold events |
| `debounce-press` | uint ms | `30` | 1..1000 | press-recognition debounce (**boot-applied**) |
| `debounce-release` | uint ms | `30` | 1..1000 | release debounce (**boot-applied**) |
| `click-timeout` | uint ms | `500` | 50..5000 | max press length still counted a click (**boot-applied**) |
| `hold-time` | uint ms | `1000` | 100..10000 | press length that becomes a hold (**boot-applied**) |
| `addr` | addr | `auto` | hex / auto / random | SDK base: the node's 32-bit radio address |
| `identity` | str | *(empty)* | ≤ 32 chars | SDK base: friendly device name |

**Event master enables gate the whole path.** A disabled event is ignored *entirely* — not
reported over radio and not shown on the LED. The default is the coherent **gesture** scheme:
`click` + `hold` on, `press` + `release` off (a click is a quick press+release, so reporting all
four is redundant and their LED shapes blend on a tap). Enable `press`/`release` for raw-edge
reporting.

**Timing is recognition, not indication.** `debounce-*`, `click-timeout`, and `hold-time`
define what the physical input *means*; the LED feedback is a separate fixed indicator (below).

### LED behaviour

- **Boot signature (every TOWER board):** 500 ms off → 2 s on → 500 ms off
  (`Board::boot_indicator`), then the app's own behaviour begins. A 3-second power-on
  fingerprint common to all firmwares.
- **Per-event feedback (fixed constants, not configurable):** Press = a single 250 ms pulse ·
  Release = 250 ms · Hold = 1000 ms · Click = a **double-blink** (2 × 100 ms). The click's
  distinct shape is deliberate: a quick tap fires Press on the down edge and Release+Click
  together on the up edge, so a shared shape would blend on the one LED. The LED reacts only to
  *enabled* events.

### `/button simulate <ms>`

Injects a synthetic press of `<ms>` (1..10000) through the **real** recognition machine
(debounce → click/hold classification) and on into reporting — the console "finger" for testing
without a physical press. A `100` ms press is a click (> debounce, < click-timeout); a `1500` ms
press is a hold. Used by the HIL gateway test.

### Pairing (node side)

- **OTA** — hold the button ≥ 1 s while unprovisioned: the node runs the 3-way JOIN against any
  gateway with an open window and persists `(gw_addr, key, band, channel)` in `NS_APP`.
- **Cable** — on USB the node serves the management channel: `Describe` (role probe), `Provision`
  (host-minted credentials — the key never rides the shell history), and `JoinOpen`
  (host-initiated OTA join). The node reboots into its new identity; watch for the fresh `Hello`.

---

## The gateway dongle (`radio_dongle_gateway`)

One main loop owns `Net` + the registry + the queue (no `&mut Net` shared across tasks) and
multiplexes: recv slices, management requests, the pairing window, and the stats tick.

### Node registry (`tower-gw-core::registry`, EEPROM)

Paired `(id, key, flags, name)` records, persisted in `NS_APP` postcard **buckets** — never
RAM-resident (bucket locals are ~270 B of transient stack), mirrored into the net peer table at
boot. `PER_BUCKET = 6`, `BUCKETS = 3` (18 slots ≥ capacity), **`CAPACITY = 16`** (matches the net
layer's `MAX_PEERS`), `MAX_NAME = 16`. Written only on add/remove/rename/pairing-commit — never
per-uplink (that would burn the part). Keys: `KEY_FORMAT = 0x00`, buckets from
`KEY_BUCKET_BASE = 0x10`.

### Downlink queue (`tower-gw-core::queue`, RAM-only)

Opaque host-built `NodeCmd` envelopes awaiting a sleeping node's next uplink. Global pool
`QUEUE_CAP = 4`, per-node FIFO `PER_NODE_CAP = 2`, `MAX_ITEM = 74` B. Stable monotonic `u16` ids
(from 1; `0` reserved = "not a queue item" in `RadioStat::Tx`). TTL expiry drops stale items and
reports `TX_EXPIRED`. **The queue is RAM-only:** a gateway reboot drops it, made visible by the
`Hello` `session_id` bump — the host re-queues. Delivery: each confirmed uplink's ACK carries a
*pending* flag; when set, the node opens a short RX window and the gateway delivers one item per
uplink cycle.

### Settings

| setting | kind | default | range / values | effect |
|---|---|---|---|---|
| `stats-period` | uint ms | `1000` | 0..60000 | ambient channel-RSSI cadence; `0` = off. `StatsConfig` overrides at runtime |
| `band` | enum | `eu868` | eu868 / us915 | radio band (**boot-applied**) |
| `channel` | uint | `0` | 0..2 | radio channel (**boot-applied**) |
| `addr` | addr | `auto` | hex / auto / random | SDK base: the coordinator's radio address |
| `identity` | str | *(empty)* | ≤ 32 chars | SDK base: friendly name |

### Pairing (gateway side)

- **OTA** — host sends `PairingOpen{window_s, key}`; the gateway opens the radio window, and on a
  join commits the registry entry (flags `UNNAMED` until the host auto-names it from the first
  `NodeInfo` via `NodeUpdate`) and answers the *delayed* `Paired`, or `MGMT_TIMEOUT` on expiry.
- **Cable** — the host (which alone can reach the node's serial port) provisions the node
  directly, then registers it on the gateway with `NodeAdd{addr, key, name, flags}`.

### RAM budget

flip-link splits the 20 KB SRAM into **statics at the top** (`.data`/`.bss`/`.uninit`) and the
**stack below** them, so a stack overflow faults below the RAM base instead of silently corrupting
`.bss`. Everything an app keeps *resident* eats the statics half and shrinks the stack half:

| App | Statics | Stack budget | Measured peak (SWD stack-paint, 2026-07-11) |
|---|---:|---:|---|
| `radio_push_button` | ~10.8 KB | ~9.4 KB | **~6.8 KB** (radio-send + button + KV; stable under compaction churn) |
| `radio_dongle_gateway` | ~11.4 KB | ~8.6 KB | ~3.3 KB console-idle; deep registry/mgmt paths not bench-driveable standalone, **est. ~7.5 KB** |

The statics are **~67 % async task futures** — Embassy stores each task's across-`await` state
resident, so `__embassy_main`'s future alone is ~4.3 KB (the app's own main-loop state, not
framework overhead). That's *why* the stack peak is only ~7 KB and not ~15: async trades a deep
transient stack for resident static state. It also means growing resident state (a bigger
`MAX_PEERS`, more buffers held across awaits) shrinks the stack directly — which is how an
early over-budget gateway build HardFaulted at boot (stack overflow; flip-link now makes that a
link/boot fault, not `.bss` corruption). Hence `MAX_PEERS` **16** (not 32), the registry on
EEPROM (transient bucket locals), the 4-item queue, and 12-byte packed link stats.

**Guard:** `just ram-budget` (and CI) fails if any product bin leaves less than an 8 KB stack —
a regression tripwire so a future-inflating change is caught at PR time, not at a bench HardFault.
Re-check `just size app radio_dongle_gateway` after growing anything resident; if a bin genuinely
needs more, raise the floor *and re-measure the high-water mark on hardware*, don't just nudge it.
TODO: a two-board HIL-driven stack-paint read of the gateway's deep paths (registry-bucket codec +
mgmt chunking + radio bridge) to replace the ~7.5 KB estimate with a measured number.

---

## Remote shell over the radio

The full round-trip, node asleep between events:

1. Host encodes `NodeCmd::Shell{cmd_id, line}` and `QueuePush`es it to the gateway.
2. The gateway holds it; the next uplink's ACK advertises the pending flag.
3. The node opens a short RX window, decodes the `NodeCmd`, runs `line` through the same
   `shell::run_line` dispatcher the console uses, and streams the reply back as `NodeMsg::Shell`
   chunks (each reply's ACK re-flags pending, chaining the queue).
4. The gateway forwards the chunks as `Uplink` frames and reports delivery via `RadioStat::Tx`.

No tab completion over the air — completion is a per-transport feature the radio transport
doesn't offer.

---

## Building, flashing, testing

```bash
just build app radio_push_button      # or radio_dongle_gateway
just run   app radio_push_button      # build + flash + open the console TUI
just size  app radio_dongle_gateway   # on-chip footprint (watch the RAM budget)
```

The end-to-end product path is covered by the HIL bench group `hil/tests/gateway.rs` (two boards,
`#[ignore]`d): cable-provision the node, bridge a simulated `/button simulate 100` click, run a
remote shell command through the downlink queue, and assert the `RadioStat` stream — driven from
the `tower-hil` repo with `just hil`.

## See also

- [`radio.md`](radio.md) — the secured radio stack (CCM, confirmed delivery, pairing handshake,
  FHSS, duty cycle) this product rides on.
- [`console.md`](console.md) — the console framing, the full MsgType table, and the settings
  framework (`Kind`, base settings, result codes).
- `tower-cli` README (*Gateway & MQTT bridge*) — the host commands (`tower gateway`, `tower nodes`,
  `tower net`) and the MQTT topic tree (the gateway's public API, in `src/gateway/topics.rs`).
