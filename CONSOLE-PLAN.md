# TOWER Console — host↔target console subsystem (phased plan)

> **STATUS: all 6 phases built + hardware-verified (2026-06-27); the TUI is build-only
> (no TTY in CI).** Built on two Radio Dongles (`/dev/cu.usbserial-120` / `-140`,
> STM32L08x): framed logs/print/events, the shell with KV-backed identity persisting
> across reset, target-authoritative TAB completion, and the `tower console` TUI. The
> four shaping decisions are locked (§9). Build record. Spec: `CONSOLE.md`; style: `FOTA.md`.
>
> **Deviations from this plan, forced during HW bring-up:** (1) **TX/RX use embassy
> `BufferedUart` (interrupt-driven async, no DMA), not async DMA** — the WS2812 strip
> owns the `DMA1_CHANNEL2_3` IRQ group, so DMA is out (§5.1/§5.5). BufferedUart first
> appeared to fail (TX dead) — **root cause: the low-power STOP executor gates the USART
> clock while the writer awaits the TXE interrupt; fix: hold a `WakeGuard` per transmit
> burst** (→ WFI). (2) **`seq` is assigned by the writer at send time**, not producers,
> so queue drops don't consume sequence numbers (§4). (3) The shell/settings framework
> is now **generalized**: a walkable tree of `Entry::{Menu, Cmd}` where each `Cmd` holds
> a handler **fn pointer** (so apps extend it), and a declarative `Setting` table drives
> `/system settings print|set|get` + `/export` with no per-setting code (§6 note).

---

## Resume in one minute

- **Goal:** replace today's transmit-only text console with a reliable, framed,
  **bidirectional** link carrying three entities — **device logs**, **shell
  commands/responses**, and **unsolicited events** — plus a host CLI/TUI `tower`
  that renders them. The UART is *always* framed; `tower` replaces `jolt monitor`.
- **Transport:** Postcard payload, COBS frame sync (`0x00` delimiter), CRC‑32,
  a `ver_type` byte (version + message type) and a 2-byte `seq` for gap detection.
  Single source of truth for the wire format is a new no_std crate
  **`tower-protocol`**, depended on by both the firmware and `tower-cli`.
- **Locked decisions (§9):** PA10↔bridge RX is wired ⇒ **full-duplex USART1**;
  the shell is **target-authoritative** (firmware owns parsing, the command tree,
  execution *and* TAB completion); **`tower` replaces the raw-text monitor** (no
  dual-mode — `print!` becomes a framed `Print` message); the shared schema lives
  in a **dedicated `tower-protocol` crate**.
- **First action:** Phase 0 — turn this repo into a Cargo **workspace**, add the
  `tower-protocol` crate (envelope + `MsgType` + CRC‑32 + payload structs), set
  PA12 to `Pull::Down`, reserve the KV key range. No behaviour change yet.
- **The one non-negotiable:** the log/print path must **never block the system**
  on a full wire. Boot-time logs **buffer in the channel** until the writer task
  runs; the **panic handler** emits its final frame **directly via the PAC** (the
  executor is dead). Both are spelled out in Phase 1 — get them right first.

---

## 1. Context & goal

We have a TX-only debug console: `src/console.rs` owns a blocking `UartTx`
(USART1 TX = PA9, 115200 8N1) behind a global, and is the backend for the `log`
facade and the `print!`/`println!` macros. It renders text and is read by
`jolt monitor`. There is no receive path.

`CONSOLE.md` wants more: a structured, reliable, *two-way* link so the host can
(a) stream colorized logs with timestamps + module, (b) run a RouterOS-style
shell (one command → one response + result code), and (c) receive unsolicited
structured events — all over the one UART, framed so "anything can happen" on
the wire is handled. The host side is a `clap`/`ratatui` CLI named `tower`
(separate repo: `/Users/pavel/hardwario/github/tower-cli`).

Target part: **STM32L083CZ** — 192 KB flash, **20 KB RAM**, Cortex-M0+ @ 16 MHz
(HSI16), 6 KB data EEPROM. Single core, no atomic CAS.

## 2. Current state (build on this)

| Capability | Where | Notes |
|---|---|---|
| TX-only console + `log` backend | `src/console.rs` | Blocking `UartTx` behind a `critical_section` global; renders `[secs.ms] LEVEL module: msg`. **Becomes** an enqueue + async writer (§5). |
| `print!`/`println!` macros | `console.rs:133`/`:140` | Backend `_print` (`console.rs:121`). **Repoint** to enqueue a `Print` frame — examples keep working unchanged. |
| Panic handler | `console.rs:153` | Executor is dead at panic ⇒ emit one framed record **directly via the PAC** (`unstable-pac`), bypassing the channel (§5.2). |
| Console wiring | `board.rs:168` `init_console` | `UartTx::new_blocking(USART1, PA9, …)`. **Becomes** the full-duplex `Uart` (PA9 TX + PA10 RX) **built in Phase 1** and `split()` — writer owns TX, RX parked until Phase 3 (purely additive). |
| USB-presence power gating | `src/power.rs` `vbus_task` | USB present ⇒ `WakeGuard` ⇒ Sleep not STOP ⇒ USART clocks stay live. **Extend it to publish a presence `Watch`** so the RX reader can reset on replug (§5.5). |
| VBUS sense pin | `board.rs:116` `ExtiInput::new(PA12, …, Pull::None)` | Spec: set **`Pull::Down`** (defined low when unplugged). One line; caveat in §10. |
| Postcard codec | `postcard 1.1.3`; used by `storage.rs` | Reuse. **Not self-describing** — both ends must share exact structs ⇒ `tower-protocol`. |
| COBS framing | `cobs` crate already in `Cargo.lock` (via postcard) | Use the `cobs` crate via `tower-protocol`'s own `FrameDecoder` — **not** postcard's `CobsAccumulator`, whose "one COBS frame = one postcard message" assumption the trailing CRC breaks (§4, §5.5). |
| CRC‑32 (bitwise, no table) | `storage.rs:320` `crc32_update` | **Move into `tower-protocol::crc`** as the shared util; `storage.rs` delegates to it (behaviour unchanged) and the frame CRC uses it too — one primitive, no new crate, stronger than 16‑bit. |
| Frame-codec idiom | `radio/frame.rs` | `ver_type` byte (`VERSION<<5 | type`), typed enum + `from_u8`, explicit `encode/parse`. **Mirror this** in `tower-protocol`. |
| KV store | `storage.rs` `Kv` (postcard or raw, CRC-checked, in-place update) | Identity + shell settings. Reserve key range **`0x5500+`** (FOTA reserved `0x5400+`). |

## 3. Constraints & RAM budget

- **20 KB RAM** is the gate. Everything below is fixed-size, no-alloc, `heapless`.
- **HSI16 ±1%** (`board.rs:159`) limits reliable async-UART baud ⇒ **keep 115200**
  for v1 (~11.5 KB/s, ~87 µs/byte). Higher baud would need HSI48+CRS trimmed to
  the existing LSE — out of scope.
- **Cortex-M0+ has no atomic CAS** — `log::set_logger_racy` is already used
  (`console.rs:106`); keep the single-init-at-startup discipline.
- **RX only matters while USB is present** (host attached ⇒ no STOP). It's an async
  `BufferedUart` read (§5.5); unplugged there's no host and `read()` just parks.

Starting RAM budget — **whole subsystem** (confirm against a real build):

| Buffer | Phase | Size | Purpose |
|---|--|--:|---|
| TX frame channel | 1 | 8 × ~280 B ≈ 2.2 KB | owned `Outgoing` queue (logs/print/events + shell responses/completions); writer encodes |
| Log render scratch | 1 | ~200 B | `format_args!` → `heapless::String` before postcard (`MAX_MSG = 192`) |
| COBS encode scratch | 1 | ~272 B | writer/panic encode buffer (inner + COBS overhead) |
| `FrameDecoder` inner buf | 3 | ~272 B | reassemble one inbound frame (RX **polled via PAC** — no DMA ring) |
| **Phase 1 subtotal** | | **≈ 2.7 KB** | TX path only |
| **Phase 3 delta** | | **≈ 0.3 KB** | RX decoder only (dispatch is inline — no command channel) |
| **Total** | | **≈ 3.0 KB** | comfortably within 20 KB |

`MAX_FRAME` (inner payload, pre-COBS) target = **256 B**; `MAX_WIRE ≈ 272 B`. The
command/settings descriptor tables are **flash, not RAM**. **The console uses no DMA** —
TX and RX are interrupt-buffered (`BufferedUart`, §5.1/§5.5) because the strip owns the
`DMA1_CH2/3` IRQ group. The `BufferedUart` TX/RX ring buffers (256 B / 128 B) are extra.

## 4. Wire protocol (`tower-protocol` crate)

```
wire:   COBS( inner )  0x00
inner:  ver_type(1) | seq(2) | payload(postcard, var) | crc32(4, LE)
        ver_type = (PROTOCOL_VERSION << 5) | (msg_type & 0x1F)   // cf. frame.rs:129
        crc32    = CRC-32/IEEE over [ver_type, seq, payload...]  // verified after COBS decode
```

- **`seq`** — `u16`, monotonic per direction, for **gap detection**. Assigned by the
  **writer task at send time** (a plain `u16` counter — no atomics, no critical section),
  so `seq` counts only frames actually put on the wire: a gap means real wire loss
  (CRC/byte drop), while *queue* drops are reported separately by the `Dropped` marker.
  (Producer-assigned seq was tried first and rejected on hardware — dropped frames
  consumed seq numbers and the writer's marker reordered, both spamming false gaps.)
- **`cmd_id`/`req_id`** — request/response **correlation** lives *inside* the shell
  payloads, not in the header (a late/lost response must never be misattributed).
- **Resync** — the host discards bytes **up to and including** the first `0x00`, then
  accumulates from the next byte; the target emits a lone `0x00` (+ `Hello`) whenever
  it observes VBUS present — at boot and on each absent→present edge (§5.5) — flushing
  any partial frame and re-announcing.
- **Versioning** — `decode_frame` validates `PROTOCOL_VERSION` (in `ver_type`) on
  **every** frame — that per-frame check is the mismatch guard, independent of when the
  host attaches. `Hello` is **supplementary**: it carries the firmware-version *string*
  (for the header) and marks a fresh connection; best-effort, emitted on boot and on
  each VBUS absent→present edge (a replug doesn't reset the MCU). A host that attaches
  to an already-running device mid-stream still version-checks off the first frame it
  decodes. No host→target `Hello` — the host adapts to the target.
- **Overflow** — **per-producer policy** on the shared TX queue: the sync
  `log`/`print` backends `try_send` and **drop the incoming frame** (drop-newest) +
  bump a counter on full; the shell responder and event emitter run in async tasks
  and `send().await` (backpressure, never dropped). A `Dropped{count}` marker frame
  reports the lost logs. Drop-newest matches `Channel::try_send` and can't evict a
  queued response, which drop-oldest could.
- **Codec** — `tower-protocol` owns `encode_frame` and a byte-fed `FrameDecoder` +
  `decode_frame` (deframe on `0x00` → COBS-decode → version + CRC check → split
  `type`/`seq`/payload), used by **both** ends. It deframes itself rather than using
  postcard's `CobsAccumulator`, which would choke on the trailing CRC.

Message catalogue (representative — refine in Phase 0; borrowed `&str`/`heapless::Vec`
so it serializes no-alloc on the target and decodes zero-copy on the host):

```
PROTOCOL_VERSION: u8 = 1
MsgType (u8, low 5 bits of ver_type):
  // target -> host
  Hello            = 0   { protocol_version, firmware_version }
  Log              = 1   { level, uptime_us, module, message }
  Print            = 2   { text }                       // from print!/println!
  Event            = 3   { name, fields: Vec<(&str,&str), 8> }  // self-describing key=value pairs
  ShellResponse    = 4   { cmd_id, result, chunk, last, text }  // result: 0 = success
  ShellCompletions = 5   { req_id, token_start, common_prefix, candidates: Vec<Candidate, 16> }
                         //   Candidate { text, kind }  kind: Menu|Command|Arg|Value; >16 → `more` flag
  Dropped          = 6   { count }                      // overflow marker
  // host -> target
  ShellCommand     = 16  { cmd_id, line }
  ShellComplete    = 17  { req_id, line, cursor }       // target-authoritative TAB
```

## 5. Firmware architecture

**As built:** `src/console.rs` (single file — the global TX `Channel`, the `log`
backend + `print!` macros, the async `BufferedUart` writer task, the PAC panic handler)
and a sibling `src/shell.rs` (the opt-in shell: async-RX task + command tree + completion).
`tower-protocol` stays a **pure codec** (no I/O, no Embassy), used by both ends.
`board.rs` builds the full-duplex `BufferedUart`, parks RX (`console::take_rx`), and
spawns the writer; the shell is opt-in via `shell::serve(spawner, storage)` (§6).
(The split-module layout / DMA / presence `Watch` below were the pre-bring-up plan.)

### 5.1 Single TX arbiter — one owner, no critical section
Three producers (the sync `log` backend, the event emitter, the shell responder)
must never interleave mid-frame. Each builds an **owned** message (an `Outgoing` enum
with `heapless::String` fields — nothing borrows past the producing call) and submits
it to a single `Channel<CriticalSectionRawMutex, Outgoing, DEPTH>`: the sync
`log`/`print` backends `try_send` (drop-newest on full, §4); the async event/shell
producers `send().await` (never dropped). **One writer task owns the `UartTx`
outright**, assigns the `seq`, encodes (postcard → CRC → COBS → `0x00`), and drains the
channel. Single ownership means the UART needs **no `critical_section` Mutex** — today's
`Mutex<RefCell<Option<Console>>>` goes away; only the panic path touches the peripheral,
via the PAC (§5.2).

**TX is interrupt-buffered (`BufferedUart`), async, with a per-burst `WakeGuard`.** DMA
TX is **infeasible on this board** — the WS2812 strip binds the `DMA1_CHANNEL2_3`
interrupt (`ws2812.rs`) and USART1's DMA channels (TX `CH2/CH4`, RX `CH3/CH5`) can't
form two clash-free interrupt groups — so the console uses `BufferedUart` (interrupt-
driven, no DMA) instead. The writer:

```rust
let item = TX_CHANNEL.receive().await;          // truly idle: NO guard → STOP allowed (unplugged)
let _guard = WakeGuard::new(StopMode::Stop1);    // hold across the burst → WFI, USART clocked
// ... assign seq, encode, then async write each frame:
let _ = tx.write_all(&buf[..n]).await;
let _ = embedded_io_async::Write::flush(&mut tx).await; // drain the ring before dropping the guard
// _guard drops here
```

**Why the guard is essential** (this was the BufferedUart bug): the async write awaits
the USART TXE interrupt, but the low-power executor enters **STOP** when the writer
idles — STOP gates the USART clock, so the interrupt never fires and TX hangs (zero
bytes). The `WakeGuard` raises the stop-refcount → the executor does a plain **WFI**
instead → the USART stays clocked → the interrupt fires. (Blocking TX, tried first,
worked only because it spins in Run mode.) The guard is held **only per burst**, so the
idle `receive().await` still reaches STOP when unplugged — low power preserved. `flush`
is called via UFCS (`embedded_io_async::Write::flush`) because an inherent private
`flush` on `BufferedUartTx` shadows the trait method.

### 5.2 Emit paths: async-queued (normal + boot) and PAC-direct (panic)
- **Async-queued** (normal *and* early boot): producers enqueue; the writer task
  sends. Logs emitted before the writer task is spawned (e.g. the boot banner)
  simply **buffer in the channel** and flush the first time the app awaits — no
  special early path is needed; just size `DEPTH` to absorb boot chatter.
- **PAC-direct** (panic only): in the panic handler the executor is dead, so the
  writer never runs. The handler builds one framed `Log{ level: Error, … }` on the
  stack and blocking-emits it straight to the `USART1` registers via the PAC
  (`unstable-pac` is already enabled), guarded by a check that `USART1` is enabled
  (a pre-`init` panic just halts, as today). Same frame bytes as the async path —
  `tower` can't tell them apart.

The `log` backend is **sync and may run with IRQs off**, so it must enqueue
non-blocking and **drop the incoming frame + count** (drop-newest, §4) on a full
queue — never `.await`, never block.

### 5.3 Structured logs (formatting moves to the host)
The target stops rendering `[secs.ms] LEVEL module: msg`. It sends a `Log{ level,
uptime_us, module, message }`. The message is rendered from `format_args!` into a
bounded `heapless::String<MAX_MSG>` (truncate + mark if over). `tower` prepends
local time, colorizes by level, and lays out the columns.

### 5.4 `print!`/`println!` → `Print` frame
Repoint `_print` (`console.rs:121`) to enqueue a `Print{ text }` frame. Examples
that call `println!` keep working unchanged; `tower` renders `Print` verbatim.

### 5.5 RX path (as built: interrupt-buffered async)
The full-duplex `BufferedUart` is built and `split()` by the board; the writer owns TX,
and the `BufferedUartRx` half is parked in a static `Option` (`console::take_rx`). The
shell (`shell::serve`) takes it and spawns a task that **async-reads** it — `rx.read()`
awaits the USART RX interrupt (no busy-poll) — feeding `tower-protocol`'s `FrameDecoder`
(not `CobsAccumulator` — §4). On a complete frame it decodes a `ShellCommand`/
`ShellComplete`, copies the borrowed line into an owned `heapless::String`, and
dispatches **inline** (the host is interactive — one request at a time — so no separate
engine task / busy-NAK is needed).

RX needs no extra guard: it only matters while USB is present, when `vbus_task` already
holds STOP off → the RX interrupt fires and `read()` wakes. (An earlier polled-PAC
fallback was used while `BufferedUart` was thought broken; once the TX root cause was
fixed (§5.1) RX moved to the clean async read, dropping the busy-poll.)

**STOP / replug:** unplugged, the device STOPs and `read()` simply doesn't wake (no
host). On replug the `FrameDecoder` resyncs on the next `0x00` and the console re-emits
`Hello` (§4). A presence `Watch` from `vbus_task` is available for an explicit decoder
reset but isn't required.

## 6. Shell & settings framework (target-authoritative)

- **Command tree = a `static` declarative descriptor table** (no_std, no-alloc).
  Each node carries: path segment, optional handler, an arg schema (`name=value`,
  fixed-size arrays, `heapless::String` values), help text. The table must be
  **walkable** (list children by prefix; list a command's arg names) because the
  target owns completion too.
- **Execution:** `ShellCommand{ line }` → tokenize → walk from root (no subpath
  entry — always starts with `/`, per spec) → run handler → `ShellResponse` with a
  `result` code (`0` = success) + text.
- **Completion (target-authoritative):** `ShellComplete{ line, cursor }` →
  `ShellCompletions{ token_start, common_prefix, candidates }`. The **same** walk
  serves dispatch and completion (no forked completer → no divergence); a TAB is one
  round-trip. Detailed in Phase 4.
- **Responses are chunked-but-logically-one:** `ShellResponse` carries `chunk` +
  `last`; the host reassembles into the single response the spec promises. `result`
  is authoritative only on the `last` chunk (the handler's output streams out before
  it returns, so earlier chunks carry a pending sentinel); the `chunk` index lets the
  host detect a CRC-dropped chunk mid-response. Fixed target buffer; `/export` grows
  without truncation.
- **Settings auto-derive from the table:** a setting descriptor `{ path, type,
  validate, kv_key, render_as_set_command }` gives `print`/`set`/`/export` for
  free. `/export` walks every setting and prints its current value as the `set`
  command (spec: "print everything, not just deviations").
- **First commands:** `/system reboot` (flush the response frame **before**
  `SCB::sys_reset()`, or the host sees a truncated reply), `/system/resource print`,
  the table-derived `/system settings print|set <name>=<value>|get <name>` (`identity`
  is a `Str` setting in `Kv` under `0x5500+`), and `/export`.
- **Wiring & ownership:** logging/TX is always-on (`Board::take` spawns the writer);
  the **shell is opt-in** — an app calls `console::shell::serve(spawner, board.storage)`,
  which *consumes* `Kv` and spawns the reader + engine. Apps that don't want a shell
  keep `board.storage`; an app needing both shares it as
  `Mutex<CriticalSectionRawMutex, Kv>`. (App-extensible trees — merging an app's
  `&'static [Node]` into the SDK root — are a follow-up; v1 ships the base tree.)
- **As built (Phase 3/4 + framework):** a walkable `static` tree of
  `Entry::{Menu, Cmd}` where each `Cmd` carries a **handler fn pointer** (`fn(&mut Ctx,
  &[&str]) -> Outcome`) instead of a closed `CmdId` enum — so the tree is **open**. One
  `resolve()` walk is shared by dispatch and completion. Settings are **declarative and
  auto-derived**: a `&'static [Setting] { key, name, kind, default }` table (`Kind::{Str,
  U32, Bool}`) drives generic `/system settings print|set|get` + `/export` with no
  per-setting code, and completion of `set`/`get` enumerates the table dynamically.
  Identity is just an SDK `Str` setting (key `0x5500`). **App-extensible:**
  `serve_ext(spawner, storage, app_commands: &'static [Entry], app_settings: &'static
  [Setting])` merges an app's commands at the root and its settings into the table (the
  walker spans base ⧺ app); the host is target-authoritative so it needs **no changes**.
  Dispatch runs **inline** in the RX task (which async-reads `BufferedUartRx`; no separate
  engine task / `Mutex<Kv>`). Responses **are chunked** (the writer splits text into
  `SHELL_CHUNK`=192-byte `chunk`/`last` frames the host reassembles by `cmd_id`).

## 7. Host architecture (`tower-cli`)

- Depends on **`tower-protocol`** (same crate as the firmware) — the only thing
  preventing silent schema drift.
- One reader task runs the shared `FrameDecoder`/`decode_frame` (§4) and demuxes
  frames into channels: logs, events, shell responses (correlated by `cmd_id`),
  completions (by `req_id`). One writer for commands.
- **Reconnection:** detect USB unplug, show it, auto-reconnect, resync to the next
  `0x00`. Version is validated per-frame (§4); a fresh `Hello`, when one arrives,
  refreshes the firmware string. Auto-pick the port when exactly one (like `jolt
  list`); else `--device/--port`.
- **Commands:** `tower devices`, `tower logs [--no-colors]`, `tower events`,
  `tower shell`, `tower console`, plus `tower monitor --hex/--raw` (transport
  debugging, since a plain terminal now shows binary).
- **Host log line:** `<local-time> [<uptime>] <LEVEL> <module>: <msg>`.
- **TUI (`tower console`):** layout per `CONSOLE.md` (left: Events 25% / Shell
  Command 1 line / Shell Responses bottom-anchored; right: Logs full height).
  Decisions baked in: **Pause (F5) buffers underneath** (no drop); scrollback ring
  caps host memory; **command history + Ctrl-R persist to a history file**; the footer
  clock ticks at 1 Hz (redraws are coalesced); gray/black header & footer; Shift-Tab focus cycle;
  F3 zoom (borderless), F8 clear (Events/Responses/Logs), F10 quit.

## 8. Phased plan

Each phase ends with an on-hardware verify, mirroring the FOTA/FHSS style.

### Phase 0 — Workspace + protocol crate (no behaviour change)
- Convert the repo to a Cargo **workspace**; add `tower-protocol` (no_std) with the
  envelope, `MsgType`, payload structs, and the COBS helpers.
- **Shared CRC util:** `tower-protocol::crc` exposes `crc32_update(crc, &[u8])`
  (raw, no table) + `crc32_ieee(&[u8])` (init `0xFFFF_FFFF`, finalize `!`). The frame
  uses the one-shot; `storage.rs` `entry_crc` (`storage.rs:314`) is refactored to
  call the shared `crc32_update` — **byte-for-byte the same** init/XOR as today, so
  existing EEPROM records stay valid — and its private `crc32_update` is removed.
  (The firmware already depends on `tower-protocol`, so this adds no new edge.)
- `board.rs:116`: `Pull::None` → `Pull::Down` on PA12.
- Reserve KV range `0x5500+`; sketch the descriptor types.
- **Exit:** `cargo build`/`clippy` clean across the workspace; `storage` still
  passes its existing checks and reads back records written before the CRC move
  (proving the shared util is bit-identical); no runtime behaviour change.

### Phase 1 — Framed TX + `tower logs` (independently shippable, immediately useful)
- Restructure `console.rs` into a `console/` module (§5): build the full-duplex
  `Uart` (PA9 TX + PA10 RX) and `split()` — the writer task owns TX, the RX half is
  parked for Phase 3 (so Phase 3 is purely additive, §5.5). TX channel + single-owner
  writer task over the interrupt-driven **`BufferedUart`** (no DMA — the WS2812 strip
  owns the DMA group), holding a per-burst `WakeGuard` so the low-power STOP executor
  uses WFI and the USART stays clocked while the TXE interrupt drains the burst (§5.1);
  `log` backend enqueues `Log` frames non-blocking (drop-newest + `Dropped` marker);
  `_print` enqueues `Print`. Boot logs buffer in the channel; the panic handler silences
  the buffered ISR and emits one framed record via the PAC (leading `0x00` to flush any
  in-flight partial). Over-long log/print lines are clipped (never empty).
  (`tower` gains direct deps on `heapless` + `tower-protocol`.)
- `tower-cli`: serial reader on the shared `FrameDecoder` (COBS resync + per-frame
  version + CRC check), uses `Hello` for the firmware string when one arrives, `tower
  logs` (local time + level color + module; `--no-colors`) and `tower monitor --hex`.
- **Test (1 board):** flash an example that logs at several levels + `println!`;
  `tower logs` renders all of it with correct levels/uptime/module; a forced log
  storm shows a `Dropped` marker, **never hangs** the device; pull/replug works
  (logs resume, host reconnects). Panic prints a framed `*** panic ***` that
  `tower` shows.
- **Exit:** logs + print render end-to-end, overflow degrades gracefully, panic
  path framed, reconnect clean. (RX is wired + split but not yet read — no reader.)

### Phase 2 — Events + `tower events`
- **Payload encoding (decided):** events are **self-describing** — `Event{ name,
  fields: Vec<(&str,&str), 8> }` (key=value pairs) — so `tower events` renders any
  app's event without a shared schema (postcard isn't self-describing, so an opaque
  app blob would be undecodable). The SDK's own events use typed builders that fill
  these fields. Emit is an **async API** (`send().await`, never dropped, §4); a
  sync/IRQ caller would have to `try_send`/drop like logs.
- Target emitter + host `tower events` (prints `name` + fields, for programmatic use).
- **Test:** an example emits events on button/sensor activity; `tower events` prints
  them; they interleave correctly with logs on the shared wire (distinct `MsgType`,
  no corruption); a multi-field event round-trips.
- **Exit:** events stream reliably alongside logs and render without per-app schema.

### Phase 3 — RX + shell execution + `tower shell`
- **Built:** `shell::serve`/`serve_ext` spawns a task that **async-reads the console's
  `BufferedUartRx`** (interrupt-driven; §5.5), feeds `FrameDecoder`, and dispatches
  `ShellCommand` **inline** against the walkable tree → runs the command's handler →
  `ShellResponse` (chunked, `chunk`/`last`) with result codes. Built-ins:
  `/system settings print|set <name>=<value>|get <name>` (table-derived; `identity` is
  a `Str` setting at KV `0x5500`), `/system/resource print`, `/export`, `/system reboot`
  (flush then `SCB::sys_reset`), unknown → result 1.
- **Test:** `tower shell` (interactive) or `tower exec "<line>"` (one-shot, for
  scripts/CI) runs `/system settings set identity=tower-01`, `… print` (persists across
  reset), `/system/resource print` (multi-chunk, reassembled on the host); bad/unknown
  settings return non-zero result codes; commands work while logs stream. HW-verified on
  a dongle for `Str`/`U32`/`Bool` settings plus an app command + app settings.
- **Exit:** interactive shell with declarative, persistent settings; app-extensible tree
  (`serve_ext`); chunked responses; result codes; coexists with logs/events.

### Phase 4 — TAB completion (target-authoritative)
- **Engine (target):** the *same* tokenizer + tree-walk `dispatch` uses (no forked
  completer — that's the divergence bug) walks **to the cursor** and returns a parse
  state: deepest resolved node, the in-progress partial token + its `token_start`, and
  the position kind — **path segment** (children by prefix), **empty segment** (all
  children), **arg name** (unsupplied `name=`), or **arg value** (enum/bool variants;
  nothing for free-form `Str`/`U32`). Pure: static tree only, no KV, no I/O. The reader
  forwards `ShellComplete` to the engine task alongside `ShellCommand` (one `EngineReq`
  enum), so it still never blocks (§5.5).
- **Protocol:** `ShellComplete{ req_id, line, cursor }` → `ShellCompletions{ req_id,
  token_start, common_prefix, candidates: Vec<Candidate, 16> }`, `Candidate{ text, kind }`.
  `token_start` lets the host replace `line[token_start..cursor]` without re-tokenizing;
  per-candidate `kind` (Menu/Command/Arg/Value) drives coloring and the auto-insert
  separator (`/`/space/`=`); >16 candidates set a `more` flag. Candidates are
  `&'static str` from the tree — or, for `settings set`/`get`, the setting names from
  the table (zero-copy either way).
- **Host:** `tower shell` (`rustyline`/`reedline` `Completer`) and the TUI command pane
  fire the round-trip and wait (~150 ms) for the matching `req_id`; single candidate →
  insert + separator, multiple → insert `common_prefix` + show the list. Stale `req_id`s
  (fast typing) are discarded.
- **Test:** `/sys`→`/system`; `/system `→all children; `/system r`→`reboot`(cmd) +
  `resource`(menu), prefix `re`, kind-colored; `/system identity set ` / `…set n`→
  `name=`; an enum setting→value completion; empty line/`/`→root; no match→nothing;
  prompt while logs stream; stale `req_id`s ignored.
- **Exit:** RouterOS-like completion across all four positions, single source of truth
  on the target; the host never tokenizes.
- **Built:** the engine runs **inline** in the polled RX task (no separate engine task
  / `EngineReq`); host TAB via `rustyline`, plus a `tower complete "<line>"` command for
  testing. Verified on hardware across path / empty-segment / arg positions (no enum
  settings exist yet, so value completion is wired but unexercised).

### Phase 5 — `tower console` TUI
Almost entirely `tower-cli`, on top of the Phase 1 reader + Phase 3/4 round-trips.
- **Layout (ratatui):** outer `[Length(1) header | Min(0) body | Length(1) footer]`;
  body `[Percentage(50) left | Percentage(50) right]` (ratio configurable); left
  `[Percentage(25) Events | Length(3) Command | Min(0) Responses]`, right = Logs full
  height. Focused pane gets a highlighted border; **zoom** replaces `body` with the
  focused pane, **borderless**. Header/footer gray-bg/black-fg.
- **Event loop:** `tokio::select!` over crossterm `EventStream`, the log/event/response/
  completion channels, and a tick. **Coalesce redraws** — dirty-flag on data, draw at
  most every ~16–33 ms (immediately on keypress); a per-frame redraw under a log storm
  flickers and pins a CPU.
- **State:** bounded ring buffers (logs/events/responses, ~5000 each, oldest evicted) +
  a `LineEditor` (text/cursor/history/Ctrl-R) + focus/zoom/paused/scroll/conn/pending.
- **Keys:** global `Shift-Tab` focus cycle, `F3` zoom, `F5` pause, `F8` clear, `F10`
  quit (active toggles yellow). Command focus: edit, `Enter` send, `↑/↓` history,
  `Ctrl-R` search, `TAB` completion (Phase 4). Scrollable panes: `PageUp/Down` + `↑/↓`
  (scrolling up suspends follow-bottom; the end re-enables it). `F8` clears only the
  focused Events/Responses/Logs pane.
- **Behaviors:** Responses **bottom-anchored** (newest against the input); **Pause**
  freezes the viewport while buffers keep filling (no drop, §7); completion overlay
  above the input with kind-colored candidates; history persists to a file (XDG),
  `Ctrl-R` incremental reverse-search.
- **Lifecycle:** raw mode + alt screen, and a **panic hook / `Drop` guard that restores
  the terminal** (a crash must not wreck the user's shell). Reconnect flips the header to
  "reconnecting…" and refreshes version/path on re-`Hello`, buffers preserved; resize
  re-lays out; below a min size show a "terminal too small" guard. Footer clock on the
  1 Hz tick; level colors (Error red…Trace dim); `--no-color` honored.
- **Test:** logs+events stream while a command runs (responses bottom-anchored);
  Shift-Tab/F3/F5/F8/PageUp behave (pause keeps buffering, F8 clears only the focused
  pane); history `↑/↓` + `Ctrl-R` survive restart; TAB overlay inserts; resize + min-size
  guard work; unplug/replug reconnects (version refreshed, buffers kept); a panic/kill
  restores the terminal.
- **Exit:** the console in `CONSOLE.md` works end-to-end on hardware — all four elements,
  all five function keys, paging, history/search, completion, reconnect, clean teardown.

## 9. Decisions

**Locked (2026-06-27):**
1. **PA10 wiring:** full-duplex confirmed ⇒ shell + events in scope, no HW change.
2. **Shell model:** target-authoritative (firmware owns parsing, tree, execution,
   completion) — adds the `ShellComplete`/`ShellCompletions` round-trip (§4, §6).
3. **Text coexistence:** `tower` replaces `jolt monitor`; UART always framed;
   `print!` → `Print` frame; boot logs buffer in the channel, panic emits via the PAC (§5.2/5.4).
4. **Schema sharing:** dedicated `tower-protocol` crate; firmware + `tower-cli`
   depend on it (repo becomes a workspace; `tower-cli` git/path-deps it).

**Defaults (override if needed):**
5. Shell responses are **chunked-but-logically-one** (so `/export` never truncates).
6. Overflow = **per-producer**: `log`/`print` `try_send` (drop-newest) + `Dropped`
   marker; async responders/events `send().await` (never dropped).
7. **Baud 115200** (HSI16 accuracy; higher needs HSI48+CRS).
8. KV settings range **`0x5500+`** (clear of FOTA's `0x5400+`).
9. CRC = **one shared `tower-protocol::crc` util** (CRC‑32/IEEE); both the frame
   and `storage.rs` use it (storage delegates, behaviour unchanged).

## 10. Risks

- **Panic output (highest for Phase 1).** The executor is dead at panic, so the
  handler must emit via the PAC (§5.2) — if that's wrong, fatal output is lost at
  the worst possible moment. Build and test it first. (Boot-time output is lower
  risk: it just buffers in the channel until the writer runs.)
- **DMA/interrupt UART (RESOLVED → `BufferedUart`).** DMA was infeasible (the strip owns
  `DMA1_CHANNEL2_3`). `BufferedUart` (interrupt async TX+RX) first appeared to fail — TX
  dead — but the root cause was the **low-power executor entering STOP while the writer
  awaited the TXE interrupt**, gating the USART clock. Fixed by a **per-burst `WakeGuard`**
  (§5.1); RX is clean async read. Residual: a frame logged while *unplugged* may truncate
  if STOP is hit before the ring drains — harmless (no host listening).
- **PA12 `Pull::Down` vs a high-impedance VBUS divider** — the internal ~40 kΩ
  pull-down in parallel could drag a legitimate "plugged" level below V_IH. 30-second
  check against the divider values before committing the one-liner.
- **RAM** — the §3 budget (~3.0 KB) is an estimate; confirm against a real build,
  tune channel depth / `MAX_FRAME` if tight.
- **EEPROM write latency** — a `Kv` write busy-waits a few ms in the shell-engine
  task (cooperative). Fine for the non-timing-critical shell, but confirm it doesn't
  disturb the radio when both run; chunk/yield if it ever does.
- **Debuggability without `tower`** — a plain terminal now shows binary; mitigate
  with `tower monitor --hex/--raw`.

## 11. Pointers / references

- Console to restructure: `src/console.rs` (`_print` `:121`, macros `:133/:140`,
  panic `:153`); wiring `src/board.rs:168` (`init_console`), VBUS `board.rs:116`.
- Power gating: `src/power.rs` (`vbus_task`, `WakeGuard`).
- Codec idiom to mirror: `src/radio/frame.rs` (`ver_type`, typed enum, `encode/parse`).
- Reuse: CRC‑32 `src/storage.rs:320`; `Kv` API `storage.rs`; postcard usage there.
- Crates already present: `postcard 1.1.3`, `cobs`, `heapless` (in `Cargo.lock`).
- Spec: `CONSOLE.md`. Host repo: `/Users/pavel/hardwario/github/tower-cli`.
- External: embassy-stm32 0.6 USART (`BufferedUart` — **parked**; RX polled via PAC),
  `ratatui`, `clap`, `rustyline`, `jolt` (kitchen ref: https://github.com/hardwario/jolt).
