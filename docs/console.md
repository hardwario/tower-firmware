# TOWER Console — user guide

A framed, bidirectional **host ↔ target console** for the TOWER Core Module
(STM32L083CZ). One serial link carries everything: structured **logs**, raw
`print!` output, self-describing **events**, and a RouterOS-style **shell** with
target-authoritative TAB completion and a declarative, EEPROM-backed **settings
framework**. The host side is the **`tower`** CLI/TUI.

> **Status: complete and hardware-verified** on the Radio Dongle (STM32L08x). The
> wire codec (COBS + CRC-32 + postcard), the interrupt-driven `BufferedUart`
> transport with a low-power-correct `WakeGuard`, the synchronous panic path, log /
> print / event streaming with overflow accounting, chunked shell responses, the
> settings framework (6 kinds, range/enum validation, value completion), the
> app-extensible deep-merge command tree, and the `tower` CLI + TUI are all
> implemented and tested on real hardware. The codec has 20 host tests including
> 9000 fuzz iterations (run in the tower-protocol repo).

This is the standalone reference for using and maintaining the console subsystem —
architecture, wire protocol, the firmware + host APIs, and a worked example per feature.

---

## Two pieces

| Piece | Where | What it is |
|---|---|---|
| **Firmware SDK** | this repo: `src/console.rs`, `src/shell.rs` | the on-MCU console: logging backend, event/shell APIs, the framed UART transport |
| **`tower` host CLI** | `github.com/hardwario/tower-cli` (binary `tower`) | decodes the framed link on your machine: logs / events / shell / TUI |
| **`tower-protocol`** | `github.com/hardwario/tower-protocol` (shared, `no_std`) | the single source of truth for the wire format, used by *both* ends |

Because `tower-protocol` is shared, the wire format cannot drift between firmware
and host. Both ends pin it to the same git tag (currently `v1.3.0`).

## Hardware

The console owns **USART1** on the Core Module:

| Signal | Pin | Notes |
|---|---|---|
| TX | PA9 | target → host |
| RX | PA10 | host → target (full-duplex; wired through the dongle's USB bridge) |
| Baud | — | 115200 **8N1** |

It is **interrupt-driven (`BufferedUart`), not DMA** — the WS2812 LED strip owns the
`DMA1_CHANNEL2_3` IRQ group, so the console can't use DMA. See *Low power* below for
why this matters and how it's handled.

The link is **always framed** (binary COBS frames). A plain serial terminal (`screen`,
`minicom`, or any raw monitor) shows gibberish — use the `tower` CLI (`tower logs`),
which decodes the frames.

## Quick start

```sh
# Firmware: flash any example (it auto-starts the console).
TOWER_DEVICE=/dev/cu.usbserial-140 just flash example console_demo

# Host: build the CLI once, then stream the device.
cd ../tower-cli && cargo build --release      # produces `tower`
tower logs                                    # auto-detects a single USB serial port
tower -d /dev/cu.usbserial-140 logs           # or name the device explicitly
```

Every app gets the console for free: `Board::take` (via the `app!` macro) starts it
and emits a boot `Hello` + banner naming the app. You don't wire anything up.

```rust
#![no_std]
#![no_main]
use tower::{app, board::Board};
use log::info;

async fn run(_b: Board) {
    info!("hello from my app");   // → framed Log on USART1, rendered by `tower logs`
}
app!(run);
```

---

## Architecture

```
 producers                         one writer task                 host
 ─────────                         ───────────────                 ────
 log::{error..trace}!  ┐
 print! / println!     ├─ try_send ─► TX_CHANNEL ─► seq+encode ─► BufferedUartTx ═══► tower
 console::event().await┘  (drop-      (depth 8)    (COBS+CRC+        (PA9, 115200)
 shell responses          newest)                   postcard)
                                       ▲
 host commands ════► BufferedUartRx ──► FrameDecoder ─► shell ─► responses ─┘
 (PA10)                (interrupt)
```

- **Producers** build an owned message and `try_send` it into a bounded channel
  (depth 8). Non-blocking and safe from any context (including the `log` backend).
  If the queue is full the newest is dropped and a counter bumped.
- **One writer loop** (inside `console::manager`) owns the UART TX. It assigns the
  per-frame **`seq`** at send time (so a gap on the wire means *real* loss, never a queue
  drop), encodes the frame, and writes it. Before the next real frame it emits a
  `Dropped{count}` marker if anything was dropped.
- **The codec** (`tower-protocol`) frames every message identically — see *Wire
  protocol*.
- **RX** is read by the same manager and **routed by frame type** — shell frames to the
  shell's channel. The manager owns the whole UART so it can
  be torn down/rebuilt on USB unplug/plug — see *Low power & the dynamic console*.

### Low power & the dynamic (USB-gated) console

On the STM32L0 an **enabled USART holds embassy's STOP refcount**, so a permanently-on
console would keep the low-power executor out of STOP *forever* — an unplugged / battery
node would burn ~3.5 mA (WFI at 16 MHz) instead of idling at µA. The console is therefore
**dynamic**: `console::manager` (spawned by `Board::take`) owns USART1 + PA9/PA10 + the
`VBUS_SENSE` (PA12) EXTI, and gates the whole UART on USB presence:

- **USB present** → builds the `BufferedUart`, runs the writer + the RX frame-router, and
  re-emits a `Hello`. While USB is present the enabled USART keeps the MCU in WFI (not
  STOP) — which is what we want: the console/shell stay responsive and the node is
  powered/plugged anyway.
- **USB absent** → **drops** the UART, which disables USART1 and releases the STOP
  refcount, so the executor reaches STOP and the node idles at µA (~32 µA @3 V measured,
  vs 3527 µA with a permanently-on console). It then waits for USB on the **PA12 EXTI
  edge plus a ~500 ms RTC poll**: EXTI works in STOP and brings the console up the instant
  VBUS rises — **no reset** — while the poll is a fallback for a *missed* edge. PA12 is
  driven by the FT231X's **CBUS3** output (a push-pull ~3.3 V logic level, *not* a 5 V
  divider), which asserts only tens of ms after power-up, i.e. after the executor may have
  already armed the edge wait — so relying on the edge alone can hang; the poll closes that
  gap. The periodic wake costs sub-µA (STOP floor unchanged).

RX is owned by the manager and **routed by frame type**: `ShellCommand`/`ShellComplete`
go to the shell (`shell::dispatch_frame`; the shell registers its command tree + settings
via `shell::serve`/`serve_ext`, which just store them — no task). So the shell keeps
working across teardown/rebuild without owning the raw RX half.

**The `WakeGuard`:** even while the console is up, STOP would gate the USART clock and the
writer's awaited TXE interrupt would never fire. The writer holds a `WakeGuard(Stop1)`
**per transmit burst** (STOP → plain WFI so the USART stays clocked and the interrupt
fires), dropped between bursts.

### Dynamic console verification

The dynamic behavior (console live on plug, torn down on unplug, no reset) can only be
verified with real USB. Recommended bench procedure:

**Setup.** Power the board from a **separate supply** (battery / power-profiler), *not*
from USB, so plugging/unplugging USB only toggles `VBUS_SENSE` — it doesn't cut power.
Have the `tower` CLI on PATH and flash a console-exercising firmware (`console_full` is
ideal): `just flash example console_full`. (Flashing is unaffected — the STM32 bootloader
runs before the app's console.)

| Test | Steps | Pass |
|---|---|---|
| **A — console up on USB** | USB plugged, `tower logs` | `Hello` then streaming logs; `tower console` shell responds (`/system/resource print`, TAB) |
| **B — live plug/unplug** | `tower logs` running + current meter on VDD; unplug USB, then re-plug | On unplug: logs stop, **current drops to µA** (~50 ms). On re-plug: reconnects, fresh `Hello`, logs resume |
| **B (no-reset check)** | across the unplug/replug in B, watch the log **uptime timestamps** | Uptime is **continuous / increasing** (device kept running+sleeping). If it resets to 0 → the device rebooted, i.e. *not* the dynamic path |
| **C — headless boot → plug** | power board with **no USB** (idles at µA), then plug USB | PA12 EXTI (or the ~500 ms poll fallback) wakes the MCU; console appears in `tower logs` within ~½ s |

**If a test fails — where to look:**

- **Console never appears on plug** → PA12 isn't reading logic-high when USB is plugged.
  It's driven by the FT231X CBUS3 push-pull output (see the caveat below), which the
  ~500 ms poll already tolerates being late — so if it *never* appears, scope PA12 on
  plug: it must reach the STM32's V_IH, or the manager never sees "USB present." (Hardware
  issue — e.g. CBUS3 mis-configured in the FTDI EEPROM — not the firmware logic.)
- **Logs appear but garbled** → framing / the reused static `TX_BUF`/`RX_BUF` across rebuilds.
- **Shell silent, logs fine** → the RX router (`dispatch_frame`) or `SHELL_PARAMS` not set
  (a `no_shell` app has no shell by design).
- **Current doesn't drop on unplug** → the UART isn't being dropped; check the manager's
  unplug branch (`select3`) fires (VBUS debounce is 50 ms).
- **Host reports a `seq` gap on re-plug** → should not occur: the writer's `seq` resets
  per USB session and `tower` resets its per-link `seq` tracking when it decodes the fresh
  `Hello`. (Older CLI builds printed a benign one-line gap warning here and kept going.)

### The panic path

The executor is dead in a panic, so the channel/writer can't run. The panic handler
silences the buffered ISR and **blocking-writes one framed `Log` (level Error)
straight to the USART registers via the PAC**, leading with a `0x00` so any byte still
in the shift register can't prefix and corrupt the frame. `tower logs` shows the panic
message + location like any other error.

It then **resets** (it does not halt). Before resetting it writes the crash text into a
reset-surviving `.uninit` RAM breadcrumb (`crashlog`, zero EEPROM wear); the next boot
re-reports it as a `crash`-module ERROR frame and via `/system/crash print` — so a
battery node that faults **with USB unplugged** still surfaces its crash, after it has
already recovered. HardFaults follow the same path (with the faulting PC/LR). Crash
loops are bounded by the bootguard's EEPROM-write backoff, and the run length shows in
`/system/eeprom print`.

---

## Logging

Use the standard [`log`](https://docs.rs/log) macros. The host adds the timestamp,
color, and columns — the device only sends level + uptime + module + message.

```rust
use log::{error, warn, info, debug, trace};
error!("sensor {} fault: {:?}", id, e);
info!("interval set to {} s", interval);
```

- **Levels:** Error, Warn, Info, Debug, Trace. Default max level is **Trace**; lower
  it at runtime with `log::set_max_level(...)`.
- **`module`** is the last `::` segment of the log target (e.g. `power`).
- **Over-long lines are truncated, never dropped** — a message past `MAX_MSG` (192
  bytes) is clipped at a char boundary (a plain `heapless` write would reject it
  wholesale and log nothing).

### Raw text

`print!` / `println!` (from the `tower` crate) send a `Print` message — verbatim text
with no level/timestamp, rendered inline by `tower logs`. `println!` appends `\r\n`.

```rust
use tower::println;
println!("raw line {}", n);
```

### Overflow

If producers outrun the writer, the newest messages are dropped and the host shows a
single marker before the next frame:

```
⚠ 22 log frame(s) dropped (device queue full)
```

No sequence gap is reported, because dropped messages never consumed a `seq`.

---

## Events

Structured, **self-describing** key=value records — the host renders any app's events
with no shared per-app schema. `event()` is `async`: while USB is present it applies
**backpressure** (awaits a free queue slot, so a burst is never dropped); while USB is
unplugged there is no writer to drain the queue, so it falls back to **drop-newest with a
count** (surfaced later as the `Dropped` marker) rather than parking the low-power executor
forever on a full queue. Call it from async code.

```rust
use tower::console;
console::event("measurement", &[("temp_c", "2351"), ("rh", "41")]).await;
```

Caps (clipped if exceeded): name ≤ 24 bytes, ≤ **6** fields, each key ≤ 12 / value ≤
20 bytes — sized so a worst-case event always fits one frame. View with `tower events`:

```
12:01:07.245 EVENT measurement  temp_c=2351 rh=41
```

---

## The shell

A RouterOS-style command shell over the same link, with **target-authoritative TAB
completion**: the firmware owns parsing, the command tree, execution **and**
completion, so the host can never suggest something the device won't accept. Opt in by
serving it with the board's EEPROM storage:

```rust
use tower::{app, board::Board, shell};
async fn run(b: Board) {
    shell::serve(b.spawner, b.kv);   // base tree + settings
    // … your app; logs/events keep flowing on the same link …
}
app!(run);
```

Drive it from the host interactively or one-shot:

```sh
tower shell                              # interactive REPL; TAB completes
tower exec "/system/resource print"      # run one command, print result, exit (scripts/CI)
tower complete "/system settings set "   # ask the target what completes here
```

### Built-in commands

| Command | Does |
|---|---|
| `/system reboot` | flush the reply, then `SCB::sys_reset()` |
| `/system/resource print` | firmware/protocol version, uptime, CPU, clock, memory |
| `/system settings print` | list every setting and its value |
| `/system settings set <name>=<value>` | validate by kind + persist |
| `/system settings get <name>` | show value + constraints + default |
| `/export` | dump all settings as `settings set` lines (reproducible config) |

Unknown commands return result code 1. Tokens split on `/`, space, and tab, so
`/system settings set` and `/system/settings/set` are equivalent.

Responses longer than one frame are **chunked** (192-byte `chunk`/`last` frames) and
reassembled by the host into one response — `/system/resource print` is a 7-line
example.

---

## Settings framework

Settings are **declarative**: an app provides a `&'static [Setting]` table and the
shell derives `print` / `set` / `get` / `export` and completion from it — no
per-setting code. Each `Setting.key` is a **`u8` local** within the shell's EEPROM
namespace (`NS_SHELL`) — the shell prefixes it, so app keys can't collide with other
subsystems. **`0x00..=0x0F` is reserved for the SDK base table** (`identity` = `0x00`,
`address` = `0x01`, the rest headroom for base growth); app settings start at `0x10`.
The partition is load-bearing: a key collision silently aliases two settings' storage.

```rust
use tower::shell::{Setting, Kind};
static SETTINGS: &[Setting] = &[
    Setting { key: 0x10, name: "interval", kind: Kind::Uint { min: 1, max: 3600 }, default: "30" },
    Setting { key: 0x11, name: "verbose",  kind: Kind::Bool,                       default: "false" },
    Setting { key: 0x12, name: "mode",     kind: Kind::Enum(&["p2p","star","mesh"]),default: "star" },
    Setting { key: 0x13, name: "tx_power", kind: Kind::Int { min: -30, max: 20 },   default: "14" },
];
```

### Kinds

| `Kind` | Accepts | Stored as | Use for |
|---|---|---|---|
| `Str { max }` | 1..=`max` bytes of UTF-8 (`max` ≤ 64) | raw bytes | names, SSIDs, tokens |
| `Uint { min, max }` | decimal `u32` in range | 4 LE bytes | intervals, ports, counts, thresholds |
| `Int { min, max }` | decimal `i32` in range | 4 LE bytes | offsets, tx-power dBm, calibration |
| `Bool` | `true`/`false`, `on`/`off`, `1`/`0` | 1 byte | flags |
| `Enum(&[&str])` | one of the listed values | raw bytes | modes, regions, roles |
| `Addr` | 32-bit hex (`0x1a2b3c4d`), or `auto` / `random` | 4 LE bytes | radio addresses (`0` = auto sentinel) |

- **Ranges/choices are enforced on `set`**; an invalid value returns result 2 and
  prints the constraint:
  ```
  > /system settings set interval=99999
  invalid value for interval (uint 1..=3600)
  ```
- **`get` shows the derived metadata:**
  ```
  > /system settings get tx_power
  tx_power = -10  [int -30..=20, default 14]
  ```
- **Completion is value-aware:** completing after `=` offers the `Enum` choices or
  `Bool` true/false:
  ```
  > /system settings set mode=⇥     →  p2p  star  mesh
  ```
- Unset / unreadable / out-of-range stored values fall back to the `default`.

`identity` is just an SDK `Str` setting (key `0x00`, ≤32 chars) — nothing special.
`address` (key `0x01`, `Kind::Addr`, default `auto`) is the SDK's other base setting: the
unit's **32-bit radio address** (the frame-header src/dest, i.e. the net `my_id`). `auto`
resolves to the chip-UID-derived address; `random` mints a fresh non-zero one from the
STM32L0 hardware TRNG (`board::rand_u32`). `get` shows the *effective* address, so it never
lies about what the radio uses. It is boot-applied — reboot to re-address a live node.

---

## Extending the shell (apps)

`serve_ext` lets an app add its **own commands and settings**. App commands
**deep-merge** with the SDK tree at *every* level, so you can drop a command into an
existing menu *or* grow your own nested subtree. App settings join the same
`/system settings` table.

```rust
use core::fmt::Write;
use tower::shell::{self, Args, Ctx, Entry, Kind, Outcome, Setting};

// A handler writes its response via `write!(ctx, …)` and returns an Outcome.
fn cmd_status(ctx: &mut Ctx, _args: &[&str]) -> Outcome {
    let _ = write!(ctx, "radio: idle, last RSSI -71 dBm");
    Outcome::ok()              // or Outcome::code(n) for a non-zero result
}

static APP_COMMANDS: &[Entry] = &[
    Entry::cmd("uptime", Args::None, cmd_status),               // → /uptime  (top level)
    Entry::menu("system", &[Entry::cmd("hello", Args::None, cmd_status)]),  // → /system hello  (into SDK menu)
    Entry::menu("radio", &[                                     // → /radio …  (new subtree)
        Entry::cmd("status", Args::None, cmd_status),
        Entry::menu("test", &[Entry::cmd("ping", Args::None, cmd_status)]),  // → /radio test ping
    ]),
];

static APP_SETTINGS: &[Setting] = &[
    Setting { key: 0x10, name: "interval", kind: Kind::Uint { min: 1, max: 3600 }, default: "30" },
];

async fn run(b: Board) {
    shell::serve_ext(b.spawner, b.kv, APP_COMMANDS, APP_SETTINGS);
}
```

The handler context (`Ctx`) exposes:
- `write!(ctx, …)` — `Ctx` implements `core::fmt::Write` (the response text);
- `ctx.kv` — the shell-namespaced (`NS_SHELL`) EEPROM handle; settings are keyed by `u8`
  within it, so use keys other than the SDK base's `0x00` (`identity`);
- `ctx.settings` — the merged settings table (`iter()` / `find(name)`).

Merge rules: a menu shadows a same-named command (menus stay descendable); on a
command-name collision the SDK command wins; completion dedups names. Apps don't need
any host changes — `tower` is target-authoritative.

---

## The `tower` host CLI

```
tower [-d <device>] <command>
```

The device auto-detects when exactly one USB serial device is present; otherwise pass
`-d`/`--device`. Subcommands:

| Command | What |
|---|---|
| `devices` | list all serial ports (one bare port name per line) |
| `logs [--no-colors] [--send <text>]` | stream logs + `print!` + the `Dropped` marker; `--send` pokes the device's RX once on connect |
| `events [--no-colors]` | stream structured events |
| `shell` | interactive REPL; TAB completes (via `rustyline`); `exit`/`quit` to leave |
| `exec "<line>"` | run one command, print the (reassembled) response, exit non-zero on a device error/timeout |
| `complete "<line>"` | print what the target would complete at the cursor (handy for testing/scripts) |
| `console` | the full-screen TUI (below) |
| `monitor [--hex]` | transport debugging: dump decoded frames, or every raw byte with `--hex` |

`logs`/`events` auto-reconnect on unplug. Frame-level integrity is reported: a corrupt
frame and a `seq` gap each print a one-line warning.

### The TUI (`tower console`)

A three-pane [ratatui](https://ratatui.rs) terminal app — Device Events, the SSH-style
Interactive Shell (scrollback + a `> ` prompt with in-pane TAB hints), Device Logs — all
on one serial drain (35/65 split):

```
 HARDWARIO TOWER Console v0.1.0 — /dev/cu.usbserial-140 ●
┌Device Events──────────────┐┌Device Logs────────────────────────────────┐
│12:01 measurement temp=23.5 ││12:01 [  64.030] INFO  app: heartbeat 32    │
│                            ││12:01 [  66.031] WARN  app: link flaky      │
┌Interactive Shell──────────┐│                                            │
│> /system settings print    ││                                            │
│identity = node-7           ││                                            │
│> /system/eeprom print      ││                                            │
│flips: 3 / 100000 (0.0%)    ││                                            │
│> _                         ││                                            │
└────────────────────────────┘└────────────────────────────────────────────┘
 <Shift-Tab> Focus  <F3> Zoom  <F5> Pause  <F8> Clear  <Shift-F8> Clear All …
```

Only the `clock [uptime] LEVEL` prefix of a log line is severity-tinted; command syntax is
highlighted (paths / commands / `key=value`), and `F5` pauses the *view* only — frames keep
being captured while the viewport holds still.

| Key | Action |
|---|---|
| type + Enter | send a shell command (history with ↑/↓) |
| **Tab** | target-authoritative completion (names + enum/bool values) |
| **Shift-Tab** | move focus between panes |
| **F3** | zoom the focused pane full-screen |
| **F5** | pause the streaming panes (shell stays live) |
| **F8** | clear the focused pane |
| **PageUp/Down**, ↑/↓ | scroll a focused stream pane |
| **F10** | quit (restores the terminal; a panic restores it too) |

---

## Wire protocol (`tower-protocol`)

```
wire:   COBS( inner )  0x00
inner:  ver_type(1) │ seq(2, LE) │ payload(postcard) │ crc32(4, LE)
        ver_type = (PROTOCOL_VERSION << 5) | (msg_type & 0x1F)
        crc32    = CRC-32/IEEE over [ver_type, seq, payload…]
```

- **COBS** framing with a `0x00` delimiter (the only zero on the wire) — the host
  resynchronizes on the next delimiter after any garbage.
- **CRC-32/IEEE** over the header + payload catches corruption (the same primitive
  the EEPROM KV store uses).
- **postcard** for the payload — compact and `no_std`, but **not self-describing**, so
  both ends must share the exact struct/enum definitions. That's why `tower-protocol`
  is one crate, version-pinned.
- **`seq`** (writer-assigned) lets the host detect real wire loss.

Message types (the low 5 bits of `ver_type`; target→host are 0..15, host→target 16+):

| Type | # | Dir | Payload |
|---|---|---|---|
| `Hello` | 0 | T→H | protocol + firmware version, firmware_name, per-boot session_id (boot announce) |
| `Log` | 1 | T→H | level, uptime_us, module, message |
| `Print` | 2 | T→H | raw text |
| `Event` | 3 | T→H | name + (key, value) pairs (wire allows 8; the SDK emits ≤6) |
| `ShellResponse` | 4 | T→H | cmd_id, result, chunk, last, text |
| `ShellCompletions` | 5 | T→H | req_id, token_start, common_prefix, candidates, more |
| `Dropped` | 6 | T→H | count of dropped frames |
| `MgmtResponse` | 7 | T→H | req_id, result, chunk, last, data (chunked mgmt reply; wire v3 gateway link) |
| `Uplink` | 8 | T→H | src, counter, rssi_dbm, lqi, data (gateway forwards a node's radio payload verbatim) |
| `RadioStat` | 9 | T→H | channel RSSI sample, or a TX-delivery outcome |
| `ShellCommand` | 16 | H→T | cmd_id, line |
| `ShellComplete` | 17 | H→T | req_id, line, cursor |
| `MgmtRequest` | 18 | H→T | req_id, op (a `mgmt::MgmtOp`; wire v3 gateway link) |

A receiver feeds bytes to a `FrameDecoder` until it yields a deframed buffer, then
`decode_frame` checks version + type + CRC and returns `(MsgType, seq, payload)`.

### Limits & constants

| Constant | Value | Meaning |
|---|---|---|
| `PROTOCOL_VERSION` | 3 | top 3 bits of `ver_type` (caps at 7; bumped to 3 for the wire-v3 gateway link) |
| `MAX_FRAME` | 256 | inner frame, pre-COBS (payload budget ≈ 249) |
| `MAX_WIRE` | 272 | COBS-expanded frame + delimiter |
| `MAX_MSG` | 192 | log / print text |
| `SHELL_CHUNK` | 192 | shell-response text per frame (chunked) |
| `MAX_SETTING` | 64 | largest setting value |
| TX queue depth | 8 | producer → writer channel |

### Shell result codes

| Code | Name | Meaning |
|---|---|---|
| 0 | `R_OK` | success |
| 1 | `R_NOT_FOUND` | no such command / setting |
| 2 | `R_BAD_ARG` | bad / out-of-range value |
| 3 | `R_STORAGE` | EEPROM write failed |
| 4 | `R_TRUNCATED` | response body overflowed the cap and was truncated |

---

## Firmware API summary

`tower::console`:
- `event(name, &[(k, v)]).await` — emit a structured event
- `print!` / `println!` — raw text (re-exported macros)
- `install_logger(max_level)`, `manager(usart1, tx_pin, rx_pin, vbus)`, `boot_banner(name)`
  — wiring for the dynamic (USB-gated) console, handled for you by `Board::take` / `app!`

`tower::shell`:
- `serve(spawner, kv)` / `serve_ext(spawner, kv, app_commands, app_settings)`
- types: `Entry` (`::cmd` / `::menu`), `Command`, `Args` (`None` / `Names` / `Settings`),
  `Setting`, `Kind`, `SettingsTable`, `Ctx`, `Outcome` (`::ok` / `::code`), `Handler`
- constants: `MAX_SETTING`, `R_OK`/`R_NOT_FOUND`/`R_BAD_ARG`/`R_STORAGE`/`R_TRUNCATED`

`tower_protocol`: `encode_frame`, `decode_frame`, `FrameDecoder`, `MsgType`, the payload
structs, `PROTOCOL_VERSION`, `MAX_FRAME`, `MAX_WIRE`, `crc::{crc32_update, crc32_ieee}`.

## Examples

| Example | Shows |
|---|---|
| `console_demo` | every log level + `println!` + truncation + the `Dropped` marker |
| `console_panic` | the framed panic path (`tower logs` shows the panic record) |
| `events_demo` | structured events interleaved with logs |
| `shell_demo` | the shell: built-ins, settings of every kind, an app command + nested subtree, app settings |
| `console_full` | the showcase for `tower console` — logs + events + shell together |

Flash any of them, e.g. `TOWER_DEVICE=/dev/cu.usbserial-140 just flash example shell_demo`,
then drive with `tower shell` / `tower logs` / `tower console`.

## Testing

The wire codec lives in its own repo now
([`tower-protocol`](https://github.com/hardwario/tower-protocol)); its host tests run there:

```sh
cargo test   # in a tower-protocol checkout
```

These cover round-trips for every message, boundary frame sizes, the decoder's
overflow/reset/resync state machine, exhaustive type mapping, and **9000 deterministic
bit-flip fuzz iterations** proving every single-bit corruption is detected. (They run on the
host; the crate itself is `no_std` and also builds for `thumbv6m`.) The firmware-side paths
(logs, events, shell, settings, completion, panic, persistence) are verified on the dongle.

## Known limitations & caveats

- **No DMA:** the WS2812 strip owns the DMA group, so the console is interrupt-driven.
  This is why the writer holds a `WakeGuard` per burst (above).
- **`fw ?` in the header** appears only if you attach `tower console` and never see a
  `Hello` — the firmware announces it on boot and on each USB plug-in (when the dynamic
  console rebuilds), not on host connect. The TUI
  header otherwise shows the **`tower` CLI** version; the firmware version is in
  `/system/resource print`.
- **`VBUS_SENSE` (PA12):** the dynamic console gates on PA12 reading logic-high when USB
  is plugged. PA12 is driven by the FT231X's **CBUS3** pin (a push-pull ~3.3 V logic
  output configured in the FTDI EEPROM — e.g. `DRIVE1` on the Core Module, `SLEEP#` on the
  Radio Dongle) — *not* a resistor divider off USB 5 V. Two consequences: (1) CBUS3 asserts
  only tens of ms after power-up, which is why the manager also **polls VBUS every ~500 ms**
  (a missed rising edge still brings the console up within ½ s); (2) if CBUS3 is
  mis-configured in the EEPROM, PA12 won't reach the STM32's V_IH and the console won't come
  up — scope PA12 on plug to confirm. The pin uses an internal pull-down so it reads a
  defined low when unplugged (no false "present"). PA12 is also the USB DP pin, used here
  purely as a VBUS-sense GPIO (no USB device peripheral is enabled).
- **Setting values can't contain `/`, space, or tab** — those are command tokenizers.
  Enum values dodge this by being a fixed set.
- **postcard is not self-describing:** any change to a payload struct/enum is a wire
  change — bump `PROTOCOL_VERSION` and re-tag `tower-protocol`; both ends (firmware + host)
  pin the new tag in lockstep. Today's tag is `v1.3.0`.
- **A response is capped at `MAX_SETTING`/`MAX_RESP`-sized text** before chunking; very
  large dumps are clipped. Raise the caps (and `MAX_RESP`) if you need more.
