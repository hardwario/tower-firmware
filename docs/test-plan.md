# HARDWARIO TOWER Firmware SDK — Hardware Test Plan

A **reusable**, end-to-end validation suite for the TOWER firmware SDK (STM32L083CZ +
SPIRIT1). It covers every subsystem — drivers, console/shell, crypto, storage, the full
radio/network stack, spectrum-access (EU AFA / US FHSS), and signed A/B FOTA — with concrete
**pass signals** and a catalogue of **edge cases**. Run it whole for a release gate, or pick a
section for a focused regression. Companion guides: `docs/console.md`, `docs/radio.md`,
`docs/fota.md`. Host-only unit tests: `just test`.

> This document is intended to be re-run by a human or by an agent. Section 3 explains the
> capture mechanics that make the firmware's "print once at boot" examples observable; the
> harness lives in `tools/hwtest/`.

---

## 1. Scope & philosophy

- **Self-checking first.** Most non-interactive examples are *known-answer tests* (KATs) or
  invariant checkers that print a single verdict line ending in `ALL PASS ***` / `PASS ***` /
  `MATCH ***`, or an `error!` line (`FAIL` / `MISMATCH` / `VIOLATION`). The pass criterion is
  "the verdict line is present and positive." These need no human judgement.
- **Two-board RF tests** assert link-level behaviour (delivery, ordering, ACK, RSSI, hopping).
  Pass = both ends log the expected exchange with no `FAIL`/`VIOLATION`/`MISMATCH`.
- **Interactive/physical tests** (button, accelerometer dice/tilt, LED visual, LED strip)
  require a human; the plan documents the action and the expected log/visual so they can be
  signed off by eye. An agent without hands records "boot OK + instructions emitted; physical
  step deferred."
- **Regulatory citations are load-bearing.** FCC §15.247 (US FHSS) and EN 300 220 (EU duty /
  LBT+AFA) behaviours are tested by `net_duty_kat`, `fhss_kat`, `fhss_compliance`. Treat a
  failure here as a compliance regression, not a flake.

## 2. Hardware setup & board roster

| Item | Value |
|---|---|
| MCU | STM32L083CZ (Cortex-M0+), chip id `0x447` |
| Target | `thumbv6m-none-eabi` |
| Flash transport | Dongle: STM32 UART bootloader via `tower flash` (jolt engine, ~30 s/image). Core: **SWD via probe-rs** (J-Link) — doesn't need the FTDI, so the Core can be measured with USB unplugged. |
| Console | USART1 PA9/PA10, 115200 8N1, COBS+CRC32+postcard framed (`tower-protocol`) |
| Radio | SPIRIT1 / SPSGRF, SPI1 PB3/PB5/PB4, CS PA15, SDN PB7, nIRQ PA7 |
| Reset/boot | NRST/BOOT0 on FTDI **aux** lines (not DTR/RTS) → opening a port does **not** reset |

**The bench (re-confirm every session — names re-enumerate):**

| Port | Board | Instruments | Role |
|---|---|---|---|
| `/dev/cu.usbserial-2120` | **TOWER Core Module** | SEGGER **J-Link** (SWD flash via `probe-rs`) + Nordic **PPK2** (scriptable current supply/measure) | radio NODE; the **power** target (STOP-floor measurements) |
| `/dev/cu.usbserial-2140` | **TOWER Radio Dongle** | USB-powered (VBUS); flashed via `tower flash` | radio GATEWAY; the **smoke** target (one-shot KAT verdicts) |

Confirm ports with `tower devices`; the automated harness (`tools/hil`, run via `just hil` /
`just hil-power` / `just hil-full`) reads this roster from `tools/hil/hil.toml` and re-resolves it
at startup, failing fast if a board is absent. Confirm I2C population by flashing `i2cscan`: with
no sensor module attached, **`thermometer` and `accelerometer` report a sensor-absent error** —
expected on a bare Core, not a fault. A sensor module additionally shows `0x49` (TMP112) and `0x19`
(LIS2DH12).

Default radio role assignment for two-board tests:

- **NODE / sender / peer-a** → `/dev/cu.usbserial-2120` (Core Module)
- **GATEWAY / receiver / peer-b** → `/dev/cu.usbserial-2140` (Radio Dongle)

> **Power measurements need the FTDI UNPLUGGED from the Core.** USB/VBUS present keeps the SDK's
> console alive, which inhibits STOP by design — so a plugged-in board never reaches the µA floor.
> The PPK2 supplies the Core at **1.8 V** (the regulator/brown-out knee, not 3 V) after a
> power-cycle (clears the ~200 µA debug-domain residual a probe leaves); readings are never sampled
> mid-SWD-flash (the PPK2 CDC can desync and report tens of mA of garbage). The `just hil-power`
> test enforces all three, and skips with an "unplug the FTDI" message if the console still answers.

## 3. Capture methodology (read this before running)

Two facts shape every observation:

1. **No host handshake.** `Board::take` brings up the console and the app logs immediately.
   A KAT prints its verdict within milliseconds of reset, then idles in `loop { Timer::after_secs(5) }`.
2. **Flash resets into the app at the end** (unless `--no-run`). So by the time a *separate*
   `tower logs` attaches, a one-shot verdict is already gone. `tower logs` has **no** reset-on-attach.

Therefore use **two capture modes**:

### Mode A — `tower logs` (decoded, pretty) — for *continuous* output
For examples that keep printing (radio links, net, `console_demo`, `events_demo`, `net_persist`,
reference apps). Attach after flashing; you'll catch the steady-state stream.

```sh
python3 tools/hwtest/cap.py <seconds> tower -d <PORT> logs --no-colors
```

### Mode B — `jolt monitor --reset` (raw bytes) + `strings` — for *one-shot* output
`jolt monitor --reset` pulses NRST **then** reads, atomically, so it catches boot output. The
console payload's **message text is plain ASCII and survives COBS unchanged** (COBS only rewrites
zero bytes; ASCII has none), so `strings` recovers every log/event message verbatim. Binary
fields (uptime, level, seq, CRC) become noise — irrelevant for verdict checking.

```sh
python3 tools/hwtest/cap.py <seconds> jolt monitor --reset -d <PORT> > raw.bin
strings -n 3 raw.bin            # human-readable log/event text, incl. the verdict line
```

Use Mode B for: all crypto/frame/duty/fhss KATs, `edge_recovery`, `radio_id/state/regdump`,
`storage`/`net_persist` first-boot lines, `console_panic`. Use a **longer window** for tests that
sweep/iterate before printing: `fhss_sweep` (~80 channel locks), `fhss_compliance` (90 slots ×
300 ms ≈ 27 s), `edge_recovery` (10 × 120 ms timeouts).

### Throughput optimisation
The two ports are independent. **Build sequentially** (shared `target/`, single `firmware.bin`),
**flash/capture concurrently** to halve wall-clock on single-board tests. Build each image to its
own path first:

```sh
tools/hwtest/build.sh <example> /tmp/twr/bin/<example>.bin "<features>"
tower -d /dev/cu.usbserial-120 flash /tmp/twr/bin/A.bin &   # concurrent flashes OK
tower -d /dev/cu.usbserial-140 flash /tmp/twr/bin/B.bin &
```

### Interactive shell tests (no TUI needed)
`shell_demo` and any shell-serving app are fully drivable headless:

```sh
tower -d <PORT> exec "/system/resource print"
tower -d <PORT> exec "/system settings set interval=60"
tower -d <PORT> complete "/system settings set m"   # TAB-completion candidates
```

## 4. The role/feature matrix

Radio examples select node/gateway/peer behaviour at **compile time** via cargo features
(`TOWER_FEATURES=...`). Both ends must share band/channel.

> **Important (verified on HW):** every two-board example gates **only on `role-node` vs
> default** (`#[cfg(feature="role-node")]` / `#[cfg(not(...))]`). **`role-gateway` is a no-op** —
> it compiles to the *default* branch. So the recipe is always: **one board built `role-node`,
> the other built with no feature** (or `role-gateway`, equivalently). The receiver/gateway/
> sender/host/master side **is** the default branch. `net_p2p` is the exception (`role-peer-a` /
> `role-peer-b`); `net_star` node B adds `node-2` on top of `role-node`. Which physical role the
> `role-node` build plays differs per example — `net_bulk`/`net_pairing` make `role-node` the
> *active* side (requester/joiner), most others make it the sender.
>
> **Flash sequentially, not concurrently.** Flashing two FTDIs at once over one USB bus is
> unreliable (Write-Memory timeouts, port re-enumeration `-120`→`-2120`). `tools/hwtest/tb.sh`
> flashes node then gateway in series, then captures both concurrently. Re-resolve ports each
> run: `P1=$(ls /dev/cu.usbserial-* | sort | head -1)` etc.
>
> **Capturing the gateway:** with sequential flashing the gateway boots ~38 s after the node;
> a sender/receiver pair is fine (sender keeps transmitting), but for a clean gateway-side log,
> flash the gateway and capture it from its own boot.

| Example | NODE/sender/A feature | GW/receiver/B feature | Boards |
|---|---|---|---|
| radio_id, radio_state, radio_regdump | — | — | 1 |
| radio_cw | `role-node` (TX CW) | *(default)* RX/RSSI | 2 |
| radio_linkdiag | `role-node` (TX) | *(default)* RX | 2 |
| radio_beacon | `role-node` (TX) | *(default)* RX | 2 |
| radio_csma | `role-node` (sender) | *(default)* jammer | 2 |
| radio_sleep | `role-node` (duty-cycled) | *(default)* always-on RX | 2 |
| net_secure_ping | `role-node` | `role-gateway` | 2 |
| net_confirmed | `role-node` | `role-gateway` | 2 |
| net_channel | `role-node` | `role-gateway` | 2 (ch2) |
| net_bulk / net_bulk_stress | `role-node` (requester) | *(default)* sender/server | 2 |
| net_bulk_stream | `role-node` | *(default)* | 2 |
| net_pairing | `role-node` (joiner) | *(default)* host | 2 |
| net_star | `role-node` (A) / `role-node,node-2` (B) | *(default)* hub | 2–3 |
| net_p2p | `role-peer-a` | `role-peer-b` | 2 |
| radio_band | `role-node` | `role-gateway` | 2 |
| radio_afa | `role-node` (LBT+agility node) | *(default)* scanner/gateway | 2 |
| radio_fhss | `role-node` (follower node) | *(default)* hop-master | 2 |
| edge_rapid | `role-node` | `role-gateway` checker | 2 |
| radio_interop | `role-node` | `role-gateway` | 2 |
| radio_gateway / radio_node | (separate examples) | | 2 |
| fota_ota | `role-node,fota-active` (+`fota-v2` for v2 image) | `role-gateway` | 2 |

KAT/single-board radio examples that need **no** feature and **no** peer: `net_persist`,
`net_duty_kat`, `fhss_kat`, `fhss_sweep`, `fhss_compliance`, `edge_recovery`,
`crypto_*`, `edge_frame_limits`, `fota_stage`, `fota_app`.

---

## 5. Test matrix

Legend for **Pass**: the exact substring to grep in the capture. `*` = mode (A=tower logs,
B=jolt --reset+strings, H=host, X=interactive, S=shell-exec).

### A. Host-side unit tests (`just test`) — mode H
| ID | What | Pass |
|---|---|---|
| H1 | `tower-kv` codec (12 tests: append/latest-wins, torn-append, torn-flip, torn-superblock fail-closed, compaction, full, migrate legacy/fresh, key-zero reserved) | `test result: ok. 12 passed` |
| H2 | `fota-sign` interop (3 tests: dalek↔salty verify, DEV_SEED↔VENDOR_PUBKEY pin, host/device digest agree) | `test result: ok. 3 passed` |
| H3 | bootloader size-check (≤18432 B budget; hard limit 20480) | prints `B used / … region` and exits 0 |

### B. Board bring-up & peripheral drivers
| ID | Example | Mode | Pass / observable | Edge cases |
|---|---|---|---|---|
| B1 | i2cscan | B | `scan complete - N device(s)`; lists `0x64` (+`0x49`,`0x19` if sensor board) | wrong/missing pull-ups → 0 devices; TMP112 in shutdown still ACKs |
| B2 | blinky | X (visual) | boot banner; LED heartbeat (40 ms on / 1960 off) + double-blink every 5 s preempts it | queue depth 8 overflow drops oldest; zero-duration step must not hang; `set_background(None)` idempotent |
| B3 | button | X | `press/release/click/hold` lines + LED flash on each | click ≤500 ms vs hold ≥1 s boundary; debounce 20 ms; button held at boot; rapid double-tap; bounce on release |
| B4 | thermometer | B | on sensor board: `NN.NN deg. C` every 2 s; **on these bare boards: `read failed` (NACK) — expected** | negative temps sign-extend; wrong strap → persistent NACK; I2C lockup |
| B5 | accelerometer | X+B | on sensor board: `dice: 1..6` per orientation, `tilt!` on shake (rate-limited 500 ms); **bare boards: WHO_AM_I/absent error** | all 6 faces (opposite faces sum to 7); 45° tilt → no face; sensitivity Ultra vs Low; min_interval throttle |
| B6 | strip | X (visual) | boot banner; scrolling rainbow on WS2812 (needs a strip on PA1) | brightness 0/100/gamma; RGB vs RGBW; zero-length; len>buffer panics; DMA1_CH3 contention |
| B7 | storage | B then reset+B | boot 1: `settings initialized to defaults`, `boot #1`; after reset `boot #2`, `#3`… (counter persists) | missing key→default; bad CRC→default; wrong-type read→None; compaction when full; power-loss mid-write keeps prior value |

### C. Console / shell / events
| ID | Example | Mode | Pass / observable | Edge cases |
|---|---|---|---|---|
| C1 | console_demo | A | logs at ERROR/WARN/INFO/DEBUG/TRACE + `println!`; long line truncated at 192 B on a char boundary; `Dropped{count}` marker after a burst overflows the depth-8 TX queue | UTF-8 (`°C`) not split at truncation; seq increments (check with `tower monitor`); drop-newest policy |
| C2 | console_panic | B | countdown `alive — deliberate panic in Ns`, then one ERROR frame `panicked at … console_panic.rs:LINE` emitted by the PAC-level panic handler | panic frame leads with `0x00` to flush a partial frame; halts silently if console not up; message clipped to 192 B |
| C3 | events_demo | A (`tower events`) | `EVENT measurement count=… temp_c=… unit=cdeg` each second + periodic `heartbeat`; interleaves with logs in `tower logs` | field cap 6; value/name length clip on char boundary; events apply backpressure (never dropped) |
| C4 | shell_demo | S | `tower exec` of `/system/resource print`, `/system/settings print`; set/get all 5 kinds (Str/Uint/Int/Bool/Enum); app cmds merged under `/system`; nested `/radio …`; `/export` | out-of-range rejected (`R_BAD_ARG`); unknown cmd (`R_NOT_FOUND`); TAB ambiguous vs unique (`tower complete`); enum/bool value completion; **persistence across reset**; chunked `resource print` (>192 B → multi-frame) |
| C5 | console_full | X (TUI) / S | `tower console` shows 4 live panes; or drive shell via `tower exec` | pane focus/zoom/pause; rapid updates no flicker |

### D. Crypto & frame KATs (single board, no peer) — mode B
| ID | Example | Pass | Vector / edge |
|---|---|---|---|
| D1 | crypto_aes_kat | `AES-128 ECB FIPS-197 vector: MATCH ***`; `got` == `69c4e0d8…c55a` | FIPS-197 App. B; mismatch ⇒ key/word/byte order or AES clock |
| D2 | crypto_ccm_kat | `AES-128-CCM RFC 3610 #1 + tamper: ALL PASS ***`; tag `…17e8d12cfdf926e0` | seal CT+tag match; open recovers PT; tampered byte ⇒ REJECTED |
| D3 | crypto_frame_loopback | `frame codec + nonce + CCM loopback: ALL PASS ***` | DATA hdr round-trip + tamper + wrong-key reject; BULK 3-byte index in nonce |
| D4 | edge_frame_limits | `edge_frame_limits: ALL PASS ***` (12 checks) | MTU 74/64 accept, 75/65 reject `PayloadTooLong`; `BadVersion`/`BadType`/`TooShort`/`AuthFail` |

### E. Storage / counter persistence
| ID | Example | Mode | Pass | Edge |
|---|---|---|---|---|
| E1 | storage | B×2 | see B7 — boot counter increments across resets | — |
| E2 | net_persist | B×2 | boot 1 `resumed tx_counter=1 reserve_watermark=1025`; advance to ~1025; **after reset**: `resumed tx_counter≥1025 reserve_watermark=2049` (never reuses) | counter must not start at 0 or go backward; watermark jumps by RESERVE=1024 |

### F. Radio bring-up (single board) — mode B
| ID | Example | Pass | Edge |
|---|---|---|---|
| F1 | radio_id | `partnum=0x01 version=0x30 … SPIRIT1 verified` | all-0x00/0xFF ⇒ SPI/CS/SDN wiring |
| F2 | radio_state | `-> READY ok (STATE=0x03…)`, `-> STANDBY ok (STATE=0x40…)`, `nIRQ asserted…`/`released` | nIRQ stuck ⇒ PA7/EXTI; state stuck ⇒ wedged |
| F3 | radio_regdump | register read-backs match `expect`; no `MISMATCH` | all 0xFF/0x00 ⇒ SPI read; field mismatch ⇒ config::apply |
| F4 | edge_recovery | `edge_recovery: ALL PASS *** (state machine never wedged)` (10/10 RX-timeouts→READY, FIFO flush 0/0, RX→READY cycle, device ID ok) | SABORT path; flush; chip responsive after abuse |
| F5 | fhss_sweep | `F1 PASS *** all FHSS channels lock`; reports `max retune+lock` + recommended GUARD | band edges 903/926.7 MHz lock; lock-time spread |

### G. Radio RF link (two boards) — mode A both ends
| ID | Example | Pass (node / gateway) | Edge |
|---|---|---|---|
| G1 | radio_cw | TX: `CW ON: state=0x5F`; RX RSSI jumps floor (~−110) → carrier (~−40 dBm) | RSSI stuck at floor ⇒ no emission; `error_lock=true` ⇒ synth |
| G2 | radio_linkdiag | TX: `fifo_loaded=16 … states=03→4F→5F→03 … tx_sent=true fifo_after=0` | FIFO load/drain; IRQ path; state trace |
| G3 | radio_beacon | TX `tx seq=N ok`; RX `rx len=16 seq=N rssi=… pqi=… sqi=… afc=…`; gap-free seq | data-rate/deviation/filter; gaps = loss; large AFC = drift |
| G4 | radio_csma | jammer `carrier ON/OFF`; sender `Busy — CCA backed off` during jam, `ok (channel clear)` after | never goes Busy ⇒ CCA broken; backoff jitter 0–100 ms |
| G5 | radio_sleep | node `→ SLEEP/SHUTDOWN`, `woke … in N µs`; GW monotonic `rx seq` re-links after each | SLEEP wake ≪ SHUTDOWN wake; never wakes ⇒ SDN/RCO |

### H. Network layer (two boards) — mode A both ends
| ID | Example | Pass | Edge |
|---|---|---|---|
| H4 | net_secure_ping | RX `AUTH OK: src=… cnt=… "ping NNN"`; forged ⇒ `CCM auth FAIL — dropped` | nonce uniqueness; key mismatch |
| H5 | net_confirmed | node `seq=N Delivered (ms)`; GW `…(ACKed, per-node key)`; retransmit doesn't double-deliver | NotDelivered storms ⇒ ACK path; replay (cnt≤last_seen) dropped |
| H6 | net_channel | same as H5 but on **ch2** (868.5 MHz); both ends must set CHANNEL=2 | per-channel VCO lock |
| H7 | radio_gateway+radio_node | node `seq=N Delivered … vbat=… temp=…`; GW decodes telemetry | marginal link ⇒ high NotDelivered |

### I. Bulk transfer (two boards) — mode A
| ID | Example | Pass | Edge |
|---|---|---|---|
| I1 | net_bulk | requester `fetched 200 bytes (… chunks), verify OK ***` | chunk order; nonce per chunk; 30 s session timeout |
| I2 | net_bulk_stream | `… B in … chunks, … ms, … bps, PASS ***` cycling 4/16/32/64 KB (US915, fast) | constant RAM across sizes (streaming); 64 KB exceeds old monolithic ceiling |
| I3 | net_bulk_stress | `fetched 4096 bytes (64 chunks), verify OK ***`; later `DutyLimited` as EU bucket drains | duty enforcement on bulk; CRC-32 |

### J. Pairing & topologies (two boards) — mode A
| ID | Example | Pass | Edge |
|---|---|---|---|
| J1 | net_pairing | host `PAIRED *** node id=… key[..4]=…`; joiner `JOINED *** … (expect …)` matching key | joiner picks own ID; PAIRING_KEY public (sniffable — by design); 60 s window |
| J2 | net_star | GW `2 peers registered`; `rx node A … "hello_a"`, `rx node B … "hello_b"` each under its own key | per-peer key + replay lane; B undecodable under A's key |
| J3 | net_p2p | both `PING/PONG … Delivered`; bidirectional | shared LINK_KEY; RX-window timing |

### K. Spectrum access — band / AFA / FHSS
| ID | Example | Mode | Pass | Regulatory |
|---|---|---|---|---|
| K1 | net_duty_kat | B | `duty governor KAT: ALL PASS ***` (ToA 30B=18 ms/96B=45 ms; bucket consume/refill/cap) | EN 300 220 1 %/h token bucket |
| K2 | fhss_kat | B | `fhss_kat: ALL PASS ***` (F5 permutation, F4 dwell ≤300 ms/20 s, F3 beacon round-trip) | FCC §15.247 dwell + ≥50 ch |
| K3 | fhss_compliance | B (≥30 s) | `F10 COMPLIANCE: PASS *** (80 ch, max …ms ≤ 400ms)`; `channels used = 80/80`; band edges used | §15.247(a)(1)(i) evidence |
| K4 | radio_band | A (2 brd) | node `now on 868/915 MHz`; GW `Delivered (868 tag)` + `(915 tag)` | runtime retune; 915 single-ch is bench-only |
| K5 | radio_afa | A (2 brd) | node `seq=N ch=X Delivered`; channel varies under contention; GW follows | EN 300 220 LBT+AFA, CCA −90 dBm |
| K6 | radio_fhss | A (2 brd) | master `beaconing slot=… ch=…`; node `LOCKED` then `tx seq=N ch=… Delivered`; re-sync after master restart | 80-ch, 300 ms slot, 24 s cycle |

### L. Stress / recovery / soak (two boards) — mode A
| ID | Example | Pass | Edge |
|---|---|---|---|
| L1 | edge_rapid | node `sent N (delivered=N other=0)`; GW `violations=0 (monotonic OK)` — never `ORDER VIOLATION` | back-to-back confirms; strict-monotonic accept; no double-accept |
| L2 | radio_interop | GW `VERDICT: PASS`; node logs `seed=…` (replayable); invariants: CRC, order, duty, confirm-resolution, oversize-rejected | run long (minutes–hours); `LATCHED FAIL` lights LED solid |

### M. FOTA (signed A/B OTA)
| ID | Example | Boards | How | Pass |
|---|---|---|---|---|
| M1 | fota_stage | 1 (FOTA-linked) | `just flash-fota fota_stage` ; mode B (longer window — stages several sizes) | `fota_stage: … ALL PASS ***` (write/read-back/digest of staged images) |
| M2 | fota_app | 1 (FOTA-linked) | `just flash-fota fota_app`; observe self-swap + confirm, then revert path | `*** SWAP CONFIRMED ***` (and revert on unconfirmed boot) |
| M3 | fota_ota E2E | 2 | node: `TOWER_FEATURES=role-node just flash-fota fota_ota`; GW: `TOWER_FEATURES=role-gateway just flash example fota_ota`; build+sign the update `just fota-update`; serve `tower -d <GW> fota serve --image target/fota-update.bin --manifest target/fota-update.fmanifest`; watch node | node pulls → bootloader verifies Ed25519+SHA → swap → `*** UPDATE CONFIRMED ***` running v2 |

FOTA happy-path detail and the bootloader verify gate live in `docs/fota.md`.

---

## 6. Cross-cutting edge-case catalogue

These are higher-value "be creative" cases to fold into runs as time allows. Many require a code
tweak or fault injection (note them as separate experiments, don't leave the tree dirty):

**Security / crypto**
- FOTA tampered image: flip one byte of the served `.bin` → bootloader must reject (no swap).
- FOTA wrong key: sign with a non-DEV_SEED key → reject.
- FOTA downgrade: serve a *lower* version than installed → rollback policy refuses.
- FOTA interrupted download: power-cut the node mid-pull → resumes from the EEPROM high-water mark.
- FOTA power-loss during swap: reset during bootloader swap → A/B leaves a consistent slot (no brick).
- FOTA unconfirmed boot: app doesn't call confirm → next boot reverts to the old slot.
- Replay: re-send a captured frame (cnt ≤ last_seen) → dropped; forged high counter → CCM-rejected *before* it can poison last_seen.
- Counter saturation: near 2³²−1 the link fails closed (no nonce reuse / wrap).

**Framing / console**
- Oversized log (>192 B) and oversized shell response (>256 B) clip on a char boundary, never mid-UTF-8.
- TX-queue overflow (depth 8) → `Dropped{count}` accurate; producers use drop-newest, events use backpressure.
- Panic with a partial frame in flight → leading `0x00` flushes the host decoder; panic frame stands alone.
- Seq wrap (u16) over a long run → host reports no false gaps.
- CRC corruption / COBS resync: garbage + delimiter → host resyncs on next frame.

**Storage / KV (also covered by host tests H1)**
- Torn append, torn flip, torn superblock-commit → fail-closed, prior keys survive.
- Compaction when half is full; `Error::Full` when live set can't fit a half.
- Wrong-type / corrupt read → falls back to default, never panics.

**Radio / RF**
- RX-completion: `PCKT_FLT_OPTIONS` bit6 must be clear or RX_DATA_READY never fires (regression guard = any working RX test).
- State-machine wedge after RX timeout (F4/edge_recovery): must SABORT→READY.
- CSMA under sustained jam: stays `Busy`, never silently transmits.
- Channel boundaries: ch0/1/2 (EU), all 80 FHSS channels incl. edges.
- Duty exhaustion on EU: repeated bulk pulls → `DutyLimited`, refill over time.
- Sleep/shutdown re-link: wake latency SLEEP ≪ SHUTDOWN; both re-establish the link.
- Pairing window expiry with no joiner; joiner-ID collision (later add_peer overwrites).

**Power / low-power**
- USB present → console UART up, so the enabled USART holds embassy's STOP refcount → STOP inhibited (plain Sleep/WFI); console + EXTI live.
- Unplugged → `console::manager` drops the UART → STOP reached when idle (~32 µA @3 V); a PA12 edge or the ~500 ms VBUS poll brings the console back on plug-in. UART flush-on-complete prevents truncated lines (regression: last log line not cut off).

---

## 7. Results log

Record one row per test, per run. Template (`tools/hwtest/RESULTS.md` is generated per session):

```
| ID | Example | Features | Port(s) | Verdict | Evidence (grep'd line) | Notes |
|----|---------|----------|---------|---------|------------------------|-------|
```

Verdict ∈ {PASS, FAIL, SKIP(reason), MANUAL(deferred)}. Keep the raw captures
(`/tmp/twr/results/<id>.txt`) for any FAIL.
