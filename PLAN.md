# TOWER Radio (SPIRIT1) — Implementation Plan

## Deliverables (in addition to the code)

- **`/Users/pavel/hardwario/embassy/PLAN.md`** — this plan, copied to the project root and
  kept alongside the code as the living implementation checklist (the numbered steps with
  checkboxes are ticked off as work lands).
- **`/Users/pavel/hardwario/embassy/docs/radio.md`** — a **user-facing** guide to the radio
  implementation and protocol, written as part of the work (see Step 19). This is distinct from
  `RADIO.md` (the internal design spec): it documents how to *use* the stack — the public API
  (radio + network layer), configuration (band/channel/power/role), the wire protocol (frame
  layout, frame types/flags, AES-CCM nonce, counters/replay), the topologies (star/P2P),
  pairing, bulk/downlink-pull, the duty governor, and a worked example per use case. Updated
  incrementally as each layer is implemented so it never drifts from the code.
- **Implementation runs autonomously ("auto mode")** — proceed step by step without pausing
  for per-step approval; the per-step on-hardware verification gates remain the quality bar.

## Context

`RADIO.md` is a finalized specification for a bi-directional sub-GHz radio stack on the
**SPIRIT1** transceiver (SPSGRF module) wired to the STM32L083CZ on the Core Module. No
radio, SPI, or AES code exists in the `tower` crate yet — this plan builds the whole stack
from bring-up to a comprehensive on-hardware test campaign.

The work is decomposed into **small, independently flashable steps**, each ending with a
concrete verification on the real hardware (two boards: **Node** `/dev/cu.usbserial-111140`,
**Gateway** `/dev/cu.usbserial-11140`). Every step compiles to an `examples/radio_*.rs`
binary and isolates exactly one new failure domain on top of an already-proven base, so a
regression is localized the moment it appears. A final **semi-fuzzy soak campaign** on both
boards is the acceptance net for interaction bugs the per-step gates can't reach.

**Decisions made with the user:**
- **Role selection = Cargo build features** (`role-node`, `role-gateway`, `role-peer-a/b`).
  Each two-board example is one source file built twice with different features.
- **RF verification uses the two boards only** (no SDR/analyzer). The CW and RF-config steps
  are proven by the partner board's RSSI and by the modulated two-board link; RX-bandwidth
  narrowing is done from logged `AFC_CORR` over temperature, not lab instruments.
- **EU 868 first.** Implement and verify 868.1/868.3/868.5 MHz now; keep the `Band`
  abstraction so US 915 can be added later (it is provisional per §2.2 / §15).

**Refinements to the spec discovered during exploration (apply these):**
- The L0 **AES block *is* in `stm32-metapac`** (`embassy_stm32::pac::AES`, `aes_v1`: `cr/sr/
  dinr/doutr/keyr(n)/ivr(n)`, with ECB+CTR `chmod`, `datatype` byte-swap, and a `ccf`
  completion flag). RADIO.md §6 says embassy-stm32 0.6.0 doesn't *wrap* it — true, but we go
  through the PAC directly, so the **hardware AES is the primary path**, not the fallback.
- Use **blocking SPI** for the radio (`Spi::new_blocking` on SPI1, PB3/PB5/PB4, AF0). SPI1's
  only DMA channels are fixed-function `DMA1_CH2/CH3`, and `DMA1_CH3` is already owned by the
  WS2812 strip (`board.rs`). FIFO bursts are ≤96 B at ≤10 MHz (~80 µs), so blocking is simpler
  and avoids the collision. Operation sequencing is still async/IRQ-driven via EXTI on nIRQ.
- **nIRQ (PA7) = EXTI line 7**, on the **already-bound `EXTI4_15`** group (`board.rs`). No new
  `bind_interrupts!` needed; PH0 (GPIO1, EXTI line 0) is free as an optional 2nd IRQ.

---

## Architecture

### Module layout (new `src/radio/` subtree; one `pub mod radio;` added to `lib.rs`)

```
src/radio/
  mod.rs        Public façade + re-exports (the only thing lib.rs sees).

  ── Radio layer (SPIRIT1 + crypto + wire) ──
  regs.rs       SPIRIT1 register/command addresses, IRQ-mask bits, MC_STATE codes,
                GPIO-conf values, PA table. Pure consts (style: src/lis2dh12.rs).
  spi.rs        `Spirit1Spi`: owns blocking `Spi` + software-CS `Output<PA15>`; enforces
                ≥2 µs CS setup; read/write regs, command, read/write FIFO; returns MC_STATE.
  device.rs     `Spirit1`: chip handle over Spirit1Spi + SDN `Output<PB7>`. Power-state
                transitions (poll MC_STATE w/ timeout), device-ID verify (304/48), XTAL 50 MHz,
                ST work-arounds, VCO/RCO calibration, CW test, FIFO flush, SABORT recovery,
                AFC_CORR/RSSI/LQI/SQI read. Pure register sequencing, no async.
  config.rs     RF config types + datasheet register math: `Band{Eu868,(Us915)}`, `Channel`,
                `RfConfig`, `TxPower`, `SignalQuality`; base-freq/VCO, datarate 19200, fdev
                20 kHz, RX BW ~210 kHz, sync 0xDB624715, CRC mode 3, PA ramp, AFC/AGC/IF, CSMA.
  driver.rs     IRQ-driven async operation owner. `#[task] radio_task` owns `Spirit1` + the
                PA7 `ExtiInput`, serves requests over a `static Channel`, wakes on nIRQ.
                State machine, TX(+CSMA), RX(timeout), FIFO fill/drain, IRQ decode, quality.
                Returns a cheap `Radio` handle = §10 radio API (tx/rx/set_state/read_afc_hz/
                cw_test). Pattern: src/button.rs init_exti → scan_task → Channel → handle.
  aes.rs        Register-level L0 AES over pac::AES: enable RCC.aesen, load key/IV, ECB block,
                CTR. Poll CCF; handle datatype byte-order. (soft `aes` crate behind a feature.)
  ccm.rs        AES-128-CCM (SP 800-38C) on aes.rs: CBC-MAC tag + CTR. N=13, L=2, 8-byte tag.
                `seal(key,nonce,aad,&mut pt)->tag`, `open(...,tag)->Result<(),AuthFail>`.
  frame.rs      Wire codec (§3): Header/Flags/FrameType, encode/decode the 96-B FIFO buffer,
                AAD slice, `nonce_for(src,counter,bulk_index)` (single audited function), MTU
                checks. Pure, no_std — the most unit-testable module.

  ── Network layer ──
  net/mod.rs    `Net` handle + `NetConfig{role,id,key,band,channel}`; spawns net_task; the
                full §10 network API (send/recv/signal_quality/bulk_send/bulk_recv/
                poll_downlink/add_peer/remove_peer/open_pairing/close_pairing/join).
  net/peers.rs  Peer table (gateway ≤64 / node→gateway / P2P ≤8): (id,key,last-seen) over
                storage::Kv; replay check (CCM-verify-then-compare).
  net/counter.rs TX counter + reserve-ahead watermark wear-ring (RESERVE=1024, hard-stop at
                2³²−1); receiver last-seen lazy-persist (P=32) / per-sender ring. Over Kv.
  net/delivery.rs Confirmed delivery: 200 ms ACK window, random 0–100 ms backoff, reps 1–10,
                cached-ACK retransmit, ACK build/parse (acked counter, dl-pending+len, RSSI).
  net/duty.rs   EU duty governor: per-sub-band rolling-hour airtime, ToA from length, defer/refuse.
  net/bulk.rs   Bulk/pull state machine (announce→BULK_REQ/BULK_DATA→complete), 24-bit index,
                last-chunk, 30 s idle timeout, streaming source/sink traits.
  net/pairing.rs OTA 3-way join under the fixed public pairing key; window timeout; commit-on-confirm.
  net/topology.rs Star vs P2P policy (who listens, pull rules, table limits).
```

### Supporting changes
- **`src/board.rs`** — additive only: hand out radio resources — `Peri<PB7>` (SDN),
  `Peri<PA15>` (CS), a pre-built blocking `Spi` on SPI1 (PB3/PB5/PB4), and an
  `ExtiInput<'static, Async>` on PA7 (nIRQ); optionally `Peri<PH0>`. These pins are currently
  unbound. Keep `storage` reachable by `Net` (it needs EEPROM for counters/keys/last-seen).
- **`Cargo.toml`** — add embassy-stm32 feature `"spi"`; add `[features]` for roles
  (`role-node`, `role-gateway`, `role-peer-a`, `role-peer-b`, `node-1`, `node-2`,
  `test-hooks`, `afc-sweep`); optional `bitflags = "2"`; optional `soft-crypto` feature gating
  `aes`/`ccm` crates as a fallback. AES needs **no** embassy feature (PAC + `RCC.aesen`). No
  DMA feature for the radio. `embedded-hal` (present) reused for a HAL-independent SPI bound.
- **`justfile`** — extend `build`/`flash` to thread `--features` through to `cargo objcopy`
  (e.g. `just flash net_uplink role-gateway -p $GW`). Small, mechanical edit.
- A small **shared test-identity table** (throwaway IDs/keys) `include!`d by the examples so
  the two boards address each other without a provisioning step.

---

## Implementation Steps

> Each step: build the named example, flash to the board(s) shown, observe in `jolt monitor`.
> Commands: `just flash <name> <role-feature> -p <port>` then `jolt monitor -p <port>`
> (`--reset` to catch boot). **Do not start the next step until the verify box is checked.**

### Phase 1 — Bring-up

- [x] **1. Board wiring + SPI transport + device ID.** Add radio pins/SPI1 to `Board`;
  implement `regs.rs` (status/ID consts), `spi.rs` (`Spirit1Spi`, ≥2 µs CS, MC_STATE
  readback), and `device.rs` `exit_shutdown()` + `read_device_id()`.
  *Reuse:* `board.rs` init, `lis2dh12.rs` register style, `Spi::new_blocking`, `console.rs`.
  - [x] **Verify** (`radio_id`, 1 board): ✅ on Gateway — `radio reached READY`, then
        `partnum=0x01 version=0x30 (part_number=304) - SPIRIT1 verified`. SPI+CS+SDN proven.

- [x] **2. Power-state machine + nIRQ.** State transitions (`ready/standby/sleep/shutdown`)
  with `MC_STATE`-poll-until-settled + timeout (§9 stuck-state). Configure GPIO0=nIRQ, bind the
  PA7 `ExtiInput`, confirm the line toggles on a benign IRQ source.
  *Reuse:* EXTI4_15 already bound (`board.rs`); `button.rs`/`power.rs` `ExtiInput` await.
  - [x] **Verify** (`radio_state`, 1 board): ✅ READY (0x03) ↔ STANDBY (0x40) transitions
        log expected codes; nIRQ asserts on READY (IRQ_STATUS bit 16) and releases after the
        status read. SLEEP state (0x36) deferred to Step 6 (needs RCO cal + wake timer).

### Phase 2 — RF configuration *(highest RF risk)*

- [x] **3. RF config + CW test (EU 868).** Implement `config.rs` register derivation (base
  freq, VCO+RCO cal, 19200 bps, fdev 20 kHz, RX BW ~210 kHz, sync `0xDB624715`, 16-bit CRC,
  whitening, PA table+ramp, AFC freeze-on-sync, AGC, IF, RSSI offset). Add `device.cw_test(on)`.
  Key fixes found on HW: **REFDIV=1** (÷2 PLL ref for the 50 MHz xtal, SYNT doubled), **SEL_TSPLIT=1**,
  **TXSOURCE=PN9 for CW** (else TX underflows), RSSI = raw/2−130 and **latches only on SABORT**.
  - [x] **Verify** (`radio_cw`, two boards): ✅ Gateway RX reads **−63 dBm (CARRIER)** during the
        Node's CW-on periods and **−106 dBm (floor)** during off, alternating with the 3s/2s cycle.
        TX reaches state 0x5F; synth locks; both boards agree on 868.1 MHz. (No SDR needed.)

- [x] **4. Raw TX / RX (unencrypted) — FULL LINK WORKING.** Async `Spirit1::tx`/`rx` (nIRQ-driven
  via `ExtiInput`, FIFO fill/drain, RSSI/LQI/SQI/AFC capture, CSMA gate). `radio_beacon` (TX) /
  `radio_sniffer` (RX), plus deep diagnostics `radio_rxdiag`/`radio_linkdiag`/`radio_rxirq`.
  - [x] **TX verified**: FIFO loads (`fifo_loaded=16`), state trace `…→5F→03`, `tx_sent=true`, FIFO drains.
  - [x] **RX verified**: ✅ `rx len=16 seq=31,32,…` sequential, **no gaps**, CRC + whitening on,
        `rssi=-36 dBm pqi=135 sqi=32 afc=5`. **First true bidirectional link.**
  - **Root cause of the long RX block (infrastructure, not RF):** never set the RX-timeout stop
        condition. Reset `PCKT_FLT_OPTIONS` has `RX_TIMEOUT_AND_OR_SELECT=1` → "timeout cannot be
        stopped" (datasheet Table 30/§9.3) → a full packet sits in the FIFO and the part stays in RX
        forever, never raising RX_DATA_READY. Setting `PCKT_FLT_OPTIONS` bit6=0 (+ AUTO_PCKT_FLT,
        + clear source/control filters) → "reception ends at packet reception" → RX_DATA_READY fires.
        `afc=5` confirms the crystals are close; the earlier bandwidth detour was a red herring.

- [ ] **5. CSMA + full IRQ surface + stuck-state recovery.** CSMA/CCA before initiating TX
  (−90 dBm, ≤100 ms backoff, max-backoff IRQ); wire all IRQ events into `RadioEvent`; SABORT→READY
  watchdog (§9).
  - [ ] **Verify** (`radio_csma`, two Nodes → one Gateway): monitor shows CCA deferrals and a
        `busy`/max-backoff event when the channel is held (jam with a held TX). It reports, never hangs.

- [ ] **6. Low-power sleep/wake.** Node uses SPIRIT1 SLEEP (wake-timer) and SHUTDOWN between
  transfers; MCU drops to STOP; nIRQ/PA7 + a timer wake it. Validate fast wake (SLEEP→READY ~125 µs)
  vs re-init (SHUTDOWN→READY ~650 µs).
  *Reuse:* `power.rs` `WakeGuard`/STOP, the auto-spawned `vbus_task`, `board.rs` STOP executor.
  - [ ] **Verify** (`radio_sleep`): with USB unplugged (STOP allowed), Node wakes on cadence,
        TXes, sleeps; Gateway keeps receiving; re-links correctly after both SLEEP and SHUTDOWN wake.

### Phase 3 — Security *(crypto correctness, no radio)*

- [x] **7. L0 AES register driver.** `aes.rs` over `pac::AES` (`unstable-pac` feature):
  enable `RCC.ahbenr.crypen`, load `keyr(3-i)` big-endian, `CR` mode=encrypt/chmod=ECB/
  datatype=BYTE, write `dinr`×4 (little-endian, engine swaps), poll `sr.ccf`, read `doutr`×4,
  clear `ccfc`. ECB single-block primitive (CBC-MAC/CTR built in `ccm.rs`).
  - [x] **Verify** (`crypto_aes_kat`, 1 board): ✅ FIPS-197 AES-128 ECB vector → **MATCH**
        (`69c4e0d8…b4c55a`). Byte order: key big-endian, data little-endian + datatype=BYTE swap.

- [x] **8. AES-128-CCM.** `ccm.rs` (CBC-MAC + CTR, N=13/L=2/8-B tag, constant-time tag compare)
  on `aes.rs`.
  - [x] **Verify** (`crypto_ccm_kat`, 1 board): ✅ RFC 3610 Packet Vector #1 ciphertext + tag
        **MATCH** (`17e8d12c…26e0`); valid `open` recovers plaintext; tampered ciphertext correctly
        **REJECTED**. Pure compute, one board.

### Phase 4 — Wire format & network layer

- [x] **9. Frame codec + secured packet (codec verified; OTA gated on RX demod).** `frame.rs`:
  `Header`/`FrameType`/`flags`, encode/parse, `nonce_for(src,counter,bulk_index)`, MTU checks,
  `seal_frame`/`open_frame` tying the layout to CCM.
  - [x] **Verify** (`crypto_frame_loopback`, 1 board, no radio): ✅ secured DATA frame round-trips
        (header+payload MATCH); tampered frame and wrong key → AuthFail; bulk frame (17 B hdr + 64 B
        chunk) round-trips with the 3-byte index in the nonce. **ALL PASS.**
  - [x] **OTA verified** (`net_secure_ping`, two boards): ✅ Node sends CCM-sealed DATA frames;
        Gateway logs `AUTH OK: src=11111111 cnt=N confirmed=true rssi=-35dBm | "ping NNN"` — full
        stack (radio link + frame codec + AES-CCM auth + decrypt) working end-to-end. Sequential,
        no gaps. Tampered/forged frames would fail the CCM tag; CRC-corrupt frames dropped by HW.

- [x] **10. Confirmed delivery + ACK + retransmit.** `net.rs`: `Net` with `send(confirmed,reps)`
  / `recv()`, 200 ms ACK window, random 0–100 ms backoff, reps 1–10, cached-ACK retransmit, and the
  counter/replay rule (counter > last-seen accept; == retransmit/resend cached ACK; < drop). ACK
  uses the ACKer's own fresh counter; acked counter rides in the payload (§6). `net_confirmed` example.
  - [x] **Verify** (`net_confirmed`, two boards): ✅ Node `Delivered (59 ms)` every cycle; Gateway
        receives + auto-ACKs. Key fix: **20 ms ACK turnaround** on the receiver — the ACK must wait
        for the sender to finish its TX→RX switch (an 8 ms turnaround raced the RX set-up and the ACK
        was missed). Retransmit path exercised when ACKs are lost (→ `NotDelivered` after N reps).
  - [ ] Adversarial cases (forced ACK loss, replay rejection) folded into Step 11 + the soak (Step 18).

- [ ] **11. Replay protection + counter persistence.** `net/counter.rs` (reserve-ahead
  watermark ring, hard-stop at 2³²−1) + `net/peers.rs` last-seen (gateway lazy-persist P=32; node
  ring) over `storage::Kv`. CCM-verify-then-compare ordering.
  *Reuse:* `storage::Kv` in-place same-size update (fixed-width ring cells); `examples/storage.rs` idiom.
  - [ ] **Verify** (`net_replay` + `net_counter_persist`): Gateway accepts increasing counters,
        **drops** a replayed lower/equal counter (replay state untouched). Power-cycle the Node →
        it resumes **at-or-above** its watermark (log the resumed value; never reused). Power-cycle
        the Gateway → replay window ≤ P. *Security-critical persistence checkpoint.*

- [ ] **12. Duty governor (EU).** `net/duty.rs`: per-sub-band rolling-hour airtime, ToA per
  frame (§2.6), defer/refuse over 1 %.
  - [ ] **Verify** (`net_duty`): drive the Node above 1 % (tight max-frame loop); monitor shows
        accumulating airtime and `DutyLimited`/deferral once the budget is hit, then resumption as
        the window rolls. Confirm the Gateway is governed too (ACK airtime counts).

- [ ] **13. Bulk transfer + downlink pull.** `net/bulk.rs` (announce → BULK_REQ/BULK_DATA,
  24-bit index, last-chunk, 30 s idle timeout, streaming source/sink) + `bulk_send`/`bulk_recv`/
  `poll_downlink`.
  *Reuse:* `net/delivery.rs` confirmed mechanism (BULK_REQ is confirmed); bulk header (17 B + ≤64 B).
  - [ ] **Verify** (`net_downlink_pull` + `net_bulk`): Gateway announces a downlink (ACK
        dl-pending+len); Node pulls chunk-by-chunk and reassembles a known blob (length+CRC OK).
        Reboot the *requester* mid-pull → Gateway times out (30 s) and frees; requester restarts on
        next announce. Matches §7.7(3).

- [ ] **14. OTA pairing (3-way join).** `net/pairing.rs` (fixed public pairing key,
  JOIN_REQ/RESP/CONFIRM, window timeout, commit-on-confirm) + `open_pairing`/`close_pairing`/`join`.
  *Reuse:* `ccm.rs` under the public key; `net/{counter,peers}.rs` to init counters/last-seen on commit (§6).
  - [ ] **Verify** (`net_pairing`): Gateway `open_pairing`; Node `join` → 3-way completes, both
        commit, then a normal confirmed `send` works under the new per-node key. Drop the CONFIRM →
        Gateway window times out, discards the tentative entry, Node retries. Two Nodes in one window
        → Gateway pairs the first, ignores the second. Matches §7.7(4).

- [ ] **15. Topologies (star + P2P) + full `Net` API.** `net/topology.rs` policy; finish
  `net/mod.rs` to the complete §10 API (`add_peer`/`remove_peer`, `signal_quality`, role handling,
  ≤64 star / ≤8 P2P limits).
  - [ ] **Verify** (`net_star` + `net_p2p`): Star — multiple Nodes (re-flash the spare board with
        different IDs across runs) each do confirmed uplink + pull downlink against the Gateway. P2P —
        two boards as peers, one listening, confirmed exchange both directions under the per-link key.

### Phase 5 — Polish & robustness

- [ ] **16. Public API + docs + reference apps.** `pub mod radio;` in `lib.rs`; finalize
  re-exports; SDK-style doc comments; write the shipped reference apps `examples/radio_gateway.rs`
  and `examples/radio_node.rs`.
  - [ ] **Verify**: `just samples` lists the radio examples; `just run radio_gateway` /
        `radio_node` on the two boards demonstrates the full happy path with clean logs.

- [ ] **17. Edge-case examples.** Land the focused edge-case binaries (catalog below):
  oversized-payload reject, unknown version/type drop, CCM auth-fail drop, FIFO over/underflow
  recovery, stuck-state recovery, CSMA/hidden-node contention, rapid back-to-back, sleep cadence,
  channel switch.
  - [ ] **Verify**: each edge example logs the expected drop/recover/reject behavior per §9.

### Phase 6 — Comprehensive semi-fuzzy campaign *(final acceptance)*

- [ ] **18. Fuzzy interop soak.** Implement `examples/radio_interop.rs` (one file, built
  `--features role-node` and `--features role-gateway`) — details in the campaign section below.
  - [ ] **Verify**: smoke (5 min) → standard (1–2 h) → soak (12–24 h). **PASS requires both
        boards' verdicts PASS and zero latched fail-LEDs.** Covers all of §14, including the AFC_CORR
        temperature sweep that justifies narrowing RX BW (then re-confirm all three channels are
        simultaneously usable per §2.1).

- [ ] **19. User documentation (`docs/radio.md`).** Write the user-facing guide (drafted
  incrementally from Step 9 onward, finalized here): public API reference (radio + network layer,
  mapping §10), configuration (band/channel/power/role features), the wire protocol (frame layout,
  types/flags, AES-CCM nonce derivation, counter/replay rules), topologies (star/P2P), OTA pairing,
  bulk/downlink-pull, the EU duty governor, low-power behavior, and a worked code snippet per use
  case keyed to the examples catalog. Keep `RADIO.md` (internal spec) and `docs/radio.md`
  (user guide) cross-linked.
  - [ ] **Verify**: a reader following only `docs/radio.md` can flash `radio_gateway`/`radio_node`,
        send a confirmed message, pair a node, and run a bulk transfer without consulting the source.

---

## Examples Catalog

All multi-board examples are **one file built twice** via role features; interactive variants
(pairing lost-confirm/two-joiner, bulk reboot, channel switch) are **button-driven** (PA8) re-runs
of one file. **LED (PH1):** heartbeat = running, short blink = transfer OK, **solid = a checked
invariant FAILED** (visible overnight without scrollback).

| Example | Step | Boards / roles | Demonstrates (spec) |
|---|---|---|---|
| `radio_id` | 1 | 1 | device-ID check, abort on mismatch (§1.2, §8) |
| `radio_state` | 2 | 1 | state transitions + nIRQ edge (§1.2, §4) |
| `radio_cw` / `radio_rssi` | 3 | TX+RX | CW carrier proven via partner RSSI; channel switch (§2.2, §2.4) |
| `radio_beacon` / `radio_sniffer` | 4 | TX+RX | raw TX/RX, per-packet RSSI/LQI/SQI/AFC_CORR (§2.7, §2.8) |
| `radio_csma` | 5 | 2 Nodes + GW | CCA deferral + max-backoff IRQ (§2.5, §9) |
| `radio_sleep` | 6 | Node + GW | SLEEP/SHUTDOWN wake, STOP integration (§4, §11) |
| `crypto_aes_kat` | 7 | 1 | FIPS-197 ECB/CTR known-answer test (§6) |
| `crypto_ccm_kat` | 8 | 1 | CCM vectors + tamper→AuthFail (§5, §6) |
| `net_secure_ping` | 9 | Node + GW | one CCM-sealed frame end-to-end (§3, §6) |
| `net_uplink` | 10 | Node + GW | confirmed/unconfirmed uplink + ACK (§7.3, §7.7-1) |
| `net_ack_retransmit` | 10 | Node + GW | forced ACK loss → identical retransmit, cached ACK (§7.3, §7.7-2) |
| `net_replay` | 11 | Node + GW | replay rejection, state untouched (§6, §7.4, §9) |
| `net_counter_persist` | 11 | Node + GW | reserve-ahead watermark survives reboot (§6, §7.4) |
| `net_duty` | 12 | Node + GW | duty governor defers/refuses over 1 % (§2.2, §9) |
| `net_downlink_pull` | 13 | Node + GW | pull-based downlink (§7.2, §7.5, §7.7-3) |
| `net_bulk` | 13 | Sender + Requester | bulk both ways + requester-reboot mid-pull (§7.5, §9) |
| `net_pairing` | 14 | Host + Joiner(s) | 3-way join; lost-confirm; two-joiner (§7.6, §7.7-4) |
| `net_star` | 15 | GW + ≥2 Nodes | star, per-node table + quality (§7.2, §7.4) |
| `net_p2p` | 15 | Peer-A + Peer-B | P2P confirmed exchange (§7.2) |
| `net_channel_band` | 17 | Node + GW | channel switch re-runs VCO cal; shared-channel rule (§2.2, §8) |
| `edge_payload_limits` | 17 | Node + GW | 1 B / 74 B OK; >74 B no-bulk → rejected (§3, MTU) |
| `edge_bad_frames` | 17 | injector + GW | unknown ver/type drop, CCM auth-fail drop (§3, §6, §9) |
| `edge_fifo_recovery` | 17 | Node + GW | FIFO over/underflow → flush, abort, resume (§4, §9) |
| `edge_stuck_state` | 17 | 1 | MC_STATE stuck → SABORT→READY, re-init on repeat (§9) |
| `edge_contention` | 17 | 2 Nodes + GW | CSMA + hidden-node, collisions absorbed by retransmit (§2.5, §4) |
| `edge_rapid` | 17 | Node + GW | back-to-back transfers, one-at-a-time, monotonic counters (§4, §6) |
| `radio_gateway` / `radio_node` | 16 | GW + Node | shipped reference apps, full happy path (§10) |

Fault injectors (ACK-drop, malformed frame, FIFO error, stale-replay) live behind the `test-hooks`
feature so `net_*`/`edge_*` don't duplicate addressing/fault code.

---

## Comprehensive Semi-Fuzzy Test Campaign (`radio_interop`)

One file, built `--features role-node` and `--features role-gateway`; long unattended runs on the
two boards.

**Deterministic randomness.** An LCG/xorshift **seeded from the 32-bit device ID** (or a build-time
`SEED` const to force a replay) — the goal is reproducibility, not entropy. The boot banner logs the
seed so a failing run replays bit-for-bit. Each board runs an independent stream; the gateway derives
its checks from the node's **self-describing payload metadata** `(seq, len, crc32)`, so the boards
need not be in lockstep.

**The node randomizes per iteration:** transfer type (single DATA / bulk uplink / downlink-pull /
P2P / BULK_REQ, weighted) · confirmed vs unconfirmed · repetitions 1–10 · TX power · payload size
1–74 B (with elevated probability on the 1 B and 74 B/full-64 B-chunk edges) · timing jitter ·
low-probability faults (self-reboot mid-transfer, forced sleep, scheduled-epoch channel switch agreed
by both boards, an intentional stale-frame replay, an oversized `send()` expecting local reject).

**Invariants checked (→ §14):**
- **No nonce reuse** — gateway records every accepted `(src,counter,bulk_index)`; a repeat that isn't
  a byte-identical retransmit ⇒ FAIL.
- **No accepted replay** — any accepted frame with counter ≤ last-seen ⇒ FAIL.
- **Confirmed delivered-or-reported** — every confirmed transfer ends `Delivered` or `NotDelivered`.
- **CCM tag valid on every accepted frame** — verify-first by construction; injected tamper never accepted.
- **Payload integrity & ordering** — gateway recomputes crc32 vs the embedded header; bulk indices
  complete and in order.
- **Duty budget respected** — both boards track airtime; a TX over 1 %/h must be `DutyLimited`, never sent.
- **Counter monotonicity & persistence** — live counter strictly increases; after a random reboot it
  resumes ≥ last-sent; any reuse ⇒ FAIL.
- **Replay window across gateway reboot ≤ P.**
- **Bulk reboot recovery** — requester reboot ⇒ sender frees ≤30 s, pull restarts, completes/reported.
- **Pairing** — periodic open-window/join cycles incl. lost-confirm and two-joiner; never half-paired.
- **FIFO / stuck recovery** — injected FIFO errors recover to RX; stuck MC_STATE recovers via SABORT.

**Surfacing:** rolling console counters every N transfers (`tx ok/not-delivered/busy/duty-limited`,
`rx accepted/dropped(replay|auth|ver|type|crc)`, `bulk done`, `pairings`) + a one-line
**`VERDICT: PASS`** or **`VERDICT: FAIL <invariant + offending (src,counter,index,seed)>`** with the
failing frame dumped. LED latches solid on any violation. Tallies persisted to `storage::Kv` so a
reboot doesn't lose them; boot banner prints cumulative totals + seed.

**Schedule:** smoke 5 min (every build) → standard 1–2 h (≥1 full duty rolling hour, many reboots,
several pairings/bulk cycles) → soak 12–24 h (rare-fault accumulation, EEPROM wear-ring rotation, AFC
drift across day/night temperature — pairs with `radio_sniffer --features afc-sweep` to gather the
AFC_CORR-vs-temperature data and **narrow RX BW** per §2.1, then re-verify all 3 channels usable).
PASS = both boards' verdicts PASS and zero latched LEDs.

---

## Top Risks & Where the Checkpoint Catches Them

| Risk | Why uncertain | De-risking gate |
|---|---|---|
| RF config register values | C reference uses wrong values (§13); encodings re-derived for the 50 MHz XO | Step 3 (CW via partner RSSI) + Step 4 (clean link, sane RSSI) |
| RX BW / AFC_CORR scaling | 210 kHz is a worst-case guess; ch1 unusable until measured (§2.1) | Step 4 starts logging; Step 18 soak sweeps temp and narrows |
| L0 AES register sequencing | hand-written PAC poking: CCF timing, datatype byte-swap, key/IV order | Step 7 FIPS-197 KAT localizes to AES alone |
| CCM nonce uniqueness | whole security argument rests on no `(key,nonce)` repeat (§6) | Step 8 KAT+tamper for the primitive; Step 11 for the counter invariants; one `nonce_for` fn audited once |
| Counter persistence wear-ring | bug ⇒ counter reuse (security) or EEPROM wear | Step 11 power-cycle test: resumed counter ≥ last persisted |
| Duty governor accounting | rolling-hour, all-TX-counted, both sides; off-by-one ⇒ breach or false-limit | Step 12 over-budget test, cross-checked vs §2.6 ToA by hand |
| Half-duplex / hidden node | CSMA mitigates, can't eliminate (§4) | Step 5 (CSMA) + Step 18 (contention soak) |

---

## Critical Files

- `/Users/pavel/hardwario/embassy/RADIO.md` — the spec (§3 frame, §6 CCM/nonce/counter, §7.4
  persistence, §10 API, §12 parameters are authoritative).
- `/Users/pavel/hardwario/embassy/src/board.rs` — extend with radio pins (PB7/PA15/SPI1+PB3/PB4/PB5),
  PA7 nIRQ `ExtiInput`, PH0; the single integration point with `Board::take`/`app!` and the bound
  `EXTI4_15`.
- `/Users/pavel/hardwario/embassy/src/button.rs` — exact reuse template for `radio/driver.rs`
  (`init_exti` → `#[task] scan_task` → `static Channel` → cheap handle).
- `/Users/pavel/hardwario/embassy/src/storage.rs` — `Kv` (in-place same-size update; postcard
  `set`/`get`) backs `net/counter.rs` (watermark ring), `net/peers.rs` (id/key/last-seen), and the
  campaign tallies.
- `/Users/pavel/hardwario/embassy/src/lis2dh12.rs` — register-map-constants + `read_reg`/`write_reg`
  discipline to copy into `radio/{regs,spi,device}.rs`.
- `/Users/pavel/hardwario/embassy/src/power.rs` — `WakeGuard`/STOP model for Step 6 sleep gating.
- `/Users/pavel/hardwario/embassy/examples/{storage,button}.rs` — the persistence-loop and
  button+LED idioms the radio examples mirror.
- `/Users/pavel/hardwario/embassy/Cargo.toml`, `justfile` — feature flags + `--features` passthrough.
- New: `/Users/pavel/hardwario/embassy/src/radio/` subtree; `pub mod radio;` in `src/lib.rs`.

## Verification (how to run any step)

```sh
# build/flash with a role feature, then watch from boot
just flash <example> role-gateway -p /dev/cu.usbserial-11140    # Gateway
just flash <example> role-node     -p /dev/cu.usbserial-111140  # Node
jolt monitor --reset -p /dev/cu.usbserial-11140                 # observe (per board)
```

Crypto KAT steps (7, 8) and single-board steps (1, 2) need only one board. All `net_*`/`edge_*`
steps need both boards flashed and both monitors observed. The final `radio_interop` campaign runs
unattended; a latched LED or `VERDICT: FAIL` line is the failure signal.
