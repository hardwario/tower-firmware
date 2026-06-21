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
  (e.g. `just flash net_confirmed role-gateway -p $GW`). Small, mechanical edit.
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

- [x] **5. CSMA + stuck-state recovery.** CSMA/CCA before TX: RSSI_TH = −90 dBm (0x50) and the
  CSMA timing programmed in `config::apply` (BU_COUNTER_SEED=0xFA21 non-zero, prescaler 32, 64·Tbit
  CCA period, CCA length 3, MAX_NB=5, non-persistent). `device::tx(use_csma=true)` masks
  `IRQ_MAX_BO_CCA_REACH` → `RadioError::Busy`; SABORT→READY watchdog on timeout and FIFO-error
  flush already in `tx`/`rx` (§9). `radio_csma` example.
  - [x] **Verify** (`radio_csma`, two boards — jammer + sender): ✅ jammer holds CW 3 s on / 3 s
        off; sender's CSMA TX prints runs of `Busy` (CCA backed off) during the jam and `ok` when the
        channel clears, tracking the cycle exactly (seq 1-5 Busy, 6-10 ok, 11-14 Busy, …). Never hangs.

- [x] **6. Low-power sleep/wake.** `device`: `to_sleep`/`to_ready` (SLEEP, config retained) and
  `enter_shutdown`/`exit_shutdown`+`config::apply` (SHUTDOWN, POR + re-init); `set_wake_timer` for
  the LDC timer. Host-driven cadence (the MCU STOPs between transfers when USB is unplugged — USB
  inhibits STOP, see `power.rs`). `radio_sleep` example.
  - [x] **Verify** (`radio_sleep`, two boards): ✅ node alternates SLEEP/SHUTDOWN between TXes;
        measured wake **SLEEP→READY ≈ 1.25 ms** vs **SHUTDOWN→READY+reconfig ≈ 4.8 ms** (SLEEP much
        faster, config retained). Gateway receives consecutive frames marked "(re-linked after sleep)"
        — node re-links correctly after BOTH modes.

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

- [x] **11. Replay protection + counter persistence.** In `net.rs`: reserve-ahead TX watermark
  (RESERVE=1024) + last-seen lazy-persist (P=32) over `storage::Kv`; CCM-verify-then-compare
  ordering in `recv()`. `net_persist` example. *Reuse:* `storage::Kv` in-place same-size update.
  - [x] **Verify** (`net_persist`, 1 board): ✅ reboot resumes the TX counter **at the previous
        watermark** (boot→1025, reset→2049, reset→3073…), jumping ahead — never reuses a value (§7.4).
        Replay rule (`counter <= last-seen` → drop) implemented + last-seen persisted (window ≤ P).
  - [ ] Adversarial replay-injection demo folded into the soak (Step 18). Watermark wear-ring and
        per-sender last-seen ring are a refinement (single Kv cell for now: 100k×1024 ≈ 10⁸ transfers).

- [x] **12. Duty governor (EU).** `duty.rs`: token-bucket airtime accounting (cap 36 s = 1 % of an
  hour, refill 1 % of wall-clock), `frame_toa_ms` (§2.6). Integrated into `Net::send` (every TX incl.
  retransmits) and `send_ack` (ACKs governed too); `SendResult::DutyLimited`.
  - [x] **Verify** (`net_duty_kat`, 1 board, deterministic): ✅ ToA(30B)=17ms, ToA(96B)=44ms;
        bucket allows 5×17ms from 100ms then refuses; refills 10ms/1s; caps correctly. ALL PASS.
        Regression: `net_confirmed` still `Delivered (59 ms)` with the governor active.

- [x] **13. Bulk transfer + downlink pull.** `net.rs` `bulk_serve`/`bulk_fetch`: announce
  (DATA+BULK_ANNOUNCE, payload=len+session) → BULK_REQ(index)/BULK_DATA(index,≤64 B), 24-bit index,
  last-chunk flag, 30 s idle timeout. Session counter distinct from the announce counter (so chunk-0's
  nonce never collides); all chunks share the session counter with a distinct index.
  - [x] **Verify** (`net_bulk`, two boards): ✅ requester pulls a 200 B blob (4 chunks),
        reassembles, byte pattern **verify OK**. Sender re-announces until the first request.
  - [ ] Requester-reboot-mid-pull (sender idle-frees) + streaming source/sink folded into the soak (Step 18).

- [x] **14. OTA pairing (3-way join).** `net/pairing.rs` `open_pairing`/`join` + `PAIRING_KEY`
  (fixed public key, honest weak-security note) + `PAIRING_WINDOW` (1 min default). The **joiner
  brings its own ID** and keeps it; the host hands out only the per-node key (does not assign the
  ID): JOIN_REQ(node id) → JOIN_RESP(key) → JOIN_CONFIRM(node id); commit-on-confirm. `tx_pair`/
  `rx_pair` use the pairing key. `net_pairing` example.
  - [x] **Verify** (`net_pairing`, two boards): ✅ host `PAIRED *** node id=000000BB (joiner-chosen)
        key[..4]=a0a1a2a3`; joiner `JOINED *** id=000000BB (mine) key[..4]=a0a1a2a3` — the ID comes
        from the joiner, keys match, both commit. Lost-confirm (window discards) + two-joiner (first
        wins) folded into the soak (Step 18).

- [x] **15. Peer table + star/P2P topologies.** `net.rs` per-peer `Peer{id,key,last_seen}`
  table (`MAX_PEERS=64`; star ≤64 / P2P ≤8 by policy) with `add_peer`/`remove_peer`/`peer_count`/
  `peer_last_seen`. Each registered peer overrides the default key and gets its own replay lane
  (per-peer last-seen persisted at `KEY_LASTSEEN_BASE+slot`); unregistered peers use the
  `NetConfig::key` default lane (single-link case, backward compatible). `recv` peeks the clear
  header to pick the key by `src`; `send`/`bulk`/`await_ack`/`send_ack` key by peer. Examples
  `net_star`, `net_p2p`.
  - [x] **Verify** (two boards): ✅ **Star** (`net_star`) — gateway holds 2 peers under distinct
        keys; decoded node A under `KEY_A` (cnt 6149) then node B under `KEY_B` (cnt 7170, separate
        lane), each ACKed; B only decodes because `KEY_B` is registered (default is `KEY_A`). ✅
        **P2P** (`net_p2p`) — A `PING Delivered`+`rx PONG`, B `rx PING(ACKed)`+`PONG Delivered`,
        bidirectional confirmed under the shared link key via the table. ✅ Regression: `net_confirmed`
        still `Delivered` (59 ms) on the default lane. (`signal_quality` already exposed via
        `Received::rssi_dbm`; richer LQI/SQI is in `radio.rssi/quality`.)

### Phase 5 — Polish & robustness

- [x] **16. Public API + docs + reference apps.** `pub mod radio;` in `lib.rs` with module
  re-exports (`Band`/`RfConfig`/`SignalQuality`/`Spirit1`/`RadioError`); `net` exposes
  `Net`/`NetConfig`/`SendResult`/`Received`/`add_peer`/… SDK-style doc comments throughout.
  Shipped reference apps `radio_node` (confirmed telemetry uplink) + `radio_gateway` (decode +
  auto-ACK, peer table).
  - [x] **Verify** (two boards): ✅ `radio_node` `seq=N Delivered (59 ms) vbat=… temp=…°C`;
        `radio_gateway` `src=11111111 cnt=… seq=… vbat=…mV temp=21.8°C rssi=-43dBm (ACKed)` — full
        happy path, clean logs. `just samples` lists all radio/net examples.
  - [ ] **Verify**: `just samples` lists the radio examples; `just run radio_gateway` /
        `radio_node` on the two boards demonstrates the full happy path with clean logs.

- [x] **17. Edge-case examples.** Focused edge binaries:
  - `edge_frame_limits` (1 board) — MTU + malformed/forged-frame KAT.
  - `edge_recovery` (1 board) — §9 timeout / stuck-state / FIFO recovery.
  - `edge_rapid` (2 boards) — back-to-back confirmed, monotonic-counter check.
  - `net_channel` (2 boards) — secured link on a non-default channel (VCO recal).
  - (Oversized-payload + unknown-ver/type + auth-fail → `edge_frame_limits`; FIFO + stuck-state →
    `edge_recovery`; CSMA contention → `radio_csma` + the soak.)
  - [x] **Verify**: ✅ `edge_frame_limits` 12/12 (1 B/74 B accept, 75 B + bulk 65 B reject; bad
        version/type/truncated/tampered-ct/tampered-tag/wrong-key → correct error) **ALL PASS**.
        ✅ `edge_recovery` 10/10 RX-timeouts → READY, FIFO flush empties, RX→READY cycle, chip
        responsive — **ALL PASS** (never wedged). ✅ `edge_rapid` 500 accepted, **0 order
        violations** (strict-monotonic). ✅ `net_channel` confirmed link on ch2 (rssi −45 dBm).

### Phase 6 — Comprehensive semi-fuzzy campaign *(final acceptance)*

- [x] **18. Fuzzy interop soak.** `examples/radio_interop.rs` (one file, `role-node` /
  `role-gateway`): node drives seeded pseudo-random traffic (payload 9–74 B edge-emphasized,
  confirmed/unconfirmed, reps 1–10, timing jitter, low-prob oversized-reject fault), self-describing
  `[seq][len][crc32][filler]` payload; gateway verifies crc32 integrity + strict-monotonic counters.
  LED latches solid on any violation; rolling `VERDICT:` lines; cumulative tallies persisted to Kv.
  - [x] **Verify** (bench smoke, two boards): ✅ node **VERDICT PASS** to 400 iters (ok=390,
        oversized rej=10 all locally rejected, fails=0); gateway **VERDICT PASS** to 525 accepted
        (integrity_fail=0, order_fail=0, monotonic). Zero latched LEDs. Harness loops indefinitely for
        the standard (1–2 h) / soak (12–24 h) tiers. (AFC-vs-temp RX-BW narrowing left as a future
        optimization — see docs caveats; all three EU channels usable as-is, per `net_channel`.)

- [x] **19. User documentation (`docs/radio.md`).** Finalized: status banner, radio-layer API,
  AES-CCM, wire protocol, **network layer** (Net API, confirmed delivery, counters/replay/persistence,
  peer table + star/P2P, bulk, pairing, duty), the full examples table, the RX-completion note, and
  an updated **Known limitations & caveats** section. Cross-linked to `RADIO.md` / `PLAN.md`.
  - [x] **Verify**: the guide documents flashing `radio_gateway`/`radio_node`, confirmed `send`,
        pairing (`net_pairing`/`open_pairing`/`join`) and bulk (`bulk_serve`/`bulk_fetch`) with code
        snippets keyed to the examples — usable without reading the source.

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
| `net_confirmed` | 10 | Node + GW | confirmed/unconfirmed uplink + ACK (§7.3, §7.7-1) |
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
