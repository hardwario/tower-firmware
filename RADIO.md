# TOWER Radio

A bi-directional sub-GHz radio stack for the TOWER ecosystem, built on the **SPIRIT1**
transceiver (ST, in the **SPSGRF** module) on the Core Module board.

This is the agreed specification: requirements, the hardware analysis grounding them,
and the resolved design decisions. Deployment caveats are in [§15](#15-open-items--caveats).
An implementation plan follows separately.

**Conventions.** All multi-byte wire and crypto fields are **little-endian**. *Node* = a
(often battery) endpoint; *gateway* = the always-listening hub (TOWER Radio Dongle);
*peer* = an endpoint in P2P. *Uplink* = node→gateway, *downlink* = gateway→node. A
*transfer* is one logical message (one packet, or a multi-chunk bulk); a *frame*/*packet*
is one over-the-air unit. Glossary: [§17](#17-glossary).

---

## 1. Hardware

### 1.1 Pin mapping (SPIRIT1 ↔ STM32L083CZ)

| SPIRIT1 | MCU pin | Notes |
|---|---|---|
| SDN | PB7 | 1 MΩ hardware pull-up → boots into SHUTDOWN; MCU must drive **low** to enable |
| SPI_CS | PA15 | **software-controlled CS** (≥2 µs setup) |
| SPI_SCLK / MOSI / MISO | PB3 / PB5 / PB4 | SPI ≤ 10 MHz |
| GPIO_0 | PA7 | **nIRQ** (active-low). EXTI line 7 → shares `EXTI4_15` (no line collision) |
| GPIO_1 | PH0 | spare status (EXTI line 0); optional 2nd IRQ |
| GPIO_2 / GPIO_3 | — | not connected |

### 1.2 SPIRIT1 key parameters (datasheet DS022758)

- **TX/RX FIFO: 96 bytes each** — the hard cap on one radio packet's payload (= our
  network frame; see §3).
- **SPI:** header `[A/C][R/W][6-bit addr/cmd]`; `MC_STATE` status returned on MISO every
  transaction. **`t_su(CS) ≥ 2 µs`** → software CS with a ≥2 µs delay after asserting.
- **Crystal:** 50 MHz (`SetXtalFrequency(50_000_000)`).
- **AES:** an on-chip AES block exists, but we use the **MCU's** AES (§6).
- **Power states:** SHUTDOWN→STANDBY→SLEEP(wake-timer)→READY→LOCK→RX/TX; SDN→READY ≈ 650 µs
  (+re-init), STANDBY/SLEEP→READY ≈ 125 µs, READY→RX/TX ≈ tens µs.
- **Device ID:** part 304, version 48 — verify at init.

---

## 2. RF configuration

| Parameter | Value | Notes |
|---|---|---|
| Bands / channels | **868 / 915 MHz** / **3** | §2.2 |
| Modulation / bit rate | **GFSK** BT 1 / **19 200 bps** | |
| Frequency deviation | **20 kHz** | h ≈ 2.1; tunable |
| Whitening | **on** | DC balance |
| Preamble | **4 bytes** (tunable, §2.7) | HW-generated |
| Sync word | **`0xDB624715`** | balanced, max run 3, low autocorrelation (§2.3) |
| CRC | **16-bit** `0x1021` | SPIRIT1 CRC mode 3 (§5) |
| RX bandwidth | **≈ 210 kHz** (bring-up); narrow after measurement | §2.1 |
| Crystal accuracy | **±40 ppm/unit** — *conservative; measure & narrow* | §2.1 |

### 2.1 Receiver bandwidth (the central RF trade-off)

±40 ppm crystal error → ±40 ppm carrier error; two units differ by **80 ppm** worst case:

| Band | 80 ppm offset | signal BW (Carson) | required RX filter ≈ signal + 2·offset |
|---|---|---|---|
| 868 MHz | 69.4 kHz | 59.2 kHz | **≈ 198 kHz** |
| 915 MHz | 73.2 kHz | 59.2 kHz | **≈ 206 kHz** |

→ nearest SPIRIT1 step **≥ ~210 kHz** for bring-up. AFC re-centers the residual, but the
analog filter must *pass* the offset signal, so ~210 kHz is required regardless of AFC.

**±40 ppm is a conservative assumption, not measured** (the SPSGRF is a 50 MHz crystal,
not a TCXO). **Measure it:** the SPIRIT1 reports each packet's offset in `AFC_CORR`. Bring
up wide (~210–300 kHz), log `AFC_CORR` between the real node and gateway (sweep
temperature), then **set RX BW to the measured worst case + margin**. A real ~±20 ppm part
→ ~130 kHz → ~2 dB more range. *Optional:* persist the per-peer offset in EEPROM and apply
it as a static correction to narrow further.

**Shipped default & channel impact.** The **bring-up default is 210 kHz**, at which the
200 kHz channel spacing (§2.2) overlaps — so until the BW is measured and narrowed to
≤ ~180 kHz, **only ch0 and ch2 (400 kHz apart) are simultaneously usable**; all three
become usable after narrowing. The "3 channels" headline assumes the narrowed operational
BW.

### 2.2 Channel plan, regulatory envelope & agreement

**Channels are tuned, not spanned.** The receiver sits on **one** channel; the RX BW is
crystal-drift tolerance *around that center*, not a window over all three. A receiver on
one channel does **not** hear the others — so **a node and its gateway must share one
channel.** The 3 channels exist to run co-located networks without interference, not for a
node to roam.

**Band** is region/config-time (EU→868, US→915), identical on gateway and all its nodes;
no OTA band negotiation. Channels follow `f = base + channel × spacing`. **Channel** is a
**configured default per network (`ch0`)**, shared and fixed at provisioning/pairing; **no
scanning/OTA negotiation in v1**. A **channel or band change re-runs VCO calibration**
(§8) and settles within the synth lock time.

**EU 868 — sub-band 868.0–868.6 MHz** (EN 300 220 / ERC 70-03 "g1": **≤ +14 dBm ERP,
≤ 1 % duty**, LBT+AFA may relax duty):
- `base 868.1 MHz`, `spacing 200 kHz` → **ch0 868.1 / ch1 868.3 / ch2 868.5**.
- Keep **ERP ≤ +14 dBm**: +11.6 dBm conducted allows ≤ ~2 dBi antenna, else reduce power.

**US 915 — 902–928 MHz ISM** (FCC Part 15): `base 915.0 MHz`, `spacing 300 kHz` →
**915.0 / 915.3 / 915.6**. ⚠️ **Provisional** — a fixed narrowband system at +11.6 dBm
needs an FCC strategy (§15.247 digital-mod wants ≥ 500 kHz BW; §15.249 caps power; FHSS
wants ≥ 25–50 channels). Resolve before US deployment.

**Duty cycle (EU)** is **per sub-band, per device** — and **all** TX counts (data, ACK,
bulk, retransmits, JOIN). The stack runs a **duty governor**: it tracks transmitted
airtime per sub-band over a rolling hour and defers/refuses a TX that would exceed the
limit (~36 s/h at 1 %). **The gateway is duty-governed too** (regulatory, independent of
its mains power). With ~45 ms max frames (§2.6), 1 % ≈ ~800 max frames/h/device; a busy
gateway answering 64 nodes must budget its ACK airtime accordingly. LBT+AFA (§2.5) may
relax the limit where permitted.

> **Regulatory note.** These are sensible engineering defaults; **final compliance —
> current EN 300 220 revision, FCC strategy, ERP with the real antenna, lab testing — is
> the integrator's responsibility before shipping.**

### 2.3 Sync word

**`0xDB624715`** — autocorrelation-searched: 16/16 balanced, max run 3, peak sidelobe 5
(near-optimal at length 32), starts `1101…` (first bit opposite the preamble's trailing
`0`, breaks the `1010…` pattern).

### 2.4 Output power

Default **+11.6 dBm** (module max), per-transfer. Use **PA ramping** (8-step table
1.0→11.6 dBm) to avoid the brown-out a hard step to max causes on a battery.

### 2.5 Listen-before-talk (CSMA/CCA)

CSMA precedes an **initiating** TX (data, bulk/downlink requests, JOIN) and runs *before*
keying the PA — it has no relation to the post-TX ACK window. SPIRIT1 CSMA/CA engine:
RSSI threshold **−90 dBm**, **CCA persistence ~one bit-window**, exponential backoff over
a small number of stages **bounded to ~100 ms** total before the max-backoff IRQ fires
(parameters in §12). **ACK replies are exempt** — sent immediately (at most one short
CCA), since the channel was just used by the frame they answer.

### 2.6 Time-on-air

`ToA = (preamble + sync + length + frame + CRC) bytes × 8 / bit-rate`. With 4 B preamble,
4 B sync, 1 B length, 2 B CRC at 19 200 bps:
- **Max non-bulk frame (96 B):** (4+4+1+96+2)·8/19 200 ≈ **44.6 ms**.
- **Typical ACK (~31 B frame):** (4+4+1+31+2)·8/19 200 ≈ **17.5 ms**.

So the 200 ms ACK window (§7.3) comfortably covers peer-processing + RX→TX turnaround +
~18 ms ACK ToA, and the duty budget (§2.2) is in max-frames/hour.

### 2.7 RX chain (AFC / AGC / IF / RSSI)

- **AFC** on, with **freeze-on-sync** so the offset estimate locks once the sync word is
  found. Loop/period parameters: §12.
- **AGC** enabled at defaults appropriate for the ~210 kHz BW (tune during bring-up).
- **IF / image rejection** set per the datasheet for the chosen BW.
- **RSSI offset calibrated** so the −90 dBm CSMA threshold and the logged per-packet RSSI
  are meaningful absolute values.
- **Preamble length** (4 B default) is a tunable; verify it covers AFC settling + clock
  recovery at low SNR / max range, and lengthen if needed (it trades against airtime/duty).

### 2.8 Signal quality (per reception)

**RSSI** (dBm, 0.5 dB), **LQI**, **SQI** (the SPIRIT1 has no direct SNR — LQI/SQI serve
that role). Exposed via `signal_quality()`; an application may use RSSI for adaptive TX
power. Link-decision thresholds are application policy (not fixed here).

---

## 3. Packet & framing

The SPIRIT1 **Basic** format adds preamble + 32-bit sync + 8-bit length + 16-bit CRC in
hardware (outside the FIFO). The **96-byte FIFO holds our entire network frame**:

| Field | Bytes | Protection | Purpose |
|---|---|---|---|
| `ver_type` | 1 | clear (AAD) | bits[7:5] version (=1), bits[4:0] frame type (§3.1) |
| `flags` | 1 | clear (AAD) | confirmed, downlink-pending, last-chunk (§3.1) |
| Source ID | 4 | clear (AAD) | 32-bit sender (§7.1) |
| Dest ID | 4 | clear (AAD) | 32-bit recipient |
| Counter | 4 | clear (AAD) | sender's monotonic counter; seeds the nonce (§6) |
| Bulk index | 3 | clear (AAD) | **bulk frames only**; chunk number |
| Encrypted payload | ≤ **74** (≤ 64 bulk) | encrypted | application data |
| CCM tag | 8 | tag | authenticates AAD + payload (§6) |

**Budget (FIFO = 96 B):** non-bulk `14 + 74 + 8 = 96`; bulk `17 + 64 + 8 = 89` (7 B spare).
The whole cleartext header is the **AAD**. A 3-bit **protocol version** (=1) lets the wire
format evolve; receivers drop unknown versions. The **`flags` bit takes precedence** over
the frame type where they overlap (e.g. a `BULK_DATA` frame's `last-chunk` flag is
authoritative for completion).

**Reserved IDs:** `0x00000000` = unassigned/none, `0xFFFFFFFF` = reserved (no broadcast in
v1). IDs must be unique within a network; collision handling is the provisioner's
responsibility.

**MTU policy:** `send()` of > 74 B of application data **without** the bulk flag is a
caller error (rejected) — there is no silent single-packet fragmentation; use a bulk
transfer (§7.5).

No SPIRIT1 hardware address filtering is used: the radio passes every CRC-valid packet up;
the **network layer** filters on dest ID and verifies the CCM tag.

### 3.1 Frame types & flags

| Type | Name | Purpose |
|---|---|---|
| 0 | `DATA` | application message (also announces a pending bulk, via the bulk flag + a length payload) |
| 1 | `ACK` | acknowledges a confirmed frame; payload carries the acked counter, downlink-pending + length, RSSI |
| 2 | `BULK_REQ` | request chunk `index` of a transfer (carries the transfer's counter/session id) |
| 3 | `BULK_DATA` | one chunk (≤ 64 B); `last-chunk` flag on the final one |
| 4 | `JOIN_REQ` | OTA pairing: joiner offers proposed ID, requests assignment (§7.6) |
| 5 | `JOIN_RESP` | pairing host: assigned ID + per-node key |
| 6 | `JOIN_CONFIRM` | joiner confirms receipt → both commit |
| 7–31 | reserved | |

**Flags:** `bit0` confirmed-requested, `bit1` downlink-pending (ACK), `bit2` last-chunk
(BULK_DATA), `bit3` bulk-announce (DATA), others reserved (0).

---

## 4. Operation model & concurrency

- **Single half-duplex resource.** The radio can't RX and TX at once and runs one
  operation at a time; it is owned by one driver task behind an async mutex, and the
  network layer serializes transfers through it. A device runs **one transfer at a time**.
- **Interrupt-driven:** nIRQ (PA7)→EXTI wakes the driver on TX-done, RX-ready, RX-timeout,
  CRC/FIFO error, CCA-max-backoff. No busy-polling; every wait is bounded.
- **States.** *Node:* `SLEEP → (wake) → CSMA → TX → RX(200 ms ACK window) → [pull?] →
  SLEEP`. *Gateway:* `RX(persistent) → TX(ACK/answer) → RX`.
- **Half-duplex deafness:** while the gateway transmits it can't hear an uplink → that
  uplink is lost and recovered by the sender's no-ACK retransmit (§7.3). With many nodes
  this is ALOHA-with-CSMA.
- **Capacity / hidden node:** CSMA mitigates but does not eliminate collisions, and it
  does **not** solve the hidden-node case (two nodes that can't hear each other both hit
  the gateway). Keep the aggregate offered load well under the channel airtime / duty
  budget (§2.2); confirmed retransmit absorbs the residual collisions. Rough budget:
  `N nodes × frame ToA × rate ≪ 1 % airtime`.
- **No broadcast/multicast** in v1 (the pull model can't reach sleeping nodes; a broadcast
  to awake nodes is possible future work — see LDC, §11).

---

## 5. Integrity (two layers)

- **Radio CRC (HW pre-filter):** 16-bit `0x1021` drops corrupted packets in hardware —
  cheap error detection, *not* security (CRC is linear; an attacker recomputes it).
- **Cryptographic integrity (AES-CCM):** forgery/tamper resistance via the CCM tag at the
  radio layer (§6), from the start.

---

## 6. Security (on-MCU AES-128-CCM)

Cryptography runs on the **STM32L083 hardware AES engine** (keys never leave the MCU),
using **AES-128-CCM** (NIST SP 800-38C; CCM = CTR + CBC-MAC, single key) — confidentiality
**and** integrity in one AEAD. *embassy-stm32 0.6.0 does not wrap the L0 AES (`aes` is
WBA-only, `cryp` is F2/F4/L4/H7), so we write a thin register-level L0 AES driver (`CR`,
`KEYRx`, `IVRx`, `DINR`, `DOUTR`; ECB block + CTR) and build CCM in firmware. Software
`aes`+`ccm` crates are the fallback.*

- **Per-node keys.** Each node has its own AES-128 key; the gateway holds one per node.
  **The same per-node key protects both directions** of that link.
- **AAD = the whole cleartext header**; **payload encrypted**; **8-byte (64-bit) tag**.

**Counter & nonce — the uniqueness rule (this is the crux):**
- Each device has **one monotonic 32-bit TX counter**, advanced by **one per transfer**
  (a single packet, an ACK, a `BULK_REQ`, or an entire bulk-DATA transfer each consume one
  value). A **retransmission re-sends the byte-identical frame** with the same counter —
  safe, because the ciphertext is identical (no new information).
- **Nonce (13 B, not transmitted, reconstructed from the clear header):**
  `src_id[4] ‖ counter[4] ‖ bulk_index[3] ‖ 0x0000` (LE; CCM N=13, L=2). `bulk_index = 0`
  for non-bulk frames; for bulk-DATA it is the chunk number (all chunks of a transfer
  share the one transfer counter, so the index keeps their nonces distinct).
- **Why it's unique per (key, frame):** the key is per-node; `src_id` fixes the sender, so
  the two directions never collide even at equal counter values (no `dir` field needed);
  the counter is fresh per transfer; the bulk index separates chunks; retransmits are
  byte-identical. **An ACK therefore uses the ACKer's own fresh counter** (the
  *acknowledged* counter is carried in the ACK *payload*, never as the nonce counter) — so
  an ACK and the frame it answers never share a nonce.

**Replay (§7.4) & receive order.** **CCM-verify first** (authenticates the header incl. the
counter, decrypts only on a valid tag), **then** compare the counter to the per-peer
last-seen and update it. A forged high counter can't poison replay state, and
forged/tampered frames are dropped before the network layer acts.

**Initialization & re-key.** On key install (join or re-key), the TX counter starts at
**1** (`0` = "never sent") and the receiver's per-peer last-seen starts at **0** (accept
iff counter > last-seen → first accepted frame is counter 1). **A re-key resets both
counters to their initial values** — safe because a new key is a disjoint nonce space, and
old captured ciphertext can't be replayed under the new key.

**Key storage** in EEPROM is only as safe as the chip's readout protection — enable flash
**RDP** for production.

---

## 7. Network layer

Layered: a **radio layer** (SPIRIT1 driver — config, TX, RX, CSMA, FIFO, IRQ, quality,
AFC, calibration) and a **network layer** (addressing, topologies, confirmed delivery,
security, replay, bulk, pairing). Single-packet traffic carries none of the bulk overhead.

### 7.1 Addressing / identity

A **32-bit unique ID, supplied at init** (derivation is the caller's concern). It rides in
the clear (AAD) header; the network layer filters/routes on it. See reserved IDs (§3).

### 7.2 Topologies

**Star:** gateway always listens, holds ≤ **64 nodes**; a node sleeps or receives.
**Downlink to a sleeping node is node-initiated (pull):** the node's uplink is ACKed, the
**ACK carries downlink-pending + length**, and the node then pulls the downlink
chunk-by-chunk (§7.5). **Downlink latency is therefore bounded by the node's uplink
cadence** — for responsive downlink, nodes should uplink/poll at a configured minimum
interval.

**P2P:** ≤ **8 peers**; **a sleeping peer cannot initiate to a sleeping peer** (one side
must listen; no async wake-up).

### 7.3 Confirmed / unconfirmed delivery & ACK

- **Timing.** Sender CSMAs (≤100 ms), transmits, then opens a **200 ms** ACK window. On
  timeout it waits a **random 0–100 ms** (de-syncs collided senders) and retransmits;
  repetitions **1–10, default 3**. The *first* attempt has no inter-rep backoff; with ACK
  window `W=200`, random `R≤100`, CSMA `C≤100`, ToA `T≈45`:
  `latency ≤ (C+T+W) + (N−1)·(R+C+T+W)` → **≈ 1.06 s** at N=3, **≈ 3.55 s** at N=10
  (worst case). After the last failed repeat the transfer reports **not-delivered**.
- The receiver ACKs **within 200 ms** — ample: peer-process + RX→TX + ~18 ms ACK ToA
  (ACKs skip the full CSMA, §2.5).
- **ACK** (`ACK` frame, CCM with the per-node key, its own fresh counter) carries: the
  **acknowledged counter**, **downlink-pending + pending length** (star), and **RSSI**.
- **Retransmit vs replay.** Counter `>` last-seen → new (accept). Counter `==` last-seen of
  the most recent confirmed frame → benign **retransmit**: re-send the **cached identical
  ACK** (same bytes, safe) and do not re-deliver. Counter `<` last-seen → **replay**: drop
  silently. (The initiator accepts an incoming ACK that CCM-verifies and whose
  *acknowledged counter* matches its outstanding transfer.)

### 7.4 Replay protection & counter persistence

A receiver accepts a transfer iff its counter `>` the **per-peer last-seen** for that
sender, then updates it (within a bulk transfer the counter is constant — accept once,
track chunks by index). State lives in the **data EEPROM** (byte-writable, ~100k
cycles/cell — `storage::Kv`). State per side:

- **Each device:** one TX counter + its **reserve-ahead watermark**.
- **Gateway:** per node, the node's ID, key, and **last-seen** (uplink).
- **Node:** the gateway's **last-seen** (downlink).

Persistence:
- **Sender — reserve-ahead.** Live counter in RAM; persist only a reservation watermark.
  On boot resume **at** the watermark (> any value actually sent → never reused; ≤ one
  block skipped). Advance the watermark by `RESERVE` (default **1024**) and persist once
  when reached. Two distinct limits: **counter wrap = 2³² transfers** (centuries at real
  rates; hard-stop + re-key at `2³²−1` to avoid nonce reuse); **watermark-cell wear ≈
  100k × RESERVE ≈ 10⁸ transfers** *for that one cell* — the watermark uses a **small
  rotating ring** so a high-rate node (e.g. 1 Hz) doesn't wear one cell out within its
  service life.
- **Receiver last-seen.** A **mains gateway** caches in RAM and persists **every `P`
  accepted transfers** (default `P = 32`) → the **replay window across a (rare) reboot is
  ≤ P transfers**, an explicit, bounded exposure. A **battery peer** spreads writes across
  a **rotating ring** per sender for a zero-window guarantee.

**EEPROM budget (gateway):** 64 × (ID 4 + key 16 + last-seen 4) + watermark ring ≈
**~1.5–2 KB of 6 KB**.

### 7.5 Bulk transfers & downlink pull (one pull-based mechanism)

The **receiver pulls; the sender announces.** The transfer's **session id is its counter**.

1. **Announce.** The sender advertises a transfer's **total length** (≤ 16 MiB): on the
   uplink **ACK's** downlink-pending length (star downlink), or a `DATA` frame with the
   **bulk-announce** flag (P2P / bulk uplink).
2. **Pull.** The requester sends `BULK_REQ(session=counter, index)` (confirmed); the sender
   replies `BULK_DATA(index, ≤64 B)`. **One request outstanding at a time** (no windowing —
   simple; bulk is rare). The confirmed mechanism retransmits a lost request/response; the
   requester re-requests a missing index. `BULK_DATA` is the *response* to `BULK_REQ` (not
   separately ACKed).
3. **Complete.** The final chunk sets `last-chunk`; the requester stops once it holds all
   chunks (count = ⌈length / 64⌉).
- 16 MiB / 64 B = **262 144 chunks** → **24-bit** index. **16 MiB is a protocol-field
  limit, not a buffer:** neither side buffers the whole transfer — the application supplies
  (sender) / consumes (requester) chunks on demand via a streaming source/sink. The radio
  stack holds only the current chunk.
- **Sessions & recovery.** The sender holds per-transfer state until completion or a
  **bulk-idle timeout** (default **30 s** with no progress), then frees it. A **requester
  reboot** → the sender times out and frees; the requester restarts the pull on the next
  announcement. A node may run only one bulk transfer at a time (§4).

### 7.6 Provisioning & pairing

A node needs its 32-bit ID + AES key; the gateway needs every node's (ID, key).

- **Programmatic API (now):** set local identity/key; add/remove a peer's (ID, key) on the
  gateway. A higher-level config interface drives these later.
- **Over-the-air pairing (now, via API):** the host opens a **pairing window** (bounded
  timeout); a **3-way handshake** follows — `JOIN_REQ` (joiner → proposed ID) → `JOIN_RESP`
  (host → assigned ID + per-node key) → `JOIN_CONFIRM` (joiner). **Both sides commit only
  after the confirm** (no half-paired state; a lost confirm → the host's window times out
  and it discards the tentative entry, the joiner retries). On commit, counters/last-seen
  initialize per §6. If **multiple joiners** answer in one window, the host pairs the
  first valid `JOIN_REQ` and ignores others until re-opened.
- **Security (be honest):** pairing frames are protected by CCM under a **fixed,
  publicly-known pairing key** — this gives a uniform frame format with integrity and
  in-session replay protection, but **no confidentiality** (the key is public, so a passive
  sniffer in range during the window recovers the delivered per-node key) and **no mutual
  authentication** (anyone in-window can answer or join). Mitigations: short window,
  proximity, reduced TX power, user-initiated. A real upgrade (an out-of-band install-code
  KEK that AES-wraps the delivered key, or ECDH key agreement) is future work.

### 7.7 Example exchanges

```
(1) Confirmed uplink, ACK OK
  Node ──DATA(cnt=Cn, confirmed)──▶ Gateway
  Node ◀──ACK(ack=Cn, dl=0)─────── Gateway        (within 200 ms)

(2) Confirmed uplink, ACK lost → retransmit
  Node ──DATA(cnt=Cn)──▶ Gateway   (gateway ACKs, ACK lost)
  Node  ...200 ms, no ACK; wait rand 0–100 ms...
  Node ──DATA(cnt=Cn)──▶ Gateway   (same counter = identical frame)
  Gateway: cnt==last-seen ⇒ retransmit ⇒ re-send cached ACK
  Node ◀──ACK(ack=Cn)───── Gateway

(3) Downlink pull (star)
  Node ──DATA(cnt=Cn)────────▶ Gateway
  Node ◀──ACK(ack=Cn, dl-pending, len=L)── Gateway
  Node ──BULK_REQ(session=Cg, index=0)──▶ Gateway
  Node ◀──BULK_DATA(index=0, 64B)──────── Gateway
        ... index 1 … k …
  Node ──BULK_REQ(index=k)──▶ Gateway
  Node ◀──BULK_DATA(index=k, last-chunk)── Gateway   (done)

(4) Pairing (3-way, under the public pairing key)
  Joiner ──JOIN_REQ(proposed_id)──▶ Host   (window open)
  Joiner ◀──JOIN_RESP(assigned_id, key)── Host
  Joiner ──JOIN_CONFIRM(assigned_id)──▶ Host   (both commit)
```

---

## 8. Initialization sequence

1. Drive **SDN low** (PB7), wait SDN→READY, exit shutdown.
2. **Verify device ID** (304 / 48); abort on mismatch.
3. Set **XTAL 50 MHz**; apply the ST management work-arounds (extra-current WA, and the
   datasheet's SPI/state errata — enumerate against the current datasheet at implementation).
4. **RF config:** band + channel (→ **SetFrequencyBase**, then **VCO calibration** +
   **RCO calibration**), GFSK BT1, 19 200 bps, fdev 20 kHz, **RX BW ~210 kHz**, AFC on
   (freeze-on-sync), AGC on, IF/image set, RSSI offset calibrated, whitening on, sync
   `0xDB624715`, 16-bit CRC, PA table + ramping, CSMA (−90 dBm, ≤100 ms).
5. Configure **GPIO0 = nIRQ**, bind EXTI; enable RSSI/LQI/SQI.
6. Install **identity + keys** (init args / EEPROM); init counters/last-seen (§6).
7. Enter the role's default state (node → SLEEP; gateway → persistent RX).

**Channel/band change** repeats step 4's frequency set + **VCO calibration** and waits the
synth lock.

---

## 9. Error handling & failure modes

| Condition | Detection | Behavior |
|---|---|---|
| No ACK after N repeats | 200 ms × N | report **not-delivered** |
| Channel busy | CCA max-backoff IRQ | report **busy**; caller may retry |
| Duty budget exceeded (EU) | airtime governor (§2.2) | defer/refuse TX; report **duty-limited** |
| RX timeout | RX-timeout IRQ | report **timeout** |
| Bad CRC | HW drop | invisible (feeds link-quality stats) |
| **CCM auth fail** | tag mismatch | **drop; do not touch replay state** |
| Replay (counter < last-seen) | post-verify | drop silently |
| Retransmit (counter == last-seen) | post-verify | re-send cached ACK; no re-deliver |
| FIFO over/underflow | FIFO-error IRQ | flush FIFO, abort op, return to READY/RX |
| Frame > 96 B (TX) | length check | reject (caller error) |
| App data > 74 B, no bulk flag | `send()` check | reject (caller error; use bulk) |
| Unknown version/type | header parse | drop |
| Bulk idle | 30 s no progress | sender frees session; requester reports failure |
| Stuck state | `MC_STATE` poll w/ timeout (≈ a few ms) | force SABORT→READY; re-init on repeat |

---

## 10. API surface (sketch)

**Radio layer:** `init()`, `configure(band, channel, power)`, `tx(frame, use_csma)`,
`rx(buf, timeout) -> (len, rssi, lqi, sqi)`, `set_state(sleep|standby|ready|rx)`,
`read_afc_hz()`, `cw_test(on)`.

**Network layer:** `init(my_id, my_key)`; `add_peer(id, key)` / `remove_peer(id)`;
`open_pairing(timeout)` / `close_pairing()` / `join(target)`;
`send(dest, data, {confirmed, repetitions=3, power})` → `Delivered | NotDelivered | Busy |
DutyLimited`; `recv() -> {src, data, quality}`; `bulk_send(dest, source_stream)` /
`bulk_recv() -> sink_stream`; `poll_downlink()`; `signal_quality()`.

---

## 11. Low power

- **Node:** sleep between transfers — SPIRIT1 **SLEEP** (wake-timer, ~125 µs to READY,
  config retained) for short cadences, **SHUTDOWN** (lowest, ~650 µs + re-init) for long
  idle. The SPIRIT1 wake-timer runs off an imprecise RC oscillator, so scheduled wakeups
  drift — fine for "poll every N seconds," not for tight rendezvous. The MCU uses the SDK's
  STOP-mode executor; nIRQ (PA7) is an EXTI wake source.
- **Gateway:** never sleeps (persistent RX); mains/USB-powered, kept awake by the SDK.
- **Not used: LDC/Wake-on-Radio.** The SPIRIT1's low-duty-cycle RX (periodic sniff) *could*
  let a sleeping node be reached asynchronously, removing the pull requirement — but it
  costs a long wake-up preamble on every downlink (airtime/duty) and steady sniff current,
  and complicates timing. **v1 uses pull instead**; LDC is a candidate for a future
  downlink-latency-sensitive mode.

---

## 12. Parameters reference

| Constant | Value |
|---|---|
| Bands / channels | 868 & 915 MHz / 3 |
| EU 868 channels | 868.1 / 868.3 / 868.5 MHz (200 kHz; g1 ≤+14 dBm ERP, ≤1 % duty) |
| US 915 channels | 915.0 / 915.3 / 915.6 MHz (300 kHz; FCC strategy pending) |
| Bit rate / deviation / BT | 19 200 bps / 20 kHz / 1 |
| RX bandwidth | ≈ 210 kHz bring-up → narrow via `AFC_CORR` |
| Sync / CRC / preamble | `0xDB624715` / 16-bit `0x1021` / 4 B |
| FIFO = network frame | 96 B |
| Header (non-bulk / bulk) / tag | 14 B / 17 B / 8 B |
| Payload (single / bulk chunk) | ≤ 74 B / 64 B |
| Nonce | 13 B (`src ‖ counter ‖ bulk_index ‖ 0x0000`) |
| ToA (max frame / ACK) | ≈ 44.6 ms / ≈ 17.5 ms |
| TX power (default/max) | +11.6 dBm |
| CSMA RSSI / max backoff | −90 dBm / ~100 ms |
| ACK window / inter-rep backoff | 200 ms / random 0–100 ms |
| Repetitions (range/default) | 1–10 / 3 |
| Confirmed latency (N=3 / N=10) | ≈ 1.06 s / ≈ 3.55 s worst case |
| Replay counter | 32-bit; start 1; hard-stop + re-key at 2³²−1 |
| Sender reserve block `RESERVE` | 1024 (watermark in a wear ring) |
| Gateway lazy-persist `P` | 32 transfers (≤ P replay window across reboot) |
| Max bulk / chunk index | 16 MiB (streamed) / 24-bit |
| Bulk idle timeout | 30 s |
| Star nodes / P2P peers | ≤ 64 / ≤ 8 |
| Reserved IDs | `0x00000000`, `0xFFFFFFFF` |
| Protocol version | 1 |
| Duty cycle (EU) | ≤ 1 % per sub-band per device (governed) |

---

## 13. Reference implementation notes

C reference `~/Downloads/spirit1/spirit1.c` — guidance only, **do not copy blindly**.
Apply: 2 µs software CS; 16-bit (not 8-bit) CRC; sync `0xDB624715` (not `0x88888888`);
~210 kHz RX BW (not 100 kHz); channel spacing ≥ RX BW (not 20 kHz); MCU-side AES (the
reference doesn't encrypt). Keep: device-ID check, XO 50 MHz, GPIO0=nIRQ, PA ramping,
CSMA approach, AFC read-back, RSSI/LQI/SQI, shutdown→exit-shutdown init.

---

## 14. Testing

Test on real hardware across: bands, channels, confirmed/unconfirmed, power levels,
star + P2P, single-packet + bulk (incl. requester reboot mid-pull), sleep/wake cycles,
CSMA contention + hidden-node, range/sensitivity (CW + `AFC_CORR` sweep over temperature),
pairing (incl. lost confirm, two joiners), replay rejection, counter persistence across
power loss, and duty-governor enforcement.

- Node: `/dev/cu.usbserial-111140` · Gateway: `/dev/cu.usbserial-11140`

---

## 15. Open items & caveats

**No design items remain open.** Two deployment caveats, each flagged in place:

- **Regulatory certification** (§2.2) — the channel plan is an engineering default;
  confirm against the current EN 300 220 revision + an FCC strategy (US narrowband is
  provisional), and certify ERP with the real antenna by lab test.
- **OTA-pairing security** (§7.6) — the delivered key is sniffable in-window and pairing
  isn't mutually authenticated; acceptable for time-boxed user-initiated setup. Upgrade
  path: install-code KEK or ECDH.

---

## 16. Resources

- SPIRIT1 datasheet `~/Downloads/spirit1/spirit1.pdf` (DS022758)
- C reference `~/Downloads/spirit1/spirit1.c`
- SPSGRF module datasheet <https://www.st.com/resource/en/datasheet/spsgrf.pdf>

---

## 17. Glossary

**Gateway** always-listening hub (≤64 node keys). **Node** sleeping endpoint. **Peer** P2P
endpoint (≤8). **Uplink/downlink** node→gw / gw→node. **Transfer** one logical message;
**frame** one over-the-air unit. **AAD** cleartext header covered by the CCM tag.
**Nonce** per-frame CCM input (derived, not sent). **AFC** automatic frequency correction.
**CSMA/LBT/CCA** carrier-sense / listen-before-talk / clear-channel assessment.
**RSSI/LQI/SQI** signal strength / link quality / sync quality. **LDC/WoR** low-duty-cycle
/ wake-on-radio periodic RX sniff. **KEK** key-encryption key. **Session (bulk)** the
transfer counter identifying a bulk transfer.
