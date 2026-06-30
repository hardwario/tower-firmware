# TOWER Radio — user guide

A sub-GHz radio stack for the TOWER Core Module, built on the **SPIRIT1**
transceiver (in the SPSGRF module). This is the standalone reference for using and
maintaining the stack — API, wire protocol, the design rationale behind the RF and
security parameters, and a worked example per feature.

> **Status: complete and hardware-verified.** The full stack is implemented and
> tested on two boards: the radio layer (SPI, power states, RF config, CW, TX/RX
> with quality metrics, CSMA/CCA, SLEEP/SHUTDOWN), on-MCU **AES-128-CCM**, the
> frame codec, and the network layer — confirmed delivery + ACK/retransmit,
> replay protection + counter persistence, the EU duty governor, bulk pull,
> OTA pairing, and per-peer keys (star / P2P). A semi-fuzzy soak (`radio_interop`)
> exercises it all under randomized traffic with latched invariant checks.

## Hardware

SPIRIT1 ↔ STM32L083CZ wiring (from the board, see `src/board.rs`):

| Signal | Pin | Notes |
|---|---|---|
| SDN | PB7 | drive low to enable (hardware pull-up → boots in shutdown) |
| SPI CS | PA15 | software CS, ≥2 µs setup |
| SCLK / MOSI / MISO | PB3 / PB5 / PB4 | SPI1, mode 0, 4 MHz |
| nIRQ | PA7 | active-low, EXTI line 7 |

Crystal is **50 MHz** (not a TCXO). Band is runtime-selectable via the `Band` abstraction:
**EU 868** (ch0/1/2 = 868.1/868.3/868.5 MHz, 1 % duty) and **US 915** (ch0/1/2 =
915.0/915.2/915.4 MHz). 915 single-channel is bench-only; FCC §15.247 compliance comes from
the FHSS mode (see *Spectrum-access modes* below).

## Building & flashing

Examples live in `examples/`. Flash with the UART bootloader and watch the console:

```sh
just flash example <name>                 # build + flash (auto-detect port)
TOWER_PORT=/dev/cu.usbserial-XXXX just flash example <name>   # pick a board
tower logs                 # watch from boot

# two-board examples select a role via a Cargo feature:
TOWER_FEATURES=role-gateway just flash example <name>   # one board
TOWER_FEATURES=role-node    just flash example <name>   # the other
```

## Radio layer API (`tower::radio`)

The `Spirit1` handle owns the SPI bus, the SDN pin and the nIRQ line:

```rust
use tower::radio::{Spirit1, RfConfig, config};

let mut radio = Spirit1::new(
    b.radio_spi, b.radio_sck, b.radio_mosi, b.radio_miso,
    b.radio_cs, b.radio_sdn, b.radio_irq,
);
radio.exit_shutdown().await?;          // SDN low, wait for READY
radio.read_device_id()?;               // verify part 304 / version 48
config::apply(&mut radio, &RfConfig { band: config::Band::Eu868, channel: 0 }).await?;

// raw frames (≤96 B), nIRQ-driven, with signal quality:
radio.tx(&bytes, /*use_csma=*/ false, Duration::from_millis(200)).await?;
let (len, q) = radio.rx(&mut buf, Duration::from_secs(5)).await?;
//   q: rssi_dbm, rssi_raw, lqi (PQI), sqi, afc_raw

radio.cw_test(true).await?;            // unmodulated carrier (bring-up / range)
radio.to_standby().await?;             // low-power states: to_ready/to_standby/to_sleep
let raw = radio.rssi_sample().await?;  // on-demand channel RSSI  (dBm = raw/2 - 130)

// Retune to another band/channel at runtime (one binary runs either band):
config::set_band(&mut radio, config::Band::Us915, 0).await?;   // raw layer
// or, on the network layer (also moves the duty policy): net.set_band(band, ch)
```

**Bands.** `Band::Eu868` (default) and `Band::Us915` both lie in the SPIRIT1 high
VCO band, so they share every setting except base frequency + channel spacing; the
band is a **runtime** choice — pass it to `Net::new`/`config::apply`, or switch a
live radio with [`Net::set_band`]/`config::set_band` (rewrites only the synth
registers; the VCO auto-recalibrates on the next TX/RX). **915 MHz is for bench
testing only** — this ~216 kHz narrowband signal is not FCC 15.247-compliant
(which needs FHSS or ≥500 kHz wideband). See `radio_band` for a live-switch demo.

RF configuration (`config::apply`) programs the 50 MHz-crystal-specific setup
(REFDIV, IF offsets, SYNT/WCP, GFSK 19.2 kbps, 20 kHz deviation, ~216 kHz RX
filter, sync `0xDB624715`, 16-bit CRC, PA ramp, AFC freeze-on-sync, CSMA timing)
plus the ST management work-arounds (per-mode SMPS, VCO current). VCO calibration
is left to the part's automatic per-channel calibration (more temperature-robust
than a one-time manual cal). See `src/radio/config.rs`.

## Security: AES-128-CCM (`tower::radio::{aes, ccm}`)

Cryptography runs on the **STM32L0 hardware AES engine** (keys never leave the
MCU). `aes::Aes` is a register-level ECB-block driver; `ccm::Ccm` builds
**AES-128-CCM** (N=13, L=2, 8-byte tag — confidentiality + integrity) in firmware:

```rust
use tower::radio::ccm::Ccm;
let mut ccm = Ccm::new();
let tag = ccm.seal(&key, &nonce13, aad, &mut data);     // encrypt in place
ccm.open(&key, &nonce13, aad, &mut data, &tag)?;        // verify + decrypt
```

Verified against FIPS-197 (AES ECB) and RFC 3610 Packet Vector #1 (CCM) — see
`examples/crypto_aes_kat.rs` and `examples/crypto_ccm_kat.rs`.

## Wire protocol (`tower::radio::frame`)

Little-endian frame, fits the 96-byte FIFO:

```
| ver_type | flags | src(4) | dest(4) | counter(4) | [bulk_idx(3)] | payload | tag(8) |
```

- `ver_type`: bits[7:5] protocol version (=1), bits[4:0] frame type
  (`Data`, `Ack`, `BulkReq`, `BulkData`, `JoinReq`, `JoinResp`, `JoinConfirm`, `Beacon`).
- `flags`: `CONFIRMED`, `DOWNLINK_PENDING`, `LAST_CHUNK`, `BULK_ANNOUNCE`.
- The whole cleartext header is the CCM **AAD**; the payload is encrypted.
- **Nonce** (13 B, not transmitted, reconstructed from the header):
  `nonce_for(src, counter, bulk_index)` = `src ‖ counter ‖ bulk_index ‖ 0x0000`.
  `bulk_index` is 0 for non-bulk frames, so each (key, frame) nonce is unique.
- MTU: ≤ 74 B payload (non-bulk), ≤ 64 B per bulk chunk.

Build / open a secured frame:

```rust
use tower::radio::frame::{self, Header, FrameType, flags};
let hdr = Header { frame_type: FrameType::Data, flags: flags::CONFIRMED,
                   src, dest, counter, bulk_index: None };
let n = frame::seal_frame(&mut ccm, &key, &hdr, payload, &mut buf)?;   // → on-air bytes
let (hdr, range) = frame::open_frame(&mut ccm, &key, &mut buf[..n])?;  // verify + decrypt
```

Verified end-to-end (build → parse → open, tamper/wrong-key rejection, bulk index
in nonce) by `examples/crypto_frame_loopback.rs`.

## Network layer (`tower::radio::net`)

`Net` wraps one radio + the CCM engine + EEPROM-backed counters and serializes one
transfer at a time. It is the layer most applications use directly:

```rust
use tower::radio::net::{Net, NetConfig, SendResult};
use tower::radio::config::Band;

let mut net = Net::new(radio, Kv::new(b.storage),
    NetConfig { my_id: 0x1111_1111, key: KEY, band: Band::Eu868, channel: 0 }).await?;

// Confirmed send: TX the DATA frame, open a 200 ms ACK window, retransmit the
// byte-identical frame up to `reps` times on loss (random 0–100 ms backoff).
match net.send(GW_ID, b"hello", /*confirmed=*/ true, /*reps=*/ 3).await {
    SendResult::Delivered    => {}                   // ACKed (or unconfirmed & sent)
    SendResult::NotDelivered => {}                   // no ACK after all reps
    SendResult::Busy | SendResult::DutyLimited => {} // CSMA / airtime budget
    SendResult::Error(e)     => {}
}

// Receive: authenticate, apply the counter/replay rule, auto-ACK a confirmed
// frame (caching it so a retransmit re-sends the identical ACK, no re-deliver).
if let Some(rx) = net.recv(Duration::from_secs(10)).await {
    let _ = (rx.src, rx.counter, rx.rssi_dbm, rx.confirmed, rx.data());
}
```

The auto-ACK cache holds only the **most recent** confirmed RX. In a busy star, a
confirmed frame from another node between a node's original send and its retransmit
evicts that node's cached ACK, so its retransmit isn't deduplicated and triggers a full
re-send (still correct — never a re-delivery or nonce reuse, just extra airtime).

**Counters, replay & persistence.** Every transfer consumes one
monotonic TX counter; the counter feeds the nonce, so it must never repeat. The
watermark is persisted *ahead* in blocks of `RESERVE=1024`, so after a reboot the
device resumes **at or above** the last value it could have sent — never reusing
one (at most one block is skipped per reboot). A receiver accepts only a strictly
higher counter than it has seen from that sender and lazy-persists the last-seen
every `P=32` accepts (replay window ≤ P across a receiver reboot). CCM verify
happens *before* the replay comparison, so a forged frame can't poison the state.

**Peer table & topologies.** Keys are per-peer. `add_peer(id, &key)` binds
a sender ID to its own AES key and its own replay lane; an unregistered peer falls
back to the `NetConfig::key` default lane (the single-link case). One table holds
up to `MAX_PEERS = 64`:

```rust
net.add_peer(NODE_A, &KEY_A);          // star hub: each node under its own key
net.add_peer(NODE_B, &KEY_B);
net.peer_count();  net.remove_peer(NODE_A);  net.peer_last_seen(NODE_B);
```

- **Star** (≤64 nodes): the gateway registers every node; `recv` reads the clear
  `src`, picks that node's key, and tracks a separate last-seen per node. Each node
  ships with only its own key.
- **P2P** (≤8 peers): both ends `add_peer` the other under a shared link key and
  exchange confirmed messages in either direction.

**Bulk transfer / downlink pull.** Large blobs are *pulled*: the sender
announces `(length, session)`, the requester pulls each ≤64 B chunk with a
`BULK_REQ(index)` and reassembles. The sender reserves **two** TX counters — one for the
announce frame and a separate `session` for every chunk — so the announce's nonce can
never collide with chunk 0's; the `session` counter + 24-bit chunk index then keep each
chunk's nonce unique. The sender frees an idle session after 30 s.

```rust
net.bulk_serve(NODE_ID, &blob).await;              // sender (in-RAM slice)
let n = net.bulk_fetch(GW_ID, &mut out).await;     // requester → bytes received
```

The transfer is **streamed on both ends** — the slice calls above are thin
wrappers over `bulk_serve_from(dest, &mut source)` / `bulk_fetch_into(src, &mut
sink)`, which pull each chunk from a [`BulkSource`] and hand each chunk to a
[`BulkSink`] as it arrives. Neither side buffers the whole transfer, so RAM is
**constant regardless of size** (only the slice wrappers are bounded, by their
slice). This removes the old monolithic-buffer ceiling (~6 KB on this 20 KB part)
and is the path a flash-backed FOTA needs: serve an image from a flash reader,
stream the received image straight into a staging slot. Verified on hardware to
**64 KB (1024 chunks, firmware-sized)** with constant RAM — see `net_bulk_stream`.

```rust
// FOTA-shaped usage: implement the two traits over flash instead of a slice.
net.bulk_serve_from(NODE_ID, &mut image_reader).await;   // BulkSource: read flash → chunk
net.bulk_fetch_into(GW_ID, &mut flash_writer).await;     // BulkSink: chunk → write flash + hash
```

**OTA pairing.** A 3-way JOIN under a fixed, **publicly-known** pairing key
(`PAIRING_KEY`): `JOIN_REQ`(node id) → `JOIN_RESP`(per-node key ‖ challenge) →
`JOIN_CONFIRM`(node id ‖ challenge), both sides committing only on the confirm. The host
mints a fresh per-session **challenge** in the response that the joiner must echo in the
confirm, so a confirm replayed from a prior session is rejected (anti-replay within the
window — on top of CCM integrity; still no confidentiality or mutual auth). The **joining
node chooses its own ID** and keeps it; the host only hands out the key (it does
not assign the ID) and learns the node's ID to install the peer. The default
window is one minute (`PAIRING_WINDOW`). The pairing key gives the JOIN frames
integrity but **no confidentiality** — a sniffer in range during the window
recovers the delivered key — and **no mutual auth**. Mitigate with a short,
user-initiated window, proximity and reduced power; enable flash RDP for
production key storage.

```rust
// host: returns Some(node_id) — the joiner's own id — on commit; install (id, key)
if let Some(id) = net.open_pairing(PAIRING_WINDOW, &per_node_key).await {
    let _ = net.add_peer(id, &per_node_key);
}
// joiner: brings its own id, returns Some(per_node_key) on commit
if let Some(key) = net.join(my_id, PAIRING_WINDOW).await { /* store key */ }
```

**Duty governor.** A token-bucket meters **all** TX airtime (data, ACKs,
bulk, pairing); a send that would exceed the budget returns `DutyLimited` instead
of transmitting. The policy follows the band and is reselected by `set_band`: EU
868 enforces the 1 % / hour limit; US 915 is unrestricted (FCC 15.247 governs by
channel-dwell/PSD, not a fixed duty cycle). Time-on-air is computed per frame
length from the 19.2 kbps rate. Verified independently by `net_duty_kat`.

## Spectrum-access modes (polite high-power operation)

Beyond plain duty-cycled access, two region-specific modes give "polite" channel
access, selected at runtime (mutually exclusive, like `set_band`; `Net::access()`
reports the current [`Access`]). The arbitrary-frequency retune primitive
[`config::set_freq_hz`] (SYNT rewrite, CHNUM=0) underpins both.

**EU 868 — LBT + AFA (EN 300 220) — implemented, hardware-verified.** Listen-
Before-Talk + Adaptive Frequency Agility relaxes the 1 % duty cap. No time-sync:
the node listens (CCA) before every TX and hops to another of 8 channels
(865.2–868.0 MHz) when one is busy or in its post-TX off-time; the gateway scans
the set and ACKs on the catching channel. `net.enable_afa(role, cfg)` →
`afa_send` / `afa_serve`. Example `radio_afa` (verified: confirmed delivery + the
agility channel sweep + gateway-scan rendezvous on two boards).

> **Verify before any product claim:** the EN 300 220 CCA time/threshold,
> minimum channel count, and off-time here are bench defaults — confirm against the
> current standard (and that the chosen sub-band permits LBT+AFA in lieu of duty).

**US 915 — FHSS (FCC §15.247) — implemented, hardware-verified.** 80-channel
frequency hopping (903.0–926.7 MHz, 300 ms slots, 24 s cycle), gateway = hop
time-master + per-slot beacon, node blind-rendezvous on a fixed channel then hops
in lockstep, re-aligning on each beacon. `net.enable_fhss(role, cfg)` →
`fhss_master_tick` (gateway loop) / `fhss_node_tick` + `fhss_send` (node loop).
Compliance is **structural** — N=80, cycle 24 s > 20 s ⇒ each channel is tuned at
most once per 20 s ⇒ ≤ one 300 ms slot occupancy (25 % under the 0.4 s/20 s limit),
no per-channel governor needed; a light `[u16; 80]` airtime counter feeds the
compliance histogram. Examples `radio_fhss` (verified: node LOCKED then confirmed
delivery on hopping channels; LOST→rescan→relock on gateway loss) and
`fhss_compliance` (≥50 channels used, max per-channel airtime ≪ 0.4 s, band edges
exercised). The hop seed is key-derived (not sent on air).

> **Verify before any product claim:** the exact §15.247 channel/dwell numbers
> (FCC KDB) and that the implementation meets them under your antenna/power.

## Examples

Two-board examples are one source file built twice with a role feature (e.g.
`TOWER_FEATURES=role-node just flash example net_confirmed`).

| Example | Boards / roles | What it shows |
|---|---|---|
| **`radio_gateway`** / **`radio_node`** | gateway / node | **reference apps**: confirmed, secure telemetry uplink + decode |
| `radio_id` | 1 | device-ID check (SPI/CS/SDN bring-up) |
| `radio_state` | 1 | READY/STANDBY transitions + nIRQ |
| `radio_cw` | 2 | CW carrier (TX) detected via partner RSSI (RX) |
| `radio_beacon` | 2 | raw TX/RX link, per-packet RSSI/LQI/SQI/AFC |
| `radio_regdump` | 1 | read back the RF registers after config |
| `radio_linkdiag` | 2 | RX event/quality diagnostics over a live link |
| `radio_csma` | 2 | CSMA/CCA defers TX while a jammer holds the channel |
| `radio_sleep` | node / gateway | SLEEP vs SHUTDOWN wake latency + re-link |
| `crypto_aes_kat` | 1 | AES-128 ECB FIPS-197 known-answer test |
| `crypto_ccm_kat` | 1 | AES-128-CCM RFC 3610 vector + tamper test |
| `crypto_frame_loopback` | 1 | frame codec + nonce + CCM round-trip |
| `net_secure_ping` | node / gateway | one CCM-sealed DATA frame end-to-end |
| `net_confirmed` | node / gateway | confirmed delivery + ACK + retransmit |
| `net_persist` | 1 | TX-counter reserve-ahead survives reboot |
| `net_duty_kat` | 1 | duty-governor token-bucket KAT |
| `net_bulk` | gateway / node | bulk pull: announce → BULK_REQ/BULK_DATA |
| `net_bulk_stress` | gateway / node | large bulk (multi-KB) + CRC-32 + throughput stress |
| `net_bulk_stream` | gateway / node | streaming bulk (source/sink), 4–64 KB, constant RAM |
| `net_pairing` | gateway / node | OTA 3-way JOIN delivers a per-node key |
| `net_star` | gateway / node[,node-2] | star: per-node keys + per-node replay lanes |
| `net_p2p` | role-peer-a / role-peer-b | P2P bidirectional confirmed exchange |
| `net_channel` | node / gateway | secured link on a non-default channel (VCO recal) |
| `radio_band` | node / gateway | runtime 868↔915 switching via `set_band` (live retune) |
| `radio_afa` | node / gateway | EU LBT+AFA: listen-before-talk + frequency agility (EN 300 220) |
| `fhss_sweep` | 1 | FHSS channel-plan + 80-channel synth lock + GUARD measure (F1) |
| `fhss_kat` | 1 | FHSS hop-permutation / dwell / beacon-frame KATs (F3–F5) |
| `radio_fhss` | node / gateway | US FHSS link (FCC §15.247): lock + hopping confirmed delivery |
| `fhss_compliance` | 1 | §15.247 evidence: channel count + per-channel airtime histogram |
| `edge_frame_limits` | 1 | MTU + malformed/forged-frame rejection KAT |
| `edge_recovery` | 1 | RX-timeout / stuck-state / FIFO recovery |
| `edge_rapid` | node / gateway | back-to-back confirmed, strict-monotonic counters |
| `radio_interop` | node / gateway | semi-fuzzy soak: randomized traffic + invariant checks |

## A note on RX completion (hard-won)

`RX_DATA_READY` only fires if the **RX-timeout stop condition** is configured.
At reset, `PCKT_FLT_OPTIONS.RX_TIMEOUT_AND_OR_SELECT` = 1 means "the timeout
cannot be stopped" (datasheet Table 30 / §9.3): a complete packet lands in the RX
FIFO but the part sits in RX forever and never raises the interrupt. Clearing that
bit (`PCKT_FLT_OPTIONS` bit6 = 0, with no timeout masks) selects "reception ends
at the reception of the packet", so `RX_DATA_READY` fires normally. `config::apply`
sets this; it is unrelated to the RF/demod registers.

## Parameters reference

| Constant | Value |
|---|---|
| Device ID / crystal | part 304 / version 48 · **50 MHz** (not a TCXO) |
| Bands / channels | EU 868 & US 915 · 3 per band |
| EU 868 channels | 868.1 / 868.3 / 868.5 MHz (200 kHz spacing; "g1" ≤ +14 dBm ERP, ≤ 1 % duty) |
| US 915 channels | 915.0 / 915.2 / 915.4 MHz (200 kHz spacing) — single-channel is **bench-only**; use FHSS for a compliant US link |
| Modulation / bit rate / deviation / BT | GFSK / 19 200 bps / 20 kHz / 1 |
| RX bandwidth | ~216 kHz (wide; narrow via AFC_CORR — see Design rationale) |
| Sync / CRC / preamble | `0xDB624715` / 16-bit `0x1021` / 4 B |
| FIFO = network frame | 96 B |
| Header (non-bulk / bulk) / CCM tag | 14 B / 17 B / 8 B |
| Payload (single / bulk chunk) | ≤ 74 B / 64 B |
| Nonce | 13 B (`src ‖ counter ‖ bulk_index ‖ 0x0000`) |
| Time-on-air (max frame / ACK) | ≈ 44.6 ms (96 B) / ≈ 15.8 ms (27 B) |
| TX power (default / max) | +11.6 dBm |
| CSMA RSSI threshold / max backoff | −90 dBm / ~100 ms |
| ACK window / inter-rep backoff / RX→TX turnaround | 200 ms / random 0–100 ms / ~20 ms |
| Confirmed repetitions (range / default) | 1–10 / 3 |
| Confirmed latency (N=3 / N=10) | ≈ 1.06 s / ≈ 3.55 s worst case |
| Replay counter | 32-bit; starts at 1; **saturating** at 2³²−1 (fail-closed, not wrap) — re-key well before |
| Reserve block `RESERVE` / lazy-persist `P` | 1024 / 32 transfers |
| Max bulk / chunk index | 16 MiB (streamed) / 24-bit |
| Bulk idle timeout | 30 s |
| Star nodes / P2P peers | ≤ 64 / ≤ 8 |
| Reserved IDs | `0x00000000`, `0xFFFFFFFF` |
| Protocol version | 1 |
| FHSS (US) | 80 channels · 300 ms slot · 24 s cycle |
| EU duty cycle | ≤ 1 % per sub-band per device (governed; all TX counted) |

## Design rationale

The reasoning behind the chosen RF and security parameters — what to understand
before changing any of them.

**RX bandwidth (the central RF trade-off).** ±40 ppm crystal error per unit means
two units differ by **~80 ppm** worst case. At 868 MHz that's a ~69 kHz carrier
offset; with the ~59 kHz GFSK signal (Carson) the RX filter must pass *signal +
2·offset* ≈ **198 kHz** (≈ 206 kHz at 915), so the nearest SPIRIT1 step **≥ ~210 kHz**
is required for bring-up. **AFC re-centers the residual, but the analog filter must
still *pass* the offset signal**, so a wide filter is unavoidable until the real
drift is measured. ±40 ppm is conservative (the SPSGRF is a plain 50 MHz crystal, not
a TCXO); the SPIRIT1 reports each packet's offset in `AFC_CORR`, so the path to
narrowing is: run wide, log `AFC_CORR` between real boards over temperature, then set
RX BW to the measured worst case + margin (a true ~±20 ppm part → ~130 kHz → ~2 dB
more range). Until narrowed below ~180 kHz the 200 kHz channel spacing overlaps, so
only ch0/ch2 are simultaneously usable; all three after narrowing.

**Channels are tuned, not spanned.** The receiver sits on **one** channel; the RX BW
is crystal-drift tolerance *around that center*, not a window over all three — a
receiver on one channel does **not** hear the others, so **a node and its gateway must
share a channel**. The 3 channels exist to run co-located networks without
interference, not for a node to roam. **Band** is region/config-time (EU→868, US→915),
identical on a gateway and all its nodes, with no OTA negotiation. EU 868 uses the
868.0–868.6 MHz "g1" sub-band (EN 300 220 / ERC 70-03): **≤ +14 dBm ERP, ≤ 1 % duty**
(keep conducted +11.6 dBm to ≤ ~2 dBi antenna, else reduce power). Duty is **per
sub-band, per device** and counts **all** TX — data, ACK, bulk, retransmit, JOIN — and
the gateway is governed too. **Final regulatory compliance — the current EN 300 220
revision, an FCC strategy for the US, ERP with the real antenna, lab testing — is the
integrator's responsibility before shipping.**

**Security model (AES-128-CCM) — the nonce-uniqueness argument.** Each node has its
own AES-128 key (the gateway holds one per node; the **same key protects both
directions**). The cleartext header is the **AAD**, the payload is encrypted, and the
8-byte tag authenticates both. The 13-byte nonce is *derived*, never transmitted:
`src ‖ counter ‖ bulk_index ‖ 0x0000`. It is unique per `(key, frame)` because: the
key is per-node and **`src` fixes the sender** (the two directions never collide even
at equal counter values — no `dir` field needed); the 32-bit **counter** advances one
per transfer; **`bulk_index`** separates the chunks of one transfer; and a
**retransmission re-sends the byte-identical frame** (same counter ⇒ same ciphertext ⇒
safe). An **ACK therefore uses the ACKer's own fresh counter** — the *acknowledged*
counter rides in the ACK payload, never as the nonce counter — so an ACK and the frame
it answers never share a nonce. On receive, **CCM-verify first** (this authenticates
the header, including the counter), **then** compare to the per-peer last-seen and
update it — so a forged high counter can't poison replay state, and tampered frames are
dropped before the network layer acts. On key install the TX counter starts at **1**
(`0` = never sent) and last-seen at **0**; a **re-key resets both** (a new key is a
disjoint nonce space, so old ciphertext can't replay under it). The counter **saturates**
at 2³²−1 instead of wrapping: at the ceiling it sticks, and the strict `counter > last-seen`
rule makes the peer reject every further frame as a replay, so the link fails **closed**
rather than silently reusing a low nonce — re-key well before then (≈136 yr at 1 Hz, so not
a practical limit). EEPROM key storage is only as safe as the chip's readout protection —
**enable flash RDP for production**.

## Known limitations & caveats

- **OTA pairing has no confidentiality.** The fixed `PAIRING_KEY` is public, so a
  sniffer in range during the (short, user-initiated) pairing window recovers the
  delivered per-node key, and there is no mutual authentication. Pair at
  close range / reduced power; enable flash RDP for production key storage.
- **US 915 single-channel `Us915` is bench-test only** (runtime-switchable via
  `set_band`, hardware-verified, but **not** FCC 15.247-compliant — use FHSS for a
  compliant US link). EU 868 (duty or LBT+AFA) is the compliant default region.
- **FHSS sync** is robust. The node opens its beacon RX a guard *before* each slot
  boundary (armed before the gateway transmits), listens a wide window that ignores
  stray frames, and — crucially — **rides through a fade by predicting the channel
  on its kept clock anchor** (drift ≪ the RX window for dozens of slots), re-locking
  within one slot once RF returns rather than treating a missed beacon as a loss.
  Only after ~7 s of misses (anchor too stale / gateway restarted) does it fall back
  to rendezvous scanning. **Soak-verified: 1 lock, 0 sync losses, 1133 confirmed
  deliveries over ~6 min / ~15 cycles, max delivery gap 2 slots** (an earlier
  per-slot-strict design dropped sync 1–2×/6 min with ~23 s re-acquire each). A
  *gateway restart* still forces a one-cycle re-acquire (new epoch) — inherent.
  (`RAM note:` the FHSS per-channel state is a `[u16; 80]` counter — an earlier
  `[DutyGovernor; 80]` overflowed the 20 KB L0.)
- **RX bandwidth is set wide (~216 kHz)** to tolerate the 50 MHz-crystal tolerance
  without lab instruments; narrowing it (per the AFC-vs-temperature data — see Design
  rationale) is a future optimization. All three EU channels are usable as-is.
- **Counter persistence uses a single reserve-ahead watermark cell** (RESERVE=1024;
  ~10⁸ transfers before EEPROM wear matters). A multi-cell wear-ring is a refinement.
- **Half-duplex single radio.** `Net` serializes one transfer at a time; CSMA
  mitigates contention but cannot eliminate hidden-node collisions — confirmed
  delivery + retransmit absorbs the rest.
- **`Net::send` does not enable CSMA by default** (CSMA is a radio-layer feature
  shown in `radio_csma`); wire `use_csma=true` into the TX path if your deployment
  needs it on every frame.
