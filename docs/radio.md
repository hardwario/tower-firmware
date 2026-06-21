# TOWER Radio — user guide

A sub-GHz radio stack for the TOWER Core Module, built on the **SPIRIT1**
transceiver (in the SPSGRF module). This is the *user-facing* guide; the internal
design spec is `RADIO.md` and the implementation checklist is `PLAN.md`.

> **Status (in progress).** The radio layer (SPI, states, RF config, CW, TX, RX
> with full quality metrics), the on-MCU **AES-128-CCM** crypto, and the **frame
> codec** are implemented and verified on hardware — including a **full secured
> bidirectional link**: `net_secure_ping` sends CCM-sealed frames Node→Gateway
> that are received, authenticated and decrypted over the air. The higher network
> layer (confirmed delivery/ACK, replay+persistence, duty governor, bulk, pairing,
> topologies) is the remaining work and now builds on this working link.

## Hardware

SPIRIT1 ↔ STM32L083CZ wiring (from the board, see `src/board.rs`):

| Signal | Pin | Notes |
|---|---|---|
| SDN | PB7 | drive low to enable (hardware pull-up → boots in shutdown) |
| SPI CS | PA15 | software CS, ≥2 µs setup |
| SCLK / MOSI / MISO | PB3 / PB5 / PB4 | SPI1, mode 0, 8 MHz |
| nIRQ | PA7 | active-low, EXTI line 7 |

Crystal is **50 MHz** (not a TCXO). Band: **EU 868** (ch0/1/2 = 868.1/868.3/868.5 MHz);
US 915 is a future option behind the same `Band` abstraction.

## Building & flashing

Examples live in `examples/`. Flash with the UART bootloader and watch the console:

```sh
just flash <example>                 # build + flash (auto-detect port)
TOWER_PORT=/dev/cu.usbserial-XXXX just flash <example>   # pick a board
jolt monitor --reset                 # watch from boot

# two-board examples select a role via a Cargo feature:
TOWER_FEATURES=role-gateway just flash <example>   # one board
TOWER_FEATURES=role-node    just flash <example>   # the other
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
```

RF configuration (`config::apply`) programs the 50 MHz-crystal-specific setup
(REFDIV, IF offsets, SYNT/WCP, GFSK 19.2 kbps, 20 kHz deviation, ~216 kHz RX
filter, sync `0xDB624715`, 16-bit CRC, PA ramp, AFC freeze-on-sync) plus the ST
management work-arounds (per-mode SMPS, VCO current, one-time manual VCO
calibration). See `src/radio/config.rs`.

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
  (`Data`, `Ack`, `BulkReq`, `BulkData`, `JoinReq`, `JoinResp`, `JoinConfirm`).
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

**Counters, replay & persistence (§6/§7.4).** Every transfer consumes one
monotonic TX counter; the counter feeds the nonce, so it must never repeat. The
watermark is persisted *ahead* in blocks of `RESERVE=1024`, so after a reboot the
device resumes **at or above** the last value it could have sent — never reusing
one (at most one block is skipped per reboot). A receiver accepts only a strictly
higher counter than it has seen from that sender and lazy-persists the last-seen
every `P=32` accepts (replay window ≤ P across a receiver reboot). CCM verify
happens *before* the replay comparison, so a forged frame can't poison the state.

**Peer table & topologies (§7.2).** Keys are per-peer. `add_peer(id, &key)` binds
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

**Bulk transfer / downlink pull (§7.5).** Large blobs are *pulled*: the sender
announces `(length, session)`, the requester pulls each ≤64 B chunk with a
`BULK_REQ(index)` and reassembles. The session counter + 24-bit chunk index keep
every chunk's nonce unique; the sender frees an idle session after 30 s.

```rust
net.bulk_serve(NODE_ID, &blob).await;              // sender
let n = net.bulk_fetch(GW_ID, &mut out).await;     // requester → bytes received
```

**OTA pairing (§7.6).** A 3-way JOIN under a fixed, **publicly-known** pairing key
(`PAIRING_KEY`): `JOIN_REQ` → `JOIN_RESP`(assigned id + per-node key) →
`JOIN_CONFIRM`, both sides committing only on the confirm. The pairing key gives
the JOIN frames integrity but **no confidentiality** — a sniffer in range during
the window recovers the delivered per-node key — and **no mutual auth**. Mitigate
with a short, user-initiated window, proximity and reduced power; enable flash RDP
for production key storage.

```rust
let proposed = net.open_pairing(Duration::from_secs(10), assign_id, &assign_key).await; // host
let (id, key) = net.join(my_proposed_id, Duration::from_secs(10)).await?;               // joiner
```

**EU duty governor (§2.2).** A token-bucket meters **all** TX airtime (data, ACKs,
bulk, pairing) against the 1 % / hour EU limit; a send that would exceed the budget
returns `DutyLimited` instead of transmitting. Time-on-air is computed per frame
length from the 19.2 kbps rate. Verified independently by `net_duty_kat`.

## Examples

Two-board examples are one source file built twice with a role feature (e.g.
`TOWER_FEATURES=role-node just flash net_confirmed`).

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
| `net_confirmed` | node / gateway | confirmed delivery + ACK + retransmit (§7.3) |
| `net_persist` | 1 | TX-counter reserve-ahead survives reboot (§7.4) |
| `net_duty_kat` | 1 | duty-governor token-bucket KAT (§2.2) |
| `net_bulk` | gateway / node | bulk pull: announce → BULK_REQ/BULK_DATA (§7.5) |
| `net_pairing` | gateway / node | OTA 3-way JOIN delivers a per-node key (§7.6) |
| `net_star` | gateway / node[,node-2] | star: per-node keys + per-node replay lanes (§7.2) |
| `net_p2p` | role-peer-a / role-peer-b | P2P bidirectional confirmed exchange (§7.2) |

## A note on RX completion (hard-won)

`RX_DATA_READY` only fires if the **RX-timeout stop condition** is configured.
At reset, `PCKT_FLT_OPTIONS.RX_TIMEOUT_AND_OR_SELECT` = 1 means "the timeout
cannot be stopped" (datasheet Table 30 / §9.3): a complete packet lands in the RX
FIFO but the part sits in RX forever and never raises the interrupt. Clearing that
bit (`PCKT_FLT_OPTIONS` bit6 = 0, with no timeout masks) selects "reception ends
at the reception of the packet", so `RX_DATA_READY` fires normally. `config::apply`
sets this; it is unrelated to the RF/demod registers.

## Known limitations

- **Network layer, low-power sleep, regulatory/duty governor** — not yet
  implemented/verified (they build on the now-working bidirectional link).
- **Regulatory & OTA-pairing security caveats** — see `RADIO.md` §2.2 and §7.6.
