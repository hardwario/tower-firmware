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

## Examples

| Example | Boards | What it shows |
|---|---|---|
| `radio_id` | 1 | device-ID check (SPI/CS/SDN bring-up) |
| `radio_state` | 1 | READY/STANDBY transitions + nIRQ |
| `radio_cw` | 2 | CW carrier (TX) detected via partner RSSI (RX) |
| `radio_beacon` | 2 | raw TX beacon / RX sniffer (TX verified; RX demod WIP) |
| `radio_regdump` | 1 | read back the RF registers after config |
| `radio_rxdiag` | 1 | poll which RX events (preamble/sync/data) fire |
| `crypto_aes_kat` | 1 | AES-128 ECB FIPS-197 known-answer test |
| `crypto_ccm_kat` | 1 | AES-128-CCM RFC 3610 vector + tamper test |
| `crypto_frame_loopback` | 1 | frame codec + nonce + CCM round-trip |

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
