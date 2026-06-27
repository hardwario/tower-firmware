# FOTA — Firmware-Over-The-Air upgrade subsystem (phased plan)

> **STATUS: SUSPENDED (parked 2026-06-27).** Piece 1 of 4 (streaming transport) is
> **done and hardware-verified**; the rest is designed but not started. Work was
> paused to focus on a different implementation block. This file is the cold-resume
> record — read the "Resume in one minute" box, then jump to the first unstarted phase.

---

## Resume in one minute

- **What FOTA needs:** (1) streaming transport, (2) flash staging + bootloader/slot-swap,
  (3) image signing + rollback protection, (4) a control protocol (advertise → pull →
  verify → swap) with resume. They are layered; do them in order.
- **Done:** Piece 1 — bulk transfer streams on both ends with **constant RAM**
  (`BulkSource`/`BulkSink`, `bulk_serve_from`/`bulk_fetch_into`), verified to **64 KB /
  1024 chunks** on two boards. See `src/radio/net/bulk.rs`, example `net_bulk_stream`,
  commit `04ea514`.
- **Next action when we come back:** start **Phase 1** (FlashSink/FlashSource over
  *program* flash) — but first resolve the **Open decisions** below (bootloader choice,
  signature scheme, slot sizes). Nothing in Phases 1–5 should start before those are locked.
- **The one non-negotiable:** image **signing is mandatory before any FOTA ships**
  (Phase 3). Without it FOTA is a remote-code-execution vector, because the link key is
  recoverable by a sniffer during OTA pairing (the pairing key is publicly known). CCM on
  the wire is *not* sufficient authenticity for executable code.

---

## 1. Context & goal

Deliver a new firmware image to a deployed `tower` node over the SPIRIT1 sub-GHz link
and have it boot the new image safely — without bricking on power loss, and without
letting an attacker push code. Target part: **STM32L083CZ** — 192 KB program flash,
20 KB RAM, 6 KB data EEPROM, Cortex-M0+ @ 16 MHz, **single flash bank**.

Topology is the existing star: the **gateway distributes**, the **node pulls** (the
pull model already fits — the node controls *when* it updates, good for battery/duty).

## 2. Current state (what already exists — build on this)

| Capability | Where | Notes |
|---|---|---|
| Streaming bulk transport | `src/radio/net/bulk.rs` | `BulkSource`/`BulkSink` traits; `bulk_serve_from`/`bulk_fetch_into`; constant RAM, any size (24-bit index → 1 GB). |
| Streaming demo / template | `examples/net_bulk_stream.rs` | `PatternSource`/`CrcCheckSink` are the **templates** for `FlashSource`/`FlashSink`. Verified 4/16/32/**64** KB, US915, 4 MHz SPI. |
| Per-frame integrity/confidentiality | `src/radio/frame.rs`, `ccm.rs` | AES-128-CCM, 8-byte tag. Protects the wire — **not** image authenticity (see §6). |
| Downlink-pending flag | `frame.rs` `flags::DOWNLINK_PENDING = 1<<1` | **Defined but unused** (`net/mod.rs` `send_ack` sends `flags: 0`). Reserved for FOTA "update available" advertisement. |
| Spare FrameType | `frame.rs` `FrameType` Data=0…Beacon=7 | **8 is the next free** value for a FOTA control frame, if needed. |
| EEPROM key-value store | `src/storage.rs` `Kv` (6 KB EEPROM) | Good for FOTA state (version, resume high-water mark, pending flag). Net uses keys `0x5201/0x5202/0x5300+slot`; **FOTA should use its own range, e.g. `0x5400+`**. |
| **Gap: program-flash writer** | — | `storage.rs` wraps `Flash` for **EEPROM only** (`eeprom_read/write_slice`). FOTA needs **program-flash erase/program** (a different API on the same `embassy_stm32::flash::Flash`). Not yet wrapped — Phase 1 adds it. |
| **Gap: bootloader** | — | App is linked at `0x0800_0000` and runs directly (no application bootloader; `jolt` uses the ROM bootloader, unusable for self-update). Phase 2 adds one. |

## 3. Target constraints & budget

- **Flash 192 KB:** app is currently ~52 KB. Dual-slot A/B + bootloader fits comfortably
  (see §4 layout). Signing code (Ed25519 verify) adds a few KB to the app.
- **RAM 20 KB:** **not** a blocker for streaming FOTA — the FlashSink needs only a chunk
  buffer (64 B) + a flash-page buffer (128 B) + a hash state (SHA-256 ~110 B) + the
  signature-verify scratch (Ed25519 ~1–2 KB, only at verify time). The old monolithic-
  buffer limit is gone.
- **EEPROM 6 KB:** holds FOTA state durably (separate from program flash, byte-writable,
  100k+ endurance).
- **Single flash bank (L0):** the core stalls on flash access during erase/program. The
  bootloader's swap routine may need its flash critical section **in RAM**. **Verify**
  embassy-boot's behaviour on a single-bank L0 (this is the main bootloader risk — §10).
- **L0 program flash granularity:** 128 B page erase; word/half-page program. **Confirm**
  embassy-stm32's `WRITE_SIZE`/`ERASE_SIZE` for L0 and align slot boundaries to pages.

## 4. Proposed flash layout (verify sizes once the signed app size is known)

embassy-boot wants `ACTIVE` and `DFU` partitions of **equal size, page-aligned**, plus a
small `BOOTLOADER_STATE` partition. A starting point for the 192 KB L083:

| Region | Start | Size | Purpose |
|---|---|---|---|
| BOOTLOADER | `0x0800_0000` | 16 KB | immutable loader (verify + swap + jump); RDP + WRP in production |
| BOOTLOADER_STATE | `0x0800_4000` | 2 KB | embassy-boot swap progress/magic |
| ACTIVE | `0x0800_4800` | 80 KB | running app (linked here; bootloader jumps here) |
| DFU | `0x0801_8800` | 80 KB | staging slot for the downloaded image |
| (spare) | `0x0802_C800` | ~14 KB | margin / future |

Notes: app `memory.x` FLASH origin = ACTIVE start; bootloader has its own `memory.x` at
`0x0800_0000`. 80 KB slots give the ~52 KB app room to grow with signing. **Gateway image
storage:** the gateway must hold the image it serves — on a 192 KB Radio Dongle gateway a
64 KB image may be tight alongside its own app; decide whether the gateway streams the
image **from its own flash** (FlashSource over a reserved region) or **proxies from the
host** over USB on demand (§9, open decision).

## 5. Time / throughput budget (drives UX, not feasibility)

Measured streaming throughput ≈ **5.4–5.8 kbps** effective (per-chunk round-trip bound).
Gateway airtime ≈ **42 ms/chunk** (64 B BULK_DATA).

| Image | Chunks | US915 (unthrottled) | EU868 (1 % duty) |
|---|--:|--:|--:|
| 52 KB (current app) | 832 | ~85 s | ~50–58 min |
| 64 KB | 1024 | **~94 s (measured)** | ~12 min (36 s burst + 1 % tail) |

- **EU is slow** for a full image (correct regulatory behaviour). Mitigations: schedule
  overnight, do it in resumable sessions across the duty budget, or use US915 where legal.
- **Delta updates** (ship only changed flash pages) would cut both time and airtime
  dramatically — strong candidate for a later phase, not the first cut.

## 6. Security requirements (the gate — Phase 3 is mandatory before shipping)

1. **Independent image signature.** Sign a **manifest** `{version, size, sha256(image)}`
   with a vendor private key; verify with the vendor **public key baked immutably into
   the bootloader**. Verify **before swap**. Recommended: **Ed25519** (small, fast on
   M0+, good no_std crates) — alternative ECDSA-P256 if standard/HW alignment demands it.
   Rationale: CCM authenticates chunks only under the *link key*, which a sniffer can
   recover during OTA pairing (publicly-known `PAIRING_KEY`, no confidentiality). So CCM
   alone ⇒ anyone who captured a pairing can push code. The signature is the real gate.
2. **Rollback protection.** Reject images with `version ≤ installed_version` (a validly-
   signed *old* image must not be installable). Persist `installed_version` in EEPROM
   (and/or option bytes); the bootloader enforces it.
3. **Whole-image hash.** Bootloader recomputes `sha256` over the staged DFU image and
   checks it against the signed manifest before swap (catches partial/garbled staging;
   the signature covers the hash, so one check covers integrity + authenticity).
4. **Production hardening.** Enable flash **RDP** (readout protection) and **write-protect
   the bootloader sector**; consider a watchdog over the swap; brownout detection.

## 7. Phased plan

Each phase ends with an on-hardware verify (single board where possible, two boards for
the link). Mirrors the project's staged style (cf. the FHSS F1–F11 plan).

### Phase 0 — Decisions & scaffolding (no hardware)
- Lock the **Open decisions** (§9): bootloader, signature scheme, slot sizes, manifest
  format, resume granularity, advertisement mechanism, gateway image storage.
- Add a `src/fota/` module skeleton: `Manifest` struct + (de)serialize, flash-region
  constants (from §4), FOTA EEPROM key range (`0x5400+`), a `FotaState` enum
  (`Idle / Downloading{hwm} / Staged / Verified / PendingSwap`).
- **Exit:** decisions written into this file; `cargo build`/`clippy` clean; no behaviour.

### Phase 1 — Flash staging sink/source (transport → real flash, no bootloader yet)
- Add a **program-flash writer** wrapping `embassy_stm32::flash::Flash` (erase page /
  program half-page) — the piece `storage.rs` lacks (it's EEPROM-only).
- Implement `FlashSink: BulkSink` — `begin(total)`: erase the DFU region for `total`;
  `consume(off, chunk)`: buffer to a page, program full pages, fold into a running SHA-256.
  Implement `FlashSource: BulkSource` — read the gateway's image region chunk-by-chunk.
- **Test (1 board):** stream a generated blob (e.g. 52–64 KB) through `FlashSink` into the
  DFU region, read it back, verify SHA + CRC. **(2 boards):** gateway serves from its
  flash via `FlashSource`, node stages via `FlashSink`; compare end-to-end SHA.
- **Exit:** a full-size image lands in DFU flash byte-perfect; SHA matches; survives the
  real program/erase path at size; constant RAM confirmed.

### Phase 2 — Bootloader + A/B swap (embassy-boot), unsigned
- Add the bootloader project (embassy-boot) + partition table (§4); split `memory.x`
  (bootloader vs app); link the app for ACTIVE. Wire `FirmwareUpdater`: after a verified
  download mark DFU "ready", reboot, bootloader swaps, app runs, app **confirms** "good"
  (so it isn't reverted on next boot).
- **Test:** app **v1** (prints "v1") OTA-delivers app **v2** (prints "v2") into DFU →
  mark + reboot → bootloader swaps → "v2" runs. Then: (a) **revert** test — don't confirm,
  power-cycle, bootloader rolls back to v1; (b) **power-loss-during-swap** test — interrupt
  mid-swap, re-power, verify no brick (boots a valid image).
- **Exit:** end-to-end swap + confirm + auto-revert + power-loss safety all pass.
- **RISK to clear here:** single-bank L0 swap (flash routine may need to run from RAM) — §10.

### Phase 3 — Signing + verification + rollback (the security gate)
- Define the signed `Manifest`; bake the vendor **public key** into the bootloader.
  Verify the Ed25519 signature over the manifest, and `sha256(DFU)` == manifest hash,
  **before** allowing the swap; reject `version ≤ installed`.
- Add a **host signing tool** (small script: hash image, build manifest, sign with the
  private key). Deliver the manifest alongside/embedded-with the image.
- **Test:** valid signed newer image → installs; **tampered** image (flip one byte) →
  signature/hash fail → no swap, stays on current; **older** signed version → rejected.
- **Exit:** only correctly-signed, strictly-newer images install. Closes the RCE vector.

### Phase 4 — Control protocol + resume (the real over-the-air flow)
- **Advertise:** gateway sets `flags::DOWNLINK_PENDING` in its ACKs (or a dedicated
  FrameType=8 FOTA frame) carrying `{version, size, manifest-hash}`. Node, when idle/
  scheduled, pulls the manifest then the image via `bulk_fetch_into(FlashSink)`.
- **Resume:** persist a high-water mark (or chunk bitmap) in EEPROM; on interrupted
  download, re-request only missing chunks instead of restarting from 0. (Needs a small
  extension to `bulk_fetch_into` to start at an offset, or a higher-level driver loop.)
  Honour the duty budget / schedule (overnight on EU; fast on US).
- **Test:** interrupt mid-download (power-cycle the node) → resume completes without
  restarting; full E2E advertise → pull → verify → swap → confirm over a real link.
- **Exit:** robust, resumable, scheduled FOTA over a lossy, duty-limited link.

### Phase 5 — Hardening & production
- Flash RDP + bootloader WRP; key provisioning/storage; watchdog over swap; brownout
  handling; telemetry (report installed version + last FOTA result back to the gateway).
- **Optional/later:** delta updates (changed-page diffs) to cut EU transfer time.

## 8. Definition of done (whole subsystem)

A deployed node, told over the air that vX+1 is available, pulls and stages it (resuming
across drops and within the duty budget), the bootloader verifies signature + hash +
version and swaps atomically, the new image boots and self-confirms, a tampered/old/
corrupt image is refused, and an interrupted swap never bricks the device.

## 9. Open decisions (resolve in Phase 0, before any code)

1. **Bootloader:** `embassy-boot` (recommended — A/B, swap state machine, verified-update
   API, Embassy-native) vs a minimal custom loader.
2. **Signature scheme:** Ed25519 (recommended) vs ECDSA-P256. Pick the no_std crate.
3. **Slot sizes:** finalize once the signed app's size is known (§4 is a starting point;
   ACTIVE == DFU, page-aligned).
4. **Resume granularity:** high-water mark (simple; restart from last contiguous chunk)
   vs chunk bitmap (request any missing). Where to persist (EEPROM key range `0x5400+`).
5. **Advertisement:** reuse `DOWNLINK_PENDING` bit + a manifest frame, vs a new
   `FrameType = 8`.
6. **Manifest format & coverage:** fields + encoding; confirm one signature over the
   manifest (which carries the image hash) covers integrity + authenticity.
7. **Gateway image storage:** serve from the gateway's own flash (FlashSource over a
   reserved region) vs proxy/stream from the host over USB on demand (matters on a
   192 KB gateway that must also hold its own app + a 64 KB image).
8. **Update policy:** when the node applies an update — immediately / scheduled /
   user-triggered — and how it budgets power + duty for a long EU transfer.
9. **Delta updates:** in scope for the first cut (no — full image first) or a later phase.

## 10. Risks

- **Single-bank L0 swap (highest).** The core stalls on flash access during erase/program;
  the bootloader's flash routine may need to execute from RAM. Validate embassy-boot on
  this exact part early in Phase 2 before building on it.
- **Gateway image storage** on a 192 KB part (decision #7) — may force a host-proxy source.
- **EU transfer time** for a full image (~1 h) — mitigate with scheduling/resume, or
  deltas later. Feasibility is fine; UX needs design.
- **Security regression** — never let CCM-only stand in for the signature; Phase 3 is a
  hard gate, not optional polish.
- **Standards/format** caveats (signing scheme, RDP, manifest) are config to confirm
  against current requirements before a product claim — same posture as the radio
  regulatory caveats.

## 11. Pointers / references

- Transport: `src/radio/net/bulk.rs` (`BulkSource`/`BulkSink`, `bulk_serve_from`,
  `bulk_fetch_into`), `examples/net_bulk_stream.rs` (sink/source templates).
- Flags/types: `src/radio/frame.rs` (`flags::DOWNLINK_PENDING`, `FrameType`).
- Persistence: `src/storage.rs` (`Kv` over EEPROM — and the **missing** program-flash
  writer to add in Phase 1).
- Net wiring: `src/radio/net/mod.rs` (`send_ack` is where DOWNLINK_PENDING would be set;
  peer keys, counters).
- Bench/flash workflow: `docs/radio.md`, `PLAN.md`; flash with `TOWER_FEATURES=… just
  flash <example>`, monitor with `jolt monitor --reset`.
- External: embassy-boot (bootloader), a no_std Ed25519 crate, SHA-256 (no_std), FCC/EU
  duty numbers in `docs/radio.md`.
