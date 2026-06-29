# TOWER FOTA — firmware-over-the-air upgrade guide

Signed **firmware-over-the-air** updates for the TOWER Core Module (STM32L083CZ). A node
pulls a new image over the sub-GHz radio (the gateway streams it from the host), stages it to
a spare flash slot, and an **embassy-boot A/B bootloader verifies its Ed25519 signature +
image digest and swaps it in atomically** — with automatic rollback if the new image fails to
confirm. This is the standalone reference for the subsystem: the flash layout, the verifying
bootloader, the signed manifest, the device-side staging + control protocol, the host-proxy,
and the design rationale and (hard-won) caveats.

> **Status: built and hardware-verified, end to end** (2026-06-28, two Radio Dongles + host).
> `fota_ota` ran the full real-firmware swap: node **v1** advertised → pulled a 67.5 KB signed
> firmware over the radio (host-proxied by `tower fota serve`) → stashed the manifest → reset →
> the **bootloader verified Ed25519 + image digest → swapped → v2 booted and confirmed**
> (`*** UPDATE CONFIRMED *** booted swapped v2`). Also HW-verified: Phase 1 staging
> (`fota_stage`), Phase 2 A/B swap + confirm + revert (`fota_app`), and **download resume** — a
> download interrupted mid-transfer (reset at ~43 KB) **resumed from the persisted high-water
> mark** (`resume from 43008`), finished, and swapped. The vendor key is a **DEV key** —
> replace before shipping.

FOTA builds on the radio **bulk transport** (see [`radio.md`](radio.md)) and the **console**
(see [`console.md`](console.md) — also the host-proxy link).

---

## How an update happens

The signature/hash check runs **in the bootloader**, not the app. That keeps the Ed25519
verifier (salty, ~15 KB) out of the *duplicated* A/B app slots — the flash saving that makes a
radio + crypto OTA node fit the L083's 192 KB at all — and runs the verify on the loader's
clean, deep stack. The app's job is light and crypto-free.

The pieces:

1. **Transport** — the radio bulk-pull (`radio.md`): gateway *serves*, node *pulls* ≤64 B
   chunks, constant-RAM streaming.
2. **Host-proxy** — the gateway holds no image; it fetches each chunk from the **host** over
   the console link on demand (`tower fota serve`; see *Design decisions*).
3. **Staging** — the node streams the image into the spare **DFU** slot and stashes the signed
   **manifest** in the MANIFEST region for the bootloader.
4. **Bootloader** — on the next reset it verifies the manifest's **Ed25519 signature** and that
   the **image digest of DFU == manifest.sha256**, then arms + performs the A/B swap (rollback if the new
   image doesn't confirm).
5. **Control protocol** — advertise (a bit on existing ACKs) → pull manifest → cheap policy →
   pull image → stash manifest → reset.

Lifecycle (node = v1 in ACTIVE, the bootloader runs first on every reset):

```
  boot ─► running v1 in ACTIVE ──(idle)──► gateway advertises (DOWNLINK_PENDING on its ACK)
                                                │
                                                ▼
                       pull signed manifest (1 bulk xfer, 116 B)
                                                │  Manifest::decode + supersedes + fits-ACTIVE
                                                ▼  (cheap policy — NO signature check here)
                       pull image ──► DFU slot (streamed from the host via the gateway)
                                                │  stash the signed manifest in MANIFEST region
                                                ▼
                                          SCB::sys_reset()
   ┌─────────────────────────────────────────────┘
   ▼
  BOOTLOADER: state==Boot && manifest staged?
       ├─ verify Ed25519(manifest) + digest(DFU)==sha    ── invalid ─► erase manifest, boot v1
       ▼ valid
     erase manifest ─► mark_updated ─► SWAP ACTIVE⇄DFU (page by page, ~2.5 min) ─► jump
                                                │
                                                ▼
  v2 boots (State::Swap) ─► mark_booted ─► *** UPDATE CONFIRMED *** (persist installed_version)
                                                │
                              (no confirm + power-cycle)
                                                ▼
  BOOTLOADER reverts to v1 (State::Revert) — brick-safe
```

---

## Flash layout

Fixed partitioning of the 192 KB flash, shared by the bootloader and app. Offsets are **from
the flash base** `0x0800_0000`. Constants live in `src/fota/mod.rs` with **compile-time
guards** (a bad layout fails the *build*, not the board); the bootloader and app `memory.x`
mirror them.

| Region | Abs addr | Offset | Size | Purpose |
|---|---|---|---|---|
| BOOTLOADER | `0x0800_0000` | `0x00000` | 20 KB | loader + **Ed25519 verify (salty) + SHA-512 digest** + swap |
| BOOTLOADER_STATE | `0x0800_5000` | `0x05000` | 12 KB | embassy-boot swap magic + per-page progress |
| MANIFEST | `0x0800_8000` | `0x08000` | 256 B | the app stashes the signed manifest here for the loader |
| ACTIVE | `0x0800_8100` | `0x08100` | 77.75 KB | running app (256 B-aligned for the M0+ vector table) |
| DFU | `0x0801_B800` | `0x1B800` | 78 KB | staging slot (**must be > ACTIVE** by ≥1 page) |
| (spare) | `0x0802_F000` | `0x2F000` | 4 KB | margin / future |

Page (erase) 128 B, word (program) 4 B. BOOTLOADER is **20 KB** because it carries the verifier
(without it the slots would be 64 KB and a real OTA node wouldn't fit — see *Caveats: making it
fit*); the loader is ≈16 KB (it reuses salty's SHA-512 for the image digest rather than a second
hash crate), so 20 KB leaves ~4 KB margin (a link error if it ever overflows; `just size-check`
trips ~2 KB earlier) and the 12 KB reclaimed from the old 32 KB region went to ACTIVE/DFU. The
MANIFEST region is the only image metadata that crosses the app↔loader boundary.

---

## Opting out: full-flash builds without FOTA

FOTA is **opt-in**, and it costs flash: the 77.75 KB ACTIVE slot above is what's left after the
20 KB bootloader, 12 KB swap-state, 2 KB manifest, and the 78 KB DFU staging mirror — the
inherent price of safe A/B + rollback. A product that doesn't need over-the-air updates simply
**doesn't enable it, and the application gets the whole chip**:

| Build | Linker map | App flash |
|---|---|---|
| **default** (no `fota-active`) | full-chip `memory.x` @ `0x0800_0000` | **192 KB**, no bootloader, no A/B |
| `--features fota-active` | ACTIVE slot @ `0x0800_8100`, under the bootloader | 77.75 KB |

A non-FOTA build pays **nothing** for FOTA: `tower::fota` is dead-code-eliminated when unused, and
`salty`/`embassy-boot` are never linked (salty lives only in the bootloader crate). So `just flash
<app>` with no feature is a plain single-image firmware with ~2.5× the room — every non-FOTA
example (`blinky`, `thermometer`, `radio_node`, …) already builds exactly this way.

**One source, both ways.** Gate the FOTA-specific code on the same `fota-active` feature, so it
compiles out of a full-flash build (the feature already controls the linker map via `build.rs`):

```rust,ignore
async fn run(b: Board) {
    // ... your application (sensors, radio, …) — identical in both builds ...

    #[cfg(feature = "fota-active")]
    {
        // Confirm a freshly-swapped image, then pull updates when idle. See
        // examples/fota_ota.rs for the full node: the boot-state confirm + fota::pull_update.
        // (The confirm path uses embassy-boot, so pull it in under the same feature.)
    }
}
app!(run);
```

Build full-flash with `just flash <app>`; build FOTA-capable with `just flash-fota <app>`
(which also merges in the bootloader). A downstream product crate can wrap this in
its own `fota` feature that forwards to `tower`'s `fota-active`.

---

## The bootloader (`crates/bootloader`) — the verifying gate

A minimal `embassy-boot` loader at `0x0800_0000` that is **also the FOTA authenticity gate**.
On every reset:

1. Peek the embassy-boot state (`BlockingFirmwareState::get_state`, no swap).
2. If `State::Boot` **and** a valid manifest is staged in MANIFEST: read the DFU image, check
   `verify_signed(VENDOR_PUBKEY, manifest)` (Ed25519, salty) **and** `digest(DFU[..size]) ==
   manifest.sha256` (the digest is SHA-512 truncated to 256 bits, reusing salty's hash). **Erase
   the manifest** (clear the request), then — only if valid —
   `mark_updated()` (arm the swap).
3. `BootLoader::prepare` performs / resumes / reverts the A/B swap; jump to ACTIVE.

**Resume-safety.** The verify runs *only* on a clean `State::Boot` with a staged manifest. A
swap already in flight (`State::Swap`/`Revert`) is resumed/reverted by embassy-boot without
re-verifying. The manifest is erased *before* arming, so a power loss between the two leaves no
manifest → the node simply re-pulls; it never re-installs a stale image.

**The STM32L0 erase-value gotcha (fixed).** L0/L1 flash erases to `0x00` (not `0xFF`);
embassy-boot's swap-progress logic depends on the erased value, so the shared `embassy-boot`
gets `features = ["flash-erase-zero"]` in both `crates/bootloader` and the app's dev-deps. Any
embassy-boot use on an L0/L1 needs this.

**Swap cost.** ~2.5 min for a 77.75 KB slot (slow L0 word-program, two copies/page), silent (the
loader has no console). Verify adds ~20 s (salty Ed25519). Single-bank-L0 swap is HW-proven.

---

## Throughput & time budget

The transfer is the slow part (drives UX, not feasibility). Measured streaming throughput is
**≈ 5.4–5.8 kbps** effective — the pull is per-chunk round-trip bound (one `BULK_REQ`/`BULK_DATA`
exchange per ≤64 B chunk); gateway airtime is **≈ 42 ms** per 64 B `BULK_DATA`.

| Image | Chunks | US915 (unthrottled) | EU868 (1 % duty) |
|---|--:|--:|--:|
| ~52 KB | ~830 | ~85 s | ~50–58 min |
| 64 KB | 1024 | **~94 s (measured)** | ~12 min (36 s burst + 1 % tail) |
| ~67 KB (`fota_ota`) | ~1060 | ~100 s (one-shot, the bench) | ≈ the whole 1 %/hour budget |

A full image on EU 868 is ≈ the entire hourly 1 % airtime budget (HW-observed stalling ~82 % in),
which is correct regulatory behaviour, not a fault. **Download resume** carries it across duty
windows: each `pull_update` stages what the budget allows and returns `InProgress`; the next call
continues from the persisted mark once the bucket refills. So budget minutes-to-hours on EU, or
schedule overnight. The `fota_ota` bench uses `Band::Us915` (unrestricted duty) only to keep the
demo to one pass — switch to `Band::Eu868` for a real EU node (resume then carries it). **Delta
updates** (ship only changed pages) would cut both time and airtime dramatically — a candidate
for later, not the first cut (see *Design decisions*). Then ~20 s verify + ~2.5 min swap (silent,
in the loader) complete the install.

---

## Building, signing & flashing

The Radio Dongle has no SWD, so the bootloader + ACTIVE-linked app are **merged into one image**
flashed at `0x0800_0000` (`tools/fota_merge.py`).

```sh
# Phase 1 — flash staging, single board (no bootloader, no radio):
just run fota_stage                       # → *** ALL PASS ***

# Phase 2 — A/B self-swap test (unsigned), single board:
just flash-fota fota_app                # bootloader + fota_app, merged + flashed
just logs                                 # → *** SWAP CONFIRMED *** (~2.5 min swap)
TOWER_FEATURES=fota-no-confirm just flash-fota fota_app   # auto-revert test → *** REVERTED ***

# Real-firmware swap E2E (fota_ota) — two boards + host:
just fota-ota-v2                          # build + sign the v2 image the host serves
TOWER_PORT=<node-port> TOWER_FEATURES=role-node just flash-fota fota_ota   # node v1 (merged)
TOWER_FEATURES=role-gateway TOWER_PORT=<gw-port> just flash fota_ota    # gateway
tower -p <gw-port> fota serve --image target/fota-ota-v2.bin \
                              --manifest target/fota-ota-v2.fmanifest    # host-proxy
TOWER_PORT=<node-port> just logs          # → *** UPDATE CONFIRMED *** booted swapped v2
```

Host signer (`tools/fota-sign`, a std host binary): `just fota-sign pubkey` /
`just fota-sign sign --version N --in fw.bin --out fw.fmanifest`.

---

## The signed manifest (`tower_protocol::fota`)

The install gate is one **Ed25519 signature over a fixed 52-byte manifest**, in the *shared*
`tower-protocol` crate so the host signer and the device can't drift.

```
manifest (52 B, LE):  MAGIC "TWFM" (4) | FORMAT=1 (1) | flags (1) | reserved (2)
                      | hw_id (4) | version (4) | size (4) | sha256(image) (32)
signed blob (116 B):  manifest (52) ‖ Ed25519 signature (64)
```

- **`verify_signed(VENDOR_PUBKEY, signed) → Option<Manifest>`** (salty) runs in the
  **bootloader** (behind tower-protocol's `verify` feature — which the `tower` lib does *not*
  enable, so salty stays out of the app). `VENDOR_PUBKEY` lives in `tower_protocol::fota` (the
  loader's trust anchor); **it's the DEV key** (`fota-sign pubkey`) — replace it + the host key
  before shipping.
- **App-side policy is crypto-free:** `Manifest::decode` + `supersedes(installed)` (rollback
  floor, §6.2) + size-fits-ACTIVE. Rollback stays app-side because it's app EEPROM state
  (`KEY_INSTALLED_VERSION`); the bootloader gates *authenticity + integrity*.

---

## Device-side staging + the OTA driver (`tower::fota`)

| Item | What |
|---|---|
| `Stage` | erase/program/read window over one flash slot (offsets relative to the slot base) |
| `FlashSink` | a `BulkSink` that streams a received image into DFU, folding the image digest (truncated SHA-512; used by `fota_stage`) |
| `Net::bulk_fetch_to_flash(src, base, size, start, progress_key) → usize` | the OTA staging pull, **resumable**: programs each radio chunk into the slot via `Net`'s own flash from `start`; persists the staged count to `progress_key` every ~2 KB; no hashing (the bootloader does that) |
| `HostProxySource` | a `BulkSource` the gateway serves from, fetching each chunk from the host (next section) |
| `pull_update` / `PullOutcome` | the node OTA driver (resume-aware) |

**`pull_update(net, gateway) → PullOutcome`** (pure SDK, no crypto): read the rollback floor →
`bulk_fetch` the signed manifest → `Manifest::decode` + `supersedes` + fits-ACTIVE → decide the
**resume offset** (`KEY_DOWNLOAD_HWM` if `KEY_DOWNLOAD_IDENT` == this version, else 0 + re-erase)
→ `bulk_fetch_to_flash` from there → on completion **stash the signed manifest** in MANIFEST
*last*. Returns `PullOutcome::{NoManifest, Malformed, NotNewer, TooLarge, InProgress{staged,
total}, ImageFailed, Staged{manifest}}`. On `Staged` the app **resets** (the bootloader verifies
+ swaps); on **`InProgress`** the download is partly staged (duty stall / reset) — **call
`pull_update` again to resume** from the persisted mark (no re-erase, no re-download). The HWM is
persisted periodically *and* on stop, so a power-cut mid-download resumes too — HW-verified.

**Flash ownership.** `Net` owns the one `Flash` (via its `Kv`); `bulk_fetch_to_flash` writes the
DFU/MANIFEST through `Net`'s own flash, touched only *between* radio receives.

**Advertise.** The gateway rides `flags::DOWNLINK_PENDING` on the auto-ACKs the node already
gets (`set_downlink_pending` / `take_downlink_pending`) — no extra airtime.

The node also confirms a swapped boot: `BlockingFirmwareState::get_state` → on `State::Swap`,
`mark_booted` + `set_installed_version`. **Do this in a synchronous `#[inline(never)]` helper**
— see *Caveats: making it fit*.

---

## Host-proxy serve (`tower fota serve`)

The gateway holds no image (decision #7); it streams each chunk from the host over the console
link. Two raw-payload message types (`tower-protocol`, no postcard so the host needs only
COBS+CRC):

- **`FotaReq` (target→host):** `offset(4 LE) ‖ len(2 LE)`. `offset == FOTA_MANIFEST_OFFSET`
  (`u32::MAX`) requests the signed manifest.
- **`FotaData` (host→target):** `offset(4 LE) ‖ bytes`.

`HostProxySource::connect(rx)` fetches the manifest once (its `size` becomes the served length),
then `read(offset, out)` does one host round-trip per radio chunk. A short/late reply is
rejected by the length-checked bulk fetcher and the node's `BULK_REQ` retransmit drives another
fetch — a slow/dead host degrades to a failed pull, never a corrupt image.

Host side: **`tower fota serve --image <bin> --manifest <fmanifest>`** opens the serial port and
answers `FotaReq` with `FotaData` (reconnects if the gateway resets). It ships in the `tower` CLI
([tower-cli](https://github.com/hardwario/tower-cli)), which pins the shared `tower-protocol`
crate at the same tag as the firmware.

---

## Examples

| Example | Boards / features | What it shows |
|---|---|---|
| `fota_stage` | 1 | Phase 1: stream 4 K/16 K/64 K into DFU via `FlashSink` → `*** ALL PASS ***` |
| `fota_app` | 1 · `fota-active` [`,fota-no-confirm`] | Phase 2: unsigned self-swap → `*** SWAP CONFIRMED ***` / `*** REVERTED ***` (sets `Swap` directly; bypasses the manifest gate — a bench test) |
| `fota_ota` | 2 + host · `role-gateway` / `role-node,fota-active` [`,fota-v2`] [`,fota-diag`] | the real-firmware OTA swap: host-proxy serve + ACTIVE-linked node → bootloader-verified swap → `*** UPDATE CONFIRMED ***` |

`fota_ota` is the flagship. The node is ACTIVE-linked (merged with the bootloader); the gateway
proxies the host-served image. Build a v2 with `fota-v2` (version bump); the bootloader installs
it over v1. `fota-diag` adds stage logging.

---

## Parameters reference

| Constant | Value |
|---|---|
| Flash base / page / word | `0x0800_0000` / 128 B / 4 B |
| BOOTLOADER / STATE / MANIFEST / ACTIVE / DFU | 20 K@0 / 12 K@0x5000 / 256 B@0x8000 / 77.75 K@0x8100 / 78 K@0x1B800 |
| Manifest / signature / signed blob | 52 B / 64 B (Ed25519) / 116 B |
| Verify location | **bootloader** (salty); rollback (version) = app-side policy |
| Image digest | SHA-512 truncated to 256 bits (bootloader reuses salty's hash; no separate sha2) |
| EEPROM keys | `KEY_DOWNLOAD_HWM 0x5401`, `KEY_INSTALLED_VERSION 0x5402`, `KEY_DOWNLOAD_IDENT 0x5403` |
| Host-proxy msgs | `FotaReq` (=7, target→host), `FotaData` (=18, host→target), raw payloads |
| Swap time / verify time | ~2.5 min (77.75 KB slot) / ~20 s (Ed25519 @16 MHz) |
| Node size (fits 77.75 K ACTIVE) | ~67 KB (fota_ota), ~10.5 KB headroom |
| Vendor key | `tower_protocol::fota::VENDOR_PUBKEY` — **DEV key**, replace for production |

---

## Security model

- **The signature is the gate, not the transport.** AES-128-CCM secures the link, but the link
  key is derivable from the public pairing key — authenticity comes only from the **Ed25519
  signature over the manifest, checked in the immutable bootloader** before the swap. An app
  can't be tricked into skipping it.
- **Integrity:** the manifest commits to the image digest (`sha256` field) + `size`; the
  bootloader recomputes the digest — SHA-512 truncated to 256 bits, reusing salty's hash — over
  the staged DFU and compares.
- **Rollback:** `version` is inside the signed bytes; the app refuses `version ≤ installed`
  (persisted in EEPROM after a confirmed boot).
- **Brick-safety:** the swap is via embassy-boot, which reverts if the new image doesn't
  `mark_booted`.
- **Caveat:** `fota_app` arms a swap directly (`mark_updated`) with no manifest — a bench
  shortcut the bootloader passes through (it only gates when a manifest is staged). A hardened
  product would forbid app-side `mark_updated`. Enable flash RDP + bootloader WRP for production
  key/image protection.

---

## Design decisions

The choices that shaped the subsystem (and the alternatives rejected) — the *why* behind the
as-built design above.

- **Bootloader = `embassy-boot`.** A/B + swap-state machine + verified-update API, Embassy-native
  and already proven on this stack. We don't roll our own swap journal.
- **Verify in the bootloader, not the app** *(the load-bearing pivot)*. The plan first verified
  app-side; a radio + crypto + bootloader node is then ~83 KB and won't A/B on 192 KB. Putting the
  Ed25519 verifier (salty, ~15 KB) in the single immutable loader instead of the *duplicated* A/B
  app slots drops the node to ~66 KB — and it runs on the loader's clean stack, and an app can't be
  tricked into skipping it. Rollback (version) stays app-side because it's app EEPROM state.
- **Signature scheme = Ed25519.** 64-byte sig, fast on the M0+, deterministic (no RNG at verify),
  mature no_std crates (salty on-device, ed25519-dalek on the host — RFC 8032, so they interop).
  One signature over a fixed 52-byte manifest carrying `sha256(image)` covers integrity **and**
  authenticity in one check.
- **Gateway holds no image — host-proxy over USB.** On a 192 KB part there's no room for an
  image-storage slot on the gateway, so it streams each chunk from the host on demand
  (`tower fota serve`). Consequence: there is no flash-backed serve-side source; staging validates
  only the (hard) write side.
- **Transport = reuse `bulk`, no new frame type.** The signed manifest and the image are two
  ordinary bulk transfers; advertising rides `flags::DOWNLINK_PENDING` on the auto-ACKs the node
  already gets (no extra airtime, no new `FrameType`).
- **Resume = single high-water mark** (restart from the last contiguous chunk), persisted in
  EEPROM and paired with a download-identity key so only the *same* image resumes. Simple and
  enough; a chunk-bitmap is a possible later refinement.
- **Flash ownership = write through `Net`'s own `Flash`** (rejected alternative: share the `Flash`
  via `Mutex<RefCell<Flash>>`). `bulk_fetch_to_flash` programs each chunk through the flash `Net`
  already owns, touched only *between* radio receives — no borrow conflict with `&mut self`, and no
  rewrite of `storage.rs`/`Net::new`/`board.rs`/every radio example for little gain.
- **Update policy = scheduled / idle pull.** The node controls *when* it updates (battery + duty),
  the gateway only advertises.
- **Delta updates = out of scope for the first cut.** Full image first; changed-page deltas (which
  would cut EU transfer time a lot) are a later candidate.

---

## Known limitations & caveats

- **Making it fit (the hard-won part).** A radio + crypto + bootloader OTA node is ~83 KB with
  app-side verify — too big to A/B on 192 KB (`2×83 + boot + state > 192`). Two fixes, both
  applied: (1) **verify in the bootloader**, not the app → salty (~15 KB) is in the single
  bootloader copy, not the duplicated A/B app slots → the node drops to ~67 KB and fits a 77.75 KB
  ACTIVE (the bootloader, ≈16 KB after it reuses salty's SHA-512 for the image digest, only
  needs a 20 KB region with ~4 KB margin — the slack went to ACTIVE/DFU). (2) The node's boot-state check
  (embassy-boot) must run in a **synchronous
  `#[inline(never)]`** helper: inlined into the radio task's async poll it ballooned the frame
  to **14 KB** (opt-`s` doesn't reuse stack slots across a giant inlined state machine), which
  overflowed the L0's ~10 KB stack → a **silent HardFault → reset loop** (no console, since the
  loader has no console and the app never drained). As a separate sync fn the boot-state frame
  is ~160 B and the radio poll frame is ~5.4 KB → fits. *Lesson: on a 20 KB-RAM L0, keep
  multi-KB-stack code (crypto, embassy-boot) out of a big async task — its own task or a
  synchronous `#[inline(never)]` fn.*
- **EU duty throttles a full image — resume covers it.** A 67 KB image is ≈ the entire EU 868
  1 %/hour airtime budget; HW-observed the transfer stalling ~82 % in. **Download resume** (above)
  handles this: each `pull_update` stages what the budget allows, returns `InProgress`, and the
  next call continues from the persisted mark once the duty bucket refills — so a full image
  completes across duty windows (slow on EU; budget for ~minutes-to-hours, or schedule overnight).
  The `fota_ota` **bench** uses `Band::Us915` (unrestricted duty) to transfer in one shot —
  bench-only (915 MHz single-channel isn't FCC §15.247-compliant); change to `Band::Eu868` for EU
  (resume then carries it across windows).
- **Resume granularity.** The HWM is persisted every ~2 KB (`HWM_PERSIST_CHUNKS`), so a power-cut
  re-pulls ≤ ~2 KB; it's a same-size in-place EEPROM rewrite of one cell (no compaction churn). A
  chunk-bitmap (vs a single contiguous mark) is a possible later refinement.
- **Swap ~2.5 min, verify ~20 s** — both silent/blocking; expected on the L0, fine for a rare
  update.
- **The vendor key is a DEV key** (published seed in `fota-sign`) — replace before shipping.
- **STM32L0 erases to `0x00`** — any embassy-boot use needs `flash-erase-zero`.

---

## Testing

- **Host (`just test`):** the `fota-sign` signer — host↔device Ed25519 interop (dalek signs,
  salty verifies) and the `DEV_SEED`↔`VENDOR_PUBKEY` pin. The manifest codec + Ed25519
  accept/tamper/old/wrong-key rejection + rollback gate run in the
  [`tower-protocol`](https://github.com/hardwario/tower-protocol) repo (`cargo test --features verify`).
- **On hardware:** `fota_stage` (staging), `fota_app` (swap + confirm + revert), and `fota_ota`
  (full real-firmware swap: pull → bootloader verify → swap → confirm) — all verified on Radio
  Dongles + host.

---

## File / pointer reference

- **Device FOTA module: `src/fota/`** — `mod.rs` (layout constants + guards, EEPROM keys),
  `flash.rs` (`Stage`), `sink.rs` (`FlashSink`), `ota.rs` (`pull_update`, `PullOutcome`,
  `installed_version`/`set_installed_version`), `hostproxy.rs` (`HostProxySource`).
- **Staging pull / flash reclaim:** `src/radio/net/bulk.rs` (`bulk_fetch_to_flash`),
  `src/radio/net/mod.rs` (`into_kv`, `set/take_downlink_pending`), `src/storage.rs`.
- **Shared manifest + host-proxy protocol:** the [`tower-protocol`](https://github.com/hardwario/tower-protocol)
  repo — `src/fota.rs` (`Manifest`, `verify_signed`, `VENDOR_PUBKEY`, `FOTA_MANIFEST_OFFSET`),
  `src/lib.rs` (`FotaReq`/`FotaData`, `encode_frame_raw`).
- **Bootloader (the verifier):** `crates/bootloader/` (`main.rs`, `memory.x`, `Cargo.toml`).
- **App linking:** `link/memory-fota-app.x` + `build.rs`, `fota-active` Cargo feature.
- **Host tools:** `tools/fota-sign/` (signer), `tools/fota_merge.py` (merge). `tower fota serve`
  (in `tower-cli`). `just` recipes: `flash-fota`, `fota-sign`, `fota-image`, `fota-ota-v2`.
- **Examples:** `examples/fota_stage.rs`, `examples/fota_app.rs`, `examples/fota_ota.rs`.
- **Design rationale + caveats:** the *Design decisions* and *Known limitations & caveats*
  sections above. (The original phased plan — `FOTA.md` — has been folded into this guide and
  removed; the per-phase build history lives in git, like the radio/console plans before it.)
