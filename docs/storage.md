# TOWER Storage — EEPROM key-value store & wear

The SDK persists small amounts of state — radio counters, per-boot session id, shell
settings, and whatever an app wants — in the STM32L083's **6 KiB data EEPROM**, through a
single shared key-value store (`storage::Nv`, codec in the `tower-kv` crate). This guide
covers the layout, how it wears, what the SDK writes and how often, the tuning knobs, and the
rules an app should follow so the EEPROM lasts the product's life.

> **Status: implemented and hardware-verified.** The store, the wear telemetry
> (`/system/eeprom print`), and the boot-loop backoff all run on the Core Module.

## The store

- One store over the whole data EEPROM, namespaced so subsystems can't collide: `NS_NET`
  (radio), `NS_SYS` (console session), `NS_SHELL` (settings), `NS_APP` (yours). Take a
  scoped view with `b.kv.scope(NS_APP)` — every get/set is then keyed by an 8-bit local.
- Records are `[tag(2) | len(2) | crc(4) | value]`; values up to `MAX_VALUE` (256 B).
- Power-loss-safe: a torn write never corrupts a prior key (see the `tower-kv` module docs).

## How it wears — it wear-levels

The 6144 bytes are **two 3060-byte halves** plus two 12-byte superblocks at the top.

- Writes **append** to the active half (each record at a fresh, advancing offset — no cell is
  rewritten in place). The free offset is derived by scanning, so an append touches **only**
  the record it writes.
- When the active half fills, `compact` **flips**: it packs the live set (latest record per
  key) into the *other* half and commits by writing that half's superblock with an incremented
  **generation**. The two halves alternate as active, so wear spreads across the whole region.
- The superblocks are the only fixed-location writes, but each is written once every *other*
  flip — the same rate the data cells are cycled — so there is **no disproportionate hot cell**.

Net: **wear is proportional to the flip rate, which is proportional to total bytes written.**
There is no per-write hot spot; the design spreads writes across all 6 KiB.

## Write budget

The STM32L083CZ data EEPROM endurance is **100,000 erase/write cycles per cell**
(`EEPROM_ENDURANCE_CYCLES`, datasheet-confirmed). One "cycle" is one erase plus one program of a
cell; every figure here scales with it.

Each compaction flip erases and reprograms the store's most-written cells — the re-packed
live-set prefix at each half's start, and the committed superblock. The gauge is kept
**conservative**: it charges one full erase/write cycle of the worst cell *per flip*, so

```
FLIP_BUDGET = endurance = 100,000 flips
```

In reality the store alternates two halves, so any given cell is the flip target only every
*other* flip — true life is closer to ~200,000 flips. The gauge therefore reports **more** wear
than real, never less. Either way a flip absorbs roughly one half (~3 KB) of appends, so total
write capacity is on the order of:

```
100,000 flips × ~3 KB ≈ 3 × 10⁸ bytes ≈ tens of millions of small records
```

Treat **~10⁷ writes** as the working budget. `/system/eeprom print` reports the live flip count
against `FLIP_BUDGET`.

## What the SDK writes, and how often

The SDK is deliberately frugal — nothing writes per radio message:

| State (namespace) | When it writes | Frequency |
|---|---|---|
| TX-counter watermark (`NS_NET`) | reserve-ahead: once per `RESERVE` = 1024 sends, and once at boot | amortized to ~nothing |
| Per-lane last-seen (`NS_NET`) | lazy: once per `P` = 32 *accepted receives* on a lane | **∝ RX traffic** |
| Session id (`NS_SYS`) | once per boot (suppressed while boot-looping) | ∝ boots |
| FHSS epoch (`NS_NET`) | once per gateway (master) boot | ∝ gateway boots |
| Settings (`NS_SHELL`) | on an explicit setting change | user-driven, rare |

For a typical sensor node this is a handful of writes per day → decades. The one path that
scales with load is a **busy gateway's last-seen** persistence (see `P` below).

## Tuning knobs (wear vs. behaviour)

Both live in the host-tested `crates/tower-net-core` decision crate (`txctr::RESERVE`,
`replay::P`), used by `src/radio/net/mod.rs`:

- **`RESERVE`** (watermark reserve block, default 1024): larger = fewer watermark writes, at the
  cost of a bigger counter jump per boot (counter space, not wear). Rarely needs changing.
- **`P`** (last-seen lazy-persist period, default 32): the main gateway wear knob. Larger `P` =
  fewer last-seen writes, but a **larger replay window across a reboot** — after a reset a
  receiver can accept up to `P` already-seen counters before the persisted last-seen catches up.
  Smaller `P` tightens that window at the cost of more writes. It is a security/wear trade-off,
  not a free dial.

## Wear telemetry

`/system/eeprom print` (or `Nv::flip_generation()`):

```
eeprom: 6 KiB data EEPROM
flips: 110 / 100000 (0.1%)
resets: 1
```

`flips` is the store's lifetime compaction count (the persisted superblock generation) against
`FLIP_BUDGET`; `resets` is the current consecutive-fast-reset run (see below). Reading it is a
pure EEPROM **read** — polling telemetry adds no wear.

## Boot-loop backoff

A unit stuck in a reset loop — a persistent hang caught by the watchdog, or a brown-out cycle —
would otherwise rewrite per-boot state on every reset and grind the store. `bootguard` counts
consecutive resets that never reached a **30 s healthy uptime** in a reset-surviving `.uninit`
RAM word (retained across a warm reset, never persisted → **zero EEPROM wear**). Once the run
reaches `BOOT_LOOP_THRESHOLD` (8), the SDK stops persisting the session counter (it reports a
RAM-derived id instead), so a wedged node can't wear the EEPROM one record per reset.

The **watermark is never suppressed** this way: it is the CCM nonce anti-reuse guarantee and must
keep failing closed even in a loop. Only wear-only, non-security per-boot state backs off.

## App guidance — the EEPROM is not RAM

The store is for configuration and slowly-changing state, not a scratchpad or a log:

- **Coalesce.** Keep hot values in RAM; persist only on a meaningful change, periodically at a
  low rate, or at shutdown — not every sample. One write per second exhausts the store in about
  a year; the budget is ~10⁷ writes, not unlimited.
- **Keep the live set small.** Storing many or large distinct keys that together approach the
  ~3 KB half size forces a flip on nearly every write (a flip storm). A handful of small keys
  leaves hundreds of appends between flips.
- **Batch related fields** into one record rather than writing several keys in a burst.
- Check `/system/eeprom print` during development to see how fast a workload is spending flips.
