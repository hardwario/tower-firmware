# TOWER Storage — EEPROM key-value store & wear

The SDK persists small amounts of state — radio counters, per-boot session id, shell
settings, and whatever an app wants — in the STM32L083's **6 KiB data EEPROM**, through a
single shared key-value store (`storage::Nv`, codec in the `tower-kv` crate). This guide
covers the layout, how it wears, what the SDK writes and how often, the tuning knobs, and the
rules an app should follow so the EEPROM lasts the product's life.

> **Status: implemented and hardware-verified.** The store, the wear telemetry
> (`/system/eeprom print`), and the boot-loop backoff all run on the Core Module. The
> background maintenance task (incremental compaction, below) is implemented and host-tested;
> its stall-slicing numbers are datasheet/bench-derived but not yet re-verified on the bench.

## The store

- One store over the whole data EEPROM, namespaced so subsystems can't collide: `NS_NET`
  (radio), `NS_SYS` (console session), `NS_SHELL` (settings), `NS_APP` (yours). Take a
  scoped view with `b.kv.scope(NS_APP)` — every get/set is then keyed by an 8-bit local.
- Records are `[tag(2) | len(2) | crc(4) | value]`; values up to `MAX_VALUE` (256 B).
- Power-loss-safe: a torn write never corrupts a prior key (see the `tower-kv` module docs).
- **Delete** with `kv.delete(key)`: it appends a zero-length **tombstone** record (`len == 0`),
  so the key reads back absent (`get` → `None`) and the next compaction flip drops both the value
  and the tombstone — so the live set can **shrink**, not only grow. Append-only and power-loss-safe
  like any write; a redundant delete of an absent key is a wear-free no-op. (`len == 0` is therefore
  reserved as the tombstone marker — no key stores a genuinely empty value.)

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

## The compaction CPU stall — bounded by background maintenance

A compaction is not just wear — it is **time**: on the STM32L0, every EEPROM word write
stalls the CPU (NVM-stall — instruction fetches from flash halt, so interrupt handlers
effectively halt with them). Re-packing a full half synchronously is on the order of 1,500
word writes at ~3.4 ms each ≈ **5.2 s of the whole chip frozen** (bench-measured 2026-07-05:
5.16–5.20 s, flip-counter 1:1 with the stalls). Historically, whichever append happened to
fill the active half paid for the whole compaction — the boot-time session bump (a ~5 s
"hung" boot every Nth reboot), a `settings set`, a radio watermark persist — with the console
and radio dead for the duration.

The SDK now compacts **incrementally, in the background**. A maintenance task (spawned by
default by the `app!` macro — `storage::maintenance`) runs bounded slices of at most
`MAINT_BUDGET_WORDS` = 4 word-programs ≈ **14 ms** of stall each, yielding to the other tasks
between slices. It is **event-driven**: woken by KV writes (and once at boot), it drains all
work and then sleeps on a signal — an idle node takes zero extra wakeups and STOP stays
reachable. The two-half + superblock-commit design is what makes this safe: every pre-commit
step is invisible (the source half stays active until the final CRC'd superblock write), so a
reboot mid-maintenance loses only RAM progress and the flip restarts from scratch.

What the task does, in priority order:

1. **Advance a pending incremental flip** (`tower-kv` `flip_start`/`flip_step`/`flip_commit`):
   pass-1 scan into a RAM plan, then blank + copy a few words per slice, then one commit slice
   (tail catch-up of appends that landed during the flip + the superblock write). Appends keep
   going to the source half throughout; if it fills anyway, the write itself finishes the
   flip's *remaining* steps synchronously — never a failure, just latency.
2. **Start a flip proactively** once free space in the active half drops below
   `FLIP_THRESHOLD` = 528 B (two worst-case records — headroom for the appends that land
   between trigger and commit), and only when flipping would restore that headroom.
3. **Pre-blank the dead half** (read-first, skipping already-zero words — idempotent and
   wear-free on re-runs), so the next flip's blank pass costs nothing.

Residual stall budget:

| Path | Worst-case stall |
|---|---|
| Steady state (default `app!`: maintenance task running) | ~14 ms slices; commit slice ~tens of ms |
| Sync fallback, dead half pre-blanked (store filled mid-flip / no proactive room) | ≤ ~2.6 s (copy pass only — the blank pass reads-and-skips) |
| Sync fallback, maintenance never ran (custom entry without the task) | ~5.2 s (blank + copy), as before |

The boot-time session bump still appends one small record per boot, but in steady state it can
no longer land on a full half — maintenance keeps free space above the threshold — so the ~5 s
"hung boot" is gone. Products that want a full compaction at a moment they control (e.g. during
provisioning) can call `Nv::compact_now()`.

Pre-blanking adds **no extra wear**: it programs exactly the nonzero words the next flip's
blank pass would have programmed (which then skips them), just earlier and in slices. Note the
per-boot session id and TX watermark deliberately stay ordinary KV records: the KV region
occupies the entire 6 KiB EEPROM (3060 + 3060 + 2×12 superblocks — no reserved raw area), so
relocating them to dedicated raw cells would shrink the region and force a full store
migration; bounded maintenance removes the motive.

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

`/system/eeprom print` (or `Nv::flip_generation()` / `free_bytes()` / `live_bytes()` /
`dead_half_blank()`):

```
eeprom: 6 KiB data EEPROM
flips: 110 / 100000 (0.1%)
live: 84 B
free: 2620 B
dead-half blanked: yes
resets: 1
```

`flips` is the store's lifetime compaction count (the persisted superblock generation) against
`FLIP_BUDGET` — proactive background flips count here too. They trigger at the 528 B threshold
rather than at full, so each flip absorbs ~2.5 KB of appends instead of ~3 KB: a ~20 % higher
flip rate, well inside the conservative budget (the gauge already under-reports true life ~2×). `live` is the packed size of the
latest record per key; `free` the appendable room left in the active half (a flip reclaims
`used − live`); `dead-half blanked` reports whether maintenance has pre-blanked the inactive
half (the next flip then skips its blank pass). `resets` is the current consecutive-fast-reset
run (see below). Reading it is a pure EEPROM **read** — polling telemetry adds no wear.

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
