# HARDWARIO TOWER Firmware SDK (Embassy)

An [Embassy](https://embassy.dev) firmware SDK for the **HARDWARIO TOWER Core
Module** (STM32L083CZ). The crate is a **library** of reusable blocks (LED,
button, TMP112 thermometer, LIS2DH12 accelerometer, addressable-LED strip, a
framed host↔target **console** (logs/events/shell), EEPROM storage, USB-gated low
power) plus a **SPIRIT1 sub-GHz radio stack** (secured AES-128-CCM network layer —
confirmed delivery, replay protection, bulk transfer, OTA pairing); flashable
programs live in [`examples/`](examples) and are built/flashed by name with
[`just`](https://just.systems). The console and radio each have a guide:
[`docs/console.md`](docs/console.md) and [`docs/radio.md`](docs/radio.md).

| | |
|---|---|
| MCU | STM32L083CZ (Arm Cortex-M0+) |
| Target | `thumbv6m-none-eabi` |
| Clock | sysclk = HSI16 (16 MHz); RTC ← LSE 32.768 kHz crystal (PC14/PC15), STOP-mode wake |
| LED | PH1, active-high |
| Button | PA8, active-high (external pull-down), EXTI |
| I2C | I2C2 — PB10/PB11 (AF6), 100 kHz; TMP112 @ `0x49`, LIS2DH12 @ `0x19` |
| Accelerometer | LIS2DH12 — INT1 → PB6 (EXTI); orientation/dice + tilt |
| Console | USART1 — TX PA9 / RX PA10, 115200 8N1; framed host↔target link (logs/events/shell), see [`docs/console.md`](docs/console.md) |
| RGB strip | WS2812B/SK6812 on PA1 — TIM2_CH2 PWM + DMA1_CH3 |
| EEPROM | 6 KB byte-addressable data EEPROM @ `0x0808_0000` (no erase, ~100k+ cycles) |
| USB sense | VBUS on PA12 — gates STOP (stay awake while plugged in) |
| Radio | SPIRIT1 (SPSGRF) — SPI1 on PB3/PB5/PB4, CS PA15, SDN PB7, nIRQ PA7 (EXTI); EU 868 / US 915 (runtime-switchable); see [`docs/radio.md`](docs/radio.md) |

## Quick start

```sh
# One-time: cargo install just cargo-binutils   (+ rustup component add llvm-tools)
#           (add probe-rs-tools only for SWD `cargo run`; jolt UART flashing needs neither)
just samples              # list the example apps
just run thermometer      # build + flash, then watch the console from boot
just monitor              # (re)attach to a running MCU without resetting it
```

## Module layout

The library (`src/lib.rs`) exposes these reusable blocks:

| Module | Responsibility |
|---|---|
| `src/button.rs` | Debounced button driver (click/hold) over any GPIO; `init_exti` (low-power, sleeps when idle) or `init_polled` (when the EXTI line is taken) |
| `src/console.rs` | Framed host↔target console (`tower-protocol`): `log` backend, `print!`/`println!`, structured `event`s, and chunked shell responses over an interrupt-driven UART — paired with the `tower` host CLI; see [`docs/console.md`](docs/console.md) |
| `src/shell.rs` | RouterOS-style shell with target-authoritative TAB completion and a declarative, EEPROM-backed settings framework (`Str`/`Uint`/`Int`/`Bool`/`Enum`); apps deep-merge their own commands + settings via `serve_ext` — see [`docs/console.md`](docs/console.md) |
| `src/led.rs` | Non-blocking LED blink dispatcher (background pattern + priority instant sequences) |
| `src/lis2dh12.rs` | LIS2DH12 accelerometer (HAL-independent): 10 Hz/normal mode, `dice()` orientation (1–6), and a hardware tilt/movement interrupt with selectable sensitivity + report `min_interval` |
| `src/power.rs` | `vbus_task` — gates STOP on USB presence via a `WakeGuard` |
| `src/storage.rs` | Non-volatile storage in the data EEPROM: a raw byte area (`read`/`write` at offset) and a key-value store (`Kv`) — values as raw scalars or postcard structs, CRC-framed, updated in place |
| `src/strip.rs` | LED-strip effects over `ws2812`: solid/compound/gradient + rainbow, chase, breathe, scanner, sparkle; brightness 0–100 % with gamma |
| `src/tmp112.rs` | TMP112 driver, generic over `embedded_hal::i2c::I2c` (HAL-independent) |
| `src/ws2812.rs` | WS2812B/SK6812 strip driver (PA1) — TIM2 PWM + DMA, RGB & RGBW, arbitrary length |
| `src/radio/` | SPIRIT1 sub-GHz radio stack: chip driver (SPI/state machine/CSMA/sleep), RF config, hardware AES-128-CCM, frame codec, EU duty governor, and a secured network layer (`net`) with per-peer keys, confirmed delivery, replay protection, bulk transfer and OTA pairing — see [`docs/radio.md`](docs/radio.md) |
| `src/board.rs` | `Board::take()` + `app!` — the common entry: clock, console, TMP112→one-shot, EXTI, radio pins, and USB-aware low power (auto-spawns `vbus_task`); logs a uniform `Example booted: <name>` banner and hands the app ready resources |

### LED indication

[`led`](src/led.rs) runs a dispatcher task that owns the pin (any GPIO via
embassy's type-erased `Output`, with an `active_high` polarity flag). The app
holds a cheap, copyable [`Led`] handle:

- `set_background(Some(pattern))` — a looping status pattern (`None` clears it).
- `play(sequence)` — a one-shot sequence that **preempts** the background, plays
  once, then the background resumes.
- `set_switch_delay(d)` — the off-gap inserted before an instant sequence
  interrupts a running background, so the two read as distinct (default 250 ms).

Patterns are `'static` slices of `Step` (`Step::on(ms)` / `Step::off(ms)`). The
`blinky` example sets a ~2 s heartbeat background and plays a double-blink every
5 s that preempts it; `button` flashes the LED on click/hold. A background
pattern wakes the MCU once per period even on battery, so clear it when
minimizing STOP power.

## Low power

The firmware runs on embassy-stm32's STOP-mode thread executor instead of the
default `embassy-executor` one. It is selected with the `executor`/`entry`
arguments on the `#[embassy_executor::main]` attribute, and enabled by the
`executor-thread` + `low-power` features on `embassy-stm32` (with the matching
features **removed** from `embassy-executor`, so there is only one `__pender`).

Whenever every task is idle, the executor drops the core into the deepest
available STOP mode. A general-purpose timer keeps `embassy-time` while the core
runs; on entering STOP that time is handed off to the **RTC**, which is clocked
from the LSE crystal (`config.rcc.ls = LsConfig::default_lse()`) and programmed
to fire the wake-up. So during an idle gap the MCU draws ~µA rather than
running continuously (~hundreds of µA). The TMP112 is independently low-power:
one-shot + shutdown keeps it at ~1 µA between conversions.

While USB is connected the board keeps itself awake instead:
[`Board::take`](src/board.rs) auto-spawns [`power::vbus_task`](src/power.rs),
which holds a `WakeGuard` whenever VBUS (PA12) is high — dropping idle to plain
Sleep (clocks live, so the console and EXTI stay responsive) rather than STOP.
Unplug and it returns to STOP. Every app gets this for free, so a debug session
never has to fight STOP latency or a clock-gated UART.

Two settings in [`board::init`](src/board.rs) matter here:

- `config.min_stop_pause` (`0`) — STOP threshold. **In embassy-stm32 0.6.0 a
  nonzero value is a hard floor on the shortest awaitable delay, not a power
  knob:** if the next alarm is sooner than the threshold, the time driver skips
  arming the RTC wake-up but the executor still enters STOP, leaving no wake
  source (the TIM is clock-gated) → the core hangs. Setting it to zero hands
  every idle off to the RTC, so any wait length is safe; the RTC alarm clamps
  sub-tick requests to a ~61 µs floor and wakes slightly early (the executor
  re-sleeps), so correctness holds and power stays optimal for realistic waits.
  See the long comment at the `min_stop_pause` assignment for how to turn this
  back into a tunable power knob (requires a fixed/newer embassy-stm32).
- `config.enable_debug_during_sleep` (`false`) — gating the debug clock domain
  is what actually lowers STOP current. Set it to `true` if you need SWD/RTT to
  stay alive while stopped (e.g. for `probe-rs`), at much higher STOP current.

> Keeping the blocking `I2c` alive does not block STOP on the L0: an enabled I2C
> only raises the *minimum* stop refcount, which still permits the L0's single
> STOP mode (it would only forbid the deeper STOP2 that the L0 doesn't have).
> A debug probe attached with `enable_debug_during_sleep = false` will lose the
> core during STOP — measure real current standalone.

## Examples

Each file in [`examples/`](examples) is a complete, flashable program. Add your
own by dropping a `.rs` there — it's picked up automatically (`just samples`).

| Example | Demonstrates |
|---|---|
| `blinky` | The `led` block — background heartbeat + priority instant blink |
| `button` | The `button` block — log press/release/click/hold, flash the LED |
| `thermometer` | `tmp112` — log the temperature every 2 s |
| `accelerometer` | `lis2dh12` — report die face 1–6 as you turn the board; opt-in tilt alert |
| `strip` | `ws2812` + `strip` — a scrolling rainbow on PA1 |
| `storage` | `storage` — a key-value store in EEPROM: a raw boot counter + a postcard settings struct, surviving reset |
| `i2cscan` | Probe the I2C2 bus and log responding addresses (diagnostic) |

The radio stack adds ~20 more (`radio_*`, `net_*`, `crypto_*`, `edge_*`) — the
reference apps `radio_gateway`/`radio_node` are the happy path; see the full table
and protocol guide in [`docs/radio.md`](docs/radio.md). Two-board examples are one
file built twice with a role feature, e.g. `TOWER_FEATURES=role-gateway just flash
net_confirmed`.

### Writing an app

The `app!` macro supplies the entry point and the always-on board setup —
clock, the serial console, the TMP112 put into one-shot (shutdown) mode, and
USB-aware low power (see above) — logs a uniform `Example booted: <name>` line,
then hands you a [`Board`](src/board.rs) of ready resources. A whole app is just:

```rust
#![no_std]
#![no_main]
use tower::{app, board::Board};

async fn run(mut b: Board) {
    // b.spawner, b.tmp112 (shut down), b.led, b.button, b.accel_int, b.storage, b.strip_* …
    loop {
        if let Ok(raw) = b.tmp112.oneshot().await {
            log::info!("{} raw", raw);
        }
    }
}
app!(run);
```

See `thermometer.rs` (≈12 lines of logic) for the minimal real example.

## Build / flash / monitor

Prerequisites (one-time): `cargo install just cargo-binutils probe-rs-tools`
and `rustup component add llvm-tools`.

```sh
just samples                 # list examples
just build blinky            # → target/firmware.bin (+ size)
just flash blinky            # build + flash over the UART bootloader (jolt)
just run thermometer         # build + flash, then monitor (resets on attach → catches boot)
just monitor                 # attach to the running MCU (no reset); add --reset to restart
just flash blinky --no-verify  # extra args pass through to `jolt flash`
```

Set `TOWER_PORT=/dev/cu.usbserial-XXXX` if more than one serial port is present.
`cargo run --release --example blinky` also flashes via the SWD probe-rs runner
in `.cargo/config.toml` if you use a J-Link/ST-Link instead of `jolt`.

`just build NAME` runs `cargo objcopy --example NAME` → `target/firmware.bin`
(linked at `0x08000000`). `jolt monitor` opens the port without resetting the
firmware (BOOT0/NRST are on the FTDI's auxiliary lines, not DTR/RTS); **close the
monitor before flashing** (`jolt flash` needs exclusive port access). Any serial
terminal works too, e.g. `picocom -b 115200 /dev/cu.usbserial-XXXX`.

## Tweaking

App-level constants (sensor address, intervals, pixel count, LED patterns, …)
live in each `examples/*.rs` and are meant to be edited or copied. Common knobs:

- **TMP112 address** — `Tmp112::new(i2c, tmp112::ADDR_VPLUS)` (`0x49`, this board
  straps ADD0 → V+); also `ADDR_GND` (`0x48`), `ADDR_SDA`, `ADDR_SCL`.
- **LED / button polarity** — `led::Polarity::ActiveHigh` / `ActiveLow` (same for
  `button::Polarity`).
- **Accelerometer** — `Lis2dh12::new(i2c, lis2dh12::ADDR_DEFAULT)` (`0x19`);
  `Accel::dice()` → face 1–6; tilt is opt-in via the example's `TILT` const
  (`TiltConfig { sensitivity: Sensitivity::{Low,Medium,High,Ultra}, min_interval }`).
  The accelerometer shares the I2C bus with the TMP112 — reclaim it with
  `b.tmp112.release()`.
- **Strip** — `Strip::new(.., LedKind::Rgb | Rgbw, brightness)`; effects take a
  frame counter you advance.
- **Storage** — wrap `b.storage` in `Kv::new(..)` for keyed values:
  `kv.set_bytes(key, &x.to_le_bytes())` / `kv.get_bytes` for scalars, or
  `kv.set(key, &value)` / `kv.get::<T>(key)` (postcard) for structs. Add a new
  `u16` key to persist new data without disturbing existing keys. Or use
  `b.storage.read/write(offset, ..)` for a raw byte layout.
- **I2C speed / pull-ups** — `i2c_config.frequency`, `scl_pullup`/`sda_pullup`.
- **Clock & low power** — all in [`board::init`](src/board.rs): sysclk, RTC
  source, `min_stop_pause`, debug-during-sleep.
- **TMP112 conversion-wait** — `POLL_MS` × `POLL_TRIES` in `src/tmp112.rs`.

## License

MIT — see [LICENSE](LICENSE). © 2026 HARDWARIO a.s.
