# HARDWARIO TOWER Firmware SDK — task runner (https://just.systems)
#
# The crate is a library; flashable programs come in two kinds — `example` (educational,
# examples/) and `app` (a ready-made TOWER IoT Kit product, apps/). build/flash/run/size take
# the kind then the name. Run `just` (or `just default`) for the full recipe list. Common flow:
#
#   just examples                  # list example names    (just apps  lists product names)
#   just flash example blinky      # build + flash the blinky example
#   just run app radio_push_button # build + flash a product, then stream the (framed) console logs
#
# Override the serial device: TOWER_DEVICE=/dev/cu.usbserial-XXXX just flash example blinky
# Pass cargo features:        TOWER_FEATURES=role-gateway just flash example net_confirmed
#
# Flashing + console use the `tower` CLI (https://github.com/hardwario/tower-cli): it programs
# the STM32L0 over the UART bootloader (the jolt engine, as a library) AND decodes the firmware's
# framed console (`just logs`, not a raw serial terminal). Install it on your PATH (e.g.
# `cargo install --path ../github/tower-cli`, or grab a release binary).
#
# CROSS-PLATFORM: every recipe is a single program invocation (no bash, no sed/awk/ls pipelines),
# so the same `just` recipes run on Linux, macOS, and Windows. The one genuinely script-shaped
# step lives in Python (tools/protocol_pin_check.py) rather than inline shell.
# Windows uses cmd.exe (below) so exit codes propagate to CI; the one directory listing that must
# differ per-OS has [unix] / [windows] variants.

# On Windows, run recipe lines with cmd.exe (always present; propagates child exit codes — unlike
# `-Command` PowerShell). Unix keeps the default `sh`. See the cross-platform note above.
set windows-shell := ["cmd.exe", "/c"]

# Merged firmware image written by `build` and flashed by `flash`.
bin := "target/firmware.bin"

# Python launcher: `python3` on Unix, `python` on Windows (the python.org installer name).
python := if os() == "windows" { "python" } else { "python3" }

# Optional serial device; empty => let `tower` auto-detect the only USB serial device present.
# Read from TOWER_DEVICE; passed to `tower`/`jolt` as `-d`.
device := env_var_or_default("TOWER_DEVICE", "")
_device_flag := if device == "" { "" } else { "-d " + device }

# Optional cargo features, e.g. a radio example's role selection:
#   TOWER_FEATURES=role-gateway just flash example net_confirmed
features := env_var_or_default("TOWER_FEATURES", "")
_feat_flag := if features == "" { "" } else { "--features " + features }

# Host triple — host-side tests must build for the host, since the workspace default target
# (thumbv6m, see .cargo/config.toml) has no libtest / panic handler. `--print host-tuple` avoids
# parsing `rustc -vV` and works on every OS.
host := trim(`rustc --print host-tuple`)

# NOTE on comments below: `just --list` shows the LAST comment line above a recipe, so each recipe
# ends its comment block with a one-line summary (examples/detail, if any, come first).

# List the available recipes.
default:
    @just --list


# === Targets ======================================================================================
#
# A buildable target has a KIND and a NAME, and `build`/`flash`/`run`/`size` take both:
#   example  — educational, demonstrates one block      (examples/*.rs, cargo --example)
#   app      — a ready-made TOWER IoT Kit product        (apps/*.rs,     cargo --bin)
#     just build example blinky
#     just flash app radio_push_button
# `just examples` / `just apps` list the names of each kind.

# List the example apps (examples/*.rs) — build/flash/run them as kind `example`.
[unix]
examples:
    @ls examples/*.rs | sed 's|examples/||; s|\.rs$||'

[windows]
examples:
    @powershell -NoProfile -Command "Get-ChildItem examples/*.rs | Select-Object -ExpandProperty BaseName"

# List the TOWER IoT Kit product firmwares (apps/*.rs) — build/flash/run them as kind `app`.
[unix]
apps:
    @ls apps/*.rs | sed 's|apps/||; s|\.rs$||'

[windows]
apps:
    @powershell -NoProfile -Command "Get-ChildItem apps/*.rs | Select-Object -ExpandProperty BaseName"

# `kind` is example|app; set TOWER_FEATURES to pass cargo features (e.g. a radio example's role):
#   just build example blinky
#   just build app radio_push_button
# Build a target into target/firmware.bin, then print its on-chip size.
build kind name: && (size kind name)
    cargo objcopy --release {{ if kind == "example" { "--example" } else if kind == "app" { "--bin" } else { error("kind must be 'example' or 'app' (got '" + kind + "')") } }} {{name}} {{_feat_flag}} -- -O binary {{bin}}

# On-chip footprint (text/data/bss) of a target. `kind` is example|app.
size kind name:
    cargo size --release {{ if kind == "example" { "--example" } else if kind == "app" { "--bin" } else { error("kind must be 'example' or 'app' (got '" + kind + "')") } }} {{name}} {{_feat_flag}}


# === Flash & run ==================================================================================

# `kind` is example|app; extra args after the name pass through to `tower flash`; set
# TOWER_FEATURES for cargo features:
#   just flash example blinky --no-verify
#   TOWER_FEATURES=role-gateway just flash example net_confirmed
# Build + flash a target — erase/write/verify/reset.
flash kind name *args: (build kind name)
    tower {{_device_flag}} flash {{bin}} {{args}}

# Build + flash a target (kind example|app), then stream its framed console logs (resets first).
run kind name: (flash kind name)
    tower {{_device_flag}} logs


# === Console & device control =====================================================================

# Stream the decoded framed console logs from the running MCU (extra args -> `tower logs`).
logs *args:
    tower {{_device_flag}} logs {{args}}

# Open the full-screen TUI console (logs + events + interactive shell).
console:
    tower {{_device_flag}} console

# Reset the MCU into the application (add `--bootloader` to enter the system bootloader).
reset *args:
    tower {{_device_flag}} reset {{args}}

# Erase the entire device flash, then reset into the application.
erase:
    tower {{_device_flag}} erase

# List the connected serial devices (TOWER boards).
devices:
    tower devices


# === Tests & checks ===============================================================================

# Builds for the host triple (the firmware itself is no_std): `tower-kv` (the EEPROM key-value
# codec) and `tower-radio-core` (the EU/US/FHSS duty token-bucket math + the fixed FHSS hop
# permutation — the regulatory arithmetic). The shared wire-protocol tests live in
# github.com/hardwario/tower-protocol.
# Run the firmware's host-side unit tests.
test *args:
    cargo test -p tower-kv --target {{host}} {{args}}
    cargo test -p tower-radio-core --target {{host}} {{args}}

# postcard is NOT self-describing, so a tower-protocol tag mismatch silently mis-decodes the wire
# (never a build error) — this makes it a hard failure. Local half of the CI `lockstep` job (which
# also fetches tower-cli's pin); the golden rule in the repo CLAUDE.md. tools/protocol_pin_check.py.
# Verify the tower-protocol git-tag pin is identical across all in-repo manifests.
check-protocol-pin:
    {{python}} tools/protocol_pin_check.py


# === Hardware-in-the-loop (HIL) ===================================================================
#
# The HIL harness (tools/hil, a std host crate excluded from the workspace) drives
# the real bench: a TOWER Core Module (J-Link SWD + Nordic PPK2) + a TOWER Radio Dongle (USB). It
# decodes the framed console natively (tower-protocol) and asserts on typed Log/Event + seq-gaps.
# The bench roster lives in tools/hil/hil.toml (re-resolved against `tower devices` at startup).
#
# `--test-threads=1` is LOAD-BEARING: the serial ports are exclusive, so tests must not run
# concurrently. HW-touching tests are `#[ignore]`d, so `--ignored` opts INTO the bench run; a plain
# `cargo test` (which `cargo check`/CI would do) only COMPILES them.

# Run the smoke + radio HIL groups on the bench (needs the Dongle + Core; NOT run in CI).
hil *args:
    cargo test --manifest-path tools/hil/Cargo.toml --target {{host}} -- --ignored --test-threads=1 {{args}}

# Run the feature-gated power HIL group (needs the Core on J-Link + PPK2, FTDI UNPLUGGED).
hil-power *args:
    cargo test --manifest-path tools/hil/Cargo.toml --target {{host}} --features power -- --ignored --test-threads=1 {{args}}

# Run every HIL group (smoke + radio + power) on the fully-cabled bench.
hil-full *args:
    cargo test --manifest-path tools/hil/Cargo.toml --target {{host}} --features power -- --ignored --test-threads=1 {{args}}


# === Maintenance ==================================================================================

# Remove build artifacts.
clean:
    cargo clean
