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
# Override the serial port:   TOWER_PORT=/dev/cu.usbserial-XXXX just flash example blinky
# Pass cargo features:        TOWER_FEATURES=role-gateway just flash example net_confirmed
#
# Flashing + console use the `tower` CLI (https://github.com/hardwario/tower-cli): it programs
# the STM32L0 over the UART bootloader (the jolt engine, as a library) AND decodes the firmware's
# framed console (`just logs`, not a raw serial terminal). Install it on your PATH (e.g.
# `cargo install --path ../github/tower-cli`, or grab a release binary).
#
# CROSS-PLATFORM: every recipe is a single program invocation (no bash, no sed/awk/ls pipelines),
# so the same `just` recipes run on Linux, macOS, and Windows. The two genuinely script-shaped
# steps live in Python (tools/size_check.py, tools/fota_merge.py) rather than inline shell.
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
# (The env var stays TOWER_PORT for continuity; it's passed to `tower`/`jolt` as `-d`.)
port := env_var_or_default("TOWER_PORT", "")
_port_flag := if port == "" { "" } else { "-d " + port }

# Optional cargo features, e.g. a radio example's role selection:
#   TOWER_FEATURES=role-gateway just flash example net_confirmed
features := env_var_or_default("TOWER_FEATURES", "")
_feat_flag := if features == "" { "" } else { "--features " + features }
# A FOTA build always needs `fota-active` (link the app into the ACTIVE slot); append any extras.
_fota_features := if features == "" { "fota-active" } else { "fota-active," + features }

# Host triple — host-side tests must build for the host, since the workspace default target
# (thumbv6m, see .cargo/config.toml) has no libtest / panic handler. `--print host-tuple` avoids
# parsing `rustc -vV` and works on every OS.
host := trim(`rustc --print host-tuple`)

# Bootloader code budget (bytes) for `size-check`: the 20 KB BOOTLOADER region (BOOTLOADER_SIZE in
# src/fota/mod.rs) minus a 2 KB reserve. The loader is ≈16 KB today; this warns well before it could
# overflow its partition — which on the SWD-less Radio Dongle is an unrecoverable brick.
boot_budget := "18432"

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
# Build + flash a target (plain full-flash at 0x0800_0000, no bootloader) — erase/write/verify/reset.
flash kind name *args: (build kind name)
    tower {{_port_flag}} flash {{bin}} {{args}}

# The app links into the ACTIVE slot (@0x0800_8100), merged with the bootloader (@0x0800_0000) into
# one image (see `fota-image`); `fota-active` is added automatically, TOWER_FEATURES appended:
#   just flash-fota fota_app                              # FOTA self-swap demo
#   TOWER_FEATURES=role-node just flash-fota fota_ota     # the real OTA node (v1)
# Build + flash the FOTA-capable (bootloader + ACTIVE-linked app) image; read it with `just logs`.
flash-fota name *args: (fota-image name _fota_features)
    tower {{_port_flag}} flash target/fota-merged.bin {{args}}

# Build + flash a target (kind example|app), then stream its framed console logs (resets first).
run kind name: (flash kind name)
    tower {{_port_flag}} logs

# Build + flash a FOTA image, then stream its framed console logs (watch the swap + confirm).
run-fota name: (flash-fota name)
    tower {{_port_flag}} logs


# === Console & device control =====================================================================

# Stream the decoded framed console logs from the running MCU (extra args -> `tower logs`).
logs *args:
    tower {{_port_flag}} logs {{args}}

# Open the full-screen TUI console (logs + events + interactive shell).
console:
    tower {{_port_flag}} console

# Reset the MCU into the application (add `--bootloader` to enter the system bootloader).
reset *args:
    tower {{_port_flag}} reset {{args}}

# Erase the entire device flash, then reset into the application.
erase:
    tower {{_port_flag}} erase

# List the available serial ports / TOWER devices.
ports:
    tower devices


# === FOTA tooling =================================================================================

# Combines the bootloader (@0x0800_0000) + the ACTIVE-linked app (@0x0800_8100) into
# target/fota-merged.bin via tools/fota_merge.py. `flash-fota` calls this for you:
#   just fota-image                                  # fota_app (swap+confirm)
#   just fota-image fota_ota role-node,fota-active   # the real OTA node (v1)
# Build the merged FOTA image WITHOUT flashing (e.g. for CI).
fota-image example="fota_app" features="fota-active":
    cargo objcopy --release -p tower-bootloader -- -O binary target/fota-boot.bin
    cargo objcopy --release --example {{example}} --features "{{features}}" -- -O binary target/fota-app.bin
    {{python}} tools/fota_merge.py target/fota-boot.bin target/fota-app.bin target/fota-merged.bin

# A newer fota_ota build (version 2 by default, via the `fota-v2` feature) that supersedes the v1
# a device received from `flash-fota`. App-only + a signed manifest — the on-device bootloader
# verifies & swaps it; nothing is flashed here. Produces target/fota-update.{bin,fmanifest},
# served with `tower fota serve` (see docs/fota.md).
# Build + sign the over-the-air firmware update the host serves to a node (real-firmware swap E2E).
fota-update version="2":
    cargo objcopy --release --example fota_ota --features role-node,fota-active,fota-v2 -- -O binary target/fota-update.bin
    just fota-sign sign --version {{version}} --in target/fota-update.bin --out target/fota-update.fmanifest

# Signs a firmware image into a signed FOTA manifest (docs/fota.md):
#   just fota-sign pubkey
#   just fota-sign sign --version 2 --in target/firmware.bin --out target/fw.fmanifest
# Run the host signer (tools/fota-sign, a std host binary built for the host triple).
fota-sign *args:
    cargo run --quiet --manifest-path tools/fota-sign/Cargo.toml --target {{host}} -- {{args}}


# === Tests & checks ===============================================================================

# Builds for the host triple (the firmware itself is no_std): `tower-kv` (the EEPROM key-value
# codec), `tower-radio-core` (the EU/US/FHSS duty token-bucket math + the fixed FHSS hop
# permutation — the regulatory arithmetic), and `fota-sign` (host<->device Ed25519 interop + the
# DEV_SEED<->VENDOR_PUBKEY pin). The shared wire-protocol tests live in
# github.com/hardwario/tower-protocol. Runs `size-check` first.
# Run the firmware's host-side unit tests.
test *args: size-check
    cargo test -p tower-kv --target {{host}} {{args}}
    cargo test -p tower-radio-core --target {{host}} {{args}}
    cargo test --manifest-path tools/fota-sign/Cargo.toml --target {{host}} {{args}}

# The linker only hard-fails at the full 20 KB region (= brick on the SWD-less Dongle), so this trips
# ~2 KB earlier (`boot_budget`) as an early warning. Wired into `just test`; also run it in CI.
# Guard the bootloader against silently eating its flash-region margin (tools/size_check.py).
size-check:
    {{python}} tools/size_check.py {{boot_budget}}

# postcard is NOT self-describing, so a tower-protocol tag mismatch silently mis-decodes the wire
# (never a build error) — this makes it a hard failure. Local half of the CI `lockstep` job (which
# also fetches tower-cli's pin); the golden rule in the repo CLAUDE.md. tools/protocol_pin_check.py.
# Verify the tower-protocol git-tag pin is identical across all four in-repo manifests.
check-protocol-pin:
    {{python}} tools/protocol_pin_check.py


# === Hardware-in-the-loop (HIL) ===================================================================
#
# The HIL harness (tools/hil, a std host crate excluded from the workspace like fota-sign) drives
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
