# HARDWARIO TOWER Firmware SDK — task runner (https://just.systems)
#
# The crate is a library; flashable programs live in examples/. Run `just` (or `just default`)
# for the full recipe list. Common flow:
#
#   just examples            # list the example apps you can build/flash
#   just flash blinky        # build + flash the blinky example
#   just run thermometer     # build + flash, then stream the (framed) console logs
#
# Override the serial port:   TOWER_PORT=/dev/cu.usbserial-XXXX just flash blinky
# Pass cargo features:        TOWER_FEATURES=role-gateway just flash net_confirmed
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

# Optional serial port; empty => let `tower` auto-detect the only USB serial device present.
port := env_var_or_default("TOWER_PORT", "")
_port_flag := if port == "" { "" } else { "-p " + port }

# Optional cargo features, e.g. a radio example's role selection:
#   TOWER_FEATURES=role-gateway just flash net_confirmed
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


# === Examples =====================================================================================

# List the example apps you can build/flash (one per examples/*.rs).
[unix]
examples:
    @ls examples/*.rs | sed 's|examples/||; s|\.rs$||'

[windows]
examples:
    @powershell -NoProfile -Command "Get-ChildItem examples/*.rs | Select-Object -ExpandProperty BaseName"

# Set TOWER_FEATURES to pass cargo features (e.g. a radio example's role).
# Build an example into target/firmware.bin, then print its on-chip size.
build name: && (size name)
    cargo objcopy --release --example {{name}} {{_feat_flag}} -- -O binary {{bin}}

# On-chip footprint (text/data/bss) of an example.
size name:
    cargo size --release --example {{name}} {{_feat_flag}}


# === Flash & run ==================================================================================

# Extra args after the name pass through to `tower flash`; set TOWER_FEATURES for cargo features:
#   just flash blinky --no-verify
#   TOWER_FEATURES=role-gateway just flash net_confirmed
# Build + flash an example (plain full-flash at 0x0800_0000, no bootloader) — erase/write/verify/reset.
flash name *args: (build name)
    tower {{_port_flag}} flash {{bin}} {{args}}

# The app links into the ACTIVE slot (@0x0800_8100), merged with the bootloader (@0x0800_0000) into
# one image (see `fota-image`); `fota-active` is added automatically, TOWER_FEATURES appended:
#   just flash-fota fota_app                              # FOTA self-swap demo
#   TOWER_FEATURES=role-node just flash-fota fota_ota     # the real OTA node (v1)
# Build + flash the FOTA-capable (bootloader + ACTIVE-linked app) image; read it with `just logs`.
flash-fota name *args: (fota-image name _fota_features)
    tower {{_port_flag}} flash target/fota-merged.bin {{args}}

# Build + flash a plain example, then stream its framed console logs (resets into the app first).
run name: (flash name)
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

# Produces target/fota-ota-v2.{bin,fmanifest}; serve with `tower fota serve` (see docs/fota.md).
# Build + sign the fota_ota "v2" image the host serves to the node (real-firmware swap E2E).
fota-ota-v2 version="2":
    cargo objcopy --release --example fota_ota --features role-node,fota-active,fota-v2 -- -O binary target/fota-ota-v2.bin
    just fota-sign sign --version {{version}} --in target/fota-ota-v2.bin --out target/fota-ota-v2.fmanifest

# Signs a firmware image into a signed FOTA manifest (docs/fota.md):
#   just fota-sign pubkey
#   just fota-sign sign --version 2 --in target/firmware.bin --out target/fw.fmanifest
# Run the host signer (tools/fota-sign, a std host binary built for the host triple).
fota-sign *args:
    cargo run --quiet --manifest-path tools/fota-sign/Cargo.toml --target {{host}} -- {{args}}


# === Tests & checks ===============================================================================

# Builds for the host triple (the firmware itself is no_std): `tower-kv` (the EEPROM key-value codec)
# and `fota-sign` (host<->device Ed25519 interop + the DEV_SEED<->VENDOR_PUBKEY pin). The shared
# wire-protocol tests live in github.com/hardwario/tower-protocol. Runs `size-check` first.
# Run the firmware's host-side unit tests.
test *args: size-check
    cargo test -p tower-kv --target {{host}} {{args}}
    cargo test --manifest-path tools/fota-sign/Cargo.toml --target {{host}} {{args}}

# The linker only hard-fails at the full 20 KB region (= brick on the SWD-less Dongle), so this trips
# ~2 KB earlier (`boot_budget`) as an early warning. Wired into `just test`; also run it in CI.
# Guard the bootloader against silently eating its flash-region margin (tools/size_check.py).
size-check:
    {{python}} tools/size_check.py {{boot_budget}}


# === Maintenance ==================================================================================

# Remove build artifacts.
clean:
    cargo clean
