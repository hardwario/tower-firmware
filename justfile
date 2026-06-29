# HARDWARIO TOWER Firmware SDK — task runner (https://just.systems)
#
# The crate is a library; flashable programs live in examples/. Build/flash one
# by name:
#   just samples            # list available examples
#   just flash blinky       # build + flash the blinky sample
#   just run thermometer    # build + flash, then stream the (framed) console logs
# Override the serial port if needed: TOWER_PORT=/dev/cu.usbserial-XXXX just flash blinky
#
# Flashing + console use the `tower` CLI (https://github.com/hardwario/tower-cli): it
# programs the STM32L0 over the UART bootloader (the jolt engine, as a library) AND decodes
# the firmware's framed console. Install it on your PATH (e.g. `cargo install --path
# ../github/tower-cli`, or grab a release binary). The old `jolt monitor` shows the framed
# console as raw bytes — use `tower logs` instead.

bin := "target/firmware.bin"

# Optional serial port; empty => let `tower` auto-detect the only USB serial device present.
port := env_var_or_default("TOWER_PORT", "")
_port_flag := if port == "" { "" } else { "-p " + port }

# Optional cargo features, e.g. the radio examples' role selection:
#   TOWER_FEATURES=role-gateway just flash net_confirmed
features := env_var_or_default("TOWER_FEATURES", "")
_feat_flag := if features == "" { "" } else { "--features " + features }

# Host triple — host-side tests must build for the host, since the workspace
# default target (thumbv6m, see .cargo/config.toml) has no libtest / panic handler.
host := `rustc -vV | sed -n 's/^host: //p'`

# Bootloader code budget (bytes) for `size-check`: the 20 KB BOOTLOADER region (BOOTLOADER_SIZE
# in src/fota/mod.rs) minus a 2 KB reserve. The loader is ≈16 KB today; this warns well before it
# could overflow its partition — which on the SWD-less Radio Dongle is an unrecoverable brick.
boot_budget := "18432"

# List available recipes.
default:
    @just --list

# Guard the bootloader against silently eating its flash-region margin. The linker only hard-fails
# at the *full* 20 KB region (= brick on the SWD-less Dongle), so this trips ~2 KB earlier
# (`boot_budget`) as an early warning. Wired into `just test`; also run it in CI. If it fires:
# trim the loader, or *deliberately* raise `boot_budget` together with BOOTLOADER_SIZE.
size-check:
    #!/usr/bin/env bash
    set -euo pipefail
    text=$(cargo size --release -p tower-bootloader 2>/dev/null | tail -1 | awk '{print $1}')
    region=20480
    printf 'bootloader: %s B used / %s B region (budget %s B; %s B reserve to the hard limit)\n' \
        "$text" "$region" "{{boot_budget}}" "$(( region - {{boot_budget}} ))"
    if [ "$text" -gt {{boot_budget}} ]; then
        printf 'ERROR: bootloader %s B exceeds the %s B budget — only %s B from the %s B brick limit.\n' \
            "$text" "{{boot_budget}}" "$(( region - text ))" "$region" >&2
        printf 'Trim the loader, or deliberately raise boot_budget + BOOTLOADER_SIZE (src/fota/mod.rs).\n' >&2
        exit 1
    fi

# Run the firmware's host-side tests on the host triple (the firmware itself is no_std):
#   - `tower-kv`    : the EEPROM key-value codec (record format / scan / in-place update /
#                     compaction + power-loss edges), extracted so it CAN be unit-tested.
#   - `fota-sign`   : host↔device Ed25519 interop (dalek↔salty) + the DEV_SEED↔VENDOR_PUBKEY pin.
# The shared wire-protocol codec/manifest tests live in their own repo —
# github.com/hardwario/tower-protocol (`cargo test --features verify` there). Also runs
# `size-check` (the bootloader flash-budget guard) so margin erosion fails the test run.
test *ARGS: size-check
    cargo test -p tower-kv --target {{host}} {{ARGS}}
    cargo test --manifest-path tools/fota-sign/Cargo.toml --target {{host}} {{ARGS}}

# Sign a firmware image into a signed FOTA manifest (docs/fota.md) with the host tool
# in tools/fota-sign (a std binary, built for the host triple). Examples:
#   just fota-sign pubkey
#   just fota-sign sign --version 2 --in target/firmware.bin --out target/fw.fmanifest
fota-sign *ARGS:
    cargo run --quiet --manifest-path tools/fota-sign/Cargo.toml --target {{host}} -- {{ARGS}}

# --- FOTA bootloader + ACTIVE-linked app ---
# A FOTA build links the app into the ACTIVE slot (@0x0800_8100) and merges it with the
# bootloader (@0x0800_0000) into ONE image, flashed over the UART bootloader. Flash it with
# `just flash --fota <example>` (see the `flash` recipe); read the framed console with
# `just logs`. `fota-image` below is the build-only step if you just want the merged binary
# (e.g. for CI):
#   just fota-image                                  # fota_app (swap+confirm)
#   just fota-image fota_ota role-node,fota-active   # the real OTA node (v1)
fota-image example="fota_app" features="fota-active":
    cargo objcopy --release -p tower-bootloader -- -O binary target/fota-boot.bin
    cargo objcopy --release --example {{example}} --features "{{features}}" -- -O binary target/fota-app.bin
    python3 tools/fota_merge.py target/fota-boot.bin target/fota-app.bin target/fota-merged.bin

# Build + sign the fota_ota "v2" image the host serves to the node (real-firmware swap E2E).
# Produces target/fota-ota-v2.{bin,fmanifest}; serve with `tower fota serve` (see docs/fota.md).
fota-ota-v2 version="2":
    cargo objcopy --release --example fota_ota --features role-node,fota-active,fota-v2 -- -O binary target/fota-ota-v2.bin
    just fota-sign sign --version {{version}} --in target/fota-ota-v2.bin --out target/fota-ota-v2.fmanifest

# List the example apps you can build/flash.
samples:
    @ls examples/*.rs | sed 's|examples/||; s|\.rs||'

# Build an example into target/firmware.bin, then print its on-chip size.
# Set TOWER_FEATURES to pass cargo features (e.g. a radio example's role).
build name: && (size name)
    cargo objcopy --release --example {{name}} {{_feat_flag}} -- -O binary {{bin}}

# Build + flash an example via `tower` (erase, write, verify, reset). Pass `--fota` (first) to
# build the FOTA-capable image — ACTIVE-linked, merged with the bootloader — and flash that;
# otherwise a plain full-flash image at 0x0800_0000 (the whole 192 KB). Extra args after the
# name go to `tower flash`. Set TOWER_FEATURES for cargo features; `--fota` adds `fota-active`.
#   just flash blinky                                    # full-flash, no bootloader
#   just flash --fota fota_app                           # FOTA self-swap demo
#   TOWER_FEATURES=role-node just flash --fota fota_ota  # the real OTA node (v1)
flash *args:
    #!/usr/bin/env bash
    set -euo pipefail
    set -- {{args}}
    fota=0
    if [ "${1-}" = "--fota" ]; then fota=1; shift; fi
    [ "$#" -ge 1 ] || { echo "usage: just flash [--fota] <example> [tower flash args]" >&2; exit 2; }
    name="$1"; shift
    if [ "$fota" -eq 1 ]; then
        just fota-image "$name" "fota-active${TOWER_FEATURES:+,${TOWER_FEATURES}}"
        tower {{_port_flag}} flash target/fota-merged.bin "$@"
    else
        just build "$name"
        tower {{_port_flag}} flash {{bin}} "$@"
    fi

# Build + flash an example, then stream its framed console logs (resets into the app first).
run name: (flash name)
    tower {{_port_flag}} logs

# Stream the decoded framed console logs from the running MCU (extra args -> tower logs).
logs *ARGS:
    tower {{_port_flag}} logs {{ARGS}}

# Open the full-screen TUI console (logs + events + interactive shell).
console:
    tower {{_port_flag}} console

# Reset the MCU into the application (add `--bootloader` to enter the system bootloader).
reset *ARGS:
    tower {{_port_flag}} reset {{ARGS}}

# Erase the entire device flash, then reset into the application.
erase:
    tower {{_port_flag}} erase

# List available serial ports / TOWER devices.
ports:
    tower devices

# On-chip footprint (text/data/bss) of an example.
size name:
    cargo size --release --example {{name}} {{_feat_flag}}

# Remove build artifacts.
clean:
    cargo clean
