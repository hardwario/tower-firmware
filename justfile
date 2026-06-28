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

# List available recipes.
default:
    @just --list

# Run the firmware's host-side tests on the host triple (the firmware itself is no_std):
#   - `tower-kv`    : the EEPROM key-value codec (record format / scan / in-place update /
#                     compaction + power-loss edges), extracted so it CAN be unit-tested.
#   - `fota-sign`   : host↔device Ed25519 interop (dalek↔salty) + the DEV_SEED↔VENDOR_PUBKEY pin.
# The shared wire-protocol codec/manifest tests live in their own repo —
# github.com/hardwario/tower-protocol (`cargo test --features verify` there).
test *ARGS:
    cargo test -p tower-kv --target {{host}} {{ARGS}}
    cargo test --manifest-path tools/fota-sign/Cargo.toml --target {{host}} {{ARGS}}

# Sign a firmware image into a signed FOTA manifest (docs/fota.md) with the host tool
# in tools/fota-sign (a std binary, built for the host triple). Examples:
#   just fota-sign pubkey
#   just fota-sign sign --version 2 --in target/firmware.bin --out target/fw.fmanifest
fota-sign *ARGS:
    cargo run --quiet --manifest-path tools/fota-sign/Cargo.toml --target {{host}} -- {{ARGS}}

# --- FOTA bootloader + ACTIVE-linked app ---
# The Radio Dongle has no SWD, so the bootloader (@0x0800_0000) and the ACTIVE-linked app
# (@0x0800_B800) are merged into ONE image and flashed over the UART bootloader. Read the
# framed console with `just logs` (the `tower` CLI). `example`/`features` pick the app build:
#   just fota-flash                                  # fota_app self-swap (swap+confirm)
#   just fota-flash fota_app fota-active,fota-no-confirm   # fota_app auto-revert test
#   just fota-flash fota_ota role-node,fota-active        # the real OTA node (v1)

# Build the merged FOTA image (bootloader + the given ACTIVE-linked example) -> fota-merged.bin.
fota-image example="fota_app" features="fota-active":
    cargo objcopy --release -p tower-bootloader -- -O binary target/fota-boot.bin
    cargo objcopy --release --example {{example}} --features "{{features}}" -- -O binary target/fota-app.bin
    python3 tools/fota_merge.py target/fota-boot.bin target/fota-app.bin target/fota-merged.bin

# Build the merged FOTA image and flash it (then read logs with `just logs`).
fota-flash example="fota_app" features="fota-active": (fota-image example features)
    tower {{_port_flag}} flash target/fota-merged.bin

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

# Build + flash an example via `tower` (erase, write, verify, reset). Extra args -> tower flash.
flash name *ARGS: (build name)
    tower {{_port_flag}} flash {{bin}} {{ARGS}}

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
