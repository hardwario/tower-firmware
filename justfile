# HARDWARIO TOWER Firmware SDK — task runner (https://just.systems)
#
# The crate is a library; flashable programs come in two kinds — `example` (educational,
# examples/) and `app` (a ready-made TOWER IoT Kit product, apps/). build/flash/run/size take
# the kind then the name. Run `just` (or `just default`) for the full recipe list. Common flow:
#
#   just examples                  # list example names    (just apps  lists product names)
#   just flash example blinky      # build + flash the blinky example
#   just run app radio_push_button # build + flash a product, then open the console TUI
#
# Override the serial device: TOWER_DEVICE=/dev/cu.usbserial-XXXX just flash example blinky
# Pass cargo features:        TOWER_FEATURES=role-gateway just flash example net_confirmed
#
# Flashing + console use the `tower` CLI (https://github.com/hardwario/tower-cli): it programs
# the STM32L0 over the UART bootloader (the jolt engine, as a library) AND decodes the firmware's
# framed console (`tower logs`, not a raw serial terminal). Install it on your PATH (e.g.
# `cargo install --path ../github/tower-cli`, or grab a release binary). Device control on its
# own (logs, console, reset, erase, devices) is `tower`'s job — call the CLI directly; the
# justfile only wraps the steps that need the kind/name/features build context.
#
# The HIL bench harness lives in its own repo (github.com/hardwario/tower-hil) and builds its
# images from this checkout; the tower-protocol pin check lives in CI here
# (tools/protocol_pin_check.py) and, developer-facing, in the TOWER control plane (/lockstep).
#
# CROSS-PLATFORM: every recipe is a single program invocation (no bash, no sed/awk/ls pipelines),
# so the same `just` recipes run on Linux, macOS, and Windows. Windows uses cmd.exe (below) so
# exit codes propagate to CI.

# On Windows, run recipe lines with cmd.exe (always present; propagates child exit codes — unlike
# `-Command` PowerShell). Unix keeps the default `sh`. See the cross-platform note above.
set windows-shell := ["cmd.exe", "/c"]

# Merged firmware image written by `build` and flashed by `flash`.
bin := "target/firmware.bin"

# Optional serial device; empty => let `tower` auto-detect the only USB serial device present.
# Read from TOWER_DEVICE; passed to `tower`/`jolt` as `-d`.
device := env_var_or_default("TOWER_DEVICE", "")
_device_flag := if device == "" { "" } else { "-d " + device }

# Optional cargo features, e.g. a radio example's role selection:
#   TOWER_FEATURES=role-gateway just flash example net_confirmed
features := env_var_or_default("TOWER_FEATURES", "")
_feat_flag := if features == "" { "" } else { "--features " + features }

# Host triple, needed by `test` ONLY: the committed .cargo/config.toml pins the repo's DEFAULT
# cargo target to thumbv6m (so bare `cargo build`/`cargo run` cross-compile for the L0) — which
# also applies to `cargo test`, and thumbv6m has no libtest / panic handler. The explicit host
# --target undoes that one default; it cannot be dropped without dropping the config default.
# `--print host-tuple` avoids parsing `rustc -vV` and works on every OS.
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

# Hand-maintained (keeps the recipe OS-independent — no ls/sed vs PowerShell split): add an
# `@echo <name>` line, kept sorted, when you add examples/<name>.rs.
# List the example apps (examples/*.rs) — build/flash/run them as kind `example`.
examples:
    @echo accelerometer
    @echo blinky
    @echo button
    @echo console_demo
    @echo console_full
    @echo console_panic
    @echo crypto_aes_kat
    @echo crypto_ccm_kat
    @echo crypto_frame_loopback
    @echo edge_frame_limits
    @echo edge_rapid
    @echo edge_recovery
    @echo events_demo
    @echo fhss_compliance
    @echo fhss_kat
    @echo fhss_sweep
    @echo i2cscan
    @echo lowpower
    @echo net_bulk
    @echo net_bulk_stream
    @echo net_bulk_stress
    @echo net_channel
    @echo net_confirmed
    @echo net_duty_kat
    @echo net_p2p
    @echo net_pairing
    @echo net_persist
    @echo net_secure_ping
    @echo net_star
    @echo radio_afa
    @echo radio_band
    @echo radio_beacon
    @echo radio_csma
    @echo radio_cw
    @echo radio_fhss
    @echo radio_gateway
    @echo radio_id
    @echo radio_interop
    @echo radio_linkdiag
    @echo radio_node
    @echo radio_regdump
    @echo radio_sleep
    @echo radio_state
    @echo shell_demo
    @echo storage
    @echo strip
    @echo thermometer
    @echo watchdog

# Hand-maintained, same deal as `examples`: add an `@echo <name>` line for a new apps/<name>.rs.
# List the TOWER IoT Kit product firmwares (apps/*.rs) — build/flash/run them as kind `app`.
apps:
    @echo radio_climate_monitor
    @echo radio_dongle_gateway
    @echo radio_push_button

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

# Flash ends by resetting into the app, so the console attaches from a fresh boot. For a plain
# log stream instead of the TUI, use `tower logs` directly.
# Build + flash a target (kind example|app), then open the console TUI (logs + events + shell).
run kind name: (flash kind name)
    tower {{_device_flag}} console


# === Tests & checks ===============================================================================

# Builds for the host triple (the firmware itself is no_std): `tower-kv` (the EEPROM key-value
# codec), `tower-net-core` (the network-layer security decision kernels — replay rule, TX-counter
# watermark/fail-closed nonce safety, ACK resolution, FHSS beacon-epoch acceptance, pairing
# confirm freshness, CCM nonce construction), `tower-radio-core` (the EU/US/FHSS duty token-bucket
# math + the fixed FHSS hop permutation — the regulatory arithmetic), `tower-gw-core` (the gateway
# node-registry bucket codec + downlink-queue policy) and `tower-shell-core` (the `address` setting
# value parser + its xorshift). The shared wire-protocol tests live in the tower-protocol repo.
# Run the firmware's host-side unit tests.
test *args:
    cargo test -p tower-kv --target {{host}} {{args}}
    cargo test -p tower-net-core --target {{host}} {{args}}
    cargo test -p tower-radio-core --target {{host}} {{args}}
    cargo test -p tower-gw-core --target {{host}} {{args}}
    cargo test -p tower-shell-core --target {{host}} {{args}}


# === Maintenance ==================================================================================

# Remove build artifacts.
clean:
    cargo clean
