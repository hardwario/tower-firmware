# HARDWARIO TOWER Firmware SDK — task runner (https://just.systems)
#
# The crate is a library; flashable programs live in examples/. Build/flash one
# by name:
#   just samples            # list available examples
#   just flash blinky       # build + flash the blinky sample
#   just run thermometer    # build + flash, then open the monitor
# Override the serial port if needed: TOWER_PORT=/dev/cu.usbserial-XXXX just flash blinky

bin := "target/firmware.bin"

# Optional serial port; empty => let jolt auto-detect the only port present.
port := env_var_or_default("TOWER_PORT", "")
_port_flag := if port == "" { "" } else { "-p " + port }

# List available recipes.
default:
    @just --list

# List the example apps you can build/flash.
samples:
    @ls examples/*.rs | sed 's|examples/||; s|\.rs||'

# Build an example into target/firmware.bin, then print its on-chip size.
build name: && (size name)
    cargo objcopy --release --example {{name}} -- -O binary {{bin}}

# Build + flash an example over the UART bootloader.
# Extra args pass through to `jolt flash`, e.g. `just flash blinky --no-verify`.
flash name *ARGS: (build name)
    jolt flash {{_port_flag}} {{ARGS}} {{bin}}

# Build + flash an example, then drop into the serial monitor. Resets the MCU on
# attach (`--reset`) so you always catch the boot banner and any one-shot output.
run name: (flash name)
    jolt monitor --reset {{_port_flag}}

# Open the serial console on the running MCU (does NOT reset it, so a quiet app
# may show nothing until it logs). Add `--reset` to restart and catch boot.
monitor *ARGS:
    jolt monitor {{_port_flag}} {{ARGS}}

# List available serial ports.
ports:
    jolt list

# On-chip footprint (text/data/bss) of an example.
size name:
    cargo size --release --example {{name}}

# Remove build artifacts.
clean:
    cargo clean
