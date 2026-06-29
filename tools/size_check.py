#!/usr/bin/env python3
"""Guard the FOTA bootloader against silently eating its flash-region margin.

Usage: size_check.py <budget_bytes>

`cargo size` reports the bootloader's on-chip `text` size; this compares it to a byte
budget (the BOOTLOADER region in src/fota/mod.rs minus a reserve, passed in by the justfile
as `boot_budget`) and fails if it is exceeded. The linker only hard-errors at the *full*
20 KB BOOTLOADER region — which on the SWD-less Radio Dongle is an unrecoverable brick — so
this trips ~2 KB earlier as an early warning. Wired into `just test`; also run it in CI.

This is plain `python3`/`python` (no shell, no coreutils) so `just size-check` runs the same
way on Linux, macOS, and Windows. If it fires: trim the loader, or *deliberately* raise
`boot_budget` (justfile) together with BOOTLOADER_SIZE (src/fota/mod.rs).
"""

import subprocess
import sys

REGION = 20480  # BOOTLOADER region size, bytes (== BOOTLOADER_SIZE in src/fota/mod.rs)


def bootloader_text_bytes() -> int:
    # `cargo size` prints a header row then one data row:
    #   text    data     bss     dec     hex filename
    #  16384       0    1024   17408    4400 tower-bootloader
    # We want the `text` column of the data row. Let build progress / errors flow to stderr.
    proc = subprocess.run(
        ["cargo", "size", "--release", "-p", "tower-bootloader"],
        stdout=subprocess.PIPE,
        text=True,
    )
    if proc.returncode != 0:
        sys.exit(proc.returncode)  # cargo already printed the reason on stderr
    rows = [line for line in proc.stdout.splitlines() if line.strip()]
    if not rows:
        sys.exit("size-check: `cargo size` produced no output (is cargo-binutils installed?)")
    return int(rows[-1].split()[0])


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit("usage: size_check.py <budget_bytes>")
    budget = int(sys.argv[1])
    text = bootloader_text_bytes()
    reserve = REGION - budget
    print(
        f"bootloader: {text} B used / {REGION} B region "
        f"(budget {budget} B; {reserve} B reserve to the hard limit)"
    )
    if text > budget:
        sys.exit(
            f"ERROR: bootloader {text} B exceeds the {budget} B budget — "
            f"only {REGION - text} B from the {REGION} B brick limit.\n"
            "Trim the loader, or deliberately raise boot_budget (justfile) + "
            "BOOTLOADER_SIZE (src/fota/mod.rs)."
        )


if __name__ == "__main__":
    main()
