#!/usr/bin/env python3
"""Merge the FOTA bootloader + ACTIVE-linked app into one jolt-flashable image.

Usage: fota_merge.py <boot.bin> <app.bin> <out.bin>

The Radio Dongle has no SWD, so the bootloader and app can't be placed separately by a
probe — they're combined into one image flashed at 0x0800_0000 over jolt's UART ROM
bootloader. Layout (docs/fota.md):

    0x00000  bootloader (from <boot.bin>, linked at 0x0800_0000; ≤20K)
    ...      0xFF padding  (incl. the state region 0x5000..0x8000 and the manifest region
             0x8000..0x8800, so a fresh boot reads "no swap" + "no manifest")
    0x08800  app (from <app.bin>, linked into ACTIVE at 0x0800_8800)

The DFU slot (0x1_B800) is left out of the image; the app erases + writes it when it
stages, so its prior contents don't matter.
"""

import struct
import sys

ACTIVE = 0x8800  # ACTIVE offset = 0x0800_8800 - flash base 0x0800_0000 (docs/fota.md)


def main() -> None:
    if len(sys.argv) != 4:
        sys.exit("usage: fota_merge.py <boot.bin> <app.bin> <out.bin>")
    boot_path, app_path, out_path = sys.argv[1:4]
    boot = open(boot_path, "rb").read()
    app = open(app_path, "rb").read()
    if len(boot) > ACTIVE:
        sys.exit(f"bootloader {len(boot)} B exceeds the ACTIVE offset {ACTIVE} (0x{ACTIVE:x})")
    img = boot + b"\xff" * (ACTIVE - len(boot)) + app
    with open(out_path, "wb") as f:
        f.write(img)
    sp, reset = struct.unpack("<II", img[ACTIVE : ACTIVE + 8])
    print(
        f"merged {len(img)} B ({len(img)//1024} KB) -> {out_path}  "
        f"[boot {len(boot)} B + app {len(app)} B; app SP=0x{sp:08x} reset=0x{reset:08x}]"
    )


if __name__ == "__main__":
    main()
