#!/usr/bin/env python3
"""Verify the tower-protocol git-tag pin is identical everywhere (the golden lockstep rule).

The console wire format lives in the separate `tower-protocol` repo, pinned here by git
tag in THREE manifests. postcard is NOT self-describing, so a tag mismatch does not error at
build time — it silently mis-decodes bytes on the wire. This guard turns that latent hazard
into a hard failure.

Usage:
    protocol_pin_check.py                       # local half: the firmware manifests agree
    protocol_pin_check.py --cli-url <raw-url>   # also fetch tower-cli's Cargo.toml and compare

The in-repo manifests (see the repo CLAUDE.md lockstep note):
    Cargo.toml
    crates/tower-kv/Cargo.toml
    tools/hil/Cargo.toml

Plain python3/python (no shell, no coreutils) so it runs the same on Linux, macOS, Windows —
matching the justfile's cross-platform convention. `just check-protocol-pin` runs the local
half; CI adds --cli-url to also pin the host CLI in lockstep.
"""

import os
import re
import sys
import urllib.request

# The tower-protocol pins that MUST all carry the same tag (relative to the repo root).
MANIFESTS = [
    "Cargo.toml",
    "crates/tower-kv/Cargo.toml",
    "tools/hil/Cargo.toml",
]

# Match e.g.:  tower-protocol = { git = "...", tag = "v1.0.0", features = [...] }
# Capture the tag value. We only require that a `tower-protocol` dependency line carries a
# `tag = "..."`; the surrounding fields (features, git url) are free to differ.
_PIN = re.compile(r"""tower-protocol\s*=\s*\{[^}]*?\btag\s*=\s*"([^"]+)"[^}]*\}""")


def tag_in(text: str, where: str) -> str:
    m = _PIN.search(text)
    if not m:
        sys.exit(f"protocol-pin: no `tower-protocol = {{ ... tag = \"...\" }}` found in {where}")
    return m.group(1)


def repo_root() -> str:
    # tools/protocol_pin_check.py -> repo root is the parent of tools/.
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main() -> None:
    root = repo_root()
    tags = {}
    for rel in MANIFESTS:
        path = os.path.join(root, rel)
        try:
            with open(path, encoding="utf-8") as f:
                tags[rel] = tag_in(f.read(), rel)
        except FileNotFoundError:
            sys.exit(f"protocol-pin: missing manifest {rel}")

    distinct = set(tags.values())
    for rel, tag in tags.items():
        print(f"  {tag}  {rel}")
    if len(distinct) != 1:
        sys.exit(
            "ERROR: tower-protocol tag MISMATCH across firmware manifests "
            f"({sorted(distinct)}). postcard is not self-describing — a mismatch silently "
            "mis-decodes the wire. Align both pins in the same change-set."
        )
    firmware_tag = distinct.pop()
    print(f"firmware tower-protocol pin: {firmware_tag} (all {len(MANIFESTS)} manifests agree)")

    # Optional: cross-repo lockstep against the host CLI.
    cli_url = None
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--cli-url" and i + 1 < len(args):
            cli_url = args[i + 1]
            i += 2
        else:
            sys.exit(f"protocol-pin: unexpected argument {args[i]!r}")
    if cli_url is None:
        return

    print(f"fetching tower-cli Cargo.toml: {cli_url}")
    try:
        with urllib.request.urlopen(cli_url, timeout=30) as resp:
            cli_toml = resp.read().decode("utf-8")
    except Exception as e:  # noqa: BLE001 — surface any fetch error as a clear failure
        sys.exit(f"protocol-pin: failed to fetch tower-cli Cargo.toml: {e}")
    cli_tag = tag_in(cli_toml, "tower-cli Cargo.toml")
    print(f"tower-cli tower-protocol pin: {cli_tag}")
    if cli_tag != firmware_tag:
        sys.exit(
            f"ERROR: tower-protocol LOCKSTEP BROKEN — firmware pins {firmware_tag} but "
            f"tower-cli pins {cli_tag}. The two repos MUST move together (silent mis-decode "
            "otherwise). Bump both in the same change-set."
        )
    print(f"lockstep OK: firmware and tower-cli both pin {firmware_tag}")


if __name__ == "__main__":
    main()
