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

# The RESOLVED source line in a Cargo.lock `[[package]]` block for tower-protocol, e.g.:
#   name = "tower-protocol"
#   version = "1.0.0"
#   source = "git+https://github.com/hardwario/tower-protocol?tag=v1.0.0#<40-hex-sha>"
# The tag string above is only a *label*; cargo resolves it to a commit SHA ONCE and records it
# here. A re-cut tag (same name, new commit — as happened 2026-07-02) makes two repos build
# DIFFERENT code while every tag-string check still says "aligned". Comparing the SHAs catches it.
_LOCK = re.compile(
    r'name\s*=\s*"tower-protocol"\s*\n'
    r'version\s*=\s*"[^"]*"\s*\n'
    r'source\s*=\s*"git\+[^"?]*(?:\?tag=([^#"]+))?#([0-9a-fA-F]{7,40})"'
)

# In-repo lockfiles that pin the resolved tower-protocol SHA (the workspace lock covers the root
# crate + tower-kv; the out-of-workspace HIL harness has its own).
LOCKFILES = [
    "Cargo.lock",
    "tools/hil/Cargo.lock",
]


def tag_in(text: str, where: str) -> str:
    m = _PIN.search(text)
    if not m:
        sys.exit(f"protocol-pin: no `tower-protocol = {{ ... tag = \"...\" }}` found in {where}")
    return m.group(1)


def sha_in_lock(text: str):
    """Return the resolved (tag, sha) for tower-protocol in a Cargo.lock, or None if not found."""
    m = _LOCK.search(text)
    if not m:
        return None
    return (m.group(1), m.group(2))


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

    # Resolved-SHA cross-check across the in-repo lockfiles. Tag strings agreeing is necessary but
    # NOT sufficient: a re-cut tag resolves to a different commit, and a warm cargo git cache can
    # leave one lockfile on the old SHA while another has the new one — both still labelled the
    # same tag. Compare the SHAs so that silent drift fails here rather than on the wire.
    firmware_sha = None
    lock_shas = {}
    for rel in LOCKFILES:
        path = os.path.join(root, rel)
        try:
            with open(path, encoding="utf-8") as f:
                got = sha_in_lock(f.read())
        except FileNotFoundError:
            print(f"  (note: {rel} not present — skipping its SHA check)")
            continue
        if got is None:
            print(f"  (note: no resolved tower-protocol source in {rel} — skipping)")
            continue
        _lock_tag, sha = got
        lock_shas[rel] = sha
        print(f"  {sha}  {rel}")
    distinct_shas = set(lock_shas.values())
    if len(distinct_shas) > 1:
        sys.exit(
            "ERROR: tower-protocol RESOLVED-SHA mismatch across in-repo lockfiles "
            f"({ {rel: s[:12] for rel, s in lock_shas.items()} }). The tag strings agree but the "
            "lockfiles resolved to different commits — a re-cut tag / stale cargo git cache. "
            "Run `cargo update -p tower-protocol` (both the workspace and tools/hil) so every lock "
            "resolves to the same commit."
        )
    if distinct_shas:
        firmware_sha = distinct_shas.pop()
        print(f"firmware tower-protocol resolved SHA: {firmware_sha}")

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

    # Resolved-SHA cross-check against the CLI too (the re-cut hazard crosses repos). Derive the
    # CLI's Cargo.lock URL from its Cargo.toml URL (side by side); best-effort — tolerate its
    # absence, but a determinable SHA mismatch under a matching tag is the exact drift to catch.
    if firmware_sha is not None and cli_url.endswith("Cargo.toml"):
        cli_lock_url = cli_url[: -len("Cargo.toml")] + "Cargo.lock"
        try:
            with urllib.request.urlopen(cli_lock_url, timeout=30) as resp:
                got = sha_in_lock(resp.read().decode("utf-8"))
        except Exception as e:  # noqa: BLE001 — best-effort; don't fail if the lock isn't published
            print(f"  (note: could not fetch/parse tower-cli Cargo.lock ({e}) — SHA check skipped)")
            got = None
        if got is not None:
            cli_sha = got[1]
            print(f"tower-cli tower-protocol resolved SHA: {cli_sha}")
            if cli_sha != firmware_sha:
                sys.exit(
                    f"ERROR: tower-protocol RESOLVED-SHA mismatch — firmware locked {firmware_sha} "
                    f"but tower-cli locked {cli_sha}, though both label tag {firmware_tag}. This "
                    "is the re-cut-tag hazard (same tag name, different commit). Re-run "
                    "`cargo update -p tower-protocol` in both repos so they resolve identically."
                )
            print(f"lockstep SHA OK: firmware and tower-cli both resolve {firmware_sha}")


if __name__ == "__main__":
    main()
