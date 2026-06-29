# tower-firmware — working notes for Claude

Embassy-based, `no_std` firmware SDK for the HARDWARIO TOWER Core Module (STM32L083CZ,
Cortex-M0+). The crate is a library; runnable programs live in `examples/`. Build/flash with
`just` (`just flash <example>`, `just run <example>`, `just logs`) over the UART bootloader via the
`tower` CLI. Subsystem guides: `docs/radio.md`, `docs/console.md`, `docs/fota.md`. Host tests:
`just test` (the `fota-sign` signer — the firmware itself is `no_std` and can't `cargo test`).

## Shared wire protocol (`tower-protocol`) — keep it in lockstep

The console/FOTA wire format lives in a **separate repo**, github.com/hardwario/tower-protocol,
pinned here **by git tag in three places**: `Cargo.toml`, `crates/bootloader/Cargo.toml`, and
`tools/fota-sign/Cargo.toml` (the latter two add `features = ["verify"]`). The host CLI `tower-cli`
pins the **same tag** — the two repos MUST move together, because postcard isn't self-describing
(mismatched versions silently mis-decode).

**If you change the protocol, or need a newer tower-protocol:** make the change in the
tower-protocol repo and follow its `CLAUDE.md` release runbook, then bump it here in the same
change-set:

```sh
# set tag = "vX.Y.Z" in Cargo.toml, crates/bootloader/Cargo.toml, tools/fota-sign/Cargo.toml
cargo update -p tower-protocol
cargo update --manifest-path tools/fota-sign/Cargo.toml -p tower-protocol
just test            # + build a FOTA example
```

…and bump **tower-cli** to the same tag too. The protocol's own codec/manifest tests run in the
tower-protocol repo, not here. For local protocol co-dev, add `paths = ["/abs/path/to/tower-protocol"]`
to your `~/.cargo/config.toml` (this repo's `.cargo/config.toml` is committed for the build target,
so the override can't live there).

## Conventions

- The whole FOTA subsystem and `tower-protocol` were developed here; design rationale + hard-won
  caveats live in the `docs/*.md` guides and in code comments (don't strip them).
- Radio is regulatory-sensitive: FCC `§15.247` / EU duty citations in code are real — keep them.

## Git workflow

- This repo is developed straight on `main`. **Do not create new Git branches** (or open PRs)
  unless the user explicitly asks — work on the current branch and commit there.
- Commit and push only when the user requests it.
