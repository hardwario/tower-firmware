# tower-firmware — working notes for Claude

Embassy-based, `no_std` firmware SDK for the HARDWARIO TOWER Core Module (STM32L083CZ,
Cortex-M0+). The crate is a library; runnable programs come in two kinds — educational
`examples/` (Cargo `--example`) and TOWER IoT Kit **products** in `apps/` (Cargo
`[[bin]]`, `--bin`). Two apps are complete, HW-verified radio products — `radio_push_button`
(sleeping sensor node) and `radio_dongle_gateway` (radio↔serial bridge + node coordinator);
`radio_climate_monitor` is still a starter skeleton (non-radio logic runs; radio send is a
TODO). Build/flash with `just`, which takes the kind then the name
(`just flash example blinky`, `just build app radio_push_button`, `just run example <name>`)
over the UART bootloader via the
`tower` CLI. Standalone device control (logs/console/reset/erase/devices) is the CLI's own
job — call `tower` directly, there are no `just` wrappers for it. Subsystem guides: `docs/radio.md`, `docs/console.md`, `docs/storage.md`, `docs/gateway.md` (the push-button + gateway product). Host tests:
`just test` (the `tower-kv` + `tower-net-core` + `tower-radio-core` + `tower-gw-core` + `tower-shell-core` crates —
the firmware itself is `no_std` and can't `cargo test`).

## Shared wire protocol (`tower-protocol`) — keep it in lockstep

The console wire format lives in a **separate repo**, github.com/hardwario/tower-protocol,
pinned here **by git tag in two places**: `Cargo.toml` and `crates/tower-kv/Cargo.toml`.
The host CLI `tower-cli` and the bench harness `tower-hil` (its own repo,
github.com/hardwario/tower-hil) pin the **same tag** — all consumers MUST move
together, because postcard isn't self-describing (mismatched versions silently mis-decode).
CI verifies the pins (`tools/protocol_pin_check.py`, which also fetches the tower-cli and
tower-hil pins); the developer-facing check lives in the TOWER control plane (`/lockstep`).

**If you change the protocol, or need a newer tower-protocol:** make the change in the
tower-protocol repo and follow its `CLAUDE.md` release runbook, then bump it here in the same
change-set:

```sh
# set tag = "vX.Y.Z" in Cargo.toml and crates/tower-kv/Cargo.toml
cargo update -p tower-protocol   # covers the workspace: root, tower-kv
just test            # + build an example
```

…and bump **tower-cli** and **tower-hil** to the same tag too. The protocol's own codec tests
run in the tower-protocol repo, not here. For local protocol co-dev, add `paths = ["/abs/path/to/tower-protocol"]`
to your `~/.cargo/config.toml` (this repo's `.cargo/config.toml` is committed for the build target,
so the override can't live there).

## Conventions

- `tower-protocol` was developed here; design rationale + hard-won caveats live in the
  `docs/*.md` guides and in code comments (don't strip them).
- Radio is regulatory-sensitive: FCC `§15.247` / EU duty citations in code are real — keep them.

## Git workflow

- This repo is developed straight on `main`. **Do not create new Git branches** (or open PRs)
  unless the user explicitly asks — work on the current branch and commit there.
- Commit and push only when the user requests it.
