# tools/hwtest — hardware test harness

Scripts that drive the reusable hardware test suite in [`docs/test-plan.md`](../../docs/test-plan.md)
on two TOWER Core Modules over their FTDI serial ports. See the test plan for the full matrix,
pass signals, and edge-case catalogue; this README is just the runner mechanics.

## Why these exist
- One-shot KAT examples print their verdict ~ms after boot then idle, and `tower logs` has no
  reset-on-attach — so `cap.py` runs `jolt monitor --reset` (resets *then* reads) and you `strings`
  the raw bytes (the postcard message text is plain ASCII and survives COBS).
- Flashing two FTDIs concurrently is unreliable (Write-Memory timeouts, port re-enumeration), so
  `tb.sh` flashes **sequentially**, then captures both ports **concurrently**.

## Scripts
| script | purpose |
|---|---|
| `cap.py SECS CMD...` | run CMD for SECS seconds, capture stdout, kill the process group (no `timeout` on macOS) |
| `build.sh <example> <outbin> [features]` | `cargo objcopy` an example to a raw `.bin` |
| `fc.sh PORT BIN SECS OUT` | flash BIN to PORT, then mode-B capture (`jolt --reset`) for SECS |
| `tb.sh NODE_BIN NP GW_BIN GP SECS NOUT GOUT` | two-board: flash node then gw (sequential), capture both |
| `m3.sh NODE_PORT GW_PORT` | full FOTA OTA E2E: flash v1 node + gw, `fota serve` v2, watch node pull→swap→confirm |

## Quick recipes
```sh
# resolve current ports (they re-enumerate; re-run each session)
P1=$(ls /dev/cu.usbserial-* | sort | head -1); P2=$(ls /dev/cu.usbserial-* | sort | tail -1)

# host unit tests + bootloader size guard
just test

# a one-shot KAT (mode B): flash + capture-from-reset + grep the verdict
tools/hwtest/build.sh crypto_ccm_kat /tmp/ccm.bin
tower -p "$P1" flash /tmp/ccm.bin
python3 tools/hwtest/cap.py 5 jolt monitor --reset -p "$P1" | strings -n 3 | grep -iE 'PASS|FAIL'

# a continuous example (mode A): flash, then decoded logs
tower -p "$P1" flash /tmp/x.bin
python3 tools/hwtest/cap.py 6 tower -p "$P1" logs --no-colors

# two-board: one board role-node, the other default (role-gateway is a no-op == default)
tools/hwtest/build.sh net_secure_ping /tmp/nsp_n.bin role-node
tools/hwtest/build.sh net_secure_ping /tmp/nsp_d.bin
tools/hwtest/tb.sh /tmp/nsp_n.bin "$P1" /tmp/nsp_d.bin "$P2" 9 /tmp/n.txt /tmp/g.txt

# interactive shell, headless
tower -p "$P1" exec "/system/settings print"
tower -p "$P1" complete "/system settings set m"
```

## Notes
- `RESULTS-2026-06-29.md` is a recorded full run (≈40 PASS + 4 findings).
- The inline shell is **zsh**: no `mapfile`; `${v:+--flag "$v"}` collapses to one arg (use explicit `if`).
- FOTA swap is silent (~2.5 min) — capture ≥210 s for `fota_app`/`fota_ota` confirm lines.
