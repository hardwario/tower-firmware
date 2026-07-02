#!/usr/bin/env bash
# m3.sh NODE_PORT GW_PORT V1_MERGED_BIN GW_BIN V2_BIN V2_MANIFEST [OUTDIR] [WATCH_SECS]
# Full FOTA OTA E2E: flash the v1 node (bootloader+app merged) and the gateway, run
# `tower fota serve` for the v2 image on the gateway port, and watch the node pull → stage →
# (bootloader verify+swap, ~2.5 min, silent) → confirm. Watch ≥340 s to catch the swap.
#
# Build the inputs first, e.g.:
#   just fota-image fota_ota role-node,fota-active                                  # -> target/fota-merged.bin   (V1_MERGED)
#   cargo objcopy --release --example fota_ota --features role-gateway -- -O binary gw.bin   # (GW_BIN)
#   just fota-update                                                                # -> target/fota-update.{bin,fmanifest}
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
P1="$1"; P2="$2"; V1="$3"; GW="$4"; V2="$5"; MAN="$6"; OUT="${7:-/tmp/m3}"; SECS="${8:-340}"
mkdir -p "$OUT"
echo "[1] flash v1 node (merged) -> $P1"; tower -d "$P1" flash "$V1" >"$OUT/node.flash" 2>&1; tail -1 "$OUT/node.flash"
echo "[2] flash gateway        -> $P2"; tower -d "$P2" flash "$GW" >"$OUT/gw.flash" 2>&1; tail -1 "$OUT/gw.flash"
echo "[3] serve v2 on $P2 (background)"
tower -d "$P2" fota serve --image "$V2" --manifest "$MAN" >"$OUT/serve.log" 2>&1 &
SERVE=$!
echo "[4] watch node $P1 for ${SECS}s (pull -> verify -> swap -> confirm)"
python3 "$HERE/cap.py" "$SECS" tower -d "$P1" logs --no-colors >"$OUT/node.txt" 2>&1
kill "$SERVE" 2>/dev/null; kill -9 "$SERVE" 2>/dev/null
echo "[done] node confirm line:"; grep -iE 'confirm|NODE v|staged' "$OUT/node.txt" | tail -3
