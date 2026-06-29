#!/usr/bin/env bash
# tb.sh NODE_BIN NODE_PORT GW_BIN GW_PORT SECS NODE_OUT GW_OUT
# Two-board test: flash node then gateway SEQUENTIALLY (concurrent flashing of two FTDIs over
# one USB bus is unreliable — Write-Memory timeouts / port re-enumeration), then capture both
# ports CONCURRENTLY with `tower logs` (mode A, for continuous output).
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
nb="$1"; np="$2"; gb="$3"; gp="$4"; secs="$5"; no="$6"; go="$7"
tower -p "$np" flash "$nb" >"${no}.flash" 2>&1
tower -p "$gp" flash "$gb" >"${go}.flash" 2>&1
python3 "$HERE/cap.py" "$secs" tower -p "$np" logs --no-colors >"$no" 2>&1 &
python3 "$HERE/cap.py" "$secs" tower -p "$gp" logs --no-colors >"$go" 2>&1 &
wait
