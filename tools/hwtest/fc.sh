#!/usr/bin/env bash
# fc.sh PORT BIN SECS OUT  — flash bin to port, then mode-B capture (jolt --reset) for SECS.
# Mode B = reset-then-read, for one-shot examples that print their verdict at boot and idle.
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
port="$1"; bin="$2"; secs="$3"; out="$4"
tower -d "$port" flash "$bin" > "${out}.flash" 2>&1
python3 "$HERE/cap.py" "$secs" jolt monitor --reset -d "$port" > "$out" 2>&1
