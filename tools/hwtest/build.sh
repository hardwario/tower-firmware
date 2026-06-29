#!/usr/bin/env bash
# build.sh <example> <outbin> [features]  — cargo objcopy an example to a raw .bin.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ex="$1"; out="$2"; feats="${3:-}"
cd "$ROOT"
if [ -n "$feats" ]; then
  cargo objcopy --release --example "$ex" --features "$feats" -- -O binary "$out" 2>&1 | tail -2
else
  cargo objcopy --release --example "$ex" -- -O binary "$out" 2>&1 | tail -2
fi
echo "built $ex -> $out ($(wc -c < "$out") bytes)"
