#!/usr/bin/env bash
# Verify the vendored libGentoSDK.so matches the build host (arch / deps / symbols).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ARCH="$(uname -m)"
SO="$ROOT/third_party/gento_sdk/lib/$ARCH/libGentoSDK.so"

if [[ ! -f "$SO" ]]; then
  echo "[ABI] missing: $SO"
  exit 1
fi

echo "== file =="
file "$SO"

echo "== ldd (missing deps flagged as 'not found') =="
ldd "$SO" || true

echo "== exported FX_L1_* symbols (sample) =="
if nm -D "$SO" 2>/dev/null | grep -q FX_L1_; then
  nm -D "$SO" | grep ' T ' | grep -c FX_L1_ | sed 's/^/[ABI] FX_L1_ exported symbols: /'
else
  echo "[ABI] WARNING: no FX_L1_* dynamic symbols found"
fi

echo "[ABI] done for arch=$ARCH"
