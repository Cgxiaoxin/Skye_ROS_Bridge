#!/usr/bin/env bash
# Sync headers + current-arch libGentoSDK.so into third_party/gento_sdk
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="${GENTO_SDK_SRC:-$ROOT/../tianji-robot-SDK-Gento_Skye-Luna/C_SDK}"
DST="$ROOT/third_party/gento_sdk"
ARCH="$(uname -m)"
case "$ARCH" in
  x86_64|amd64) LIBDIR=x86_64 ;;
  aarch64|arm64) LIBDIR=aarch64 ;;
  *) echo "unsupported arch: $ARCH"; exit 1 ;;
esac

if [[ ! -d "$SRC" ]]; then
  echo "SDK source not found: $SRC"
  echo "Set GENTO_SDK_SRC=/path/to/C_SDK"
  exit 1
fi

mkdir -p "$DST/include" "$DST/lib/$LIBDIR"

for d in Common L0Control L1Robot FileClient; do
  mkdir -p "$DST/include/$d"
  cp -a "$SRC/$d/"*.h "$DST/include/$d/"
done

mkdir -p "$DST/include/Kinematics"
cp -a "$SRC/Kinematics/"*.h "$DST/include/Kinematics/" 2>/dev/null || true
for sub in ArmKinematics BaseMath DynaIdent KineCommon MotionPlanner SkyeBodyKinematics; do
  mkdir -p "$DST/include/Kinematics/$sub"
  cp -a "$SRC/Kinematics/$sub/"*.h "$DST/include/Kinematics/$sub/"
done

if [[ ! -f "$SRC/libGentoSDK.so" ]]; then
  echo "missing $SRC/libGentoSDK.so"
  exit 1
fi
cp -a "$SRC/libGentoSDK.so" "$DST/lib/$LIBDIR/"

{
  echo "4.4.2"
  echo "source: $SRC"
  echo "synced_arch: $LIBDIR"
  echo "synced_at: $(date -Iseconds)"
} > "$DST/VERSION"

echo "OK -> $DST (lib/$LIBDIR/libGentoSDK.so)"
