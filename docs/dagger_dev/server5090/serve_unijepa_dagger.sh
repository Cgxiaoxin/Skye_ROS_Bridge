#!/usr/bin/env bash
# Deploy on 5090. Wraps existing UniJEPA serve; does not talk to Gento SDK.
set -euo pipefail

UNIJEPA_ROOT="${UNIJEPA_ROOT:-/mnt/sdb/zph/UniJEPA}"
CKPT="${LEVJEPA_CKPT:-/mnt/sdb/zph/UniJEPA_ckpt/downliao_b64_4epoch_5e5/last.ckpt}"
NORM="${LEVJEPA_NORMALIZATION:-deploy/downliao_normalization.json}"
SERVE_SCRIPT="${SERVE_SCRIPT:-deploy/serve_b_levjepa.sh}"

if [[ ! -d "${UNIJEPA_ROOT}" ]]; then
  echo "[ERROR] UNIJEPA_ROOT missing: ${UNIJEPA_ROOT}" >&2
  echo "[ERROR] Copy UniJEPA tree here or set UNIJEPA_ROOT. See NEED_FROM_YOU.md" >&2
  exit 2
fi
if [[ ! -f "${UNIJEPA_ROOT}/${SERVE_SCRIPT}" ]]; then
  echo "[ERROR] Serve script missing: ${UNIJEPA_ROOT}/${SERVE_SCRIPT}" >&2
  exit 2
fi

cd "${UNIJEPA_ROOT}"
export PATH="/usr/local/cuda/bin:${PATH}"
export LEVJEPA_CKPT="${CKPT}"
export LEVJEPA_NORMALIZATION="${NORM}"
export LEVJEPA_COMPILE_ENCODER="${LEVJEPA_COMPILE_ENCODER:-0}"
export LEVJEPA_COMPILE_ACTION_HEAD="${LEVJEPA_COMPILE_ACTION_HEAD:-0}"

echo "[INFO] UniJEPA root=${UNIJEPA_ROOT}"
echo "[INFO] CKPT=${LEVJEPA_CKPT}"
echo "[INFO] NORM=${LEVJEPA_NORMALIZATION}"
exec bash "${SERVE_SCRIPT}"
