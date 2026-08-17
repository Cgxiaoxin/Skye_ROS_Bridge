#!/usr/bin/env bash
# Download marvin_ws/install (factr_teleop so + yaml) when missing after git clone.
#
# Artifacts live in GitLab Generic Package Registry (NOT in git).
# Maintainer upload: ./scripts/publish_marvin_install.sh
#
# Usage:
#   ./scripts/bootstrap_marvin_install.sh
#   # optional overrides:
#   MARVIN_INSTALL_URL=http://.../marvin_ws_install.tgz ./scripts/bootstrap_marvin_install.sh
#   GITLAB_TOKEN=glpat-... ./scripts/bootstrap_marvin_install.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MARVIN_WS="${MARVIN_WS:-${REPO_ROOT}/marvin_ws}"
INSTALL="${MARVIN_WS}/install"
MARKER="${INSTALL}/setup.bash"
SO_MARKER="${INSTALL}/lib/factr_teleop/factr_teleop.cpython-310-x86_64-linux-gnu.so"

VERSION_FILE="${SCRIPT_DIR}/marvin_install.version"
PACKAGE_NAME="${MARVIN_INSTALL_PACKAGE:-marvin-ws}"
ARCHIVE_NAME="${MARVIN_INSTALL_ARCHIVE:-marvin_ws_install.tgz}"

default_gitlab_api() {
  local remote url host project
  remote="$(git -C "${REPO_ROOT}" remote get-url alex 2>/dev/null \
    || git -C "${REPO_ROOT}" remote get-url origin 2>/dev/null \
    || true)"
  [[ -n "${remote}" ]] || return 0

  if [[ "${remote}" =~ ^git@([^:]+):(.+)\.git$ ]]; then
    host="${BASH_REMATCH[1]}"
    project="${BASH_REMATCH[2]//\//%2F}"
  elif [[ "${remote}" =~ ^https?://([^/]+)/(.+)\.git$ ]]; then
    host="${BASH_REMATCH[1]}"
    project="${BASH_REMATCH[2]//\//%2F}"
  else
    return 0
  fi
  printf 'http://%s/api/v4/projects/%s' "${host}" "${project}"
}

read_version() {
  if [[ -n "${MARVIN_INSTALL_VERSION:-}" ]]; then
    printf '%s' "${MARVIN_INSTALL_VERSION}"
  elif [[ -f "${VERSION_FILE}" ]]; then
    tr -d '[:space:]' < "${VERSION_FILE}"
  else
    printf '1.0.0'
  fi
}

install_ok() {
  [[ -f "${MARKER}" && -f "${SO_MARKER}" ]]
}

verify_install() {
  local missing=0
  for f in \
    "${MARKER}" \
    "${SO_MARKER}" \
    "${INSTALL}/share/factr_teleop/configs/grav_comp_m6_left.yaml"; do
    if [[ ! -f "${f}" ]]; then
      echo "  missing: ${f}" >&2
      missing=1
    fi
  done
  [[ "${missing}" -eq 0 ]]
}

if install_ok; then
  echo "OK: marvin_ws/install already present (${INSTALL})"
  exit 0
fi

VERSION="$(read_version)"
GITLAB_API="${GITLAB_API:-$(default_gitlab_api)}"
DOWNLOAD_URL="${MARVIN_INSTALL_URL:-}"

if [[ -z "${DOWNLOAD_URL}" && -n "${GITLAB_API}" ]]; then
  DOWNLOAD_URL="${GITLAB_API}/packages/generic/${PACKAGE_NAME}/${VERSION}/${ARCHIVE_NAME}"
fi

if [[ -z "${DOWNLOAD_URL}" ]]; then
  cat >&2 <<EOF
ERROR: marvin_ws/install missing and no download URL configured.

Options:
  1) Set MARVIN_INSTALL_URL to a direct .tgz URL, then re-run:
       MARVIN_INSTALL_URL=http://.../marvin_ws_install.tgz ./scripts/bootstrap_marvin_install.sh

  2) Maintainer: upload once to GitLab Package Registry:
       GITLAB_TOKEN=glpat-... ./scripts/publish_marvin_install.sh

  3) Manual: copy from a working machine:
       rsync -a user@host:\$REPO/marvin_ws/install/ ${INSTALL}/
EOF
  exit 1
fi

TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT
ARCHIVE="${TMP}/${ARCHIVE_NAME}"

echo "== bootstrap marvin_ws/install =="
echo "   version: ${VERSION}"
echo "   url:     ${DOWNLOAD_URL}"

CURL_ARGS=(-fsSL --retry 3 --retry-delay 2 -o "${ARCHIVE}")
if [[ -n "${GITLAB_TOKEN:-}" ]]; then
  CURL_ARGS+=(--header "PRIVATE-TOKEN: ${GITLAB_TOKEN}")
fi

if ! curl "${CURL_ARGS[@]}" "${DOWNLOAD_URL}"; then
  cat >&2 <<EOF

ERROR: download failed.

If the package was never uploaded, on a machine that has marvin_ws/install:
  tar --exclude='__pycache__' -czf /tmp/marvin_ws_install.tgz marvin_ws/install
  GITLAB_TOKEN=glpat-... ./scripts/publish_marvin_install.sh

Create token: GitLab → User Settings → Access Tokens (api scope).
EOF
  exit 1
fi

echo "   downloaded: $(du -h "${ARCHIVE}" | cut -f1)"

rm -rf "${INSTALL}"
mkdir -p "${MARVIN_WS}"
tar -xzf "${ARCHIVE}" -C "${REPO_ROOT}"

if ! verify_install; then
  echo "ERROR: archive extracted but install layout invalid" >&2
  exit 1
fi

echo "OK: installed to ${INSTALL}"
