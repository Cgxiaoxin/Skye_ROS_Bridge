#!/usr/bin/env bash
# Pack marvin_ws/install and upload to GitLab Generic Package Registry.
#
# Run ONCE on a machine that already has a working marvin_ws/install/.
# Do NOT commit the .tgz into git.
#
# Usage:
#   tar --exclude='__pycache__' -czf /tmp/marvin_ws_install.tgz marvin_ws/install   # or let this script pack
#   export GITLAB_TOKEN=glpat-xxxxxxxxxxxx
#   ./scripts/publish_marvin_install.sh
#
# Optional:
#   MARVIN_INSTALL_VERSION=1.0.1 ./scripts/publish_marvin_install.sh
#   MARVIN_INSTALL_TGZ=/tmp/marvin_ws_install.tgz ./scripts/publish_marvin_install.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VERSION_FILE="${SCRIPT_DIR}/marvin_install.version"

PACKAGE_NAME="${MARVIN_INSTALL_PACKAGE:-marvin-ws}"
ARCHIVE_NAME="${MARVIN_INSTALL_ARCHIVE:-marvin_ws_install.tgz}"
INSTALL="${REPO_ROOT}/marvin_ws/install"

read_version() {
  if [[ -n "${MARVIN_INSTALL_VERSION:-}" ]]; then
    printf '%s' "${MARVIN_INSTALL_VERSION}"
  elif [[ -f "${VERSION_FILE}" ]]; then
    tr -d '[:space:]' < "${VERSION_FILE}"
  else
    printf '1.0.1'
  fi
}

# Return 0 if ${1}/api/v4/version answers on HTTP at all (any status code,
# including 401 "Unauthorized" — that is still a live GitLab API).
gitlab_base_is_live() {
  local code
  code="$(curl -sS -o /dev/null -m 2 -w '%{http_code}' "${1}/api/v4/version" 2>/dev/null || true)"
  [[ -n "${code}" && "${code}" != "000" ]]
}

default_gitlab_api() {
  local remote host project proto tries base
  remote="$(git -C "${REPO_ROOT}" remote get-url alex 2>/dev/null \
    || git -C "${REPO_ROOT}" remote get-url origin 2>/dev/null \
    || true)"
  [[ -n "${remote}" ]] || return 1

  if [[ "${remote}" =~ ^git@([^:]+):(.+)\.git$ ]]; then
    host="${BASH_REMATCH[1]}"
    project="${BASH_REMATCH[2]//\//%2F}"
    proto="http"
  elif [[ "${remote}" =~ ^(https?)://([^/]+)/(.+)\.git$ ]]; then
    proto="${BASH_REMATCH[1]}"
    host="${BASH_REMATCH[2]}"
    project="${BASH_REMATCH[3]//\//%2F}"
  else
    return 1
  fi

  # The git remote only reveals the SSH host; GitLab's Web/API often listens on
  # a non-default HTTP port (e.g. 8929 for the official Docker image). Probe a
  # few common ports and fall back to the plain remote-derived URL.
  if [[ "${host}" == *:* ]]; then
    tries=("http://${host}" "https://${host}")
  else
    tries=("${proto}://${host}" "http://${host}:8929" "https://${host}" "http://${host}:8080")
  fi
  for base in "${tries[@]}"; do
    if gitlab_base_is_live "${base}"; then
      printf '%s/api/v4/projects/%s' "${base}" "${project}"
      return 0
    fi
  done
  printf '%s://%s/api/v4/projects/%s' "${proto}" "${host}" "${project}"
}

[[ -f "${INSTALL}/setup.bash" ]] \
  || { echo "ERROR: ${INSTALL}/setup.bash missing; nothing to publish" >&2; exit 1; }

[[ -n "${GITLAB_TOKEN:-}" ]] \
  || { echo "ERROR: set GITLAB_TOKEN (GitLab → Settings → Access Tokens, scope: api)" >&2; exit 1; }

GITLAB_API="${GITLAB_API:-$(default_gitlab_api)}"
[[ -n "${GITLAB_API}" ]] \
  || { echo "ERROR: cannot derive GITLAB_API from git remote; set GITLAB_API manually" >&2; exit 1; }

VERSION="$(read_version)"
ARCHIVE="${MARVIN_INSTALL_TGZ:-/tmp/marvin_ws_install.tgz}"

if [[ ! -f "${ARCHIVE}" ]]; then
  echo "== packing ${ARCHIVE} =="
  tar --exclude='__pycache__' -czf "${ARCHIVE}" -C "${REPO_ROOT}" marvin_ws/install
fi

UPLOAD_URL="${GITLAB_API}/packages/generic/${PACKAGE_NAME}/${VERSION}/${ARCHIVE_NAME}"

echo "== upload marvin_ws/install =="
echo "   version: ${VERSION}"
echo "   file:    ${ARCHIVE} ($(du -h "${ARCHIVE}" | cut -f1))"
echo "   url:     ${UPLOAD_URL}"

curl --fail --header "PRIVATE-TOKEN: ${GITLAB_TOKEN}" \
  --upload-file "${ARCHIVE}" \
  "${UPLOAD_URL}"

echo ""
echo "OK: uploaded ${PACKAGE_NAME}/${VERSION}/${ARCHIVE_NAME}"
echo ""
echo "New machines:"
echo "  git clone ... && cd Skye_ROS_Bridge"
echo "  GITLAB_TOKEN=\$GITLAB_TOKEN ./scripts/bootstrap_marvin_install.sh"
echo ""
echo "Download URL (for MARVIN_INSTALL_URL):"
echo "  ${UPLOAD_URL}"
