# shellcheck shell=bash
# Resolve ROBOT_PROFILE: env > marvin_ws/.skye/robot_profile > thor
resolve_robot_profile() {
  local marvin_ws="${1:?marvin_ws path required}"
  if [[ -n "${ROBOT_PROFILE:-}" ]]; then
    echo "${ROBOT_PROFILE,,}"
    return
  fi
  local profile_file="${marvin_ws}/.skye/robot_profile"
  if [[ -f "${profile_file}" ]]; then
    tr -d '[:space:]' < "${profile_file}" | tr '[:upper:]' '[:lower:]'
    return
  fi
  echo "thor"
}

validate_robot_profile() {
  case "${1,,}" in
    thor|orin) return 0 ;;
    *)
      echo "ERROR: ROBOT_PROFILE must be thor|orin (got: ${1})" >&2
      return 1
      ;;
  esac
}
