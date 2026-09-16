# shellcheck shell=bash
# Resolve ROBOT_PROFILE.
#
# Priority (highest first):
#   1) explicit CLI / caller override (pass as $1 to apply_robot_profile_arg)
#   2) ROBOT_PROFILE env
#   3) marvin_ws/.skye/robot_profile
#   4) marvin_ws/robot_profile   (fallback when .skye is not writable)
#   5) thor
#
# Known profiles today: thor | orin
# Adding a machine: drop config/profiles/<name>.yaml + configs/<name>/,
# then extend validate_robot_profile / known_robot_profiles.

known_robot_profiles() {
  echo "thor orin"
}

_robot_profile_read_file() {
  local path="${1:?}"
  [[ -f "${path}" ]] || return 1
  tr -d '[:space:]' < "${path}" | tr '[:upper:]' '[:lower:]'
}

validate_robot_profile() {
  case "${1,,}" in
    thor|orin) return 0 ;;
    *)
      echo "ERROR: robot profile must be one of: $(known_robot_profiles) (got: ${1})" >&2
      return 1
      ;;
  esac
}

resolve_robot_profile() {
  local marvin_ws="${1:?marvin_ws path required}"
  if [[ -n "${ROBOT_PROFILE:-}" ]]; then
    echo "${ROBOT_PROFILE,,}"
    return
  fi
  local value=""
  value="$(_robot_profile_read_file "${marvin_ws}/.skye/robot_profile")" && {
    echo "${value}"
    return
  }
  value="$(_robot_profile_read_file "${marvin_ws}/robot_profile")" && {
    echo "${value}"
    return
  }
  echo "thor"
}

# If $1 looks like a profile name, export ROBOT_PROFILE and shift it off.
# Usage in callers:
#   apply_robot_profile_arg "$@"
#   set -- "${_ROBOT_PROFILE_REMAINING[@]}"
apply_robot_profile_arg() {
  _ROBOT_PROFILE_REMAINING=("$@")
  if ((${#_ROBOT_PROFILE_REMAINING[@]} == 0)); then
    return 0
  fi
  local candidate="${_ROBOT_PROFILE_REMAINING[0],,}"
  case "${candidate}" in
    thor|orin)
      export ROBOT_PROFILE="${candidate}"
      _ROBOT_PROFILE_REMAINING=("${_ROBOT_PROFILE_REMAINING[@]:1}")
      ;;
  esac
}

# Best writable path for persisting profile on this machine.
robot_profile_write_path() {
  local marvin_ws="${1:?marvin_ws path required}"
  local skye_dir="${marvin_ws}/.skye"
  local skye_file="${skye_dir}/robot_profile"
  if mkdir -p "${skye_dir}" 2>/dev/null && \
     (umask 022; : > "${skye_file}") 2>/dev/null; then
    echo "${skye_file}"
    return
  fi
  echo "${marvin_ws}/robot_profile"
}
