#!/usr/bin/env bash
# Offline by default. --connect is an explicit, later hardware test action.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS="${REPO_ROOT}/skye_ros2_ws"
PARAMS="${WS}/src/skye_robot_driver/config/force_feedback_test.yaml"
PROFILE="thor"
ACTION="--check"

usage() {
  cat <<'USAGE'
Usage: bash scripts/start_force_feedback_test.sh [thor|orin] [ACTION] [monitor args]
  --check     Offline file/config/dependency check (default; no ROS node or SDK link)
  --offline   Start the driver with controller connection and grippers disabled
  --connect   Connect the selected controller, request Idle, keep grippers disabled
  --monitor   Subscribe to feedback for 30 seconds; optional monitor args follow
  --help      Show this help

Default test domain: ROS_DOMAIN_ID=218, ROS_LOCALHOST_ONLY=1.
Thor: 6.6.7.191. Orin: 6.6.7.190. Select the actual robot explicitly.
--connect resets SDK errors and requests Idle; it is not a read-only SDK link.
USAGE
}

if [[ "${1:-}" == thor || "${1:-}" == orin ]]; then
  PROFILE="$1"
  shift
fi
if (($#)); then
  ACTION="$1"
  shift
fi
case "${ACTION}" in
  --help|-h) usage; exit 0 ;;
  --check|--offline|--connect|--monitor) ;;
  *) usage >&2; exit 2 ;;
esac
if [[ "${ACTION}" != --monitor && $# -ne 0 ]]; then
  usage >&2
  exit 2
fi

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo 'Missing /opt/ros/humble/setup.bash' >&2; exit 1;
}
[[ -f "${WS}/install/setup.bash" &&
   -x "${WS}/install/skye_robot_driver/lib/skye_robot_driver/skye_robot_driver" ]] || {
  echo 'Build first: source /opt/ros/humble/setup.bash; cd skye_ros2_ws; colcon build --packages-select skye_robot_driver' >&2
  exit 1
}
export PATH="/usr/bin:${PATH}"
set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-218}"
# Keep the test's ROS graph local even if a teleop shell exported 0.
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/skye_force_feedback_ros_logs}"
export ROS_HOME="${ROS_HOME:-/tmp/skye_force_feedback_ros_home}"
# This test does not depend on the legacy marvin_ws FastDDS XML.
if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" &&
      ! -f "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]]; then
  unset FASTRTPS_DEFAULT_PROFILES_FILE
fi

if [[ "${ACTION}" == --check ]]; then
  /usr/bin/python3 - "${REPO_ROOT}" <<'PY'
import ast
import ctypes
import pathlib
import platform
import sys
import yaml
import rclpy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

root = pathlib.Path(sys.argv[1])
package = root / 'skye_ros2_ws/src/skye_robot_driver'
config = yaml.safe_load((package / 'config/force_feedback_test.yaml').read_text())
params = config['skye_robot_driver']['ros__parameters']
assert params['control_mode'] == 'idle'
assert params['connect_on_startup'] is False
assert params['enable_gripper'] is False
assert params['state_publish_hz'] == 250.0
launch = root / 'skye_ros2_ws/install/skye_robot_driver/share/skye_robot_driver/launch/skye_robot_driver.launch.py'
tree = ast.parse(launch.read_text())
remaps = next(ast.literal_eval(node.value) for node in tree.body
              if isinstance(node, ast.Assign)
              and any(isinstance(target, ast.Name) and target.id == '_REMAPS'
                      for target in node.targets))
for side in ('left', 'right'):
    for suffix in ('joint_states', 'joint_external_torque', 'flange_wrench'):
        name = f'/{side}_{suffix}'
        assert (name, '/gento' + name) in remaps, name
arch = {'arm64': 'aarch64', 'amd64': 'x86_64'}.get(platform.machine(), platform.machine())
ctypes.CDLL(str(root / 'third_party/gento_sdk/lib' / arch / 'libGentoSDK.so'))
print('PASS: test config, installed topic remaps, ROS message imports and SDK library load')
print('No ROS node started; no controller connection attempted.')
PY
  echo "Test environment: profile=${PROFILE} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
  exit 0
fi

if [[ "${ACTION}" == --monitor ]]; then
  exec /usr/bin/python3 "${SCRIPT_DIR}/debug/watch_force_feedback.py" "$@"
fi

CONNECT=false
if [[ "${ACTION}" == --connect ]]; then
  if pgrep -f 'lib/skye_robot_driver/[s]kye_robot_driver|[g]ento_robot_driver' >/dev/null; then
    echo 'A local robot driver is already running. Stop its session before this test.' >&2
    exit 1
  fi
  CONNECT=true
  echo 'Connecting controller in Idle; SDK resets errors and requests Idle at startup.'
  echo 'Use only when this controller has no other SDK client. Grippers remain disabled.'
else
  echo 'Offline driver: no controller connection; feedback messages will not be published.'
fi
echo "profile=${PROFILE} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
exec ros2 launch skye_robot_driver skye_robot_driver.launch.py \
  robot_profile:="${PROFILE}" \
  params_file:="${PARAMS}" \
  connect_on_startup:="${CONNECT}" \
  enable_gripper:=false
