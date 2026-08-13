#!/usr/bin/env bash
# Build the skye_ros2_ws workspace (main C++ teleop driver).
set -euo pipefail

WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"

# Prefer system Python so ament finds dist-packages (catkin_pkg), not conda.
if [[ -x /usr/bin/python3 ]]; then
  export PATH="/usr/bin:${PATH}"
  export PYTHON_EXECUTABLE=/usr/bin/python3
fi

# ROS setup.bash references optional unset vars; disable nounset while sourcing.
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # Avoid conda shadowing ROS Python modules during build.
  if command -v conda >/dev/null 2>&1 && [[ -n "${CONDA_PREFIX:-}" ]]; then
    conda deactivate >/dev/null 2>&1 || true
  fi
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "ROS 2 setup not found at /opt/ros/humble/setup.bash" >&2
  exit 1
fi

PKG="${1:-skye_robot_driver}"
echo "== colcon build: $PKG =="
echo "python3=$(command -v python3)"
colcon build --packages-select "$PKG" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
set +u
source install/setup.bash
set -u
echo "== built: $PKG =="
