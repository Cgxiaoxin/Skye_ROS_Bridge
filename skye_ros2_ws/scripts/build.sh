#!/usr/bin/env bash
# Build the skye_ros2_ws workspace (main C++ teleop driver).
set -euo pipefail

WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
else
  echo "ROS 2 setup not found at /opt/ros/humble/setup.bash" >&2
  exit 1
fi

PKG="${1:-skye_robot_driver}"
echo "== colcon build: $PKG =="
colcon build --packages-select "$PKG" --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
echo "== built: $PKG =="
