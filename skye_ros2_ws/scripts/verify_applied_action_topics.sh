#!/usr/bin/env bash
# Verify applied-action topics (RELIABLE QoS) and optional skye_data_recorder services.
#
# Build (from repo root): ./scripts/build.sh skye_robot_driver skye_data_recorder
# Recorder mcap: sudo apt install ros-humble-rosbag2-storage-mcap
#
# Usage: ROS_DOMAIN_ID=21 ./scripts/verify_applied_action_topics.sh
# Requires: sourced workspace, driver (+ optional recorder) running.
#
# Manual integration checklist:
# 1. Terminal A: start skye_robot_driver (imp_joint, gripper on)
# 2. Terminal B: ros2 launch skye_data_recorder data_recorder.launch.py
# 3. ros2 service call /skye/data_recorder/start std_srvs/srv/Trigger {}
# 4. Teleop briefly (or pub joint_control once with driver in position/imp)
# 5. ros2 service call /skye/data_recorder/stop std_srvs/srv/Trigger {}
# 6. Inspect mcap under /tmp/skye_data_bags/episode_XXXX for applied + joint_states
set -euo pipefail

need_topics=(
  /gento/left_joint_action_applied
  /gento/right_joint_action_applied
  /gento/left_gripper_action_applied
  /gento/right_gripper_action_applied
)

topic_list=$(ros2 topic list)
service_list=$(ros2 service list)

publisher_is_reliable() {
  local info=$1
  local publishers_section
  publishers_section=$(printf '%s\n' "$info" | awk '
    /^Publisher count:/ { p=1 }
    p { print }
    /^Subscription count:/ { exit }
  ')
  echo "$publishers_section" | grep -qi 'Reliability: RELIABLE'
}

for t in "${need_topics[@]}"; do
  grep -qx "$t" <<<"$topic_list" || { echo "missing $t"; exit 1; }
  info=$(ros2 topic info -v "$t")
  publisher_is_reliable "$info" || { echo "$t publisher not RELIABLE"; exit 1; }
  if ! timeout 2 ros2 topic echo --once "$t" >/dev/null 2>&1; then
    echo "WARN: $t present but no message within 2s (driver may be idle)"
  fi
done

grep -qx /skye/data_recorder/start <<<"$service_list" || \
  echo "WARN: recorder not running (start service missing)"
grep -qx /skye/data_recorder/stop <<<"$service_list" || true

echo "OK: applied action topics present with RELIABLE QoS"
