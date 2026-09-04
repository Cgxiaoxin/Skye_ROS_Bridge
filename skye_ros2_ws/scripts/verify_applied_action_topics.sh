#!/usr/bin/env bash
# Verify applied-action topics (RELIABLE QoS) and optional skye_data_recorder services.
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

for t in "${need_topics[@]}"; do
  ros2 topic list | grep -qx "$t" || { echo "missing $t"; exit 1; }
  info=$(ros2 topic info -v "$t")
  echo "$info" | grep -qi Reliable || { echo "$t not RELIABLE"; exit 1; }
done

ros2 service list | grep -qx /skye/data_recorder/start || \
  echo "WARN: recorder not running (start service missing)"
ros2 service list | grep -qx /skye/data_recorder/stop || true

echo "OK: applied action topics present with RELIABLE QoS"
