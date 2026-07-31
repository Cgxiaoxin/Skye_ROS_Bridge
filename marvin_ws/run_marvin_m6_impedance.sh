#!/usr/bin/env bash
set -euo pipefail

IMAGE="${1:-${IMAGE:-harbor.amigos-robot.com/tmp/marvin-m6-ros2:humble_add_impedance}}"

ROBOT_IP="${ROBOT_IP:-6.6.7.190}"
ROBOT_GRIPPER_PORT_LEFT="${ROBOT_GRIPPER_PORT_LEFT:-/dev/ttyUSB0}"
ROBOT_GRIPPER_PORT_RIGHT="${ROBOT_GRIPPER_PORT_RIGHT:-/dev/ttyUSB1}"
USE_LEFT_GRIPPER="${USE_LEFT_GRIPPER:-false}"
USE_RIGHT_GRIPPER="${USE_RIGHT_GRIPPER:-false}"

# Pass either a full /dev path or a /dev/serial/by-id basename.  _PORT_RIGHT and _PORT_LEFT should be modified
ROBOT_LEADER_DYNAMIXEL_PORT_LEFT="${ROBOT_LEADER_DYNAMIXEL_PORT_LEFT:-usb-FTDI_USB__-__Serial_Converter_FTB8HNOT-if00-port0}"
ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT="${ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT:-usb-FTDI_USB__-__Serial_Converter_FTAO51EA-if00-port0}"

DOCKER_ARGS=(
  --rm
  -it
  --net=host
  --ipc=host
  --privileged
  --group-add dialout
  --cap-add SYS_NICE
  --ulimit rtprio=99
  --ulimit memlock=-1
  -e "ROBOT_IP=${ROBOT_IP}"
  -e "ROBOT_GRIPPER_PORT_LEFT=${ROBOT_GRIPPER_PORT_LEFT}"
  -e "ROBOT_GRIPPER_PORT_RIGHT=${ROBOT_GRIPPER_PORT_RIGHT}"
  -e "USE_LEFT_GRIPPER=${USE_LEFT_GRIPPER}"
  -e "USE_RIGHT_GRIPPER=${USE_RIGHT_GRIPPER}"
  -e "ROBOT_LEADER_DYNAMIXEL_PORT_LEFT=${ROBOT_LEADER_DYNAMIXEL_PORT_LEFT}"
  -e "ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT=${ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT}"
  -v /dev:/dev
  -v /home/wsj/pycode/marvin_ws_humble_add_impedance/marvin_ws:/marvin_ws
  -w /marvin_ws
)

if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
  DOCKER_ARGS+=(
    -e "DISPLAY=${DISPLAY}"
    -v /tmp/.X11-unix:/tmp/.X11-unix
  )
fi

exec docker run "${DOCKER_ARGS[@]}" "${IMAGE}"
