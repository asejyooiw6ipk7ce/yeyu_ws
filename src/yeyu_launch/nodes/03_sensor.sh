#!/bin/bash
sleep 6
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../lib/wait_ros.sh"
source_ros_env

wait_for_device "/dev/tb3_sensor" 30

echo "[yeyu_sensor_bridge] arduino_bridge_node_serial 시작"
exec ros2 run yeyu_control arduino_bridge_node_serial --ros-args -p port:=/dev/tb3_sensor