#!/bin/bash
# ================================================================
# 3) 센서 노드 (arduino_bridge_node_serial)
#    시리얼 장치가 실제로 붙을 때까지 대기 후 시작합니다.
# ================================================================
set -uo pipefail
source "$HOME/yeyu_ws/deploy/lib/wait_ros.sh"
source_ros_env

wait_for_device "/dev/tb3_sensor" 30

echo "[03_sensor] arduino_bridge_node_serial 시작"
exec ros2 run yeyu_control arduino_bridge_node_serial --ros-args -p port:=/dev/tb3_sensor