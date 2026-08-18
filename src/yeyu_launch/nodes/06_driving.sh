#!/bin/bash
# ================================================================
# 6) 상태 관리 노드 (driving_waypoint_node)
#    Nav2의 navigate_to_pose 액션 서버가 뜬 뒤 시작합니다.
# ================================================================
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../lib/wait_ros.sh"
source_ros_env

wait_for_action "/navigate_to_pose" 120
wait_for_tf "map" "base_link" 60 

sleep 1   # ← 추가

echo "[06_driving] driving_waypoint_node 시작"
exec ros2 run yeyu_control driving_waypoint_node