#!/bin/bash
# ================================================================
# 5) Navigation2 (odom, scan이 준비된 뒤 시작)
# ================================================================
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../lib/wait_ros.sh"
source_ros_env

wait_for_topic "/odom" 60
wait_for_topic "/scan" 60

echo "[05_navigation] navigation2.launch.py 시작"
exec ros2 launch yeyu_navigation2 navigation2.launch.py \
    map:="$HOME/yeyu_ws/src/yeyu_navigation2/map/yeyu_map1.yaml" \
    use_sim_time:=false