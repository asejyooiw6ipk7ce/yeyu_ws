#!/bin/bash
# ================================================================
# 2) 카메라 노드 (bringup이 완전히 뜬 뒤 시작)
# ================================================================
set -uo pipefail
source "$HOME/yeyu_ws/deploy/lib/wait_ros.sh"
source_ros_env

wait_for_topic "/odom" 60

echo "[02_camera] camera.launch.py 시작"
exec ros2 launch yeyu_bringup camera.launch.py format:=BGR888