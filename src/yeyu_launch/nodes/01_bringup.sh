#!/bin/bash
# yeyu_bringup: TurtleBot3 Bringup
sleep 0
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../lib/wait_ros.sh"
source_ros_env

echo "[yeyu_bringup] robot.launch.py 시작"
exec ros2 launch yeyu_bringup robot.launch.py


