#!/bin/bash
# ================================================================
# 공통 유틸리티: ROS2 환경 소싱 + 토픽/액션/장치 대기 함수 모음
# nodes/*.sh 에서 source 해서 사용합니다.
# ================================================================

source_ros_env() {
    source /opt/ros/humble/setup.bash
    source "$HOME/yeyu_ws/install/setup.bash"
}

# 사용법: wait_for_topic /odom 60
wait_for_topic() {
    local topic="$1"
    local timeout_sec="${2:-60}"
    local waited=0

    echo "[wait] 토픽 대기 중: ${topic} (최대 ${timeout_sec}초)"
    until ros2 topic list 2>/dev/null | grep -qx "${topic}"; do
        sleep 1
        waited=$((waited + 1))
        if [ "${waited}" -ge "${timeout_sec}" ]; then
            echo "[wait] 타임아웃: ${topic} 이(가) ${timeout_sec}초 내에 나타나지 않았습니다. 계속 진행합니다."
            return 1
        fi
    done
    echo "[wait] 확인됨: ${topic}"
    return 0
}

# 사용법: wait_for_action /navigate_to_pose 90
wait_for_action() {
    local action="$1"
    local timeout_sec="${2:-90}"
    local waited=0

    echo "[wait] 액션 서버 대기 중: ${action} (최대 ${timeout_sec}초)"
    until ros2 action list 2>/dev/null | grep -qx "${action}"; do
        sleep 1
        waited=$((waited + 1))
        if [ "${waited}" -ge "${timeout_sec}" ]; then
            echo "[wait] 타임아웃: ${action} 이(가) ${timeout_sec}초 내에 나타나지 않았습니다. 계속 진행합니다."
            return 1
        fi
    done
    echo "[wait] 확인됨: ${action}"
    return 0
}

# 사용법: wait_for_device /dev/tb3_sensor 30
wait_for_device() {
    local device="$1"
    local timeout_sec="${2:-30}"
    local waited=0

    echo "[wait] 장치 대기 중: ${device} (최대 ${timeout_sec}초)"
    until [ -e "${device}" ]; do
        sleep 1
        waited=$((waited + 1))
        if [ "${waited}" -ge "${timeout_sec}" ]; then
            echo "[wait] 타임아웃: ${device} 이(가) ${timeout_sec}초 내에 나타나지 않았습니다. 계속 진행합니다."
            return 1
        fi
    done
    echo "[wait] 확인됨: ${device}"
    return 0
}