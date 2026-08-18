#!/bin/bash
# ================================================================
# 공통 유틸리티: ROS2 환경 소싱 + 토픽/액션/장치 대기 함수 모음
# nodes/*.sh 에서 source 해서 사용합니다.
# ================================================================

source_ros_env() {
    set +u    # ROS2 setup.bash 내부의 미정의 변수 참조를 허용하기 위해 잠시 해제
    source /opt/ros/humble/setup.bash
    source "$HOME/yeyu_ws/install/setup.bash"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI
    set -u    # 다시 켜서 나머지 스크립트는 안전하게 유지
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
# 사용법: wait_for_tf map base_link 60
wait_for_tf() {
    local target_frame="$1"
    local source_frame="$2"
    local timeout_sec="${3:-60}"
    local waited=0

    echo "[wait] tf 대기 중: ${target_frame} -> ${source_frame} (최대 ${timeout_sec}초)"
    until ros2 run tf2_ros tf2_echo "${target_frame}" "${source_frame}" --once > /dev/null 2>&1; do
        sleep 1
        waited=$((waited + 1))
        if [ "${waited}" -ge "${timeout_sec}" ]; then
            echo "[wait] 타임아웃: ${target_frame} -> ${source_frame} tf가 ${timeout_sec}초 내에 나타나지 않았습니다. 계속 진행합니다."
            return 1
        fi
    done
    echo "[wait] 확인됨: ${target_frame} -> ${source_frame}"
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
