#!/bin/bash
# ================================================================
# 4) USB 스피커 설정 후 오디오 출력 노드 시작
#    set_usb_speaker.sh 경로가 다르면 아래 SPEAKER_SCRIPT 값을 수정하세요.
# ================================================================
set -uo pipefail
source "$HOME/yeyu_ws/deploy/lib/wait_ros.sh"
source_ros_env

SPEAKER_SCRIPT="$HOME/yeyu_ws/bin/set_usb_speaker.sh"

if [ -x "${SPEAKER_SCRIPT}" ]; then
    echo "[04_audio] USB 스피커 설정 실행: ${SPEAKER_SCRIPT}"
    "${SPEAKER_SCRIPT}"
else
    echo "[04_audio] 경고: ${SPEAKER_SCRIPT} 를 찾을 수 없거나 실행 권한이 없습니다. 건너뜁니다."
fi

echo "[04_audio] audio_output_node 시작"
exec ros2 run yeyu_control audio_output_node