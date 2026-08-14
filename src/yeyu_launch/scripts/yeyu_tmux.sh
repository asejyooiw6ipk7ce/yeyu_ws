#!/bin/bash
# ================================================================
# turtlebot3_tmux.sh
# tmux 세션(yeyu_robot) 안에 6개 노드를 창(window)으로 띄웁니다.
# 각 창은 자체적으로 "무한 재시작 루프"를 돌기 때문에,
# 개별 노드가 죽어도 그 노드만 스스로 재시작됩니다.
#
# 서브커맨드:
#   boot     세션을 새로 만들고 즉시 리턴합니다 (접속 X, 블로킹 X).
#            → supervisor가 PM2 프로세스로서 이 명령만 호출합니다.
#   stop     세션을 종료합니다.
#   status   창 목록/상태를 출력합니다.
#   attach   세션에 직접 접속합니다 (Ctrl+B, D 로 분리).
#   respawn <윈도우이름>   특정 창 하나만 강제로 재생성합니다.
#                          (supervisor가 죽은 창을 스스로 고칠 때 사용)
# ================================================================
set -uo pipefail

SESSION="yeyu_robot"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODES_DIR="${SCRIPT_DIR}/nodes"
LOG_DIR="${SCRIPT_DIR}/logs/nodes"

mkdir -p "${LOG_DIR}"

# 창 이름 : 실행할 노드 스크립트 (순서대로 생성됨)
WINDOWS=(
    "01-bringup:01_bringup.sh"
    "02-camera:02_camera.sh"
    "03-sensor:03_sensor.sh"
    "04-audio:04_audio.sh"
    "05-navigation:05_navigation.sh"
    "06-driving:06_driving.sh"
)

# 각 창에서 돌아갈 명령: 무한 루프로 노드 실행, 죽으면 3초 후 재시작.
# 화면 + 로그 파일에 동시에 기록(tee).
build_window_cmd() {
    local name="$1"
    local script="$2"
    local logfile="${LOG_DIR}/${name}.log"
    cat <<EOF
while true; do
    echo "[\$(date '+%Y-%m-%d %H:%M:%S')] [${name}] 시작" | tee -a "${logfile}"
    bash "${NODES_DIR}/${script}" 2>&1 | tee -a "${logfile}"
    echo "[\$(date '+%Y-%m-%d %H:%M:%S')] [${name}] 종료됨. 3초 후 재시작합니다." | tee -a "${logfile}"
    sleep 3
done
EOF
}

script_for_window() {
    local target="$1"
    for entry in "${WINDOWS[@]}"; do
        local name="${entry%%:*}"
        local script="${entry##*:}"
        if [ "${name}" == "${target}" ]; then
            echo "${script}"
            return 0
        fi
    done
    return 1
}

# 세션만 만들고 즉시 리턴 (접속하지 않음, 블로킹하지 않음)
boot_session() {
    if tmux has-session -t "${SESSION}" 2>/dev/null; then
        echo "[tmux boot] 기존 세션(${SESSION})이 있어 종료 후 새로 만듭니다."
        tmux kill-session -t "${SESSION}"
    fi

    local first=1
    for entry in "${WINDOWS[@]}"; do
        local name="${entry%%:*}"
        local script="${entry##*:}"
        local cmd
        cmd="$(build_window_cmd "${name}" "${script}")"

        if [ "${first}" -eq 1 ]; then
            tmux new-session -d -s "${SESSION}" -n "${name}" "bash -c '${cmd}'"
            first=0
        else
            tmux new-window -t "${SESSION}" -n "${name}" "bash -c '${cmd}'"
        fi
    done

    tmux select-window -t "${SESSION}:01-bringup"
    echo "[tmux boot] 세션 생성 완료: ${SESSION} (창 ${#WINDOWS[@]}개). 접속하지 않고 종료합니다."
    return 0
}

stop_session() {
    if tmux has-session -t "${SESSION}" 2>/dev/null; then
        tmux kill-session -t "${SESSION}"
        echo "[tmux] 세션 종료됨: ${SESSION}"
    else
        echo "[tmux] 세션이 이미 없습니다: ${SESSION}"
    fi
}

status_session() {
    if tmux has-session -t "${SESSION}" 2>/dev/null; then
        echo "[tmux] 세션 ${SESSION} 실행 중. 창 목록 (pane_dead=1 이면 창이 죽은 상태):"
        tmux list-panes -a -t "${SESSION}" -F '#{window_name}\tpane_dead=#{pane_dead}\tpid=#{pane_pid}'
    else
        echo "[tmux] 세션이 없습니다: ${SESSION}"
    fi
}

attach_session() {
    tmux attach -t "${SESSION}"
}

# 특정 창 하나만 강제로 재생성 (supervisor의 자가 치유용)
respawn_window() {
    local name="$1"
    local script
    script="$(script_for_window "${name}")" || {
        echo "[tmux respawn] 알 수 없는 창 이름: ${name}"
        return 1
    }
    local cmd
    cmd="$(build_window_cmd "${name}" "${script}")"

    if tmux list-windows -t "${SESSION}" -F '#{window_name}' 2>/dev/null | grep -qx "${name}"; then
        echo "[tmux respawn] 창 재생성: ${name}"
        tmux respawn-window -k -t "${SESSION}:${name}" "bash -c '${cmd}'"
    else
        echo "[tmux respawn] 창이 존재하지 않아 새로 생성: ${name}"
        tmux new-window -t "${SESSION}" -n "${name}" "bash -c '${cmd}'"
    fi
}

case "${1:-}" in
    boot)    boot_session ;;
    stop)    stop_session ;;
    status)  status_session ;;
    attach)  attach_session ;;
    respawn) respawn_window "${2:-}" ;;
    *)
        echo "사용법: $0 {boot|stop|status|attach|respawn <윈도우이름>}"
        exit 1
        ;;
esac