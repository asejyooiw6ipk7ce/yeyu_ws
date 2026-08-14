#!/bin/bash
# ================================================================
# turtlebot3_pm2_supervisor.sh
# PM2가 감시하는 단일 프로세스입니다.
#
# 핵심 설계: 이 스크립트는 "boot으로 세션을 만들고 끝나는" 것이 아니라,
# 세션을 만든 뒤 무한 감시 루프에 들어가 절대로 스스로 정상 종료하지
# 않습니다. 그래서 PM2는 이 프로세스가 "계속 살아있는 정상 상태"로
# 인식하고, 불필요한 재시작을 반복하지 않습니다.
#
# 감시 대상 두 가지:
#   1) tmux 세션 자체가 통째로 사라졌는가?
#        → 사라졌으면 supervisor도 종료 (exit 1) → PM2가 재시작
#          → 재시작된 supervisor가 다시 boot → 세션 전체 재생성
#   2) 개별 창(윈도우)이 죽어있는가? (pane_dead=1)
#        → 정상적으로는 각 창의 while-loop이 스스로 재시작하므로
#          거의 발생하지 않지만, 만약을 대비한 2차 안전장치.
#        → 죽은 창만 개별적으로 respawn (세션 전체는 안 건드림)
# ================================================================
set -uo pipefail

SESSION="yeyu_robot"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMUX_SCRIPT="${SCRIPT_DIR}/turtlebot3_tmux.sh"

CHECK_INTERVAL_SEC=5

WINDOW_NAMES=(
    "01-bringup"
    "02-camera"
    "03-sensor"
    "04-audio"
    "05-navigation"
    "06-driving"
)

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [supervisor] $*"
}

log "tmux 세션 boot 시도"
"${TMUX_SCRIPT}" boot

if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
    log "tmux 세션 생성 실패. supervisor를 종료합니다 (PM2가 재시도합니다)."
    exit 1
fi

log "tmux 세션(${SESSION}) 감시 루프 시작 (${CHECK_INTERVAL_SEC}초 주기)"

# ---- 메인 감시 루프: 여기서 절대 정상적으로 빠져나가지 않음 ----
while true; do
    sleep "${CHECK_INTERVAL_SEC}"

    # 1) 세션 자체가 사라졌는지 확인
    if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
        log "tmux 세션(${SESSION})이 사라졌습니다. supervisor를 종료합니다 (PM2가 재시작 → 세션 전체 재생성)."
        exit 1
    fi

    # 2) 개별 창이 죽어있는지 확인 (pane_dead=1)
    while IFS=$'\t' read -r win_name dead_flag; do
        dead_flag="${dead_flag#pane_dead=}"
        if [ "${dead_flag}" == "1" ]; then
            log "경고: 창 '${win_name}' 이(가) 죽어있습니다 (pane_dead=1). 해당 창만 재생성합니다."
            "${TMUX_SCRIPT}" respawn "${win_name}"
        fi
    done < <(tmux list-panes -a -t "${SESSION}" -F '#{window_name}	pane_dead=#{pane_dead}' 2>/dev/null)
done