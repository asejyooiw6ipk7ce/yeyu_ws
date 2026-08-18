#!/bin/bash
# ================================================================
# yeyu_launch/scripts/yeyu_pm2_supervisor.sh
# PM2가 감시하는 단일 프로세스. boot 후 무한 감시 루프로 들어가
# 절대 스스로 정상 종료하지 않습니다.
# ================================================================
set -uo pipefail

SESSION="yeyu_robot"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../yeyu_launch/scripts
TMUX_SCRIPT="${SCRIPT_DIR}/yeyu_tmux.sh"                      # 같은 scripts/ 폴더 안

CHECK_INTERVAL_SEC=5

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

while true; do
    sleep "${CHECK_INTERVAL_SEC}"

    if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
        log "tmux 세션(${SESSION})이 사라졌습니다. supervisor를 종료합니다 (PM2가 재시작 → 세션 전체 재생성)."
        exit 1
    fi

    while IFS=$'\t' read -r win_name dead_flag; do
        dead_flag="${dead_flag#pane_dead=}"
        if [ "${dead_flag}" == "1" ]; then
            log "경고: 창 '${win_name}' 이(가) 죽어있습니다 (pane_dead=1). 해당 창만 재생성합니다."
            "${TMUX_SCRIPT}" respawn "${win_name}"
        fi
    done < <(tmux list-panes -a -t "${SESSION}" -F '#{window_name}	pane_dead=#{pane_dead}' 2>/dev/null)
done