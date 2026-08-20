#!/bin/bash
# ================================================================
# yeyu_launch/scripts/install_watchdog.sh
# lo_alias_watchdog.sh를 systemd 서비스로 등록하는 1회성 설치 스크립트.
# 로봇 온보드 컴퓨터에서 실행. colcon build로는 실행되지 않음
# (yeyu_launch는 COLCON_IGNORE로 colcon 빌드 대상에서 제외되어 있음).
# 재실행해도 안전(idempotent).
# ================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log() {
    echo "[install_watchdog] $*"
}

log "sudoers 규칙 설치"
sudo install -o root -g root -m 0440 \
    "${SCRIPT_DIR}/lo_alias_watchdog.sudoers" /etc/sudoers.d/lo_alias_watchdog

log "systemd 유닛 파일 설치"
sudo cp "${SCRIPT_DIR}/lo-alias-watchdog.service" /etc/systemd/system/lo-alias-watchdog.service

log "systemd 데몬 리로드"
sudo systemctl daemon-reload

log "부팅 시 자동 시작 등록 + 지금 바로 시작"
sudo systemctl enable --now lo-alias-watchdog

log "설치 완료. 상태 확인: sudo systemctl status lo-alias-watchdog"
