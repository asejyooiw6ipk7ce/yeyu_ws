#!/bin/bash
# wlan0가 죽으면 그 IP를 loopback에 별칭으로 걸어서, 로컬 프로세스간 통신이
# wlan0 상태와 무관하게 항상 그 IP로 도달 가능하게 유지한다.
# wlan0가 살아있을 때는 별칭을 반드시 제거해서 주소 충돌을 피한다.
ALIAS_IP="192.168.230.100/32"
IFACE="wlan0"
LOG="/home/yeyu/lo_alias.log"

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" >> "$LOG"
}

log "감시 시작"

while true; do
    state=$(cat /sys/class/net/$IFACE/operstate 2>/dev/null)
    has_alias=$(ip -4 addr show dev lo | grep -q "192.168.230.100/32" && echo yes || echo no)

    if [ "$state" != "up" ]; then
        if [ "$has_alias" == "no" ]; then
            sudo ip addr add $ALIAS_IP dev lo 2>>"$LOG"
            log "wlan0 state=$state -> lo에 별칭 추가"
        fi
    else
        if [ "$has_alias" == "yes" ]; then
            sudo ip addr del $ALIAS_IP dev lo 2>>"$LOG"
            log "wlan0 state=$state -> lo 별칭 제거"
        fi
    fi
    sleep 1
done
