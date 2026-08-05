#!/bin/bash
CARD_NUM=$(aplay -l | grep -i "USB Audio" | grep -oP '(?<=card )\d+' | head -1)

if [ -z "$CARD_NUM" ]; then
    echo "USB 오디오 장치를 찾을 수 없습니다."
    exit 1
fi

cat > ~/.asoundrc << ASOUNDRC
pcm.!default {
    type hw
    card ${CARD_NUM}
}
ctl.!default {
    type hw
    card ${CARD_NUM}
}
ASOUNDRC

echo "USB 스피커(card ${CARD_NUM})를 기본 출력으로 설정했습니다."
