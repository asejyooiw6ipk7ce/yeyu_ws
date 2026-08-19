from enum import Enum, auto

class DrivingMode(Enum):
    NAV_TO_START = auto()      # 초기위치 → 크랭크 시작점(wp1)
    TRACING_CRANK = auto()    # 크랭크 코스 (IR+카메라 라인트레이싱) 
    NAV_TO_S = auto()          # 크랭크 도착점(wp2) → S자 시작점(wp3)로 이동
    TRACING_S = auto()        # S자 코스 (나중에 카메라 중심점 정렬로 구현)
    NAV_TO_MAZE = auto()       # S자 도착점(wp4 )→ 미로 시작점(wp5)
    NAV_TO_SIGNAL = auto()     # 미로 시작점 → 시그널 시작점(wp6)
    SIGNAL_WAIT = auto()
    NAV_TO_ACCEL = auto()      # 시그널 시작점(wp6)-> 속도 시작점(wp7)
    ACCEL_ZONE = auto()
    NAV_TO_PARKING = auto()    # wp7(속도 시작점) → 속도가속 → wp8(경유) -> 속도정상 -> wp9(주차시작점)
    PARKING = auto()
    NAV_TO_END = auto()        # PARKING 위치 -> wp10(경유) → wp11(최종도착점)
    E_STOP = auto()
    RESULT_SUMMARY = auto()
    RETRY = auto()
    WIFI_RETURN_HOME = auto()  # wifi 단절 감지 → 진행 중이던 구간 취소하고 wp1(첫 위치)로 복귀