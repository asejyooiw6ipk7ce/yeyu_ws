# system_state.py
from enum import Enum, auto

class SystemState(Enum):
    IDLE = auto()
    STARTUP_CHECK = auto()
    NAV_WAYPOINT = auto()
    LINE_TRACING = auto()
    OBSTACLE_RESPONSE = auto()
    PARKING = auto()
    SIGNAL_WAIT = auto()
    ACCEL_ZONE = auto()
    RESULT_SUMMARY = auto()
    RETRY = auto()
    E_STOP = auto()