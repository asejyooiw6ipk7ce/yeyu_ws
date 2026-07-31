from enum import Enum, auto

class DrivingMode(Enum):
    NAV_TO_START = auto()
    NAV_TO_PARKING = auto()
    PARKING = auto()
    OBSTACLE_RESPONSE = auto()
    NAV_TO_SIGNAL = auto()
    SIGNAL_WAIT = auto()
    NAV_TO_ACCEL = auto()
    ACCEL_ZONE = auto()
    NAV_TO_END = auto()