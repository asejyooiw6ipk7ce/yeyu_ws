from enum import Enum

class LineCourseState(Enum):
    LINE_FOLLOWING = 'LINE_FOLLOWING'
    CREEPING = 'CREEPING'      # ← 추가
    TURNING = 'TURNING'
    DONE = 'DONE'
    FAILED = 'FAILED'

class SCourseState(Enum):
    TRACKING = 'TRACKING'   # 오프셋 기반 회전+직진 중
    DONE = 'DONE'
    FAILED = 'FAILED'