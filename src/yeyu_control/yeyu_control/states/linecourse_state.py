from enum import Enum

class LineCourseState(Enum):
    LINE_FOLLOWING = 'LINE_FOLLOWING'
    TURNING = 'TURNING'
    DONE = 'DONE'
    FAILED = 'FAILED'