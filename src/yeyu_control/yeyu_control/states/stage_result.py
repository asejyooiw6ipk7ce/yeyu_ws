from enum import Enum ,auto

class StageResult(Enum):
    IN_PROGRESS = auto()   # 아직 진행 중, 판정 안 남
    PASS = auto()
    FAIL = auto()