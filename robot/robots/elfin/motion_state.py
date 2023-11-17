from enum import Enum

class MotionState(Enum):
    FREE_TO_MOVE = 0
    IN_MOTION = 1
    WAITING_FOR_EXECUTION = 2
    ERROR = 3
    UNKNOWN = 4
