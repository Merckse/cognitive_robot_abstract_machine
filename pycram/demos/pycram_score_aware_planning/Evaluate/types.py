from dataclasses import dataclass
from enum import Enum
from typing import Optional

from semantic_digital_twin.spatial_types.spatial_types import Pose


# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

class ActionType(str, Enum):
    PICKUP          = "pickup"
    PLACE           = "place"
    OPEN            = "open"
    CLOSE           = "close"
    NAVIGATE        = "navigate"
    HAND_OVER       = "hand_over"
    POUR            = "pour"
    PUSH            = "push"
    DETECT          = "detect"
    CUSTOM          = "custom"


class ActionOutcome(str, Enum):
    SUCCESS              = "success"
    SUCCESS_WITH_ASSIST  = "success_with_assist"
    FAILURE_RECOVERABLE  = "failure_recoverable"
    FAILURE_UNRECOVERABLE = "failure_unrecoverable"
    SKIPPED              = "skipped"
    RUNNING              = "running"


class TaskMode(str, Enum):
    GPSR = "gpsr"
    PP = "pp"
    FD = "fd"

@dataclass
class TaskStep:
    action: ActionType
    object_name: Optional[str] = None
    object_pickup: Optional["str"] = None
    object_placement: Optional["str"] = None

@dataclass
class Task:
    task_steps: list[TaskStep]