from dataclasses import dataclass, field
from enum import Enum
from itertools import count
from typing import Optional

from semantic_digital_twin.spatial_types.spatial_types import Pose

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------


class ActionType(str, Enum):
    PICKUP = "pickup"
    PLACE = "place"
    OPEN = "open"
    CLOSE = "close"
    NAVIGATE = "navigate"
    HAND_OVER = "hand_over"
    POUR = "pour"
    PUSH = "push"
    DETECT = "detect"
    CUSTOM = "custom"


class ActionOutcome(str, Enum):
    SUCCESS = "success"
    SUCCESS_WITH_ASSIST = "success_with_assist"
    FAILURE_RECOVERABLE = "failure_recoverable"
    FAILURE_UNRECOVERABLE = "failure_unrecoverable"
    SKIPPED = "skipped"
    RUNNING = "running"


class TaskMode(str, Enum):
    GPSR = "gpsr"
    PP = "pp"
    FD = "fd"


@dataclass
class TaskStep:
    # task_step_id: int = field(default_factory=lambda: next(_taskstep_id_counter), init=False)
    action_type: ActionType
    object_name: Optional[str] = ""
    object_pickup: Optional["str"] = None
    object_placement: Optional["str"] = None


@dataclass
class Task:
    task_steps: list[TaskStep]
    task_id: int
