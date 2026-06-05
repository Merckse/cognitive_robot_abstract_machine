from dataclasses import dataclass, field
from enum import Enum
from itertools import count
from typing import Optional

from semantic_digital_twin.spatial_types.spatial_types import Pose

# TODO: Unstatify the types?
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
    PARK = "park"
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
    location : Optional[str] = ""
    object_name: Optional[str] = ""
    object_pickup: Optional["str"] = None # Possible: cooking_table, shelf, lowerTable, table, desk, dining_table, shelf_1, shelf_2
    object_placement: Optional["str"] = None # Possible:a


@dataclass
class Task:
    task_steps: list[TaskStep]
    task_id: int

@dataclass
class TaskEstimation:
    task_id : int
    task_score: int
    task_score_per_seconds : float
    task_probability : float

@dataclass
class SurfaceSpace:
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_surface: float  # top of the surface (z where objects rest)

@dataclass(kw_only=True)
class ExpectedProbabilityModel:
    task_id: int
    expected_action_probability: list[float]
    expected_probability: float
    # taskstep_id : int
    # assisted

@dataclass(kw_only=True)
class ScoreEvent:
    timestamp: float
    action_type: str
    outcome: str
    object_name: Optional[str]
    base_points: int
    penalty: int
    time_spent: float
    net_points: int
    cumulative_score: int
    cumulative_time: float
    note: Optional[str] = None