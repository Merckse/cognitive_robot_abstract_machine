from dataclasses import dataclass, field
from enum import Enum
from itertools import count
from typing import Optional

from semantic_digital_twin.spatial_types.spatial_types import Pose

from pycram.datastructures.enums import TaskStatus


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
    NOT_ASSIGNED = "not_assigned"


class TaskMode(str, Enum):
    GPSR = "gpsr"
    PP = "pp"
    FD = "fd"
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

class ScoreModel:
    current_points: int
    current_penalty: float
    time_left: float
    ScoreEvents : list[ScoreEvent]


# The action step is equivilant to an action, just has been named TaskStep to find seperation, between the two
@dataclass
class TaskStep:
    # task_step_id: int = field(default_factory=lambda: next(_taskstep_id_counter), init=False)
    action_type: ActionType
    action_probability : float= 1
    action_score : float = 0
    action_penatly: int = 0
    action_time : float = 0
    action_assisted: bool = False
    location : Optional[str] = ""
    object_name: Optional[str] = "" # Object that can be named for interaction. E.g: "bowl", with action_type: Pick_Up
    object_pickup: Optional["str"] = None # Possible: cooking_table, shelf, lowerTable, table, desk, dining_table, shelf_1, shelf_2
    object_placement: Optional["str"] = None # Possible:a

# The Task is equivilent to a whole plan, containing of exactly one task.
@dataclass
class Task:
    id: int
    task_steps: list[TaskStep]
    status : TaskStatus = ActionOutcome.NOT_ASSIGNED
    score: int = 0
    penatly: int = 0
    score_penalized: int = 0
    duration: int = 0
    score_per_seconds : float = 0
    probability : float = 1

# TODO: replace the data-type, by asserting the variables to the Task-datatype
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

# TODO: TO BE REPLACED, yb the TASKSTEPS, containing the Probability and misk
@dataclass(kw_only=True)
class ExpectedProbabilityModel:
    task_id: int
    expected_probability: float
    # taskstep_id : int
    # assisted

