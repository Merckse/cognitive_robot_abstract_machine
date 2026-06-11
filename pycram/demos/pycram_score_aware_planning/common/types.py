from dataclasses import dataclass, field
from enum import Enum
from itertools import count
from typing import Optional

from pycram.robot_plans.actions.base import ActionDescription
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
    points: int
    penalty: int
    time_spent: float
    cumulative_score: int
    cumulative_time: float

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
    uncertain: bool = False # Meaning, if not all informations to a task are known, then there has to be something done
    location : str = ""
    object_name: str = "" # Object that can be named for interaction. E.g: "bowl", with action_type: Pick_Up
    object_pickup: str = None # Possible: cooking_table, shelf, lowerTable, table, desk, dining_table, shelf_1, shelf_2
    object_placement: str = None # Possible:a

# The Task is equivilent to a whole plan, containing of exactly one task.
@dataclass
class Task:
    id: int
    task_steps: list[TaskStep]
    action_list: list[ActionDescription] = field(default_factory=list)
    uncertain: bool = False # Meaning, if not all informations to a task are known, then there has to be something done
    status : TaskStatus = ActionOutcome.NOT_ASSIGNED
    score: int = 0
    penatly: int = 0
    score_penalized: int = 0
    duration: int = 0
    score_per_seconds : float = 0
    probability : float = 1

@dataclass
class SurfaceSpace:
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_surface: float  # top of the surface (z where objects rest)
