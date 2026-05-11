# ---------------------------------------------------------------------------
# values
# ---------------------------------------------------------------------------
from typing import Optional

from demos.pycram_score_aware_planning.Evaluate.types import ActionType, ActionOutcome, TaskMode, TaskStep, Task
from pycram.datastructures.enums import TaskStatus
from semantic_digital_twin.semantic_annotations.semantic_annotations import *
from semantic_digital_twin.spatial_types.spatial_types import Pose

# Base points awarded on clean success
BASE_POINTS: dict[tuple[ActionType, Optional[str]], int]= {
    # PICKUP = 50pts
    # Pickup plate = 100pts
    (ActionType.PICKUP,   "plate"): 50 + 100,

    # Objects of category CUTLERY = 2x+50
    (ActionType.PICKUP,   "knife"): 50 + 50,
    (ActionType.PICKUP,   "fork"): 50 + 50,
    (ActionType.PICKUP,   "spoon"): 50 + 50,

    # Common object pickup = 2x-20pts
    (ActionType.PICKUP,   "bowl"): 50 - 20,
    (ActionType.PICKUP,   "milk"): 50 - 20,
    (ActionType.PICKUP,   ""): 50 + 0,

    # Placing objects = 40pts
    # Place in dishwasher = 3x70pts
    (ActionType.PLACE,    "dishwasher"): 40 + 70,

    # Place next to similar object = 2x20pts
    (ActionType.PLACE,    "PLACEHOLDER"): 40 + 20,

    # Common object place = 2x-20pts
    (ActionType.PLACE,    "bowl"): 40 - 20,
    (ActionType.PLACE,    "milk"): 40 - 20,
    (ActionType.PLACE,    ""): 40 + 0,

    # Open dishwasher = 2x200pts
    (ActionType.OPEN,     "dishwasher"): 200,
    (ActionType.OPEN,     ""): 5,

    (ActionType.CLOSE,    "dishwasher"): 200,
    (ActionType.CLOSE,    ""): 5,

    # navigate to table = 15pts
    (ActionType.NAVIGATE, "table"): 0,
    (ActionType.NAVIGATE, ""): 0,

    (ActionType.HAND_OVER, ""): 15,

    # Pour milk / cereal = 200pts
    (ActionType.POUR,     "cereal"): 200,
    (ActionType.POUR,     "milk"): 200,
    (ActionType.POUR,     ""): 10,

    (ActionType.PUSH,     ""): 0,

    # TODO: add points for detecting objects
    (ActionType.DETECT,   ""): 5,
    (ActionType.CUSTOM,   ""): 0,
}

BASE_TIME_ESTIMATE: dict[tuple[ActionType, Optional[str]], int]= {
    # PICKUP = 50pts
    # Pickup plate = 100pts
    (ActionType.PICKUP,   "plate"): 50,

    # Objects of category CUTLERY = 2x+50
    (ActionType.PICKUP,   "knife"): 100,
    (ActionType.PICKUP,   "fork"): 100,
    (ActionType.PICKUP,   "spoon"): 100,

    # Common object pickup = 2x-20pts
    (ActionType.PICKUP,   "bowl"): 30,
    (ActionType.PICKUP,   "milk"): 15,
    (ActionType.PICKUP,   ""): 30,

    # Placing objects = 40pts
    # Place in dishwasher = 3x70pts
    (ActionType.PLACE,    "dishwasher"): 20,

    # Common object place = 2x-20pts
    (ActionType.PLACE,    "bowl"): 40 - 20,
    (ActionType.PLACE,    "milk"): 40 - 20,
    (ActionType.PLACE,    ""): 40 + 0,

    # Open dishwasher = 2x200pts
    (ActionType.OPEN,     "dishwasher"): 60,
    (ActionType.OPEN,     ""): 50,

    (ActionType.CLOSE,    "dishwasher"): 50,
    (ActionType.CLOSE,    ""): 50,

    # navigate to table = 15pts
    (ActionType.NAVIGATE, "table"): 15,
    (ActionType.NAVIGATE, ""): 5,

    (ActionType.HAND_OVER, ""): 15,

    # Pour milk / cereal = 200pts
    (ActionType.POUR,     "cereal"): 200,
    (ActionType.POUR,     "milk"): 200,
    (ActionType.POUR,     ""): 10,

    (ActionType.PUSH,     ""): 8,
    (ActionType.DETECT,   ""): 5,
    (ActionType.CUSTOM,   ""): 0,
}


BASE_GRASP_PROBABILITY : dict[type, float] = {
    Bowl: 0.3,
    Milk: 0.99,
    Cereal: 0.95,
    Plate: 0.05}

# Penalties applied on top of (or instead of) base points
OUTCOME_MODIFIERS: dict[ActionOutcome|TaskStatus, float] = {
    ActionOutcome.SUCCESS:                1.0,   # full points
    TaskStatus.SUCCEEDED:                 1.0,
    ActionOutcome.SUCCESS_WITH_ASSIST:    0.5,   # half points, assist penalty applied separately
    ActionOutcome.FAILURE_RECOVERABLE:    0.0,   # no points
    ActionOutcome.FAILURE_UNRECOVERABLE:  0.0,   # no points
    TaskStatus.FAILED:                    0.0,
    ActionOutcome.SKIPPED:                0.0,
}

# Flat penalties per outcome type (negative values)
FLAT_PENALTIES: dict[ActionOutcome | TaskStatus, int] = {
    ActionOutcome.SUCCESS:                 0,
    TaskStatus.SUCCEEDED:                 0,
    ActionOutcome.SUCCESS_WITH_ASSIST:   -10,   # human assistance penalty
    ActionOutcome.FAILURE_RECOVERABLE:    -5,
    ActionOutcome.FAILURE_UNRECOVERABLE: -15,
    TaskStatus.FAILED:                   -15,
    ActionOutcome.SKIPPED:                 0,
}

MAX_TIME_ESTIMATE: dict[TaskMode, int] = {
    TaskMode.GPSR: 50,
    TaskMode.PP: 50,
    TaskMode.FD: 50,
}

# Optional: monitor name to ActionType mapping for Giskard integration
MONITOR_ACTION_MAP: dict[str, ActionType] = {
    "pickup_reached":    ActionType.PICKUP,
    "place_reached":     ActionType.PLACE,
    "gripper_opened":    ActionType.OPEN,
    "gripper_closed":    ActionType.CLOSE,
    "nav_goal_reached":  ActionType.NAVIGATE,
    "hand_over_done":    ActionType.HAND_OVER,
}

# All possible tasks for PP
TASKSTEP_PP: list[Task] = [Task(task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="bowl"),TaskStep(ActionType.PLACE, object_name="bowl")]),
                           Task([TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="plate"),TaskStep(ActionType.PLACE, object_name="plate")]),
                           Task([TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="milk"),TaskStep(ActionType.PLACE, object_name="milk")]),
                           Task([TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="cereal"),TaskStep(ActionType.PLACE, object_name="cereal")]),]

# All possible tasks for GPSR
TASKSTEP_GPSR: list[Task] = [Task(task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="bowl"),TaskStep(ActionType.PLACE, object_name="bowl")]),
                           Task([TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="plate"),TaskStep(ActionType.PLACE, object_name="plate")]),
                           Task([TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="milk"),TaskStep(ActionType.PLACE, object_name="milk")]),
                           Task([TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="cereal"),TaskStep(ActionType.PLACE, object_name="cereal")]),]

# All possible tasks for FD
TASKSTEP_FD: list[Task] = [Task(task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="bowl"),TaskStep(ActionType.PLACE, object_name="bowl")]),
                           Task([TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="plate"),TaskStep(ActionType.PLACE, object_name="plate")]),
                           Task([TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="milk"),TaskStep(ActionType.PLACE, object_name="milk")]),
                           Task([TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="cereal"),TaskStep(ActionType.PLACE, object_name="cereal")]),]

TASKS : dict[TaskMode, list[Task]] = {
    TaskMode.PP: TASKSTEP_PP,
    TaskMode.GPSR: TASKSTEP_GPSR,
    TaskMode.FD: TASKSTEP_FD
}