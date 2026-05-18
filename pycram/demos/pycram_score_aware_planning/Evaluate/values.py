# ---------------------------------------------------------------------------
# values
# ---------------------------------------------------------------------------
import math
from typing import Optional

from demos.pycram_score_aware_planning.Evaluate.types import ActionType, ActionOutcome, TaskMode, TaskStep, Task
from pycram.datastructures.enums import TaskStatus
from semantic_digital_twin.semantic_annotations.semantic_annotations import *
from semantic_digital_twin.spatial_types.spatial_types import Pose

# Base points awarded on clean success
BASE_POINTS: dict[tuple[ActionType, Optional[str]], int]= {
    (ActionType.PARK, ""): 0,

    # PICKUP = 50pts
    # Pickup plate = 100pts
    (ActionType.PICKUP, "plate"): 50 + 100,

    # Objects of category CUTLERY = 2x+50
    (ActionType.PICKUP,   "knife"): 50 + 50,
    (ActionType.PICKUP,   "fork"): 50 + 50,
    (ActionType.PICKUP,   "spoon"): 50 + 50,

    # Common object pickup = 2x-20pts
    (ActionType.PICKUP,   "bowl"): 50 - 20,
    (ActionType.PICKUP,   "milk"): 50 - 20,
    (ActionType.PICKUP,   "cereal"): 15,
    (ActionType.PICKUP,   ""): 50 + 0,

    # Placing objects = 40pts
    # Place in dishwasher = 3x70pts
    (ActionType.PLACE,    "dishwasher"): 40 + 70,

    # Place next to similar object = 2x20pts
    (ActionType.PLACE,    "PLACEHOLDER"): 40 + 20,

    # Common object place = 2x-20pts
    (ActionType.PLACE,    "bowl"): 40 - 20,
    (ActionType.PLACE,    "milk"): 40 - 20,
    (ActionType.PLACE,   "cereal"): 15,
    # random value
    (ActionType.PICKUP, "plate"): 40,
    (ActionType.PLACE, ""): 40 + 0,
    (ActionType.PLACE, ""): 40 + 0,

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
    (ActionType.PARK, ""): 0,

    # PICKUP = 50pts
    # Pickup plate = 100pts
    (ActionType.PICKUP, "plate"): 50,

    # Objects of category CUTLERY = 2x+50
    (ActionType.PICKUP,   "knife"): 100,
    (ActionType.PICKUP,   "fork"): 100,
    (ActionType.PICKUP,   "spoon"): 100,

    # Common object pickup = 2x-20pts
    (ActionType.PICKUP,   "bowl"): 30,
    (ActionType.PICKUP,   "milk"): 15,
    (ActionType.PICKUP,   "cereal"): 15,
    (ActionType.PICKUP,   ""): 30,

    # Placing objects = 40pts
    # Place in dishwasher = 3x70pts
    (ActionType.PLACE,    "dishwasher"): 20,

    # Common object place = 2x-20pts
    (ActionType.PLACE,    "bowl"): 40 - 20,
    (ActionType.PLACE,    "milk"): 40 - 20,
    (ActionType.PLACE,   "cereal"): 15,
    # random value
    (ActionType.PLACE, "plate"): 40,

    (ActionType.PLACE,    ""): 40 + 0,

    # Open dishwasher = 2x200pts
    (ActionType.OPEN,     "dishwasher"): 60,
    (ActionType.OPEN,     ""): 50,

    (ActionType.CLOSE,    "dishwasher"): 50,
    (ActionType.CLOSE,    ""): 50,

    # navigate to table = 15pts
    (ActionType.NAVIGATE, "table"): 20,
    (ActionType.NAVIGATE, ""): 30,

    (ActionType.HAND_OVER, ""): 15,

    # Pour milk / cereal = 200pts
    (ActionType.POUR,     "cereal"): 200,
    (ActionType.POUR,     "milk"): 200,
    (ActionType.POUR,     ""): 10,

    (ActionType.PUSH,     ""): 8,
    (ActionType.DETECT,   ""): 5,
    (ActionType.CUSTOM,   ""): 0,
}



BASE_PROBABILITY : dict[tuple[ActionType, str], float] = {
    (ActionType.PARK, "") : 1,
    # PICKUP = 50pts
    # Pickup plate = 100pts
    (ActionType.PICKUP, "plate"): 0.5,

    # Objects of category CUTLERY = 2x+50
    (ActionType.PICKUP, "knife"): 0.01,
    (ActionType.PICKUP, "fork"): 0.01,
    (ActionType.PICKUP, "spoon"): 0.01,

    # Common object pickup = 2x-20pts
    # TODO: geometry analysis or not
    (ActionType.PICKUP, "bowl"): 0.75,
    (ActionType.PICKUP, "milk"): 0.95,
    (ActionType.PICKUP, "cereal"): 0.95,
    (ActionType.PICKUP, ""): 0.5,

    # Placing objects = 40pts
    # Place in dishwasher = 3x70pts
    (ActionType.PLACE, "dishwasher"): 0.4,

    # Common object place = 2x-20pts
    (ActionType.PLACE, "bowl"): 0.8,
    (ActionType.PLACE, "milk"): 0.99,
    (ActionType.PLACE, "cereal"): 0.99,
    (ActionType.PLACE, "plate"): 0.01,
    # TODO: geometry analysis or not
    (ActionType.PLACE, ""): 0.6,

    # Open dishwasher = 2x200pts
    (ActionType.OPEN, "dishwasher"): 0,
    (ActionType.OPEN, ""): 0,

    (ActionType.CLOSE, "dishwasher"): 0,
    (ActionType.CLOSE, ""): 0,

    # navigate to table = 15pts
    (ActionType.NAVIGATE, "table"): 1,
    (ActionType.NAVIGATE, ""): 0.98,

    (ActionType.HAND_OVER, ""): 0.95,

    # Pour milk / cereal = 200pts
    (ActionType.POUR, "cereal"): 0,
    (ActionType.POUR, "milk"): 0,
    (ActionType.POUR, ""): 0,

    (ActionType.PUSH, ""): 0,
    (ActionType.DETECT, ""): 0.8,
    (ActionType.CUSTOM, ""): 0,
}

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
TASKSTEP_PP: list[Task] = [Task(task_id= 0, task_steps=[TaskStep(ActionType.NAVIGATE, location="cooking_table"), TaskStep(ActionType.PICKUP, object_name="bowl"), TaskStep(ActionType.PARK), TaskStep(ActionType.PARK), TaskStep(ActionType.NAVIGATE, location="counterTop"), TaskStep(ActionType.PLACE, object_placement="counterTop"), TaskStep(ActionType.PARK)]),
                           Task(task_id= 1, task_steps=[TaskStep(ActionType.NAVIGATE, location="desk"), TaskStep(ActionType.PICKUP, object_name="plate"),TaskStep(ActionType.PARK), TaskStep(ActionType.NAVIGATE, location="table"), TaskStep(ActionType.PLACE, object_placement="table"), TaskStep(ActionType.PARK)]),
                           Task(task_id= 2, task_steps=[TaskStep(ActionType.NAVIGATE, location="counterTop"), TaskStep(ActionType.PICKUP, object_name="milk"),TaskStep(ActionType.PARK), TaskStep(ActionType.NAVIGATE, location="shelf_1"), TaskStep(ActionType.PLACE, object_placement="shelf_1"), TaskStep(ActionType.PARK)]),
                           Task(task_id= 3, task_steps=[TaskStep(ActionType.NAVIGATE, location="shelf_1"), TaskStep(ActionType.PICKUP, object_name="cereal"),TaskStep(ActionType.PARK), TaskStep(ActionType.NAVIGATE, location="shelf_2"), TaskStep(ActionType.PLACE, object_placement="shelf_2"), TaskStep(ActionType.PARK)]),]

# All possible tasks for GPSR
TASKSTEP_GPSR: list[Task] = [Task(task_id= 0, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="bowl"),TaskStep(ActionType.PLACE, object_name="bowl")]),
                           Task(task_id= 1, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="plate"),TaskStep(ActionType.PLACE, object_name="plate")]),
                           Task(task_id= 2, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="milk"),TaskStep(ActionType.PLACE, object_name="milk")]),
                           Task(task_id= 3, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="cereal"),TaskStep(ActionType.PLACE, object_name="cereal")]),]

# All possible tasks for FD
TASKSTEP_FD: list[Task] = [Task(task_id= 0, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="bowl"),TaskStep(ActionType.PLACE, object_name="bowl")]),
                           Task(task_id= 1, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="plate"),TaskStep(ActionType.PLACE, object_name="plate")]),
                           Task(task_id= 2, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="milk"),TaskStep(ActionType.PLACE, object_name="milk")]),
                           Task(task_id= 3, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="cereal"),TaskStep(ActionType.PLACE, object_name="cereal")]),]

TASKS : dict[TaskMode, list[Task]] = {
    TaskMode.PP: TASKSTEP_PP,
    TaskMode.GPSR: TASKSTEP_GPSR,
    TaskMode.FD: TASKSTEP_FD
}

def _quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))

# name → (x, y, quaternion(x,y,z,w))
NAVIGATION_POSES: dict[str, tuple[float, float, tuple[float, float, float, float]]] = {
    "cooking_table": (1.3, 4.6, _quat( math.pi / 2)),   # south of table, facing north
    "dining_table":  (2.6, 4.1, _quat( math.pi / 2)),   # south of table, facing north
    "table":         (3.5, 1.8, _quat(-math.pi / 2)),   # north of table, facing south
    "lowerTable":    (3.0, 2.2, _quat( 0.0)),           # west of table,  facing east
    "desk":          (1.3, 1.2, _quat( math.pi)),        # east of desk,   facing west
    "shelf_1":       (3.3,  4.7,  _quat( 0.0)),          # west of cupboard, facing east
    "shelf_2":       (3.3,  4.7,  _quat( 0.0)),          # same approach as shelf_1
    "counterTop":    (1.859, -0.852, _quat(-math.pi / 2)), # north of counter, facing south
}
