# ---------------------------------------------------------------------------
# values
# ---------------------------------------------------------------------------
from demos.pycram_score_aware_planning.common.types import ActionType, ActionOutcome, TaskMode, TaskStep, Task
from pycram.datastructures.enums import TaskStatus

# Base points awarded on clean success
# Key = (action, object, location). Object-centric actions leave location "";
# location/furniture-centric actions (NAVIGATE, OPEN/CLOSE, PLACE target) leave object "".
BASE_POINTS: dict[tuple[ActionType, str, str], int]= {
    (ActionType.PARK, "", ""): 0,

    # ------------------ PICKUP = 50pts
    # Pickup plate = 100pts
    (ActionType.PICKUP, "plate", ""): 50 + 100,

    # Objects of category CUTLERY = 2x+50
    (ActionType.PICKUP,   "knife", ""): 50 + 50,
    (ActionType.PICKUP,   "fork", ""): 50 + 50,
    (ActionType.PICKUP,   "spoon", ""): 50 + 50,

    # Common object pickup = 2x-20pts
    (ActionType.PICKUP,   "bowl", ""): 50 - 20,
    (ActionType.PICKUP,   "milk", ""): 50 - 20,
    (ActionType.PICKUP,   "cereal", ""): 15,
    (ActionType.PICKUP,   "", ""): 50 + 0,

    # ------------------ Placing objects = 40pts
    # Place in dishwasher = 3x70pts
    (ActionType.PLACE,    "", "dishwasher"): 40 + 70,

    # Place next to similar object = 2x20pts
    (ActionType.PLACE,    "PLACEHOLDER", ""): 40 + 20,

    # Objects of category CUTLERY = 2x+50
    (ActionType.PLACE, "knife", ""): 50 + 50,
    (ActionType.PLACE, "fork", ""): 50 + 50,
    (ActionType.PLACE, "spoon", ""): 50 + 50,

    # Common object place = 2x-20pts
    (ActionType.PLACE,    "bowl", ""): 40 - 20,
    (ActionType.PLACE,    "milk", ""): 40 - 20,
    (ActionType.PLACE,   "cereal", ""): 15,
    # random value
    (ActionType.PLACE, "plate", ""): 40,
    (ActionType.PLACE, "", ""): 40 + 0,

    #  ------------------ Open/Close dishwasher = 2x200pts
    (ActionType.OPEN,     "", "dishwasher"): 200,
    (ActionType.OPEN,     "", ""): 0,

    (ActionType.CLOSE,    "", "dishwasher"): 200,
    (ActionType.CLOSE,    "", ""): 0,

    #  ------------------ navigate to table = 15pts
    (ActionType.NAVIGATE, "", "table"): 0,
    (ActionType.NAVIGATE, "", ""): 0,

    #  ------------------ Handover
    (ActionType.HAND_OVER, "", ""): 15,

    #  ------------------ Detect
    # TODO: add points for detecting objects
    (ActionType.DETECT,   "", ""): 5,
    (ActionType.CUSTOM,   "", ""): 0,
}

# TODO: ADD correct penalties
# Roughly scaled to the reward & difficulty of the matching BASE_POINTS entry:
# the more valuable / harder the action, the more a failure costs.
BASE_PENALTIES: dict[tuple[ActionType, str, str], int] = {
    (ActionType.PARK, "", ""): 0,

    # ------------------ PICKUP = -20pts
    # Pickup plate = high value, costly failure
    (ActionType.PICKUP, "plate", ""): -20 - 30,

    # Objects of category CUTLERY = hard to grasp, costly failure
    (ActionType.PICKUP,   "knife", ""): -20 - 10,
    (ActionType.PICKUP,   "fork", ""): -20 - 10,
    (ActionType.PICKUP,   "spoon", ""): -20 - 10,

    # Common object pickup = cheap failure
    (ActionType.PICKUP,   "bowl", ""): -20 + 10,
    (ActionType.PICKUP,   "milk", ""): -20 + 10,
    (ActionType.PICKUP,   "cereal", ""): -10,
    (ActionType.PICKUP,   "", ""): -20 + 0,

    # ------------------ Placing objects = -15pts
    # Place in dishwasher = high precision, costly failure
    (ActionType.PLACE,    "", "dishwasher"): -15 - 25,

    # Place next to similar object
    (ActionType.PLACE,    "PLACEHOLDER", ""): -15 - 5,

    # Objects of category CUTLERY = costly failure
    (ActionType.PLACE, "knife", ""): -15 - 15,
    (ActionType.PLACE, "fork", ""): -15 - 15,
    (ActionType.PLACE, "spoon", ""): -15 - 15,

    # Common object place = cheap failure
    (ActionType.PLACE,    "bowl", ""): -15 + 5,
    (ActionType.PLACE,    "milk", ""): -15 + 5,
    (ActionType.PLACE,   "cereal", ""): -10,
    # random value
    (ActionType.PLACE, "plate", ""): -15,
    (ActionType.PLACE, "", ""): -15 + 0,

    #  ------------------ Open/Close dishwasher = -50pts
    (ActionType.OPEN,     "", "dishwasher"): -50,
    (ActionType.OPEN,     "", ""): 0,

    (ActionType.CLOSE,    "", "dishwasher"): -50,
    (ActionType.CLOSE,    "", ""): 0,

    #  ------------------ navigate to table
    (ActionType.NAVIGATE, "", "table"): 0,
    (ActionType.NAVIGATE, "", ""): 0,

    #  ------------------ Handover
    (ActionType.HAND_OVER, "", ""): -10,

    #  ------------------ Detect
    # TODO: add penalties for failed detections
    (ActionType.DETECT,   "", ""): -5,
    (ActionType.CUSTOM,   "", ""): 0,
}

BASE_TIME_ESTIMATE: dict[tuple[ActionType, str, str], int]= {
    (ActionType.PARK, "", ""): 0,

    # ------------------ PICKUP ~30s
    # Pickup plate is slower (larger, heavier) = ~50s
    (ActionType.PICKUP, "plate", ""): 50,

    # Objects of category CUTLERY are harder to locate/grasp = ~100s
    (ActionType.PICKUP,   "knife", ""): 100,
    (ActionType.PICKUP,   "fork", ""): 100,
    (ActionType.PICKUP,   "spoon", ""): 100,

    # Common object pickup
    (ActionType.PICKUP,   "bowl", ""): 30,
    (ActionType.PICKUP,   "milk", ""): 15,
    (ActionType.PICKUP,   "cereal", ""): 15,
    (ActionType.PICKUP,   "", ""): 30,

    # ------------------ Placing objects ~40s
    # Place in dishwasher requires precision (close proximity) = ~20s
    (ActionType.PLACE,    "", "dishwasher"): 20,

    # Common object placement
    (ActionType.PLACE,    "bowl", ""): 40 - 20,
    (ActionType.PLACE,    "milk", ""): 40 - 20,
    (ActionType.PLACE,   "cereal", ""): 15,
    (ActionType.PLACE, "plate", ""): 40,
    (ActionType.PLACE,    "", ""): 40 + 0,

    #  ------------------ Open/Close dishwasher ~50-60s
    (ActionType.OPEN,     "", "dishwasher"): 60,
    (ActionType.OPEN,     "", ""): 50,

    (ActionType.CLOSE,    "", "dishwasher"): 50,
    (ActionType.CLOSE,    "", ""): 50,

    #  ------------------ Navigate ~20-30s
    (ActionType.NAVIGATE, "", "table"): 20,
    (ActionType.NAVIGATE, "", ""): 30,

    #  ------------------ Handover
    (ActionType.HAND_OVER, "", ""): 15,

    #  ------------------ Pour ~200s
    (ActionType.POUR,     "cereal", ""): 200,
    (ActionType.POUR,     "milk", ""): 200,
    (ActionType.POUR,     "", ""): 10,

    (ActionType.PUSH,     "", ""): 8,
    (ActionType.DETECT,   "", ""): 5,
    (ActionType.CUSTOM,   "", ""): 0,
}


BASE_PROBABILITY : dict[tuple[ActionType, str, str], float] = {
    (ActionType.PARK, "", "") : 1,

    # ------------------ PICKUP base = 0.5
    # Pickup plate is challenging (large, heavier object) = 0.5
    (ActionType.PICKUP, "plate", ""): 0.5,

    # Objects of category CUTLERY are very hard to grasp = 0.01
    (ActionType.PICKUP, "knife", ""): 0.01,
    (ActionType.PICKUP, "fork", ""): 0.01,
    (ActionType.PICKUP, "spoon", ""): 0.01,

    # Common object pickup
    # TODO: geometry analysis or not
    (ActionType.PICKUP, "bowl", ""): 0.75,
    (ActionType.PICKUP, "milk", ""): 0.95,
    (ActionType.PICKUP, "cereal", ""): 0.95,
    (ActionType.PICKUP, "", ""): 0.5,

    # ------------------ Placing objects base = 0.6
    # Place in dishwasher requires high precision = 0.4
    (ActionType.PLACE, "", "dishwasher"): 0.4,

    # Common object placement
    (ActionType.PLACE, "bowl", ""): 0.8,
    (ActionType.PLACE, "milk", ""): 0.99,
    (ActionType.PLACE, "cereal", ""): 0.99,
    (ActionType.PLACE, "plate", ""): 0.01,
    # TODO: geometry analysis or not
    (ActionType.PLACE, "", ""): 0.6,

    #  ------------------ Open/Close dishwasher = 0 (not yet implemented)
    (ActionType.OPEN, "", "dishwasher"): 0,
    (ActionType.OPEN, "", ""): 0,

    (ActionType.CLOSE, "", "dishwasher"): 0,
    (ActionType.CLOSE, "", ""): 0,

    #  ------------------ Navigate = ~1.0
    (ActionType.NAVIGATE, "", "table"): 1,
    (ActionType.NAVIGATE, "", ""): 0.98,

    #  ------------------ Handover
    (ActionType.HAND_OVER, "", ""): 0.95,

    #  ------------------ Pour = 0 (not yet implemented)
    (ActionType.POUR, "cereal", ""): 0,
    (ActionType.POUR, "milk", ""): 0,
    (ActionType.POUR, "", ""): 0,

    (ActionType.PUSH, "", ""): 0,
    (ActionType.DETECT, "", ""): 0.8,
    (ActionType.CUSTOM, "", ""): 0,
}

# Penalties applied on top of (or instead of) base points
OUTCOME_MODIFIERS: dict[ActionOutcome|TaskStatus, float] = {
    TaskStatus.SUCCEEDED:                 1.0,
    TaskStatus.FAILED:                    0.0,
    ActionOutcome.SUCCESS: 1.0,  # full points
    ActionOutcome.SUCCESS_WITH_ASSIST: 0.5,  # half points, assist penalty applied separately
    ActionOutcome.FAILURE_RECOVERABLE: 0.0,  # no points
    ActionOutcome.FAILURE_UNRECOVERABLE: 0.0,  # no points
    ActionOutcome.SKIPPED: 0.0,
    ActionOutcome.NOT_ASSIGNED: 0,
}



# TODO: replace with proper penalty scores
# Flat penalties per outcome type (negative values)
FLAT_PENALTIES: dict[ActionOutcome | TaskStatus, int] = {
    TaskStatus.SUCCEEDED:                 0,
    TaskStatus.FAILED:                   -15,
    ActionOutcome.SUCCESS:                 0,
    ActionOutcome.SUCCESS_WITH_ASSIST: -10,  # human assistance penalty
    ActionOutcome.FAILURE_RECOVERABLE: -5,
    ActionOutcome.FAILURE_UNRECOVERABLE: -15,
    ActionOutcome.SKIPPED: 0,
    ActionOutcome.NOT_ASSIGNED: 0,
}

MAX_TIME_ESTIMATE: dict[TaskMode, int] = {
    TaskMode.GPSR: 50,
    TaskMode.PP: 50,
    TaskMode.FD: 50,
}

# All possible tasks for PP
TASKSTEP_PP: list[Task] = [Task(id= 0, task_steps=[TaskStep(ActionType.NAVIGATE, location="cooking_table"), TaskStep(ActionType.PICKUP, object_name="bowl"), TaskStep(ActionType.PARK), TaskStep(ActionType.NAVIGATE, location="counterTop"), TaskStep(ActionType.PLACE, object_placement="counterTop"), TaskStep(ActionType.PARK)]),
                           Task(id= 1, task_steps=[TaskStep(ActionType.NAVIGATE, location="desk"), TaskStep(ActionType.PICKUP, object_name="plate"), TaskStep(ActionType.PARK), TaskStep(ActionType.NAVIGATE, location="table"), TaskStep(ActionType.PLACE, object_placement="table"), TaskStep(ActionType.PARK)]),
                           Task(id= 2, task_steps=[TaskStep(ActionType.NAVIGATE, location="counterTop"), TaskStep(ActionType.PICKUP, object_name="milk"), TaskStep(ActionType.PARK), TaskStep(ActionType.NAVIGATE, location="shelf_1"), TaskStep(ActionType.PLACE, object_placement="shelf_1"), TaskStep(ActionType.PARK)]),
                           Task(id= 3, task_steps=[TaskStep(ActionType.NAVIGATE, location="shelf_1"), TaskStep(ActionType.PICKUP, object_name="cereal"), TaskStep(ActionType.PARK), TaskStep(ActionType.NAVIGATE, location="shelf_2"), TaskStep(ActionType.PLACE, object_placement="shelf_2"), TaskStep(ActionType.PARK)]), ]

# All possible tasks for GPSR
TASKSTEP_GPSR: list[Task] = [Task(id= 0, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="bowl"), TaskStep(ActionType.PLACE, object_name="bowl")]),
                             Task(id= 1, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="plate"), TaskStep(ActionType.PLACE, object_name="plate")]),
                             Task(id= 2, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="milk"), TaskStep(ActionType.PLACE, object_name="milk")]),
                             Task(id= 3, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="cereal"), TaskStep(ActionType.PLACE, object_name="cereal")]), ]

# All possible tasks for FD
TASKSTEP_FD: list[Task] = [Task(id= 0, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="bowl"), TaskStep(ActionType.PLACE, object_name="bowl")]),
                           Task(id= 1, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="plate"), TaskStep(ActionType.PLACE, object_name="plate")]),
                           Task(id= 2, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="milk"), TaskStep(ActionType.PLACE, object_name="milk")]),
                           Task(id= 3, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_name="cereal"), TaskStep(ActionType.PLACE, object_name="cereal")]), ]

TASKS : dict[TaskMode, list[Task]] = {
    TaskMode.PP: TASKSTEP_PP,
    TaskMode.GPSR: TASKSTEP_GPSR,
    TaskMode.FD: TASKSTEP_FD
}

