# ---------------------------------------------------------------------------
# values
# ---------------------------------------------------------------------------
from dataclasses import dataclass

from demos.pycram_score_aware_planning.common.types import ActionType, ActionOutcome, TaskMode, TaskStep, Task
from pycram.datastructures.enums import TaskStatus

# ---------------------------------------------------------------------------
# Action scoring table
# ---------------------------------------------------------------------------
# Single source of truth for every (action, object, location) case. Each entry
# bundles all four scoring metrics (points, penalty, time, probability) so they
# can never drift out of sync the way four parallel dicts did.
#
# Key = (action, object, location):
#   - Object-centric actions (PICKUP, PLACE object) put the object in slot 1 and
#     use ANY for location.
#   - Location/furniture-centric actions (NAVIGATE, OPEN/CLOSE, PLACE target) put
#     the location in slot 2 and use ANY for object.
#   - A catch-all uses ANY in both slots.
#
# Resolution goes through profile(): it tries the most specific key first, then
# widens each slot to ANY. A specific object beats a specific location.

ANY = "*"   # wildcard: matches any object or location in that slot


@dataclass(frozen=True)
class ActionEvaluation:
    """All scoring attributes of a single (action, object, location) case."""
    points: int          # reward on clean success
    penalty: int         # cost on failure (negative)
    time: int            # estimated duration, seconds
    probability: float   # base success probability


ACTIONS: dict[tuple[ActionType, str, str], ActionEvaluation] = {
    (ActionType.PARK, ANY, ANY):
        ActionEvaluation(points=0, penalty=0, time=0, probability=1.0),

    # ------------------ PICKUP (base: 50pts / -20 / 30s / 0.5)
    (ActionType.PICKUP, "plate", ANY):
        ActionEvaluation(points=50 + 100, penalty=-20 - 30, time=50, probability=0.01),
    # CUTLERY — hard to grasp/locate
    (ActionType.PICKUP, "knife", ANY):
        ActionEvaluation(points=50 + 50, penalty=-20 - 10, time=100, probability=0.01),
    (ActionType.PICKUP, "fork", ANY):
        ActionEvaluation(points=50 + 50, penalty=-20 - 10, time=100, probability=0.01),
    (ActionType.PICKUP, "spoon", ANY):
        ActionEvaluation(points=50 + 50, penalty=-20 - 10, time=100, probability=0.01),
    # Common objects
    (ActionType.PICKUP, "bowl", ANY):
        ActionEvaluation(points=50 - 20, penalty=-20 + 10, time=30, probability=0.75),
    (ActionType.PICKUP, "milk", ANY):
        ActionEvaluation(points=50 - 20, penalty=-20 + 10, time=15, probability=0.95),
    (ActionType.PICKUP, "cereal", ANY):
        ActionEvaluation(points=15, penalty=-10, time=15, probability=0.95),
    # any other object
    (ActionType.PICKUP, ANY, ANY):
        ActionEvaluation(points=50, penalty=-20, time=30, probability=0.5),

    # ------------------ PLACE (base: 40pts / -15 / 40s / 0.6)
    (ActionType.PLACE, ANY, "dishwasher"):
        ActionEvaluation(points=40 + 70, penalty=-15 - 25, time=20, probability=0.4),
    # place next to a similar object
    (ActionType.PLACE, "PLACEHOLDER", ANY):
        ActionEvaluation(points=40 + 20, penalty=-15 - 5, time=40, probability=0.6),
    # CUTLERY (base is 40 — fixed copy-paste from PICKUP that read 50)
    (ActionType.PLACE, "knife", ANY):
        ActionEvaluation(points=40 + 50, penalty=-15 - 15, time=40, probability=0.6),
    (ActionType.PLACE, "fork", ANY):
        ActionEvaluation(points=40 + 50, penalty=-15 - 15, time=40, probability=0.6),
    (ActionType.PLACE, "spoon", ANY):
        ActionEvaluation(points=40 + 50, penalty=-15 - 15, time=40, probability=0.6),
    # Common objects
    (ActionType.PLACE, "bowl", ANY):
        ActionEvaluation(points=40 - 20, penalty=-15 + 5, time=40 - 20, probability=0.8),
    (ActionType.PLACE, "milk", ANY):
        ActionEvaluation(points=40 - 20, penalty=-15 + 5, time=40 - 20, probability=0.99),
    (ActionType.PLACE, "cereal", ANY):
        ActionEvaluation(points=15, penalty=-10, time=15, probability=0.99),
    (ActionType.PLACE, "plate", ANY):
        ActionEvaluation(points=40, penalty=-15, time=40, probability=0.01),
    # any other object
    (ActionType.PLACE, ANY, ANY):
        ActionEvaluation(points=40, penalty=-15, time=40, probability=0.6),

    # ------------------ OPEN / CLOSE (probability 0 — not yet implemented)
    (ActionType.OPEN, ANY, "dishwasher"):
        ActionEvaluation(points=200, penalty=-50, time=60, probability=0.0),
    (ActionType.OPEN, ANY, ANY):
        ActionEvaluation(points=0, penalty=0, time=50, probability=0.0),
    (ActionType.CLOSE, ANY, "dishwasher"):
        ActionEvaluation(points=200, penalty=-50, time=50, probability=0.0),
    (ActionType.CLOSE, ANY, ANY):
        ActionEvaluation(points=0, penalty=0, time=50, probability=0.0),

    # ------------------ NAVIGATE (points/penalty always 0; ~20s)
    # NOTE: the old duplicate (NAVIGATE,"","") keys (20 then 30) collapse into the
    # single catch-all below.
    (ActionType.NAVIGATE, ANY, "table"):
        ActionEvaluation(points=0, penalty=0, time=20, probability=1.0),
    (ActionType.NAVIGATE, ANY, "cooking_table"):
        ActionEvaluation(points=0, penalty=0, time=20, probability=0.98),
    (ActionType.NAVIGATE, ANY, "desk"):
        ActionEvaluation(points=0, penalty=0, time=20, probability=0.98),
    (ActionType.NAVIGATE, ANY, "counterTop"):
        ActionEvaluation(points=0, penalty=0, time=20, probability=0.98),
    (ActionType.NAVIGATE, ANY, "shelf_1"):
        ActionEvaluation(points=0, penalty=0, time=20, probability=0.98),
    (ActionType.NAVIGATE, ANY, ANY):
        ActionEvaluation(points=0, penalty=0, time=20, probability=0.98),

    # ------------------ Handover
    (ActionType.HAND_OVER, ANY, ANY):
        ActionEvaluation(points=15, penalty=-10, time=15, probability=0.95),

    # ------------------ Pour (~200s; probability 0 — not yet implemented)
    (ActionType.POUR, "cereal", ANY):
        ActionEvaluation(points=0, penalty=0, time=200, probability=0.0),
    (ActionType.POUR, "milk", ANY):
        ActionEvaluation(points=0, penalty=0, time=200, probability=0.0),
    (ActionType.POUR, ANY, ANY):
        ActionEvaluation(points=0, penalty=0, time=10, probability=0.0),

    # ------------------ Push / Detect / Custom
    (ActionType.PUSH, ANY, ANY):
        ActionEvaluation(points=0, penalty=0, time=8, probability=0.0),
    (ActionType.DETECT, ANY, ANY):
        ActionEvaluation(points=5, penalty=-5, time=5, probability=0.8),
    (ActionType.CUSTOM, ANY, ANY):
        ActionEvaluation(points=0, penalty=0, time=0, probability=0.0),
}

# Fallback
EVALUATION = ActionEvaluation(points=0, penalty=0, time=0, probability=1.0)


def evaluation(action_type: ActionType, object_name: str = "", location: str = "") -> ActionEvaluation:
    for o in (object_name, ANY):
        for l in (location, ANY):
            hit = ACTIONS.get((action_type, o, l))
            if hit is not None:
                return hit
    return EVALUATION

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

MAX_TIME_ESTIMATE: dict[TaskMode, int] = {
    TaskMode.GPSR: 50,
    TaskMode.PP: 50,
    TaskMode.FD: 50,
}

# All possible tasks for PP
TASKSTEP_PP: list[Task] = [Task(id= 0, task_steps=[TaskStep(ActionType.NAVIGATE, location="cooking_table"),
                                                   TaskStep(ActionType.PICKUP, object_name="bowl"),
                                                   TaskStep(ActionType.PARK),
                                                   TaskStep(ActionType.NAVIGATE, location="counterTop"),
                                                   TaskStep(ActionType.PLACE, object_name="bowl", location="counterTop"),
                                                   TaskStep(ActionType.PARK)]),

                           Task(id= 1, task_steps=[TaskStep(ActionType.NAVIGATE, location="desk"),
                                                   TaskStep(ActionType.PICKUP, object_name="plate"),
                                                   TaskStep(ActionType.PARK),
                                                   TaskStep(ActionType.NAVIGATE, location="table"),
                                                   TaskStep(ActionType.PLACE, object_name="plate", location="table"),
                                                   TaskStep(ActionType.PARK)]),

                           Task(id= 2, task_steps=[TaskStep(ActionType.NAVIGATE, location="counterTop"),
                                                   TaskStep(ActionType.PICKUP, object_name="milk"),
                                                   TaskStep(ActionType.PARK),
                                                   TaskStep(ActionType.NAVIGATE, location="shelf_1"),
                                                   TaskStep(ActionType.PLACE, object_name="milk", location="shelf_1"),
                                                   TaskStep(ActionType.PARK)]),

                           Task(id= 3, task_steps=[TaskStep(ActionType.NAVIGATE, location="shelf_1"),
                                                   TaskStep(ActionType.PICKUP, object_name="cereal"),
                                                   TaskStep(ActionType.PARK),
                                                   TaskStep(ActionType.NAVIGATE, location="shelf_2"),
                                                   TaskStep(ActionType.PLACE, object_name="cereal", location="shelf_2"),
                                                   TaskStep(ActionType.PARK)]), ]

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

