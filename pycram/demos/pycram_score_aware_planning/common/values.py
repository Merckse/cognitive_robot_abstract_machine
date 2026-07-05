# ---------------------------------------------------------------------------
# values
# ---------------------------------------------------------------------------
from dataclasses import dataclass

from demos.pycram_score_aware_planning.common.cram_types import ActionType, Status, ChallengeMode, TaskStep, Task
from giskardpy.middleware.ros2.exceptions import ExecutionAbortedException
from giskardpy.motion_statechart.exceptions import CollisionViolatedError
from giskardpy.motion_statechart.goals.pick_up import ObjectDoesntFitException, ObjectNotReachableException
from pycram.datastructures.enums import TaskStatus, PlanTransformationOperator
from pycram.exceptions import MotionDidNotFinish
from pycram.plans.failures import ObjectNotGrasped
from semantic_digital_twin.exceptions import WorldEntityNotFoundError
from semantic_digital_twin.semantic_annotations.semantic_annotations import Plate, Knife, Fork, Spoon, Bowl, Milk, Cereal
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation

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
    score: int          # reward on clean success
    penalty: int         # cost on failure (negative)
    time: int            # estimated duration, seconds
    probability: float   # base success probability


ACTIONS: dict[tuple[ActionType, SemanticAnnotation, str], ActionEvaluation] = {
    (ActionType.PARK, ANY, ANY):
        ActionEvaluation(score=0, penalty=0, time=0, probability=1.0),

    # ------------------ PICKUP (base: 50pts / -20 / 30s / 0.5)
    (ActionType.PICKUP, Plate, ANY):
        ActionEvaluation(score=50 + 100, penalty=-10 - 30, time=50, probability=0.01),
    # CUTLERY — hard to grasp/locate
    (ActionType.PICKUP, Knife, ANY):
        ActionEvaluation(score=50 + 50, penalty=-20 - 10, time=100, probability=0.01),
    (ActionType.PICKUP, Fork, ANY):
        ActionEvaluation(score=50 + 50, penalty=-20 - 10, time=100, probability=0.01),
    (ActionType.PICKUP, Spoon, ANY):
        ActionEvaluation(score=50 + 50, penalty=-20 - 10, time=100, probability=0.01),
    # Common objects
    (ActionType.PICKUP, Bowl, ANY):
        ActionEvaluation(score=50 - 20, penalty=-20 + 10, time=30, probability=0.75),
    (ActionType.PICKUP, Milk, ANY):
        ActionEvaluation(score=50 - 20, penalty=-20 + 10, time=15, probability=0.95),
    (ActionType.PICKUP, Cereal, ANY):
        ActionEvaluation(score=15, penalty=-10, time=15, probability=0.95),
    # any other object
    (ActionType.PICKUP, ANY, ANY):
        ActionEvaluation(score=50, penalty=-20, time=30, probability=0.5),

    # ------------------ PLACE (base: 40pts / -15 / 40s / 0.6)
    (ActionType.PLACE, ANY, "dishwasher"):
        ActionEvaluation(score=40 + 70, penalty=-15 - 25, time=20, probability=0.4),
    # place next to a similar object
    (ActionType.PLACE, "PLACEHOLDER", ANY):
        ActionEvaluation(score=40 + 20, penalty=-15 - 5, time=40, probability=0.6),
    # CUTLERY (base is 40 — fixed copy-paste from PICKUP that read 50)
    (ActionType.PLACE, Knife, ANY):
        ActionEvaluation(score=40 + 50, penalty=-15 - 15, time=40, probability=0.6),
    (ActionType.PLACE, Fork, ANY):
        ActionEvaluation(score=40 + 50, penalty=-15 - 15, time=40, probability=0.6),
    (ActionType.PLACE, Spoon, ANY):
        ActionEvaluation(score=40 + 50, penalty=-15 - 15, time=40, probability=0.6),
    # Common objects
    (ActionType.PLACE, Bowl, ANY):
        ActionEvaluation(score=40 - 20, penalty=-15 + 5, time=40 - 20, probability=0.8),
    (ActionType.PLACE, Milk, ANY):
        ActionEvaluation(score=40 - 20, penalty=-15 + 5, time=40 - 20, probability=0.99),
    (ActionType.PLACE, Cereal, ANY):
        ActionEvaluation(score=15, penalty=-10, time=15, probability=0.99),
    (ActionType.PLACE, Plate, ANY):
        ActionEvaluation(score=40, penalty=-15, time=40, probability=0.01),
    # any other object
    (ActionType.PLACE, ANY, ANY):
        ActionEvaluation(score=40, penalty=-15, time=40, probability=0.6),

    # ------------------ OPEN / CLOSE (probability 0 — not yet implemented)
    (ActionType.OPEN, ANY, "dishwasher"):
        ActionEvaluation(score=200, penalty=-50, time=60, probability=0.0),
    (ActionType.OPEN, ANY, ANY):
        ActionEvaluation(score=0, penalty=0, time=50, probability=0.0),
    (ActionType.CLOSE, ANY, "dishwasher"):
        ActionEvaluation(score=200, penalty=-50, time=50, probability=0.0),
    (ActionType.CLOSE, ANY, ANY):
        ActionEvaluation(score=0, penalty=0, time=50, probability=0.0),

    # ------------------ NAVIGATE (points/penalty always 0; ~20s)
    # NOTE: the old duplicate (NAVIGATE,"","") keys (20 then 30) collapse into the
    # single catch-all below.
    (ActionType.NAVIGATE, ANY, ANY):
        ActionEvaluation(score=0, penalty=0, time=20, probability=0.98),

    # ------------------ Handover
    (ActionType.HAND_OVER, ANY, ANY):
        ActionEvaluation(score=15, penalty=-10, time=15, probability=0.95),

    # ------------------ Pour (~200s; probability 0 — not yet implemented)
    (ActionType.POUR, Cereal, ANY):
        ActionEvaluation(score=0, penalty=0, time=200, probability=0.0),
    (ActionType.POUR, Milk, ANY):
        ActionEvaluation(score=0, penalty=0, time=200, probability=0.0),
    (ActionType.POUR, ANY, ANY):
        ActionEvaluation(score=0, penalty=0, time=10, probability=0.0),

    # ------------------ Push / Detect / Custom
    (ActionType.PUSH, ANY, ANY):
        ActionEvaluation(score=0, penalty=0, time=8, probability=0.0),
    (ActionType.DETECT, ANY, ANY):
        ActionEvaluation(score=5, penalty=-5, time=5, probability=0.8),
    (ActionType.EXPLORE, ANY, ANY):
        ActionEvaluation(score=0, penalty=0, time=20, probability=0.7),
    (ActionType.CUSTOM, ANY, ANY):
        ActionEvaluation(score=0, penalty=0, time=0, probability=0.0),
}

# Fallback
action_evaluation = ActionEvaluation(score=0, penalty=0, time=0, probability=1.0)


def evaluation(action_type: ActionType, semantic_annotation: SemanticAnnotation = None, location: str = "") -> ActionEvaluation:
    """
    Wild card search, for the Action. In short: if the sem_anno or loc isnt in ACTIONS, it becomes ANY. If a hit is registered, it is returned
    :param action_type: action type defines the action to be evaluated
    :param semantic_annotation: semantic annotation of the action to be evaluated
    :param location: location of the action to be evaluated
    """
    for sem_anno in (semantic_annotation, ANY):
        for loc in (location, ANY):
            hit = ACTIONS.get((action_type, sem_anno, loc))
            if hit is not None:
                return hit
    return action_evaluation



# Module-level: failure type -> candidate repair operators
CANDIDATE_OPERATORS: dict[type[Exception],
list[PlanTransformationOperator]] = {
    ObjectNotReachableException:
        [PlanTransformationOperator.SUBSTITUTE_WITH_ASSISTANCE],
    ObjectDoesntFitException:
        [PlanTransformationOperator.REPLAN_WITH_ASSISTANCE,
         PlanTransformationOperator.SKIP],
    TimeoutError: [PlanTransformationOperator.SKIP,
                   PlanTransformationOperator.REPLAN,
                   PlanTransformationOperator.RETRY],
    MotionDidNotFinish: [PlanTransformationOperator.SKIP,
                         PlanTransformationOperator.REPLAN,
                         PlanTransformationOperator.RETRY],
    ObjectNotGrasped: [PlanTransformationOperator.SKIP,
                       PlanTransformationOperator.REPLAN,
                       PlanTransformationOperator.RETRY],
    CollisionViolatedError:
        [PlanTransformationOperator.REPLAN],
    WorldEntityNotFoundError:
    [PlanTransformationOperator.REPLAN,
     PlanTransformationOperator.SKIP],
    ExecutionAbortedException:
    [PlanTransformationOperator.SKIP,
                         PlanTransformationOperator.REPLAN,
                         PlanTransformationOperator.RETRY]
    # TODO: add other errors, that I want to handle
    #
}

DEFAULT_OPERATORS: list[PlanTransformationOperator] = [
    PlanTransformationOperator.SKIP,
    PlanTransformationOperator.REPLAN,
]

# keep in mind to potentially add MRO
def lookup_operators(exception: Exception) -> list[PlanTransformationOperator]:
    if type(exception) in CANDIDATE_OPERATORS:
        return CANDIDATE_OPERATORS.get(exception)
    else: return DEFAULT_OPERATORS

# Penalties applied on top of (or instead of) base points
OUTCOME_MODIFIERS: dict[Status | TaskStatus, float] = {
    TaskStatus.SUCCEEDED:                 1.0,
    TaskStatus.FAILED:                    0.0,
    Status.SUCCESS: 1.0,  # full points
    Status.SUCCESS_WITH_ASSIST: 0.5,  # half points, assist penalty applied separately
    Status.FAILURE_RECOVERABLE: 0.0,  # no points
    Status.FAILURE_UNRECOVERABLE: 0.0,  # no points
    Status.SKIPPED: 0.0,
    Status.NOT_ASSIGNED: 0,
}

CHALLENGE_DURATION: dict[ChallengeMode, int] = {
    ChallengeMode.GPSR: 50,
    ChallengeMode.PP: 500,
    ChallengeMode.FD: 50,
}

# Rooms as the coarse navigable unit: each room contains the surfaces (tables) that can be
# scanned within it. Single source of truth -- the surface names must match NAVIGATION_POSES.
ROOM_SURFACES: dict[str, list[str]] = {
    "kitchen":    ["table", "counterTop"],
    "livingroom": ["lowerTable", ],
    "office":     ["desk"],
    "dining_room": ["dining_table", "shelf_1", "shelf_2"],
    "preperation_room": ["cooking_table"],
}
# Derived inverse: surface -> room, so a known location always resolves to its room.
SURFACE_ROOM: dict[str, str] = {s: r for r, surfaces in ROOM_SURFACES.items() for s in surfaces}

# All possible tasks for PP
# TODO: implement navigationTimeOut, for unexpected length
TASKSTEP_PP: list[Task] = [Task(id= 0, task_steps=[TaskStep(ActionType.PICKUP, object_annotations=Bowl, room="preperation_room", uncertain=True),
                                                   TaskStep(ActionType.PARK),
                                                   TaskStep(ActionType.PLACE, object_annotations=Bowl, location="counterTop"),
                                                   TaskStep(ActionType.PARK)]),

                           Task(id= 1, task_steps=[TaskStep(ActionType.PICKUP, object_annotations=Plate, room="office", uncertain=True),
                                                   TaskStep(ActionType.PARK),
                                                   TaskStep(ActionType.PLACE, object_annotations=Plate, location="table"),
                                                   TaskStep(ActionType.PARK)]),

                           Task(id= 2, task_steps=[TaskStep(ActionType.PICKUP, object_annotations=Milk, room="kitchen", uncertain=True),
                                                   TaskStep(ActionType.PARK),
                                                   TaskStep(ActionType.PLACE, object_annotations=Milk, location="shelf_1"),
                                                   TaskStep(ActionType.PARK)]),

                           Task(id= 3, task_steps=[TaskStep(ActionType.PICKUP, object_annotations=Cereal, room="kitchen", uncertain=True),
                                                   TaskStep(ActionType.PARK),
                                                   TaskStep(ActionType.PLACE, object_annotations=Cereal, location="table"),
                                                   TaskStep(ActionType.PARK)]), ]

# All possible tasks for GPSR
TASKSTEP_GPSR: list[Task] = [Task(id= 0, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Bowl), TaskStep(ActionType.PLACE, object_annotations=Bowl)]),
                             Task(id= 1, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Plate), TaskStep(ActionType.PLACE, object_annotations=Plate)]),
                             Task(id= 2, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Milk), TaskStep(ActionType.PLACE, object_annotations=Milk)]),
                             Task(id= 3, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Cereal), TaskStep(ActionType.PLACE, object_annotations=Cereal)]), ]

# All possible tasks for FD
TASKSTEP_FD: list[Task] = [Task(id= 0, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Bowl), TaskStep(ActionType.PLACE, object_annotations=Bowl)]),
                           Task(id= 1, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Plate), TaskStep(ActionType.PLACE, object_annotations=Plate)]),
                           Task(id= 2, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Milk), TaskStep(ActionType.PLACE, object_annotations=Milk)]),
                           Task(id= 3, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Cereal), TaskStep(ActionType.PLACE, object_annotations=Cereal)]), ]

CHALLENGE_TASKS : dict[ChallengeMode, list[Task]] = {
    ChallengeMode.PP: TASKSTEP_PP,
    ChallengeMode.GPSR: TASKSTEP_GPSR,
    ChallengeMode.FD: TASKSTEP_FD
}

