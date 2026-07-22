# ---------------------------------------------------------------------------
# values
# ---------------------------------------------------------------------------
from dataclasses import dataclass
import random
from random import randint

from demos.pycram_score_aware_planning.common.cram_types import ActionType, Status, TaskStep, Task
from semantic_digital_twin.datastructures.definitions import RoomEnum
from giskardpy.middleware.ros2.exceptions import ExecutionAbortedException
from giskardpy.motion_statechart.exceptions import CollisionViolatedError
from giskardpy.motion_statechart.goals.pick_up import ObjectDoesntFitException, ObjectNotReachableException
from pycram.datastructures.enums import TaskStatus, PlanTransformationOperator, ChallengeMode
from pycram.exceptions import MotionDidNotFinish
from pycram.plans.failures import ObjectNotGrasped
from semantic_digital_twin.exceptions import WorldEntityNotFoundError
from semantic_digital_twin.semantic_annotations.semantic_annotations import Plate, Knife, Fork, Spoon, Bowl, Milk, \
    Cereal, \
    Apple, Orange, Cloth, Cup, Bottle, Noodles, Pringles, Food, Cabinet, Dishwasher, DishwasherTab, MustardBottle, \
    TomatoSoup, Banana
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation

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
        [PlanTransformationOperator.SKIP,
        # PlanTransformationOperator.REPLAN,
        PlanTransformationOperator.SUBSTITUTE_WITH_ASSISTANCE,
        PlanTransformationOperator.RETRY
          ],
    ObjectDoesntFitException:
        [
            # PlanTransformationOperator.REPLAN_WIT:H_ASSISTANCE,
         PlanTransformationOperator.SUBSTITUTE_WITH_ASSISTANCE,
         PlanTransformationOperator.SKIP],
    TimeoutError:
        [PlanTransformationOperator.SKIP,
        PlanTransformationOperator.RETRY,
        # PlanTransformationOperator.REPLAN,
        PlanTransformationOperator.SUBSTITUTE_WITH_ASSISTANCE,
        ],
    MotionDidNotFinish:
        [
         PlanTransformationOperator.SKIP,
         # PlanTransformationOperator.REPLAN,
         PlanTransformationOperator.SUBSTITUTE_WITH_ASSISTANCE,
         PlanTransformationOperator.RETRY
         ],
    ObjectNotGrasped:
        [PlanTransformationOperator.SKIP,
        # PlanTransformationOperator.REPLAN,
        PlanTransformationOperator.RETRY,
        PlanTransformationOperator.SUBSTITUTE_WITH_ASSISTANCE,
        ],
    CollisionViolatedError:
        [
# PlanTransformationOperator.REPLAN
         ],
    WorldEntityNotFoundError:
        [
            # PlanTransformationOperator.REPLAN,
         PlanTransformationOperator.SKIP],
    ExecutionAbortedException:
        [PlanTransformationOperator.SKIP,
         PlanTransformationOperator.REPLAN,
         PlanTransformationOperator.RETRY],

    # TODO: add other errors, that I want to handle
}

DEFAULT_OPERATORS: list[PlanTransformationOperator] = [
    PlanTransformationOperator.SKIP,
    PlanTransformationOperator.REPLAN,
    PlanTransformationOperator.RETRY,
]

# keep in mind to potentially add MRO
def lookup_operators(exception: Exception) -> list[PlanTransformationOperator]:
    if type(exception) in CANDIDATE_OPERATORS:
        return CANDIDATE_OPERATORS.get(type(exception))
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


def get_surfaces(challenge_mode : ChallengeMode, RoomEnum):
    surfaces = []
    match challenge_mode:
        case ChallengeMode.GPSR:
            surfaces = ROOM_SURFACES.get(RoomEnum)
        case ChallengeMode.PP:
            surfaces = ROOM_SURFACES_PP.get(RoomEnum)
        case ChallengeMode.FD:
            surfaces = ROOM_SURFACES.get(RoomEnum)
    return surfaces

ROOM_SURFACES: dict[RoomEnum, list[str]] = {
    RoomEnum.KITCHEN:          ["table", "counterTop"],
    RoomEnum.LIVINGROOM:       ["lowerTable", ],
    RoomEnum.OFFICE:           ["desk"],
    RoomEnum.DINING_ROOM:      ["dining_table", "shelf_1", "shelf_2"],
    RoomEnum.PREPERATION_ROOM: ["cooking_table"],
}

ROOM_SURFACES_PP: dict[RoomEnum, list[str]] = {
    RoomEnum.KITCHEN:          ["table", "counterTop"],
    RoomEnum.LIVINGROOM:       ["dishwasher", "trash"],
    RoomEnum.OFFICE:           ["desk"],
    RoomEnum.DINING_ROOM:      ["dining_table", "lowerTable", "extra_space", "shelf_1", "shelf_2", "shelf_3", "shelf_4"],
    RoomEnum.PREPERATION_ROOM: ["cooking_table"],
}


SURFACE_ROOM: dict[str, RoomEnum] = {s: r for r, surfaces in ROOM_SURFACES.items() for s in surfaces}

CHALLENGE_PP_TEST: list[Task] = [
                           # # "Picking up an object for transportation" (12x50) + "Plate" (+100)
                           # Task(id= 0, task_steps=[TaskStep(ActionType.PICKUP, object_annotations=Plate, room=RoomEnum.OFFICE, uncertain=True),
                           #                         TaskStep(ActionType.PARK),
                           #                         TaskStep(ActionType.PLACE, object_annotations=Plate, location="table"),
                           #                         TaskStep(ActionType.PARK)]),
                           #
                           # # "Cutlery" (2x+50) -- first cutlery item; Knife is spawned in the world
                           # Task(id= 1, task_steps=[TaskStep(ActionType.PICKUP, object_annotations=Knife, room=RoomEnum.DINING_ROOM, uncertain=True),
                           #                         TaskStep(ActionType.PARK),
                           #                         TaskStep(ActionType.PLACE, object_annotations=Knife, location="table"),
                           #                         TaskStep(ActionType.PARK)]),

                           # Common objects (base 12x50 pickup / 12x40 place)
                           Task(id= 2, task_steps=[TaskStep(ActionType.PICKUP, object_annotations=Bowl, room=RoomEnum.PREPERATION_ROOM, uncertain=True),
                                                   TaskStep(ActionType.PARK),
                                                   TaskStep(ActionType.PLACE, object_annotations=Bowl, location="counterTop"),
                                                   TaskStep(ActionType.PARK)]),

                           # Task(id= 3, task_steps=[TaskStep(ActionType.PICKUP, object_annotations=Milk, room=RoomEnum.KITCHEN, uncertain=True),
                           #                         TaskStep(ActionType.PARK),
                           #                         TaskStep(ActionType.PLACE, object_annotations=Milk, location="shelf_1"),
                           #                         TaskStep(ActionType.PARK)]),
                           #
                           # Task(id= 4, task_steps=[TaskStep(ActionType.PICKUP, object_annotations=Cereal, room=RoomEnum.KITCHEN, uncertain=True),
                           #                         TaskStep(ActionType.PARK),
                           #                         TaskStep(ActionType.PLACE, object_annotations=Cereal, location="table"),
                           #                         TaskStep(ActionType.PARK)
                           ]

CHALLENGE_PP: list[Task] = [

    # Place the plate in the dishwasher.
    Task(id=0, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Plate, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Plate, location="table"),
        TaskStep(ActionType.PARK)]),

    # Place the knife in the dishwasher.
    Task(id=1, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Knife, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Knife, location="table"),
        TaskStep(ActionType.PARK)]),

    # Place the fork in the dishwasher.
    Task(id=2, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Fork, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Fork, location="table"),
        TaskStep(ActionType.PARK)]),

    # Place the spoon in the dishwasher.
    Task(id=3, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Spoon, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Spoon, location="table"),
        TaskStep(ActionType.PARK)]),

    # Put the dishwasher tab in the tab slot.
    Task(id=4, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=DishwasherTab, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=DishwasherTab, location="table"),
        TaskStep(ActionType.PARK)]),

    # Open the dishwasher door.
    Task(id=5, task_steps=[
        TaskStep(ActionType.OPEN, object_annotations=Dishwasher, location="table")]),

    # Close the dishwasher door.
    Task(id=6, task_steps=[
        TaskStep(ActionType.CLOSE, object_annotations=Dishwasher, location="table")]),

    # Pull or push the dishwasher rack.
    Task(id=7, task_steps=[
        TaskStep(ActionType.PUSH, object_annotations=Dishwasher, location="table")]),

    # Place the cereal next to similar objects in the cabinet.
    Task(id=8, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Cereal, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Cereal, location="dining_table"),
        TaskStep(ActionType.PARK)]),

    # Place the milk in the cabinet.
    Task(id=9, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Milk, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Milk, location="dining_table"),
        TaskStep(ActionType.PARK)]),

    # Place the bowl on the table.
    Task(id=10, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Bowl, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Bowl, location="table"),
        TaskStep(ActionType.PARK)]),

    # # Open the milk container.
    # Task(id=11, task_steps=[
    #     TaskStep(ActionType.OPEN, object_annotations=Milk)]),
    #
    # # Pour milk into the bowl.
    # Task(id=12, task_steps=[
    #     TaskStep(ActionType.PICKUP, object_annotations=Milk, uncertain=False, location="dining_table"),
    #     TaskStep(ActionType.PARK),
    #     TaskStep(ActionType.POUR, object_annotations=Milk, location="bowl"),
    #     TaskStep(ActionType.PARK)]),

    # Pour cereal into the bowl.
    Task(id=13, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Cereal, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Cereal, location="dining_table"),
        TaskStep(ActionType.PARK)]),
    #
    # # Perceive the objects on the shelf and indicate the correct placement.
    # Task(id=14, task_steps=[
    #     TaskStep(ActionType.DETECT, location="shelf", uncertain=True)]),

    # # Place a common object from the auxiliary table.
    # Task(id=15, task_steps=[
    #     TaskStep(ActionType.PICKUP, location="dining_table", uncertain=False),
    #     TaskStep(ActionType.PARK),
    #     TaskStep(ActionType.PLACE, location="cabinet"),
    #     TaskStep(ActionType.PARK)]),
]
# All possible tasks for GPSR
CHALLENGE_GPSR: list[Task] = [Task(id= 0, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Bowl), TaskStep(ActionType.PLACE, object_annotations=Bowl)]),
                              Task(id= 1, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Plate), TaskStep(ActionType.PLACE, object_annotations=Plate)]),
                              Task(id= 2, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Milk), TaskStep(ActionType.PLACE, object_annotations=Milk)]),
                              Task(id= 3, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Cereal), TaskStep(ActionType.PLACE, object_annotations=Cereal)]), ]

# All possible tasks for FD
CHALLENGE_FD: list[Task] = [Task(id= 0, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Bowl), TaskStep(ActionType.PLACE, object_annotations=Bowl)]),
                            Task(id= 1, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Plate), TaskStep(ActionType.PLACE, object_annotations=Plate)]),
                            Task(id= 2, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Milk), TaskStep(ActionType.PLACE, object_annotations=Milk)]),
                            Task(id= 3, task_steps=[TaskStep(ActionType.NAVIGATE), TaskStep(ActionType.PICKUP, object_annotations=Cereal), TaskStep(ActionType.PLACE, object_annotations=Cereal)]), ]




GPSR_CANDIDATE_TASKS: dict[int, Task] = {
    # Take the apple from the living room to the dining table.
    0: Task(id=0, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Apple, room=RoomEnum.LIVINGROOM, uncertain=True),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Apple, location="dining table"),
        TaskStep(ActionType.PARK)]),

    # Bring me the coke from the counter.
    1: Task(id=1, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Bottle, location="counter"),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.HAND_OVER, object_annotations=Bottle),
        TaskStep(ActionType.PARK)]),

    # Get the cereal from the cupboard and deliver it to me.
    2: Task(id=2, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Cereal, location="cupboard"),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.HAND_OVER, object_annotations=Cereal),
        TaskStep(ActionType.PARK)]),

    # Take the sponge from the side table and put it on the end table.
    3: Task(id=3, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Cloth, location="side table"),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Cloth, location="end table"),
        TaskStep(ActionType.PARK)]),

    # Put the bowl on the storage table.
    4: Task(id=4, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Bowl, uncertain=True),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Bowl, location="storage table"),
        TaskStep(ActionType.PARK)]),

    # Go to the bookcase, find the pringles, and deliver it to Michael at the couch.
    5: Task(id=5, task_steps=[
        TaskStep(ActionType.DETECT, object_annotations=Pringles, location="bookcase"),
        TaskStep(ActionType.PICKUP, object_annotations=Pringles, location="bookcase"),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.HAND_OVER, object_annotations=Pringles, location="couch"),
        TaskStep(ActionType.PARK)]),

    # Take the tray to the dining table.
    6: Task(id=6, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Bowl, uncertain=True),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Bowl, location="dining table"),
        TaskStep(ActionType.PARK)]),

    # Bring me the left most object from the cupboard.
    7: Task(id=7, task_steps=[
        TaskStep(ActionType.DETECT, location="cupboard", uncertain=True),
        TaskStep(ActionType.PICKUP, location="cupboard", uncertain=True),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.HAND_OVER),
        TaskStep(ActionType.PARK)]),

    # Bring me the object on top of the bowl from the storage table.
    8: Task(id=8, task_steps=[
        TaskStep(ActionType.DETECT, location="storage table", uncertain=True),
        TaskStep(ActionType.PICKUP, location="storage table", uncertain=True),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.HAND_OVER),
        TaskStep(ActionType.PARK)]),

    # Bring me the biggest object from the bookcase.
    9: Task(id=9, task_steps=[
        TaskStep(ActionType.DETECT, location="bookcase", uncertain=True),
        TaskStep(ActionType.PICKUP, location="bookcase", uncertain=True),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.HAND_OVER),
        TaskStep(ActionType.PARK)]),


    # Take out the garbage.
    11: Task(id=11, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Bottle, uncertain=True),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.THROW_AWAY, location="trash")]),

    # Find the fork in the kitchen.
    12: Task(id=12, task_steps=[
        TaskStep(ActionType.DETECT, object_annotations=Fork, room=RoomEnum.KITCHEN, uncertain=True)]),

    # Tell me how many drinks there are on the counter.
    13: Task(id=13, task_steps=[
        TaskStep(ActionType.DETECT, location="counter", uncertain=True)]),

    # Tell me which are the three largest objects on the bookcase.
    15: Task(id=15, task_steps=[
        TaskStep(ActionType.DETECT, location="bookcase", uncertain=True)]),

    # Tell me the name of the person at the couch.
    16: Task(id=16, task_steps=[
        TaskStep(ActionType.DETECT, location="couch", uncertain=True)]),

    # Tell me how many people in the living room are standing.
    17: Task(id=17, task_steps=[
        TaskStep(ActionType.DETECT, room=RoomEnum.LIVINGROOM, uncertain=True)]),

    # -------------------- Part B: EGPSR (10) --------------------

    # Put the TomatoSoup into the cabinet.
    20: Task(id=20, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=TomatoSoup, uncertain=True),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.OPEN, object_annotations=Cabinet, location="cabinet"),
        TaskStep(ActionType.PLACE, object_annotations=TomatoSoup, location="shelf_3"),
        TaskStep(ActionType.CLOSE, object_annotations=Cabinet, location="cabinet"),
        TaskStep(ActionType.PARK)]),

    # Pick up a tableware from the storage table and put it into the dishwasher.
    21: Task(id=21, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Plate, location="storage table"),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.OPEN, object_annotations=Dishwasher, location="dishwasher"),
        TaskStep(ActionType.PLACE, object_annotations=Plate, location="dishwasher"),
        TaskStep(ActionType.CLOSE, object_annotations=Dishwasher, location="dishwasher"),
        TaskStep(ActionType.PARK)]),

    # Bring me some food from the cabinet.
    22: Task(id=22, task_steps=[
        TaskStep(ActionType.OPEN, object_annotations=Cabinet, location="cabinet"),
        TaskStep(ActionType.PICKUP, object_annotations=Food, location="shelf_3", uncertain=True),
        TaskStep(ActionType.CLOSE, object_annotations=Cabinet, location="cabinet"),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.HAND_OVER, object_annotations=Food),
        TaskStep(ActionType.PARK)]),

    # Bring me the blue object from the counter.
    23: Task(id=23, task_steps=[
        TaskStep(ActionType.DETECT, location="counter", uncertain=True),
        TaskStep(ActionType.PICKUP, location="counter", uncertain=True),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.HAND_OVER),
        TaskStep(ActionType.PARK)]),

    # Tell me how many food items are in the cupboard.
    24: Task(id=24, task_steps=[
        TaskStep(ActionType.DETECT, location="cupboard", uncertain=True)]),

    # Open the entrance door.
    25: Task(id=25, task_steps=[
        TaskStep(ActionType.OPEN, location="entrance door")]),

    # Close the corridor door.
    26: Task(id=26, task_steps=[
        TaskStep(ActionType.CLOSE, location="corridor door")]),

    # Describe the objects on the bookcase to me.
    27: Task(id=27, task_steps=[
        TaskStep(ActionType.DETECT, location="bookcase", uncertain=True)]),

    # Pour some coke in a bowl.
    28: Task(id=28, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Bottle, location="counter"),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.POUR, object_annotations=Bottle, location="bowl"),
        TaskStep(ActionType.PARK)]),

    # Guide the person at the bed to the exit.
    29: Task(id=29, task_steps=[
        TaskStep(ActionType.DETECT, location="bed", uncertain=True),
        TaskStep(ActionType.NAVIGATE, location="exit")]),
}


def get_gpsr(number_tasks: int) -> list[Task]:
    tasks : list[Task] = []
    for i in range(number_tasks):
        gpsr_task = GPSR_CANDIDATE_TASKS.get(randint(0,len(GPSR_CANDIDATE_TASKS)))
        if tasks.__contains__(gpsr_task):
            i-=1
            continue
        tasks.append(gpsr_task)
    return tasks

CHALLENGE_GPSR : list[Task] = get_gpsr(3)


PP_CANDIDATE_TASKS: list[Task] = [

    # Cup (tableware, always spawned on the dining table).
    Task(id=0, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Cup, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Cup, location="dishwasher"),
        TaskStep(ActionType.PARK)]),

    # Knife (the cuttlery_obj slot on the dining table is Knife-or-Fork).
    Task(id=1, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Knife, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Knife, location="dishwasher"),
        TaskStep(ActionType.PARK)]),

    # Fork (the other cuttlery_obj candidate).
    Task(id=2, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Fork, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Fork, location="dishwasher"),
        TaskStep(ActionType.PARK)]),
    #
    # # Dishwasher tab, spawned in extra_space -> tab slot.
    Task(id=3, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=DishwasherTab, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.OPEN, object_annotations=Dishwasher, location="dishwasher_rack", action_assisted=True),
        TaskStep(ActionType.PLACE, object_annotations=DishwasherTab, location="dishwasher_rack"),
        TaskStep(ActionType.PARK)]),

    # Open the dishwasher door.
    # Task(id=4, task_steps=[
    #     TaskStep(ActionType.OPEN, object_annotations=Dishwasher, location="dishwasher")]),
    # #
    # # Close the dishwasher door.
    # Task(id=5, task_steps=[
    #     TaskStep(ActionType.CLOSE, object_annotations=Dishwasher, location="dishwasher")]),
    #
    # # Pull or push the dishwasher rack.
    # Task(id=6, task_steps=[
    #     TaskStep(ActionType.PUSH, object_annotations=Dishwasher, location="dishwasher")]),

    # Bottle, when drawn as trash rather than a common item
    Task(id=7, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Bottle, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.THROW_AWAY, object_annotations=Bottle, location="trash_can")]),
    Task(id=7, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Bottle, uncertain=True, room=RoomEnum.LIVINGROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.THROW_AWAY, object_annotations=Bottle, location="trash_can")]),
    # Pringles, when drawn as trash rather than a common item.
    Task(id=8, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Pringles, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.THROW_AWAY, location="trash_can")]),

    # Apple, when drawn as trash rather than a common item.
    Task(id=9, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Apple, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.THROW_AWAY, location="trash_can")]),

    # --- common items (dining_table/extra_space) -> cabinet ----------------
    # Pringles, when drawn as a common item rather than trash.
    Task(id=10, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Pringles, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Pringles, location="shelf_3"),
        TaskStep(ActionType.PARK)]),

    # Apple, when drawn as a common item rather than trash.
    Task(id=11, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Apple, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Apple, location="shelf_3"),
        TaskStep(ActionType.PARK)]),


    # MustardBottle (common item, never eligible as trash).
    Task(id=13, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=MustardBottle, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=MustardBottle, location="shelf_3"),
        TaskStep(ActionType.PARK)]),

    # TomatoSoup (common item, never eligible as trash).
    Task(id=14, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=TomatoSoup, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=TomatoSoup, location="shelf_3"),
        TaskStep(ActionType.PARK)]),

    # Banana (common item, never eligible as trash).
    Task(id=15, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Banana, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.THROW_AWAY, object_annotations=Banana, location="trash_can"),
        TaskStep(ActionType.PARK)]),
    Task(id=15, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Banana, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.THROW_AWAY, object_annotations=Banana, location="trash_can"),
        TaskStep(ActionType.PARK)]),
    # --- lowerTable breakfast items -> dining_table -------------------------
    # Bowl, spawned on lowerTable.
    Task(id=16, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Bowl, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Bowl, location="dining_table"),
        TaskStep(ActionType.PARK)]),

    # Spoon, spawned on lowerTable.
    Task(id=17, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Spoon, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Spoon, location="dining_table"),
        TaskStep(ActionType.PARK)]),

    # --- cabinet_objects -> dining_table -------------------------------------
    # Milk, spawned near the cabinet.
    Task(id=18, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Milk, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Milk, location="dining_table"),
        TaskStep(ActionType.PARK)]),

    # Cereal, spawned near the cabinet.
    Task(id=19, task_steps=[
        TaskStep(ActionType.PICKUP, object_annotations=Cereal, uncertain=True, room=RoomEnum.DINING_ROOM),
        TaskStep(ActionType.PARK),
        TaskStep(ActionType.PLACE, object_annotations=Cereal, location="dining_table"),
        TaskStep(ActionType.PARK)]),
]

CHALLENGE_TASKS : dict[ChallengeMode, list[Task]] = {
    ChallengeMode.PP: PP_CANDIDATE_TASKS,
    ChallengeMode.GPSR: CHALLENGE_GPSR,
    ChallengeMode.FD: CHALLENGE_FD
}

def get_pp(number_tasks: int) -> list[Task]:
    tasks : list[Task] = []
    for i in range(number_tasks):
        pp_task = random.choice(PP_CANDIDATE_TASKS)
        if tasks.__contains__(pp_task):
            i-=1
            continue
        tasks.append(pp_task)
    return tasks

CHALLENGE_PP : list[Task] = get_pp(3)
