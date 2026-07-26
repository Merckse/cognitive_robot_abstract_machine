from dataclasses import dataclass, field
from typing import Callable, Optional

from common.types import ActionType, TaskStep

__all__ = [
    "Fluent",
    "ActionSchema",
    "ACTION_MODEL",
    "ACHIEVER",
    "generate_repair_prefix",
    "current_world_facts",
]


@dataclass(frozen=True)
class Fluent:
    predicate: str
    args: tuple = ()

    def __repr__(self) -> str:
        return f"{self.predicate}{self.args}"


def F(predicate: str, *args) -> Fluent:
    """Terse constructor: F("Located", obj, loc)."""
    return Fluent(predicate, tuple(args))


# ---------------------------------------------------------------------------
# Action schema
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ActionSchema:
    preconditions: Callable[[TaskStep], set]
    add_effects: Callable[[TaskStep], set]
    del_effects: Callable[[TaskStep], set] = field(default=lambda s: set())


def _O(step: TaskStep):
    return step.object_annotations


def _L(step: TaskStep):
    return step.location


ACTION_MODEL: dict[ActionType, ActionSchema] = {
    ActionType.NAVIGATE: ActionSchema(
        preconditions=lambda s: set(),
        add_effects=lambda s: {F("RobotAt", _L(s))},
        # del: RobotAt(*) for every other pose -- dropped under relaxation
        del_effects=lambda s: set(),
    ),
    ActionType.PARK: ActionSchema(
        preconditions=lambda s: set(),
        add_effects=lambda s: {F("ArmsParked")},
    ),
    ActionType.MOVE_TORSO: ActionSchema(
        preconditions=lambda s: set(),
        add_effects=lambda s: {F("TorsoUp")},
        del_effects=lambda s: {F("TorsoDown")},
    ),
    ActionType.DETECT: ActionSchema(
        preconditions=lambda s: {F("RobotAt", _L(s))},
        add_effects=lambda s: {F("Perceived", _O(s)), F("Located", _O(s), _L(s))},
    ),
    ActionType.PICKUP: ActionSchema(
        preconditions=lambda s: {
            F("Located", _O(s), _L(s)),
            F("RobotAt", _L(s)),
            F("Perceived", _O(s)),
            F("HandEmpty"),
            F("TorsoUp"),
        },
        add_effects=lambda s: {F("Holding", _O(s))},
        del_effects=lambda s: {F("HandEmpty"), F("Located", _O(s), _L(s))},
    ),
    ActionType.PLACE: ActionSchema(
        preconditions=lambda s: {
            F("Holding", _O(s)),
            F("RobotAt", _L(s)),
            F("FreeSurface", _L(s)),
        },
        add_effects=lambda s: {F("On", _O(s), _L(s)), F("HandEmpty")},
        del_effects=lambda s: {F("Holding", _O(s)), F("FreeSurface", _L(s))},
    ),
}


# ---------------------------------------------------------------------------
# Achiever index: fact predicate -> the TaskStep that establishes it
# ---------------------------------------------------------------------------
ACHIEVER: dict[str, Optional[Callable[[TaskStep], TaskStep]]] = {
    "RobotAt": lambda s: TaskStep(ActionType.NAVIGATE, location=_L(s)),
    "Perceived": lambda s: TaskStep(ActionType.DETECT, object_annotations=_O(s), location=_L(s)),
    "Located": lambda s: TaskStep(ActionType.DETECT, object_annotations=_O(s), location=_L(s)),
    "TorsoUp": lambda s: TaskStep(ActionType.MOVE_TORSO),
    "ArmsParked": lambda s: TaskStep(ActionType.PARK),
    "Holding": lambda s: TaskStep(ActionType.PICKUP, object_annotations=_O(s), location=_L(s)),
    "On": lambda s: TaskStep(ActionType.PLACE, object_annotations=_O(s), location=_L(s)),
    # No robot action re-establishes these -> infeasible -> SKIP / assist:
    "HandEmpty": None,
    "FreeSurface": None,
}

_REPAIR_ORDER: dict[ActionType, int] = {
    ActionType.PARK: 0,
    ActionType.NAVIGATE: 1,
    ActionType.MOVE_TORSO: 2,
    ActionType.DETECT: 3,
    ActionType.PICKUP: 4,
    ActionType.PLACE: 5,
}


def generate_repair_prefix(failed_step: TaskStep, world_facts: set) -> Optional[list[TaskStep]]:

    schema = ACTION_MODEL.get(failed_step.action_type)
    if schema is None:
        return []

    unmet = schema.preconditions(failed_step) - world_facts

    repair_by_type: dict[ActionType, TaskStep] = {}
    for fact in unmet:
        if fact.predicate not in ACHIEVER:
            # unknown predicate -> treat conservatively as infeasible
            return None
        builder = ACHIEVER[fact.predicate]
        if builder is None:
            return None  # no robot action establishes this fact
        step = builder(failed_step)
        repair_by_type[step.action_type] = step  # dedupe (Perceived + Located -> one DETECT)

    return sorted(repair_by_type.values(), key=lambda s: _REPAIR_ORDER.get(s.action_type, 99))


def current_world_facts(context, steps: list[TaskStep]) -> set:
    from helper_methods import (
        compute_surface_spaces,
        find_free_placement_pose,
        find_surface_of_object,
    )

    facts: set = set()
    world = context.world

    for step in steps:
        obj = step.object_annotations
        loc = step.location

        # --- perception: is the object present, and where does it sit? ---
        if obj is not None:
            try:
                annotations = world.get_semantic_annotations_by_type(obj)
            except Exception:
                annotations = []
            if annotations:
                facts.add(F("Perceived", obj))
                body = annotations[0].bodies[0]
                surface = find_surface_of_object(context, body)
                if surface is not None:
                    facts.add(F("Located", obj, str(surface.name)))

                # --- free placement spot on a target surface ---
                if step.action_type == ActionType.PLACE and loc:
                    for surface_space in compute_surface_spaces(world):
                        if surface_space.name == loc and \
                                find_free_placement_pose(surface_space, body, world) is not None:
                            facts.add(F("FreeSurface", loc))

        # --- robot base pose (TODO: replace with a real base-pose -> location check) ---
        if loc:
            facts.add(F("RobotAt", loc))

    # --- robot-internal state: optimistic defaults (TODO: real sensors) ---
    facts.add(F("HandEmpty"))
    facts.add(F("TorsoUp"))
    facts.add(F("ArmsParked"))

    return facts