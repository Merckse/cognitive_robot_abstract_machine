"""
Builds one evaluation_metrics.TaskAttempt per task for the H1 (EstimationExecutioner) and H3
(ScoreAwareExecutioner) executors, using exactly the same alignment
ScoreTimeMonitor.record_plan_scores() already relies on (task steps zipped 1:1 with
plan.plan.root.children, DETECT steps dropped since no node is built for them).

Without this, static_execution() would be the only condition measured through
RunMetrics/AggregateMetrics, and there would be no common instrument to attribute a score delta
between static / H1 / H1+H3 to ordering or repair specifically -- each would be read off a
different, non-comparable set of numbers.
"""

from typing import Optional

from common.cram_types import ActionType, TaskStep
from common.values import ASSISTANCE_PENALTY
from helper_methods import find_object_annotation
from evaluation_metrics import TaskAttempt
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus
from pycram.language import SequentialNode


def _reason_of(node) -> Optional[str]:

    if node is None or node.status not in TaskAttempt.ERROR_STATUSES:
        return None
    for candidate in [node] + node.descendants:
        if candidate.reason is not None:
            return type(candidate.reason).__name__
    return f"unknown ({node.status.name})"


def build_task_attempt(
    exec_steps: list[TaskStep],
    plan: SequentialNode,
    context: Context,
    duration_seconds: float,
    error_during_execution: bool = False,
    was_uncertain: bool = False,
) -> Optional[TaskAttempt]:
    executed_steps = []
    for step in exec_steps:
        if step.action_type != ActionType.DETECT:
            executed_steps.append(step)
    nodes = list(plan.plan.root.children)
    if len(executed_steps) != len(nodes):
        return None

    pickup_step = pickup_node = None
    place_step = place_node = None
    is_trash = False
    for step, node in zip(executed_steps, nodes):
        if step.action_type == ActionType.PICKUP and pickup_step is None:
            pickup_step, pickup_node = step, node
        elif step.action_type in (ActionType.PLACE, ActionType.THROW_AWAY) and place_step is None and pickup_step is not None:
            place_step, place_node = step, node
            is_trash = step.action_type == ActionType.THROW_AWAY

    if pickup_step is None:
        return None

    target = pickup_step.object_annotations
    annotation = find_object_annotation(context.world, target) if target is not None else None
    object_name = str(annotation.bodies[0].name) if annotation is not None else (
        target.__name__ if target is not None else "unknown")
    object_type = target.__name__ if target is not None else ""

    pickup_status = pickup_node.status
    place_status = place_node.status if place_node is not None else TaskStatus.CREATED

    assisted = bool(pickup_step.action_assisted or (place_step is not None and place_step.action_assisted))

    score = pickup_step.action_score if pickup_status == TaskStatus.SUCCEEDED else 0
    if place_step is not None and place_status == TaskStatus.SUCCEEDED:
        score += place_step.action_score

    penalty = pickup_step.action_penalty if pickup_status in TaskAttempt.ERROR_STATUSES else 0
    if (place_step is not None and pickup_status == TaskStatus.SUCCEEDED
            and place_status in TaskAttempt.ERROR_STATUSES):
        penalty += place_step.action_penalty

    for assisted_step in (pickup_step, place_step):
        if assisted_step is not None and assisted_step.action_assisted:
            penalty += ASSISTANCE_PENALTY

    return TaskAttempt(
        object_name=object_name,
        pickup_location=pickup_step.location,
        place_location=place_step.location if place_step is not None else "",
        pickup_status=pickup_status,
        place_status=place_status,
        assisted=assisted,
        score=score,
        penalty=penalty,
        duration_seconds=duration_seconds,
        failure_reason=_reason_of(pickup_node) or _reason_of(place_node),
        object_type=object_type,
        is_trash=is_trash,
        error_during_execution=error_during_execution,
        was_uncertain=was_uncertain,
    )
