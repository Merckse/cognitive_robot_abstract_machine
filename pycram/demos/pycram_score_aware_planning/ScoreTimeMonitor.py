import datetime

from _pytest import outcomes

from common.types import ActionOutcome, ScoreEvent, TaskStep
from common.values import FLAT_PENALTIES, get_values
from giskardpy.tree.behaviors import time
from pycram.datastructures.enums import TaskStatus
from pycram.language import SequentialNode
from pycram.robot_plans.actions.base import ActionDescription


class ScoreTimeMonitor:
    _start_time = datetime.datetime.now()
    _score: int = 0
    _time: int = 0
    _score_events : list[ScoreEvent] = []

    def __init__(self):
        pass

    def record_score(self, task_step: TaskStep, plan: SequentialNode):
        outcome = plan.status
        duration = plan.end_time - plan.start_time
        action_start_time = (plan.start_time - self._start_time).seconds
        points, penalty, _, _ = get_values(action_type=task_step.action_type, object_name=task_step.object_name, location=task_step.location)

        if outcome == TaskStatus.SUCCEEDED:
            self._score += points
        if outcome == TaskStatus.FAILED:
            # TODO: replace with proper Reference
            self._score -= penalty
        else:
            return

        event = ScoreEvent(
            timestamp=action_start_time,
            action_type=task_step.action_type,
            outcome=str(outcome),
            object_name=task_step.object_name,
            base_points=points,
            penalty=penalty,
            time_spent=duration.seconds,
            cumulative_score=self._score,
            cumulative_time=self._time,
        )
        self._score_events.append(event)