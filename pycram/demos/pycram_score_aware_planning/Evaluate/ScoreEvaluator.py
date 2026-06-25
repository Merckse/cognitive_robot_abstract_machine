from __future__ import annotations
import time
from dataclasses import dataclass, field
from typing import Optional

from common.cram_types import Status, TaskStep
from demos.pycram_score_aware_planning.common.cram_types import (
    Task, ScoreEvent,
)
from demos.pycram_score_aware_planning.common.values import (
    evaluation,
)
from pycram.datastructures.enums import TaskStatus

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------
"""
This is the actual score event that is recorded.

:param timestamp:      The time at which the event occurred.
:param action_type:    The type of action performed.
:param outcome:        The outcome of the action.
:param object_name:    Optional label for the object involved.
:param base_points:    The base points earned for this action.
:param penalty:        The penalty applied for this action.
:param time_spent:     The actual time spent on the action.
:param net_points:     The total points earned for this action.
:param cumulative_score: The total score so far.
:param cumulative_time: The total time so far.
:param note:           Free-text note attached to this event.
"""

"""
This is the estimator for the Expected Score.

:param action_type:    The type of action performed.
:param outcome:        The outcome of the action.
:param object_name:    Optional label for the object involved.
:param expected_time:  The expected time spent on the action.
:param expected_score: The expected score created by completing the action.
"""

@dataclass(kw_only=True)
class RobotScorer:
    """
    Central scoring tracker. Call record() after each action completes.
    Thread-safe reads; not designed for concurrent writes.
    """

    """
    The context providing world and robot.
    """

    _score: int = field(default=0, init=False, repr=False)
    _time: int = field(default=0, init=False, repr=False)
    _start_time_action: float = field(default=time.time(), init=False, repr=False)

    # ------------------------------------------------------------------
    # Core API
    # ------------------------------------------------------------------

    """
    Estimating the expected time and score created by completing a task.
    """

    def estimate(
        self, task_list: list[Task], finished_task_ids: Optional[list[int]] = [],
    ) -> list[Task]:
        for task in task_list:
            total_score :int= 0
            total_time : int= 0
            total_score_penalized : int = 0
            outcome : TaskStatus | Status = TaskStatus.SUCCEEDED

            if task.id in finished_task_ids:
                continue
            for step in task.task_steps:
                base_score: int = 0
                expected_time: int = 0
                penalty: int = 0

                # if action is already successful, dont give score, else as expected
                if step.action_outcome == Status.SUCCESS:
                    pass
                else:
                    step_profile = evaluation(
                    step.action_type, step.object_annotations, step.location
                    )
                    base_score = step_profile.score
                    expected_time = step_profile.time

                    if step.action_assisted:
                        outcome = Status.SUCCESS_WITH_ASSIST
                        base_score : int = 0




                # calculation of total scores
                total_score += base_score
                total_time += expected_time
                total_score_penalized += base_score + penalty

                # Asserting  evaluated score and time
                step.action_score = base_score
                step.action_penatly = penalty
                step.action_time = expected_time
                step.task_score_penalized = base_score+penalty


            # asserting all needed values on the task
            task.score = total_score
            task.duration = total_time
            task.score_penalized = total_score_penalized
            task.score_per_seconds = total_score_penalized / total_time
            task.status = TaskStatus.CREATED
        return task_list

    def get_score_task_list(self, task_list : list[TaskStep]) -> int:
        return sum(t.action_score for t in task_list)