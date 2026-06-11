"""
robot_scorer.py
---------------
Score-aware evaluation module for robot action execution.
Tracks points and penalties per action, integrates with Giskard monitor callbacks.

Usage:
    scorer = RobotScorer()
    scorer.record(ActionType.PICKUP, ActionOutcome.SUCCESS, object_name="bowl")
    scorer.record(ActionType.PLACE, ActionOutcome.FAILURE_WITH_ASSIST)
    print(scorer.summary())

    # Export event log for the dashboard
    scorer.export_json("score_log.json")

    # Giskard monitor integration example:
    #   def on_monitor_triggered(monitor_name, state):
    #       outcome = ActionOutcome.SUCCESS if state == "success" else ActionOutcome.FAILURE_RECOVERABLE
    #       scorer.record_from_monitor(monitor_name, outcome)
"""

from __future__ import annotations
import time
from dataclasses import dataclass, field
from typing import Optional

from demos.pycram_score_aware_planning.common.types import (
    TaskMode,
    ActionType,
    ActionOutcome,
    Task, ScoreEvent,
)
from demos.pycram_score_aware_planning.common.values import (
    MAX_TIME_ESTIMATE,
    BASE_TIME_ESTIMATE,
    OUTCOME_MODIFIERS,
    FLAT_PENALTIES,
    BASE_POINTS,
)
from pycram.datastructures.dataclasses import Context
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
class ExpectedScoreEvent:
    task_id: int = 0
    # action_type: str TODO: Do i want to pain or not?
    outcome: str
    # object_name: Optional[str] TODO: Well more pain here
    expected_time: float
    expected_score: int
    expected_score_per_seconds: float
    task_step_count: int = 0


@dataclass(kw_only=True)
class RobotScorer:
    """
    Central scoring tracker. Call record() after each action completes.
    Thread-safe reads; not designed for concurrent writes.
    """

    """
    The context providing world and robot.
    """

    events: list[ScoreEvent] = field(default_factory=list)
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
        self, task_list: list[Task], finished_task_ids: Optional[list[int]] = [], probability_threshold: float = 0.2
    ) -> list[Task]:
        for task in task_list:
            total_score :int= 0
            total_time : int= 0
            total_score_penalized : int = 0
            if task.id in finished_task_ids:
                continue
            for step in task.task_steps:
                base_score: int = 0
                expected_time: int = 0
                penalty: int = 0

                if step.action_probability <= probability_threshold:
                    penalty : int = FLAT_PENALTIES.get(outcome)

                    step.action_assisted = True

                base_score += BASE_POINTS.get(
                    (step.action_type, step.object_name or ""), 1
                )
                expected_time += BASE_TIME_ESTIMATE.get(
                    (step.action_type, step.object_name or ""), 1
                )

                # calculation of total scores
                total_score += base_score
                total_time += expected_time
                total_score_penalized += base_score - penalty

                # Asserting  evaluated score and time
                step.action_score = base_score
                step.action_penatly = penalty
                step.action_time = expected_time
                step.task_score_penalized = base_score-penalty


            # asserting all needed values on the task
            task.score = total_score
            task.duration = total_time
            task.score_penalized = total_score_penalized
            task.score_per_seconds = total_score / total_time
            task.status = TaskStatus.CREATED
        return task_list



    def record_from_monitor(
        self,
        monitor_name: str,
        outcome: ActionOutcome,
        object_name: Optional[str] = None,
        note: Optional[str] = None,
    ) -> Optional[ScoreEvent]:
        """
        TODO: Think about implementing monitoring for tasks; Could be useful for failure handling
        """

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------
    @property
    def score(self) -> int:
        return self._score

    @property
    def total_actions(self) -> int:
        return len(self.events)

    @property
    def success_rate(self) -> float:
        if not self.events:
            return 0.0
        successes = sum(
            1
            for e in self.events
            if e.outcome
            in (ActionOutcome.SUCCESS.value, ActionOutcome.SUCCESS_WITH_ASSIST.value)
        )
        return round(successes / len(self.events), 3)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------
    def _log(self, event: ScoreEvent) -> None:
        sign = "+" if event.net_points >= 0 else ""
        print(
            f"[RobotScorer] {event.action_type:<20} "
            f"{event.outcome:<28} "
            f"{sign}{event.net_points:>4} pts   "
            f"(total: {event.cumulative_score})"
        )
