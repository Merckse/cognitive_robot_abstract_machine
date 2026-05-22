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

from pycram_score_aware_planning.common.types import (
    TaskMode,
    ActionType,
    ActionOutcome,
    Task, ScoreEvent,
)
from pycram_score_aware_planning.common.values import (
    MAX_TIME_ESTIMATE,
    BASE_TIME_ESTIMATE,
    OUTCOME_MODIFIERS,
    FLAT_PENALTIES,
    BASE_POINTS,
    TASKS,
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

    task_name: str = "unnamed_task"
    task_mode: TaskMode = TaskMode.PP
    challenge_starting_time: int = MAX_TIME_ESTIMATE.get(task_mode, 0)
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
        self, taskmode: TaskMode, finished_task_ids: Optional[list[int]] = []
    ) -> list[ExpectedScoreEvent]:
        tasks: list[Task] = TASKS.get(taskmode, [])
        estimates: list[ExpectedScoreEvent] = []

        for task in tasks:
            base_score = 0
            expected_time = 0
            print(task.task_id)
            if task.task_id in finished_task_ids:
                continue
            for step in task.task_steps:
                base_score += BASE_POINTS.get(
                    (step.action_type, step.object_name or ""), 0
                )
                expected_time += BASE_TIME_ESTIMATE.get(
                    (step.action_type, step.object_name or ""), 0
                )

            estimates.append(
                ExpectedScoreEvent(
                    task_id=task.task_id,
                    # action_type=step.action_type.value,
                    outcome=ActionOutcome.SUCCESS.value,
                    # object_name=step.object_name,
                    expected_time=expected_time,
                    expected_score=base_score,
                    expected_score_per_seconds=base_score / expected_time,
                    task_step_count=len(task.task_steps),
                )
            )
        estimates = sorted(
            estimates, key=lambda x: x.expected_score_per_seconds, reverse=True
        )
        return estimates

    def record(
        self,
        action_type: ActionType,
        outcome: ActionOutcome | TaskStatus,
        object_name: Optional[str] = "",
        custom_points: Optional[int] = None,
        note: Optional[str] = None,
        **kwargs,
    ) -> ScoreEvent:
        """
        Record an action result and update the running score.

        Args:
            action_type:    The type of action performed.
            outcome:        The outcome of the action.
            object_name:    Optional label for the object involved.
            custom_points:  Override base points for this action only.
            note:           Free-text note attached to this event.

        Returns:
            The ScoreEvent that was recorded.
        """
        # Actual score calculation
        base: int = (
            custom_points
            if custom_points is not None
            else BASE_POINTS.get((action_type, object_name.lower()), 0)
        )
        modifier = OUTCOME_MODIFIERS.get(outcome, 0.0)
        penalty = FLAT_PENALTIES.get(outcome, 0)

        earned = int(base * modifier) + penalty
        time_spent = round(time.time() - self._start_time_action, 2)
        _start_time_action = time.time()

        # cumulative score & time so far
        self._score += earned
        self._time += time_spent

        event = ScoreEvent(
            timestamp=time.time(),
            action_type=action_type.value,
            outcome=str(outcome),
            object_name=object_name,
            base_points=base,
            penalty=penalty,
            time_spent=time_spent,
            net_points=earned,
            cumulative_score=self._score,
            cumulative_time=self._time,
            note=note,
        )
        self.events.append(event)
        self._log(event)
        return event

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

    def summary_estimate(self, estimates: list[ExpectedScoreEvent]) -> None:
        sum_score = 0
        sum_time = 0
        sum_score_per_seconds = 0
        sum_tasksteps = 0

        if not estimates:
            print(f"There were no estimates.")
        best = estimates[0]
        lines = [
            f"Estimate",
            "=" * 56,
            f"{'Rank':<6} {'Task ID':<10} {'Score':>8} {'Time (s)':>10} {'pts/s':>8} {'Number of tasks':>10}",
            "-" * 56,
        ]
        for rank, e in enumerate(estimates, 1):
            lines.append(
                f"{rank:<6} {e.task_id:<10} {e.expected_score:>8} {e.expected_time:>10.1f} {e.expected_score_per_seconds:>8.2f} {e.task_step_count:<10}"
            )
            sum_time += e.expected_time
            sum_score += e.expected_score
            sum_tasksteps += e.task_step_count
        sum_score_per_seconds = sum_score / sum_time
        lines += [
            "=" * 28 + "SUM" + "=" * 28,
            f"{"-":<6} {"-":<10} {sum_score:>8} {sum_time:>10.1f} {sum_score_per_seconds:>8.2f} {sum_tasksteps:<10}",
        ]
        lines += [
            "=" * 59,
            f"  Recommended: Task {best.task_id}  "
            f"({best.expected_score} pts in {best.expected_time:.1f}s = {best.expected_score_per_seconds:.2f} pts/s)",
        ]
        print("\n".join(lines))

    def summary(self) -> str:
        lines = [
            f"Task:          {self.task_name}",
            f"Total score:   {self._score}",
            f"Actions:       {self.total_actions}",
            f"Success rate:  {self.success_rate * 100:.1f}%",
            "",
            f"{'#':<4} {'Action':<22} {'Outcome':<26} {'Object':<16} {'Net pts':<10} {'Cumul.'}",
            "-" * 88,
        ]
        for i, e in enumerate(self.events, 1):
            lines.append(
                f"{i:<4} {e.action_type:<22} {e.outcome:<26} "
                f"{(e.object_name or '-'):<16} {e.net_points:<10} {e.cumulative_score}"
            )
        return "\n".join(lines)

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
