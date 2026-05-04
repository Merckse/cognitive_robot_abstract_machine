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

import json
import time
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Optional

from pycram.datastructures.enums import TaskStatus


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class ActionType(str, Enum):
    PICKUP          = "pickup"
    PLACE           = "place"
    OPEN            = "open"
    CLOSE           = "close"
    NAVIGATE        = "navigate"
    HAND_OVER       = "hand_over"
    POUR            = "pour"
    PUSH            = "push"
    DETECT          = "detect"
    CUSTOM          = "custom"


class ActionOutcome(str, Enum):
    SUCCESS              = "success"
    SUCCESS_WITH_ASSIST  = "success_with_assist"
    FAILURE_RECOVERABLE  = "failure_recoverable"
    FAILURE_UNRECOVERABLE = "failure_unrecoverable"
    SKIPPED              = "skipped"


# ---------------------------------------------------------------------------
# Scoring table  — edit these values to match your RoboCup rubric
# ---------------------------------------------------------------------------

# Base points awarded on clean success
BASE_POINTS: dict[ActionType, int] = {
    ActionType.PICKUP:    10,
    ActionType.PLACE:     10,
    ActionType.OPEN:       5,
    ActionType.CLOSE:      5,
    ActionType.NAVIGATE:   5,
    ActionType.HAND_OVER: 15,
    ActionType.POUR:      10,
    ActionType.PUSH:       8,
    ActionType.DETECT:     5,
    ActionType.CUSTOM:     0,
}

# Penalties applied on top of (or instead of) base points
OUTCOME_MODIFIERS: dict[ActionOutcome|TaskStatus, float] = {
    ActionOutcome.SUCCESS:                1.0,   # full points
    TaskStatus.SUCCEEDED:                 1.0,
    ActionOutcome.SUCCESS_WITH_ASSIST:    0.5,   # half points, assist penalty applied separately
    ActionOutcome.FAILURE_RECOVERABLE:    0.0,   # no points
    ActionOutcome.FAILURE_UNRECOVERABLE:  0.0,   # no points
    TaskStatus.FAILED:                    0.0,
    ActionOutcome.SKIPPED:                0.0,
}

# Flat penalties per outcome type (negative values)
FLAT_PENALTIES: dict[ActionOutcome | TaskStatus, int] = {
    ActionOutcome.SUCCESS:                 0,
    TaskStatus.SUCCEEDED:                 0,
    ActionOutcome.SUCCESS_WITH_ASSIST:   -10,   # human assistance penalty
    ActionOutcome.FAILURE_RECOVERABLE:    -5,
    ActionOutcome.FAILURE_UNRECOVERABLE: -15,
    TaskStatus.FAILED:                   -15,
    ActionOutcome.SKIPPED:                 0,
}

# Optional: monitor name → ActionType mapping for Giskard integration
MONITOR_ACTION_MAP: dict[str, ActionType] = {
    "pickup_reached":    ActionType.PICKUP,
    "place_reached":     ActionType.PLACE,
    "gripper_opened":    ActionType.OPEN,
    "gripper_closed":    ActionType.CLOSE,
    "nav_goal_reached":  ActionType.NAVIGATE,
    "hand_over_done":    ActionType.HAND_OVER,
}


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class ScoreEvent:
    timestamp: float
    action_type: str
    outcome: str
    object_name: Optional[str]
    base_points: int
    penalty: int
    net_points: int
    cumulative_score: int
    note: Optional[str] = None


@dataclass
class RobotScorer:
    """
    Central scoring tracker. Call record() after each action completes.
    Thread-safe reads; not designed for concurrent writes.
    """
    task_name: str = "unnamed_task"
    events: list[ScoreEvent] = field(default_factory=list)
    _score: int = field(default=0, init=False, repr=False)

    # ------------------------------------------------------------------
    # Core API
    # ------------------------------------------------------------------

    def record(
        self,
        action_type: ActionType,
        outcome: ActionOutcome | TaskStatus,
        object_name: Optional[str] = None,
        custom_points: Optional[int] = None,
        note: Optional[str] = None,
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
        base = custom_points if custom_points is not None else BASE_POINTS.get(action_type, 0)
        modifier = OUTCOME_MODIFIERS.get(outcome, 0.0)
        penalty = FLAT_PENALTIES.get(outcome, 0)

        earned = int(base * modifier) + penalty
        self._score += earned

        event = ScoreEvent(
            timestamp=time.time(),
            action_type=action_type.value,
            outcome=str(outcome),
            object_name=object_name,
            base_points=base,
            penalty=penalty,
            net_points=earned,
            cumulative_score=self._score,
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
        Convenience wrapper for Giskard monitor callbacks.
        Maps monitor_name → ActionType via MONITOR_ACTION_MAP.

        Example Giskard integration:
            def giskard_monitor_callback(monitor_name, triggered):
                if triggered:
                    outcome = ActionOutcome.SUCCESS
                else:
                    outcome = ActionOutcome.FAILURE_RECOVERABLE
                scorer.record_from_monitor(monitor_name, outcome)
        """
        action_type = MONITOR_ACTION_MAP.get(monitor_name)
        if action_type is None:
            print(f"[RobotScorer] Warning: unknown monitor '{monitor_name}', skipping.")
            return None
        return self.record(action_type, outcome, object_name=object_name, note=f"monitor:{monitor_name} {note or ''}")

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
            1 for e in self.events
            if e.outcome in (ActionOutcome.SUCCESS.value, ActionOutcome.SUCCESS_WITH_ASSIST.value)
        )
        return round(successes / len(self.events), 3)

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
    # Import / Export
    # ------------------------------------------------------------------

    def export_json(self, path: str) -> None:
        """Export full event log as JSON for the live dashboard."""
        payload = {
            "task_name": self.task_name,
            "final_score": self._score,
            "success_rate": self.success_rate,
            "total_actions": self.total_actions,
            "events": [asdict(e) for e in self.events],
        }
        with open(path, "w") as f:
            json.dump(payload, f, indent=2)
        print(f"[RobotScorer] Exported {len(self.events)} events to {path}")

    @classmethod
    def from_json(cls, path: str) -> "RobotScorer":
        """Reload a scorer from a previously exported JSON file."""
        with open(path) as f:
            data = json.load(f)
        scorer = cls(task_name=data["task_name"])
        for e in data["events"]:
            event = ScoreEvent(**e)
            scorer.events.append(event)
            scorer._score = event.cumulative_score
        return scorer

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


# ---------------------------------------------------------------------------
# Quick demo — run this file directly to see it in action
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    scorer = RobotScorer(task_name="clean_table_demo")

    scorer.record(ActionType.NAVIGATE, ActionOutcome.SUCCESS, note="approach table")
    scorer.record(ActionType.DETECT,   ActionOutcome.SUCCESS,   object_name="bowl")
    scorer.record(ActionType.PICKUP,   ActionOutcome.SUCCESS,   object_name="bowl")
    scorer.record(ActionType.PLACE,    ActionOutcome.SUCCESS,   object_name="bowl")
    scorer.record(ActionType.PICKUP,   ActionOutcome.FAILURE_RECOVERABLE, object_name="cup")
    scorer.record(ActionType.PICKUP,   ActionOutcome.SUCCESS_WITH_ASSIST, object_name="cup")
    scorer.record(ActionType.OPEN,     ActionOutcome.SUCCESS,   object_name="dishwasher")
    scorer.record(ActionType.PLACE,    ActionOutcome.FAILURE_UNRECOVERABLE, object_name="cup")
    scorer.record(ActionType.CLOSE,    ActionOutcome.SUCCESS,   object_name="dishwasher")

    print()
    print(scorer.summary())
    scorer.export_json("/tmp/score_log.json")