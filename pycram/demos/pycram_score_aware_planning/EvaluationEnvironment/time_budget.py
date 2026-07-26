import logging
from typing import Optional

logger = logging.getLogger(__name__)


class TimeBudget:

    def __init__(self, total_seconds: Optional[float] = None):
        self.total_seconds = total_seconds
        self.spent_seconds: float = 0.0
        self.skipped_over_budget: int = 0

    @property
    def unlimited(self) -> bool:
        return self.total_seconds is None

    @property
    def remaining_seconds(self) -> Optional[float]:
        return None if self.unlimited else max(0.0, self.total_seconds - self.spent_seconds)

    def fits(self, cost_seconds: float) -> bool:
        """Whether an item of this modelled cost still fits in what is left."""
        return self.unlimited or (self.spent_seconds + cost_seconds) <= self.total_seconds

    def spend(self, cost_seconds: float) -> None:
        self.spent_seconds += max(0.0, float(cost_seconds))

    def note_skipped(self, label: str, cost_seconds: float) -> None:
        """Record that an item was passed over because it did not fit."""
        self.skipped_over_budget += 1
        logger.info(f"[budget] {label} needs {cost_seconds:.0f}s, only "
                    f"{self.remaining_seconds:.0f}s of {self.total_seconds:.0f}s left -> skipping")

    @property
    def exhausted(self) -> bool:
        return not self.unlimited and self.remaining_seconds < 1.0


def task_cost_seconds(task) -> float:
    return float(sum(step.action_time or 0 for step in task.task_steps))
