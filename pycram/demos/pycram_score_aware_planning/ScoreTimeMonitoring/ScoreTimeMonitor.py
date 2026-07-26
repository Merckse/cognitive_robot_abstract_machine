import datetime

import json
import logging
import os
import time

from dataclasses import asdict, dataclass, field

from common.cram_types import ActionType, TaskStep, ScoreEvent, Task, Status
from common.values import CHALLENGE_DURATION, CHALLENGE_TASKS
from helper_methods import get_values
from pycram.datastructures.enums import TaskStatus, ChallengeMode
from pycram.language import SequentialNode

logger = logging.getLogger(__name__)


def _json_default(o):
    """Fallback encoder: SemanticAnnotations are carried as the class itself, emit its name."""
    if isinstance(o, type):
        return o.__name__
    return str(o)

@dataclass
class ScoreTimeMonitor:

    challenge_mode: ChallengeMode = field(default=None)
    _score_events: list[ScoreEvent] = field(default_factory=list)
    _start_time : datetime.datetime = field(default_factory=datetime.datetime.now)
    """Start of the challenge, also used for naming the log file."""
    _score: int = field(default=0)
    _time: int = field(default=0)
    _challenge_duration: int = field(default=0, init=False)

    def __post_init__(self):
        self._challenge_duration = CHALLENGE_DURATION.get(self.challenge_mode, 0)

    @staticmethod
    def _carries_failure_reason(node) -> bool:
        """
        Whether this node or anything below it actually raised. This separates the one step that
        really failed from the downstream steps that were interrupted before they ever ran.
        """
        for candidate in [node] + node.descendants:
            if candidate.reason is not None:
                return True
        return False

    def record_score(self, task_step: TaskStep, plan: SequentialNode, duration: float = None):

        outcome = plan.status
        succeeded = outcome == TaskStatus.SUCCEEDED
        failed = outcome in (TaskStatus.FAILED, TaskStatus.INTERRUPTED)
        if not succeeded and not failed:
            return

        if outcome == TaskStatus.INTERRUPTED and not self._carries_failure_reason(plan):
            return

        penalty : int = 0
        points : int = 0
        action_start_time = (plan.start_time - self._start_time).total_seconds()
        if duration is None:
            # an interrupted node never stamps end_time, so fall back to now
            end_time = plan.end_time if plan.end_time is not None else datetime.datetime.now()
            duration = (end_time - plan.start_time).total_seconds()
        self._time = self.time_taken_seconds_total()

        max_points, max_penalty, _, _ = get_values(action_type=task_step.action_type, semantic_annotation=task_step.object_annotations, location=task_step.location)

        if succeeded:
            if task_step.action_assisted:
                points = 0
            else:
                self._score += max_points
                points = max_points
        else:
            self._score += max_penalty
            penalty = max_penalty

        event = ScoreEvent(
            assisted=task_step.action_assisted,
            timestamp=action_start_time,
            action_type=task_step.action_type,
            outcome=outcome.name,
            semantic_annotation=task_step.object_annotations,
            points=points,
            penalty=penalty,
            time_spent=duration,
            cumulative_score=self._score,
            cumulative_time=self._time,
        )
        self._score_events.append(event)
        self._log(event=event, start_time=self._start_time)

    def record_plan_scores(self, task_steps: list[TaskStep], plan: SequentialNode):
        executed_steps = []
        for task_step in task_steps:
            if task_step.action_type != ActionType.DETECT:
                executed_steps.append(task_step)
        previous_end = plan.start_time
        for task_step, node in zip(executed_steps, plan.plan.root.children):
            node_end = node.end_time if node.end_time is not None else datetime.datetime.now()
            duration = max(0.0, (node_end - previous_end).total_seconds())
            self.record_score(task_step=task_step, plan=node, duration=duration)
            previous_end = node_end

    def record_task_start(self, task: Task):
        if task.status == Status.RUNNING and task.task_begin is not None:
            return
        task.status = Status.RUNNING
        task.task_begin = datetime.datetime.now()

    def record_task_interrupt(self, task: Task):
        task.status = Status.SKIPPED
        task.actual_duration += self.task_elapsed_seconds(task)
        task.task_begin = None

    def record_task_end(self, task: Task):
        task.status = Status.SUCCESS
        task.actual_duration += self.task_elapsed_seconds(task)
        task.task_begin = None

    @property
    def score(self) -> int:
        return self._score

    @property
    def score_events(self) -> list[ScoreEvent]:
        return list(self._score_events)

    def task_elapsed_seconds(self, task: Task) -> float:
        """Seconds since this task started executing."""
        if task.task_begin is None:
            return 0.0
        return (datetime.datetime.now() - task.task_begin).total_seconds()

    def task_overtime_seconds(self, task: Task) -> float:
        """Positive = running over the estimated duration. Negative = ahead of schedule."""
        return self.task_elapsed_seconds(task) - task.duration

    def time_remaining_seconds_total(self):
        remaining_time = self._challenge_duration - (datetime.datetime.now() - self._start_time).total_seconds()
        return remaining_time

    def time_remaining_seconds_task(self, task: Task):
        remaining_time = task.duration - self.task_elapsed_seconds(task)
        return remaining_time

    def time_taken_seconds_total(self):
        time_taken : int = round((datetime.datetime.now() - self._start_time).total_seconds())
        return time_taken

    def time_expected_seconds_task_list(self, task_list: list[TaskStep]) -> float:
        expected_time : float = sum(t.action_time for t in task_list)
        return expected_time

    def time_expected_seconds_task_total(self, task: Task):
        expected_time = self.time_expected_seconds_task_list(task.task_steps)
        return expected_time

    def _log(self, event: ScoreEvent, start_time: datetime.datetime):
        filename = "plan_" + start_time.strftime("%Y%m%d-%H%M%S") + ".log"
        log_dir = os.path.join(os.getcwd(), filename)
        entry = json.dumps(asdict(event), indent=2, default=_json_default)

        if not os.path.exists(log_dir):
            # first event: open the JSON array
            with open(log_dir, 'w') as f:
                f.write("[\n" + entry + "\n]")
        else:
            # splice this one event in just before the closing ']'
            with open(log_dir, 'r+') as f:
                pos = f.seek(0, os.SEEK_END)
                while pos > 0:
                    pos -= 1
                    f.seek(pos)
                    if f.read(1) == ']':
                        break
                f.seek(pos)
                f.truncate()
                f.write(",\n" + entry + "\n]")
        logger.info("recorded finished action")

    def mean_tasks(self, challenge_mode : ChallengeMode) -> float:
        """
        mean of time taken for tasks in challenge
        """
        tasks = CHALLENGE_TASKS.get(challenge_mode) or []
        if not tasks:
            return 0.0
        ts_times: list[int] = []
        for t in tasks:
            ts_time : int = 0
            for ts in t.task_steps:
                ts_time += ts.action_time
            ts_times.append(ts_time)
        avg_time = sum(ts_times) / len(tasks)
        return avg_time

    def limited_time(self) -> bool:
        avg_task_duration = self.mean_tasks(self.challenge_mode)
        challenge_remaining = self.time_remaining_seconds_total()
        limited : bool = challenge_remaining <= avg_task_duration

        return limited
