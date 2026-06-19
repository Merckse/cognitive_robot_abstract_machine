import datetime

import json
import logging
import os
import time

from dataclasses import asdict, dataclass, field

from fontTools.merge.util import avg_int

from common.types import ScoreEvent, TaskStep, Task, ChallengeMode
from common.values import CHALLENGE_DURATION, CHALLENGE_TASKS
from helper_methods import get_values
from pycram.datastructures.enums import TaskStatus
from pycram.language import SequentialNode

logger = logging.getLogger(__name__)


def _json_default(o):
    """Fallback encoder for values json can't handle natively.

    SemanticAnnotations are carried as the annotation class itself (e.g. ``Bowl``),
    which json can't serialize — emit its name instead.
    """
    if isinstance(o, type):
        return o.__name__
    return str(o)

@dataclass
class ScoreTimeMonitor:
    _score_events: list[ScoreEvent] = field(default_factory=list)
    """
    List of all score events.
    """
    challenge_mode: ChallengeMode = field(default_factory=None)
    """
    List of all score events.
    """
    _start_time : datetime.datetime = field(default_factory=datetime.datetime.now)
    """
    The starting time of the entire challenge, this is also used for naming the log file.
    """
    _score: int = field(default=0)
    """
    The total score of the entire challenge.
    """
    _time: int = field(default=0)
    """
    The total time of the entire challenge.
    """
    _challenge_duration: int = field(default_factory=CHALLENGE_DURATION.get(challenge_mode))
    """
    the duration that the challenge has run for.
    """

    def record_score(self, task_step: TaskStep, plan: SequentialNode):
        penalty : int = 0
        points : int = 0
        action_start_time = (plan.start_time - self._start_time).total_seconds()
        duration = (plan.end_time - plan.start_time).total_seconds()
        self._time += duration

        outcome = plan.status
        max_points, max_penalty, _, _ = get_values(action_type=task_step.action_type, semantic_annotation=task_step.object_annotations, location=task_step.location)

        if outcome == TaskStatus.SUCCEEDED:
            self._score += max_points
            points = max_points
        elif outcome == TaskStatus.FAILED:
            # TODO: replace with proper Reference
            self._score -= max_penalty
            penalty = max_penalty
        else:
            return

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


    def record_task_start(self, task: Task):
        """
        Call at the start of each task execution so elapsed and overtime
        can be computed mid-task by the stabilizer.
        """
        task.task_begin = datetime.datetime.now()

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
        remaining_time = task.duration - (datetime.datetime.now() - task.task_begin).total_seconds()
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
        tasks = CHALLENGE_TASKS.get(challenge_mode)
        task_number = len(tasks)
        ts_times: list[int] = []
        for t in tasks:
            task_steps = len(t.task_steps)
            ts_time : int = 0
            for ts in t.task_steps:
                ts_time += ts.action_time
            ts_times.append(ts_time)
        avg_time = sum(ts_times) / task_number
        return avg_time

    def limited_time(self) -> bool:
        """
        A function, that just returns if there is limited time. Limited time is here defined by a limit
        of 10 percent of the overall time and the median of all task durations.
        """
        avg_task_duration = self.mean_tasks(self.challenge_mode)
        challenge_remaining = self._total_time - self._challenge_duration
        limited : bool = False if challenge_remaining > avg_task_duration else True

        return limited
