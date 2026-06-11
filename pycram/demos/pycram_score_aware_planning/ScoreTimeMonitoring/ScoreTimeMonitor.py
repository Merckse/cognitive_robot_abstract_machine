import datetime

import json
import os
import time

from dataclasses import asdict

from common.types import ScoreEvent, TaskStep
from helper_methods import get_values
from pycram.datastructures.enums import TaskStatus
from pycram.language import SequentialNode

class ScoreTimeMonitor:
    _score_events: list[ScoreEvent] = []
    """
    List of all score events.
    """
    _start_time = datetime.datetime.now()
    """
    The starting time of the entire challenge, this is also used for naming the log file.
    """
    _score: int = 0
    """
    The total score of the entire challenge.
    """
    _time: int = 0
    """
    The total time of the entire challenge.
    """
    _challenge_duration: int = 0
    """
    the duration that the challenge has run for.
    """

    def __init__(self, challenge_duration_seconds: int):
        self._challenge_duration: int = challenge_duration_seconds

    def record_score(self, task_step: TaskStep, plan: SequentialNode):
        penalty : int = 0
        points : int = 0
        action_start_time = (plan.start_time - self._start_time).seconds
        duration = plan.end_time - plan.start_time
        outcome = plan.status
        max_points, max_penalty, _, _ = get_values(action_type=task_step.action_type, object_name=task_step.object_name, location=task_step.location)

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
            timestamp=action_start_time,
            action_type=task_step.action_type,
            outcome=outcome.name,
            object_name=task_step.object_name,
            points=points,
            penalty=penalty,
            time_spent=duration.seconds,
            cumulative_score=self._score,
            cumulative_time=self._time,
        )
        self._score_events.append(event)
        self._log(event=event, start_time=self._start_time)

    def _log(self, event: ScoreEvent, start_time: datetime.datetime):
        print(event)
        filename = "plan_" + start_time.strftime("%Y%m%d-%H%M%S") + ".log"
        log_dir = os.path.join(os.getcwd(), filename)
        entry = json.dumps(asdict(event), indent=2)

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
        print("task has been recorded")
