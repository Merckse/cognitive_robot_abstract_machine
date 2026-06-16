from dataclasses import dataclass

from common.types import Task
from giskardpy.motion_statechart.goals.pick_up import ObjectNotReachableException


@dataclass(kw_only=True)
class PlanStabilizer:

    def __init__(self):
        pass

    """
    Notes:
        - Dependent on different method and situation there are different solutions
        - meaning, find out error 
        - then try to fix error
        - then go on
    """
    def stabilize(self, task: Task, exception: Exception):
        for t in task.task_steps:
            t.
        if isinstance(exception, ObjectNotReachableException):
            self.stabilize_unreachable(task)
        else:
            self.stabilize_failure_any(task)
        raise NotImplementedError

    def stabilize_unreachable(self, task: Task):
        pass

    def stabilize_failure_any(self, task: Task):
        pass

    def place_stabilize(self, task):
        raise NotImplementedError

    def pickup_stabilize(self, task):
        raise NotImplementedError