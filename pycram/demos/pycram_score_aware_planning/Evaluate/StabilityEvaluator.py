from dataclasses import dataclass

from common.cram_types import Task


# TODO: implement RobotPlanStability
@dataclass(kw_only=True)
class RobotPlanStability:

    def record(self, **kwargs):
        pass

    def evaluate(self, task: Task):
        pass

    def summary(self, **kwargs):
        pass