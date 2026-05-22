from dataclasses import dataclass


# TODO: implement RobotPlanStability
@dataclass(kw_only=True)
class RobotPlanStability():

    def record(self, **kwargs):
        pass

    def evaluate(self, **kwargs):
        pass

    def summary(self, **kwargs):
        pass