from dataclasses import dataclass

from demos.pycram_score_aware_planning.Evaluate.Evaluator import Evaluator

@dataclass(kw_only=True)
class RobotProbability(Evaluator):

    def record(self, **kwargs):
        pass

    def evaluate(self, **kwargs):
        pass

    def summary(self, **kwargs):
        pass