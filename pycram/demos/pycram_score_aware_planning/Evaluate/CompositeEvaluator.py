from typing import Optional

from demos.pycram_score_aware_planning.Evaluate.Evaluator import Evaluator
from demos.pycram_score_aware_planning.Evaluate.ProbabilityEvaluator import RobotProbability
from demos.pycram_score_aware_planning.Evaluate.ScoreEvaluator import RobotScorer
from demos.pycram_score_aware_planning.Evaluate.StabilityEvaluator import RobotPlanStability
from demos.pycram_score_aware_planning.Evaluate.types import ActionType, ActionOutcome
from pycram.datastructures.enums import TaskStatus


class CompositeEvaluator(Evaluator):

    def __init__(self):
        self.score_evaluator = RobotScorer()
        self.plan_stability_evaluator = RobotPlanStability()
        self.probability_evaluator = RobotProbability()

    def record(self, action_type : Optional[ActionType]= None,
               outcome : Optional[ActionOutcome] | Optional[TaskStatus]= None, object_name : Optional[str]= None,**kwargs):
        if action_type is not None:
            if outcome is None:
                outcome = TaskStatus.RUNNING
            self.score_evaluator.record(action_type=action_type,outcome=outcome,object_name=object_name)
        self.plan_stability_evaluator.record()
        self.probability_evaluator.record()
        pass

    def evaluate(self):
        pass

    def summary(self):
        pass
