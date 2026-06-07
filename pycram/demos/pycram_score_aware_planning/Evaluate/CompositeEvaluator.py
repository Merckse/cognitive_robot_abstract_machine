from dataclasses import dataclass
from typing import Optional

from Evaluate.ScoreEvaluator import ExpectedScoreEvent
from common.types import TaskEstimation, ExpectedProbabilityModel, Task
from common.values import TASKS
from demos.pycram_score_aware_planning.Evaluate.ProbabilityEvaluator import RobotProbability
from demos.pycram_score_aware_planning.Evaluate.ScoreEvaluator import RobotScorer, ScoreEvent
from demos.pycram_score_aware_planning.common.types import ActionType, ActionOutcome
from helper_methods import normalize_task_estimation
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus

# TODO: implement, this hasnt been touched in a week
@dataclass(kw_only=True)
class CompositeEvaluator:

    def __init__(self, context: Context):
        self.score_evaluator = RobotScorer(context=context)
        self.probability_evaluator = RobotProbability(context=context)
        self.context: Context = context
        # self.plan_stability_evaluator = RobotPlanStability()

    def record(self, action_type : Optional[ActionType]= None,
               outcome : Optional[ActionOutcome] | Optional[TaskStatus]= None, object_name : Optional[str]= None,**kwargs):
        if action_type is not None:
            if outcome is None:
                outcome = TaskStatus.RUNNING
            score_event : ScoreEvent = self.score_evaluator.record(action_type=action_type,outcome=outcome,object_name=object_name)
        self.plan_stability_evaluator.record()
        self.probability_evaluator.record()

    def estimate(self, task_list : list[Task]) -> tuple[list[Task], list[TaskEstimation]]:
        # Initializing lits
        task_estimation_list = []

        # estimating score and probability, with score relying on probability
        estimate_probability : list[ExpectedProbabilityModel]= self.probability_evaluator.estimate(task_list=task_list)
        estimate_score : list[ExpectedScoreEvent] = self.score_evaluator.estimate(task_list=task_list)

        # printing summaries
        self.score_evaluator.summary_estimate(estimate_score)
        self.probability_evaluator.summary_estimate(estimate_probability)

        # create natural joined list
        for task_probability in estimate_probability:
            for task_score in estimate_score:
                if task_probability.task_id == task_score.task_id:
                    task_estimation = TaskEstimation(task_id=task_probability.task_id,
                                                     task_score=task_score.expected_score,
                                                     task_score_per_seconds=task_score.expected_score_per_seconds,
                                                     task_probability=task_probability.expected_probability)
                    task_estimation_list.append(task_estimation)
                    continue

        # normalize list
        normalized_task_estimation_list = normalize_task_estimation(task_estimation_list)
        return task_list,normalized_task_estimation_list


    def summary(self):
        pass
