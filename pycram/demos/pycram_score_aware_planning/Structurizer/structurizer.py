from pycram_score_aware_planning.Evaluate.ProbabilityEvaluator import RobotProbability
from pycram_score_aware_planning.Evaluate.ScoreEvaluator import RobotScorer
from pycram_score_aware_planning.Evaluate.types import ActionType, TaskMode, Task, TaskEstimation
from pycram_score_aware_planning.Evaluate.values import TASKS

from pycram_score_aware_planning.helper_methods import normalize_task_estimation

"""
Structurizer there to create a plan and sequence, as the building block,
between the probability and the possible score, to structurize all needed and known tasks.
"""


class PlanStructurizer:

    def structurize(self, task_mode: TaskMode) -> list[Task]:
        robot_score = RobotScorer()
        robot_probability = RobotProbability()
        estimate_score = robot_score.estimate(task_mode)
        estimate_probability = robot_probability.estimate(task_mode)
        task_list = TASKS.get(task_mode)
        task_estimation_list = []

        # create natural joined list
        for task_probability in estimate_probability:
            for task_score in estimate_score:
                if task_probability.task_id == task_score.task_id:

                    task_estimation = TaskEstimation(task_id=task_probability.task_id,task_score=task_score.expected_score,task_score_per_seconds=task_score.expected_score_per_seconds,task_probability=task_probability.expected_probability)
                    task_estimation_list.append(task_estimation)
                    continue

        # normalize list
        normalized_task_estimation_list = normalize_task_estimation(task_estimation_list)

        # sort / structure list
        sorted_task_estimation = sorted(normalized_task_estimation_list, key=lambda x: x.task_score_per_seconds+x.task_probability, reverse=True)
        sorted_task_list : list[Task] = []
        for task in sorted_task_estimation:
            for task2 in task_list:
                if task.task_id == task2.task_id:
                    sorted_task_list.append(task2)
        print(sorted_task_list)
        return sorted_task_list

# pls dont test for bugs yet, thank you bye :heart: