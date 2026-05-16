from enum import Enum

from pycram.datastructures import dataclasses
from pycram_score_aware_planning.Evaluate.ProbabilityEvaluator import RobotProbability
from pycram_score_aware_planning.Evaluate.ScoreEvaluator import RobotScorer
from pycram_score_aware_planning.Evaluate.types import ActionType, TaskMode, Task
import numpy as np

class TaskEstimation:
    task_id : int
    task_score: int
    task_score_per_seconds : float
    task_probability : float

"""
Structurizer there to create a plan and sequence, as the building block,
between the probability and the possible score, to structurize all needed and known tasks.
"""


class Structurizer:

    def structurize(self, task_mode: TaskMode) -> list[Task]:
        robot_score = RobotScorer()
        robot_probability = RobotProbability()
        estimate_score = robot_score.estimate(task_mode)
        estimate_probability = robot_probability.estimate(task_mode)
        arr1 : list[tuple[int, float]] = [(1,0.2),(2,0.4)]
        arr2 : list[tuple[int, int]] = [(1,2),(2,4)]
        for task_probability in estimate_probability:
            for task_score in estimate_score:
                if task_probability[0] == task_score[0]:
                    TaskEstimation(task_id=task_probability.task_id,task_score=task_score.expected_score,task_score_per_seconds=task_score.expected_score_per_seconds,task_probability=task_probability.expected_probability)

        for task in estimate_score:
            print(task)


        task_list: list[Task] = []
        pp = np.intersect1d(arr1,arr2,)
        print(pp)
        return task_list


if __name__ == "__main__":
    structurizer = Structurizer()
    structurizer.structurize(TaskMode.PP)
