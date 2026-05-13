from enum import Enum

from pycram_score_aware_planning.Evaluate.ProbabilityEvaluator import RobotProbability
from pycram_score_aware_planning.Evaluate.ScoreEvaluator import RobotScorer
from pycram_score_aware_planning.Evaluate.types import ActionType, TaskMode, Task
from pycram_score_aware_planning.Evaluate.values import TASKS

"""
Structurizer there to create a plan and sequence, as the building block,
between the probability and the possible score, to structurize all needed and known tasks.
"""


class Structurizer:

    def structurize(self, task_mode: TaskMode) -> list[Task]:
        robot_score = RobotScorer()
        estimate_score = robot_score.estimate(task_mode)
        i = 0
        list: dict[int, Task] = {}
        for single_score in estimate_score:
            print(single_score.task_id)

        task_list: list[Task] = []
        return task_list


if __name__ == "__main__":
    structurizer = Structurizer()
    structurizer.structurize(TaskMode.PP)
