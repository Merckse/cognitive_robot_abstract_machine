from pycram_score_aware_planning.Evaluate.ProbabilityEvaluator import RobotProbability
from pycram_score_aware_planning.Evaluate.ScoreEvaluator import RobotScorer
from pycram_score_aware_planning.common.types import TaskMode, Task, TaskEstimation
from pycram_score_aware_planning.common.values import TASKS

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
        robot_score.summary_estimate(estimate_score)
        estimate_probability = robot_probability.estimate(task_mode)
        robot_probability.summary_estimate(estimate_probability)
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
        self.summary(sorted_task_estimation)
        return sorted_task_list

    @staticmethod
    def summary(task_estimations: list[TaskEstimation]) -> None:
        if not task_estimations:
            print("No task estimations to summarize.")
            return

        best = task_estimations[0]
        lines = [
            "Task Structurizer — Sorted Plan",
            "=" * 60,
            f"{'Rank':<6} {'Task ID':<10} {'Score':>8} {'pts/s':>8} {'p(task)':>9} {'combined':>10}",
            "-" * 60,
        ]
        for rank, e in enumerate(task_estimations, 1):
            combined = round(e.task_score_per_seconds + e.task_probability, 3)
            lines.append(
                f"{rank:<6} {e.task_id:<10} {e.task_score:>8} "
                f"{e.task_score_per_seconds:>8.3f} {e.task_probability:>9.3f} {combined:>10.3f}"
            )
        lines += [
            "=" * 60,
            "  Execute: " + " → ".join(f"Task {e.task_id}" for e in task_estimations),
            f"  Top pick: Task {best.task_id}  "
            f"(score/s={best.task_score_per_seconds:.3f}  p={best.task_probability:.3f}  combined={best.task_score_per_seconds + best.task_probability:.3f})",
        ]
        print("\n".join(lines))

# pls dont test for bugs yet, thank you bye :heart: