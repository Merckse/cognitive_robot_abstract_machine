from demos.pycram_score_aware_planning.Evaluate.ProbabilityEvaluator import RobotProbability
from demos.pycram_score_aware_planning.Evaluate.ScoreEvaluator import RobotScorer
from demos.pycram_score_aware_planning.common.types import TaskMode, Task, TaskEstimation
from demos.pycram_score_aware_planning.common.values import TASKS
from demos.pycram_score_aware_planning.helper_methods import normalize_task_estimation

"""
Structurizer there to create a plan and sequence, as the building block,
between the probability and the possible score, to structurize all needed and known tasks.
"""


class PlanStructurizer:
    def structurize(self, task_list : list[Task], normalized_task_estimation_list: list[TaskEstimation]) -> list[Task]:
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
            ("No task estimations to summarize.")
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