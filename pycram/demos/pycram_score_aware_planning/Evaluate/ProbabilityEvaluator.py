from dataclasses import dataclass

from demos.pycram_score_aware_planning.Evaluate.Evaluator import Evaluator
from pycram_score_aware_planning.Evaluate.types import Task, TaskMode
from pycram_score_aware_planning.Evaluate.values import BASE_PROBABILITY, TASKS


@dataclass(kw_only=True)
class ExpectedProbabilityModel:
    task_id: int
    expected_probability: float
    # taskstep_id : int
    # assisted


# TODO: implement RobotProbability
@dataclass(kw_only=True)
class RobotProbability(Evaluator):

    def estimate(self, task_mode: TaskMode) -> list[ExpectedProbabilityModel]:
        probability = 1
        expected_probability_list = []

        tasks: list[Task] = TASKS.get(task_mode)
        for task in tasks:
            for action in task.task_steps:
                print(action.action_type, action.object_name)
                probability = round(
                    probability
                    * BASE_PROBABILITY.get((action.action_type, action.object_name)),
                    2,
                )
            expected_probability_list.append(
                ExpectedProbabilityModel(
                    task_id=task.task_id, expected_probability=probability
                )
            )
            probability = 1
        sorted_list = sorted(
            expected_probability_list,
            key=lambda x: x.expected_probability,
            reverse=True,
        )
        return sorted_list

    def record(self, **kwargs):

        pass

    def evaluate(self, **kwargs):
        pass

    def summary(self, **kwargs):
        pass

    def summary_estimate(self, estimates: list[ExpectedProbabilityModel]) -> None:
        sum_score = 0
        sum_time = 0
        sum_score_per_seconds = 0
        sum_tasksteps = 0

        if not estimates:
            print(f"There were no estimates.")
        best = estimates[0]
        lines = [
            f"Estimate",
            "=" * 56,
            f"{'Rank':<6} {'Task ID':<10} {'p Percentage':>8}",
            "-" * 56,
        ]
        for rank, e in enumerate(estimates, 1):
            lines.append(f"{rank:<6} {e.task_id:<10} {e.expected_probability:>8} ")
        print("\n".join(lines))


if __name__ == "__main__":
    robot_probability = RobotProbability()
    val = robot_probability.estimate(TaskMode.PP)
    robot_probability.summary_estimate(val)
    print(val)
