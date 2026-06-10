from dataclasses import dataclass
from typing import Optional

from common.types import TaskEstimation, ExpectedProbabilityModel, Task
from common.values import TASKS
from demos.pycram_score_aware_planning.Evaluate.ProbabilityEvaluator import RobotProbability
from demos.pycram_score_aware_planning.Evaluate.ScoreEvaluator import RobotScorer, ScoreEvent
from demos.pycram_score_aware_planning.common.types import ActionType, ActionOutcome
from helper_methods import normalize_task_estimation
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus

# TODO: replace the contexts here, they shouldnt be in the Class initializations, this would mean the classes are left with unupdated fucked up contexts, very whacky.
# TODO: implement, this hasnt been touched in a week
@dataclass(kw_only=True)
class CompositeEvaluator:

    def __init__(self):
        self.score_evaluator = RobotScorer()
        self.probability_evaluator = RobotProbability()
        # self.plan_stability_evaluator = RobotPlanStability()

    def record(self,context : Context, action_type : Optional[ActionType]= None,
               outcome : Optional[ActionOutcome] | Optional[TaskStatus]= None, object_name : Optional[str]= None,**kwargs):
        if action_type is not None:
            if outcome is None:
                outcome = TaskStatus.RUNNING
            score_event : ScoreEvent = self.score_evaluator.record(action_type=action_type,outcome=outcome,object_name=object_name)
        self.plan_stability_evaluator.record()
        self.probability_evaluator.record()

    def estimate(self, context : Context, task_list : list[Task]) -> list[Task]:
        # estimating score and probability, with score relying on probability
        task_list_probability_evaluated = self.probability_evaluator.estimate(task_list=task_list)
        task_list_score_evaluated = self.score_evaluator.estimate(task_list=task_list_probability_evaluated)

        # normalize list
        normalized_evaluated_task_list = normalize_task_estimation(task_list_score_evaluated)
        self.summary(normalized_evaluated_task_list)
        return normalized_evaluated_task_list


    def summary(self, task_list: list[Task]) -> None:
        """
        Prints a full evaluation summary:
          1. Ranked overview table of all tasks.
          2. Per-task step table with every evaluated value.
        """
        if not task_list:
            print("No tasks to summarize.")
            return

        ranked = sorted(task_list, key=lambda t: t.score_per_seconds, reverse=True)
        best = ranked[0]

        # ── Overview table ────────────────────────────────────────────────────
        ov_header = (
            f"{'Rank':<6} {'Task ID':<10} {'Status':<24} {'Score':>7} "
            f"{'Penalized':>10} {'Time(s)':>8} {'pts/s':>8} {'P(success)':>12} {'Steps':>6}"
        )
        ov_width = len(ov_header)
        lines: list[str] = [
            "EVALUATION SUMMARY",
            "=" * ov_width,
            ov_header,
            "-" * ov_width,
        ]
        for rank, task in enumerate(ranked, 1):
            marker = "  <-- best" if task.id == best.id else ""
            lines.append(
                f"{rank:<6} {task.id:<10} {str(task.status):<24} {task.score:>7} "
                f"{task.score_penalized:>10} {task.duration:>8} {task.score_per_seconds:>8.3f} "
                f"{task.probability:>12.3f} {len(task.task_steps):>6}{marker}"
            )
        lines += [
            "=" * ov_width,
            f"  Best: Task {best.id}  "
            f"| score {best.score}  penalized {best.score_penalized}  "
            f"duration {best.duration}s  {best.score_per_seconds:.3f} pts/s  "
            f"p={best.probability:.3f}",
            "",
        ]

        # ── Per-task step tables ──────────────────────────────────────────────
        step_header = (
            f"  {'#':<4} {'Action':<18} {'Object':<16} {'Location':<14} "
            f"{'Score':>6} {'Penalty':>8} {'Net':>7} {'Time(s)':>8} "
            f"{'Prob':>7} {'Assisted':<10} {'Pickup':<14} {'Placement'}"
        )
        step_width = len(step_header)

        for task in ranked:
            lines += [
                f"Task {task.id}  "
                f"[status: {task.status}  score: {task.score}  "
                f"penalized: {task.score_penalized}  duration: {task.duration}s  "
                f"pts/s: {task.score_per_seconds:.3f}  p(success): {task.probability:.3f}]",
                "-" * step_width,
                step_header,
                "-" * step_width,
            ]
            for i, step in enumerate(task.task_steps, 1):
                net = getattr(step, "task_score_penalized", step.action_score - step.action_penatly)
                lines.append(
                    f"  {i:<4} {step.action_type.value:<18} {(step.object_name or '-'):<16} "
                    f"{(step.location or '-'):<14} {step.action_score:>6.1f} {step.action_penatly:>8} "
                    f"{net:>7} {step.action_time:>8.1f} {step.action_probability:>7.3f} "
                    f"{str(step.action_assisted):<10} {(step.object_pickup or '-'):<14} "
                    f"{(step.object_placement or '-')}"
                )
            lines += ["=" * step_width, ""]

        print("\n".join(lines))
