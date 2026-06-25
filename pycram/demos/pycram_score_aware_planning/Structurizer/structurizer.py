from common.types import Status
from demos.pycram_score_aware_planning.common.types import Task

"""
Structurizer there to create a plan and sequence, as the building block,
between the probability and the possible score, to structurize all needed and known tasks.
"""


class PlanStructurizer:
    def structurize(self, task_list: list[Task]) -> list[Task]:
        # sort by normalized score_per_seconds + probability (set by normalize_task_estimation)
        skipped_tasks: list[Task] = []
        for t in task_list:
            for ts in t.task_steps:
                if ts.action_outcome == Status.SKIPPED:
                    skipped_tasks.append(t)
                    task_list.remove(t)

        sorted_task_list = sorted(
            task_list,
            key=lambda t: (t.score_per_seconds) *  (t.probability**2),
            reverse=True,
        )
        sorted_skipped_tasks = sorted(
            skipped_tasks,
            key=lambda t: (t.score_per_seconds) *  (t.probability**2),
            reverse=True,
        )
        sorted_task_list.extend(sorted_skipped_tasks)
        self.summary(sorted_task_list)
        return sorted_task_list

    @staticmethod
    def summary(task_list: list[Task]) -> None:
        if not task_list:
            print("No task estimations to summarize.")
            return

        best = task_list[0]
        lines = [
            "Task Structurizer — Sorted Plan",
            "=" * 60,
            f"{'Rank':<6} {'Task ID':<10} {'Score':>8} {'pts/s':>8} {'p(task)':>9} {'combined':>10}",
            "-" * 60,
        ]
        for rank, task in enumerate(task_list, 1):
            sps = getattr(task, "task_score_per_seconds", task.score_per_seconds)
            prob = getattr(task, "task_probability", task.probability)
            score = getattr(task, "task_score", task.score)
            combined = round(sps * (prob ** 2), 3)  # same key the sort uses, so the column matches the order
            lines.append(
                f"{rank:<6} {task.id:<10} {score:>8} "
                f"{sps:>8.3f} {prob:>9.3f} {combined:>10.3f}"
            )
        best_sps = getattr(best, "task_score_per_seconds", best.score_per_seconds)
        best_prob = getattr(best, "task_probability", best.probability)
        lines += [
            "=" * 60,
            "  Execute: " + " → ".join(f"Task {t.id}" for t in task_list),
            f"  Top pick: Task {best.id}  "
            f"(score/s={best_sps:.3f}  p={best_prob:.3f}  combined={best_sps * (best_prob**2):.3f})",
        ]
        print("\n".join(lines))

# pls dont test for bugs yet, thank you bye :heart: