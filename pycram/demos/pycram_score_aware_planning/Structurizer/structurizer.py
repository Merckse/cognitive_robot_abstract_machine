from common.cram_types import Status
from common.pretty import Cell, bar_cell, paint, render_table, value_colors
from demos.pycram_score_aware_planning.common.cram_types import Task

"""
Structurizer there to create a plan and sequence, as the building block,
between the probability and the possible score, to structurize all needed and known tasks.
"""


class PlanStructurizer:
    def structurize(self, task_list: list[Task]) -> list[Task]:
        # sort by normalized score_per_seconds + probability (set by normalize_task_estimation)
        skipped_tasks: list[Task] = []
        for t in list(task_list):
            if any(ts.action_outcome == Status.SKIPPED for ts in t.task_steps):
                skipped_tasks.append(t)
                task_list.remove(t)
        sorted_task_list = sorted(
            task_list,
            key=lambda t: t.cum_expected_score,
            reverse=True,
        )
        sorted_skipped_tasks = sorted(
            skipped_tasks,
            key=lambda t: t.cum_expected_score,
            reverse=True,
        )
        sorted_task_list.extend(sorted_skipped_tasks)
        self.summary(sorted_task_list)
        return sorted_task_list

    @staticmethod
    def summary(task_list: list[Task]) -> None:
        """Prints the final execution plan as a box table matching the evaluation theme,
        ranked by E[score] (cum_expected_score) -- the same key structurize() sorts by --
        followed by the concrete execution order."""
        if not task_list:
            print("No task estimations to summarize.")
            return

        exp = lambda t: (lambda e: 0.0 if abs(e) < 5e-4 else e)(getattr(t, "cum_expected_score", 0.0))
        best = task_list[0]
        peak = max((exp(t) for t in task_list), default=0.0) or 1.0

        rows = []
        for rank, task in enumerate(task_list, 1):
            ev = exp(task)
            skipped = any(ts.action_outcome == Status.SKIPPED for ts in task.task_steps)
            is_top = rank == 1 and not skipped
            tag = "SKIP" if skipped else ("▶" if is_top else "")
            rows.append([
                Cell(f"{rank} ▶", "green", "bold") if is_top else str(rank),
                Cell(str(task.id), "bold") if is_top else str(task.id),
                Cell(f"{ev:.3f}", *value_colors(ev)),
                bar_cell(ev, peak, 12),
                str(task.score),
                f"{task.probability:.3f}",
                f"{task.duration}s",
                str(len(task.task_steps)),
                Cell(tag, "yellow" if skipped else ("green" if is_top else "dim")),
            ])
        lines = render_table(
            "TASK STRUCTURIZER · EXECUTION PLAN",
            ["Rank", "Task", "E[score]", "level", "Score", "p(task)", "Time", "Steps", "note"],
            rows,
            widths=[5, 4, 8, 12, 5, 7, 5, 5, 4],
            aligns=["^", "^", ">", "<", ">", ">", ">", ">", "^"],
            subtitle="ordered by E[score] — the risk-weighted expected value",
        )

        order = paint(" → ".join(f"Task {t.id}" for t in task_list), "cyan")
        lines += [
            "",
            paint("  ▶ Execute:  ", "bold") + order,
            paint(f"  ★ Top pick:  Task {best.id}", "green", "bold") +
            paint(f"   (E[score] {exp(best):.3f}  ·  score {best.score}  ·  p {best.probability:.3f})", "dim"),
        ]
        print("\n".join(lines))