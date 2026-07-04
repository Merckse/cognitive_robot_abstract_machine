import logging
from dataclasses import dataclass

from common.cram_types import Task, TaskStep
from common.pretty import Cell, bar_cell, paint, render_table, value_colors
from demos.pycram_score_aware_planning.Evaluate.ProbabilityEvaluator import RobotProbability
from demos.pycram_score_aware_planning.Evaluate.ScoreEvaluator import RobotScorer, ScoreEvent
from helper_methods import normalize_task_estimation
from pycram.datastructures.dataclasses import Context

logger = logging.getLogger(__name__)

@dataclass(kw_only=True)
class CompositeEvaluator:

    def __init__(self):
        self.score_evaluator = RobotScorer()
        self.probability_evaluator = RobotProbability()
        # self.plan_stability_evaluator = RobotPlanStability()

    def estimate(self, context : Context, task_list : list[Task], found_objects: dict, risk_aversion : float = 1) -> list[Task]:
        # estimating score and probability, with score relying on probability
        task_list_probability_evaluated = self.probability_evaluator.estimate(task_list=task_list, context=context, found_objects=found_objects)
        task_list_score_evaluated = self.score_evaluator.estimate(task_list=task_list_probability_evaluated)
        min_risk_aversion = 0.5
        max_risk_aversion = 4

        if not min_risk_aversion < risk_aversion < max_risk_aversion:
            risk_aversion = 2
            logger.info("risk_aversion set to", risk_aversion, "since it has been to high or too low")


        for task in task_list_score_evaluated:
            cum_expected_score = 0
            for ts in task.task_steps:
                safe_gain = ts.action_score * ts.action_probability ** risk_aversion
                expected_penalty = abs(ts.action_penatly) * (1 - ts.action_probability)
                ts.expected_score = (safe_gain - expected_penalty) / ts.action_time if ts.action_time else 0.0
                cum_expected_score += ts.expected_score
            task.cum_expected_score = cum_expected_score
        self.summary(task_list_score_evaluated)

        # normalize list
        normalized_evaluated_task_list = normalize_task_estimation(task_list_score_evaluated)

        self.summary(normalized_evaluated_task_list)
        return normalized_evaluated_task_list

    def get_score_task_list(self, task_list : list[TaskStep]) -> int:
        return self.score_evaluator.get_score_task_list(task_list=task_list)

    def get_probability_task_list(self, task_list : list[TaskStep]) -> float:
        return self.probability_evaluator.get_probability_task_list(task_list=task_list)


    def summary(self, task_list: list[Task]) -> None:
        """
        Prints the evaluation summary in three clearly separated sections:

          1. RANKED TASKS  — every task ordered by E[score] (cum_expected_score), the
             risk-weighted expected points/second that drives selection, with an inline bar.
          2. TOP TASK · STEP BREAKDOWN — a full table of the chosen task's steps showing
             how its value is built up (gain, expected penalty, per-step E), with a Σ total row.
          3. LEGEND — what the columns mean and the formula behind E.
        """
        if not task_list:
            print("No tasks to summarize.")
            return

        exp = lambda t: (lambda e: 0.0 if abs(e) < 5e-4 else e)(getattr(t, "cum_expected_score", 0.0))
        ranked = sorted(task_list, key=exp, reverse=True)
        best = ranked[0]
        peak = max((exp(t) for t in ranked), default=0.0) or 1.0

        lines: list[str] = []

        # ── SECTION 1 · ranked overview ────────────────────────────────────────
        ov_rows = []
        for i, task in enumerate(ranked, 1):
            ev = exp(task)
            is_best = task is best
            ov_rows.append([
                Cell(f"{i} ★", "green", "bold") if is_best else str(i),
                Cell(str(task.id), "bold") if is_best else str(task.id),
                Cell(f"{ev:.3f}", *value_colors(ev)),
                bar_cell(ev, peak, 12),
                str(task.score),
                f"{task.probability:.3f}",
                f"{task.duration}s",
                Cell(str(len(task.task_steps)), "dim"),
            ])
        lines += render_table(
            "RANKED TASKS",
            ["#", "Task", "E[score]", "level", "Score", "p(succ)", "Time", "Steps"],
            ov_rows,
            widths=[4, 4, 8, 12, 5, 7, 5, 5],
            aligns=["^", "^", ">", "<", ">", ">", ">", ">"],
            subtitle="ranked by E[score] — risk-weighted expected points / second",
        )

        # ── SECTION 2 · step breakdown of the chosen task ──────────────────────
        st_rows = []
        tot_score = tot_pen = tot_time = 0.0
        tot_gain = tot_epen = 0.0
        for i, step in enumerate(best.task_steps, 1):
            pen = step.action_penatly
            p = step.action_probability
            t = step.action_time
            es = getattr(step, "expected_score", 0.0)
            epen = abs(pen) * (1 - p)          # expected penalty on failure
            gain = es * t + epen               # reconstructed risk-weighted gain (E = (gain-epen)/t)
            noop = step.action_score == 0 and pen == 0
            dim = ("dim",) if noop else ()
            obj = step.object_annotations.__name__ if step.object_annotations is not None else "·"
            st_rows.append([
                Cell(str(i), "dim"),
                Cell(step.action_type.value, "grey" if noop else "cyan"),
                Cell(obj, *dim),
                Cell(step.location or "·", *dim),
                f"{step.action_score:.0f}",
                Cell(f"{pen:+d}" if pen else "0", *(("red",) if pen < 0 else ("dim",))),
                f"{p:.2f}",
                f"{t:.0f}s",
                f"{gain:.2f}",
                f"{epen:.2f}",
                Cell(f"{es:.3f}", *value_colors(es)),
            ])
            tot_score += step.action_score
            tot_pen += pen
            tot_time += t
            tot_gain += gain
            tot_epen += epen
        cum = exp(best)
        footer = [
            Cell("Σ", "bold"), Cell("total", "dim"), "", "",
            Cell(f"{tot_score:.0f}", "bold"),
            Cell(f"{tot_pen:+.0f}", *(("red",) if tot_pen < 0 else ("dim",))),
            "", f"{tot_time:.0f}s", f"{tot_gain:.2f}", f"{tot_epen:.2f}",
            Cell(f"{cum:.3f}", "bold", *value_colors(cum)),
        ]
        lines.append("")
        lines += render_table(
            f"TOP TASK {best.id} · STEP BREAKDOWN",
            ["#", "Action", "Object", "Location", "Score", "Pen", "p", "Time", "gain", "E[pen]", "E[step]"],
            st_rows,
            widths=[2, 8, 11, 13, 5, 4, 4, 5, 6, 6, 7],
            aligns=["^", "<", "<", "<", ">", ">", ">", ">", ">", ">", ">"],
            subtitle=f"E[score] {cum:.3f}   ·   score {best.score}   ·   p(success) {best.probability:.3f}",
            footer=footer,
        )

        # ── SECTION 3 · legend ─────────────────────────────────────────────────
        lines += [
            "",
            paint("  LEGEND", "bold"),
            paint("    E[step] = (score · pᵃ  −  |Pen| · (1−p)) / Time      a = risk aversion", "dim"),
            paint("    gain    = risk-weighted expected points   ·   E[pen] = expected penalty on failure", "dim"),
            paint("    E[score]= Σ E[step]  (task total, the ranking key)   ·   ★ = chosen task", "dim"),
        ]

        print("\n".join(lines))
