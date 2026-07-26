import contextlib
import io
import logging
import sys
import time
from copy import deepcopy
from pathlib import Path
from typing import Optional


def _ensure_source_roots_on_path() -> None:
    """See StaticEvaluation.py's helper of the same name -- identical reasoning."""
    evaluation_dir = Path(__file__).resolve().parent
    demo_dir = evaluation_dir.parent
    repo_root = demo_dir.parents[2]

    roots = [source_root for source_root in sorted(repo_root.glob("*/src")) if source_root.is_dir()]
    roots += [repo_root / "pycram", repo_root / "pycram" / "demos", demo_dir, evaluation_dir]

    for root in roots:
        path = str(root)
        if root.is_dir() and path not in sys.path:
            sys.path.insert(0, path)


_ensure_source_roots_on_path()

from Evaluate.CompositeEvaluator import CompositeEvaluator
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from Stabilizer.PlanStabilizer import PlanStabilizer
from common.cram_types import ActionType, Status, Task, TaskStep
from common.values import CHALLENGE_TASKS
from demos.pycram_score_aware_planning.Structurizer.structurizer import PlanStructurizer
from helper_methods import get_remaining_task_steps, reposition_task_list
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import ChallengeMode, PlanTransformationOperator

logger = logging.getLogger(__name__)

REPAIR_OPERATORS: list[PlanTransformationOperator] = [
    PlanTransformationOperator.SKIP,
    PlanTransformationOperator.RETRY,
    PlanTransformationOperator.SUBSTITUTE_WITH_ASSISTANCE,
]

FAILABLE_ACTION_TYPES = (ActionType.PICKUP, ActionType.PLACE, ActionType.OPEN, ActionType.THROW_AWAY)
"""Which step in a task gets the injected failure -- the first "real" manipulation step, mirroring
which steps actually raise exceptions during execution (PARK/NAVIGATE/DETECT never do)."""


def _pick_failure_step(task: Task, rng: "random.Random") -> Optional[TaskStep]:
    failable_steps = [step for step in task.task_steps if step.action_type in FAILABLE_ACTION_TYPES]
    if not failable_steps:
        return None
    return rng.choice(failable_steps)


def run_trial(
    trial_seed: int,
    context: Context,
    evaluator: CompositeEvaluator,
    structurizer: PlanStructurizer,
    stabilizer: PlanStabilizer,
    scoretime_monitor: ScoreTimeMonitor,
    challenge_mode: ChallengeMode,
) -> Optional[dict]:
    import random
    rng = random.Random(trial_seed)

    task_pool: list[Task] = deepcopy(CHALLENGE_TASKS.get(challenge_mode, []))
    if not task_pool:
        return None

    with contextlib.redirect_stdout(io.StringIO()):
        ranked_pool = evaluator.estimate(context=context, task_list=task_pool, found_objects={}, risk_aversion=1)
        ordered_pool = structurizer.structurize(task_list=ranked_pool)

    task_index = rng.randrange(len(ordered_pool))
    task = ordered_pool[task_index]
    task_id = task.id
    completed_task_ids = {completed_task.id for completed_task in ordered_pool[:task_index]}

    failure_step = _pick_failure_step(task, rng)
    if failure_step is None:
        return None
    failure_step.action_outcome = Status.FAILURE
    failure_step.action_failures += 1

    remaining_steps = get_remaining_task_steps(task)
    if not remaining_steps:
        return None

    original_identity = [stabilizer._step_identity(step) for step in remaining_steps]
    original_probability_stability = stabilizer._probability_stability(remaining_steps)

    # ---- Repair branch: only touches the failed task's own remaining steps ----
    repair_start = time.perf_counter()
    repair_candidates = []
    for operator in REPAIR_OPERATORS:
        candidate_steps = stabilizer._build_stabilized_task_list(deepcopy(remaining_steps), operator, context)
        expected_value = stabilizer._expected_value(task, candidate_steps, scoretime_monitor)
        repair_candidates.append((operator, expected_value, candidate_steps))
    best_operator, best_repair_value, best_repair_steps = max(repair_candidates, key=lambda row: row[1])
    repair_time = time.perf_counter() - repair_start

    repair_distance = stabilizer._stability_distance_to(original_identity, best_repair_steps)
    repair_probability_stability_after = stabilizer._probability_stability(best_repair_steps)
    repair_steps_considered = len(remaining_steps) * len(REPAIR_OPERATORS)

    replan_start = time.perf_counter()
    fresh_pool: list[Task] = [
        candidate_task for candidate_task in deepcopy(CHALLENGE_TASKS.get(challenge_mode, []))
        if candidate_task.id not in completed_task_ids
    ]
    with contextlib.redirect_stdout(io.StringIO()):
        replanned = evaluator.estimate(context=context, task_list=fresh_pool, found_objects={}, risk_aversion=1)
        replanned_ordered = structurizer.structurize(task_list=replanned)
    replan_time = time.perf_counter() - replan_start

    replanned_task = next(candidate for candidate in replanned_ordered if candidate.id == task_id)
    replan_steps = replanned_task.task_steps

    replan_distance = stabilizer._stability_distance_to(original_identity, replan_steps)
    replan_probability_stability_after = stabilizer._probability_stability(replan_steps)
    replan_steps_considered = sum(len(t.task_steps) for t in fresh_pool)

    original_task_template = deepcopy(next(t for t in CHALLENGE_TASKS.get(challenge_mode, [])
                                           if t.id == task_id))

    # ---- (a) single-task replan WITH evaluation: estimate() then ground ----
    single_task_start = time.perf_counter()
    with contextlib.redirect_stdout(io.StringIO()):
        single_task_scored = evaluator.estimate(
            context=context, task_list=[deepcopy(original_task_template)], found_objects={}, risk_aversion=1)
    single_task_steps = reposition_task_list(
        context=context, task_list=single_task_scored[0].task_steps, robot=context.robot)
    single_task_time = time.perf_counter() - single_task_start

    single_task_distance = stabilizer._stability_distance_to(original_identity, single_task_steps)
    single_task_probability_stability_after = stabilizer._probability_stability(single_task_steps)
    single_task_steps_considered = len(single_task_steps)

    no_eval_start = time.perf_counter()
    no_eval_steps = reposition_task_list(
        context=context, task_list=deepcopy(original_task_template.task_steps), robot=context.robot)
    no_eval_time = time.perf_counter() - no_eval_start

    no_eval_distance = stabilizer._stability_distance_to(original_identity, no_eval_steps)
    no_eval_probability_stability_after = stabilizer._probability_stability(no_eval_steps)
    no_eval_steps_considered = len(no_eval_steps)

    return {
        "trial_seed": trial_seed,
        "task_id": task.id,
        "already_completed_tasks": len(completed_task_ids),
        "failed_action_type": failure_step.action_type.value,
        "remaining_steps": len(remaining_steps),
        "original_probability_stability": original_probability_stability,
        "repair_operator": best_operator.name,
        "repair_time_seconds": repair_time,
        "repair_edit_distance": repair_distance,
        "repair_probability_stability_after": repair_probability_stability_after,
        "repair_steps_considered": repair_steps_considered,
        "replan_time_seconds": replan_time,
        "replan_edit_distance": replan_distance,
        "replan_probability_stability_after": replan_probability_stability_after,
        "replan_steps_considered": replan_steps_considered,
        "single_task_replan_time_seconds": single_task_time,
        "single_task_replan_edit_distance": single_task_distance,
        "single_task_replan_probability_stability_after": single_task_probability_stability_after,
        "single_task_replan_steps_considered": single_task_steps_considered,
        "single_task_replan_no_eval_time_seconds": no_eval_time,
        "single_task_replan_no_eval_edit_distance": no_eval_distance,
        "single_task_replan_no_eval_probability_stability_after": no_eval_probability_stability_after,
        "single_task_replan_no_eval_steps_considered": no_eval_steps_considered,
        "fresh_pool_size": len(fresh_pool),
        "repair_stability_delta": repair_probability_stability_after - original_probability_stability,
        "replan_stability_delta": replan_probability_stability_after - original_probability_stability,
        "single_task_replan_stability_delta":
            single_task_probability_stability_after - original_probability_stability,
        "single_task_replan_no_eval_stability_delta":
            no_eval_probability_stability_after - original_probability_stability,
        "repair_time_per_step": repair_time / repair_steps_considered if repair_steps_considered else 0.0,
        "replan_time_per_step": replan_time / replan_steps_considered if replan_steps_considered else 0.0,
        "single_task_replan_time_per_step":
            single_task_time / single_task_steps_considered if single_task_steps_considered else 0.0,
        "single_task_replan_no_eval_time_per_step":
            no_eval_time / no_eval_steps_considered if no_eval_steps_considered else 0.0,
    }


def run_h3_experiment(
    n_trials: int,
    challenge_mode: ChallengeMode = ChallengeMode.PP,
    seeds: Optional[list[int]] = None,
    checkpoint_path: Optional[Path] = None,
) -> list[dict]:
    from helper_methods import challenge_setup

    world, dispatcher, context = challenge_setup(challenge_mode)
    evaluator = CompositeEvaluator()
    structurizer = PlanStructurizer()
    stabilizer = PlanStabilizer(context=context)
    scoretime_monitor = ScoreTimeMonitor(challenge_mode=challenge_mode)

    trial_seeds = seeds if seeds is not None else list(range(n_trials))
    results: list[dict] = []
    checkpoint_file = open(checkpoint_path, "w") if checkpoint_path is not None else None
    try:
        for trial_seed in trial_seeds:
            try:
                result = run_trial(trial_seed, context, evaluator, structurizer, stabilizer,
                                   scoretime_monitor, challenge_mode)
            except KeyboardInterrupt:
                logger.warning(f"interrupted at trial seed {trial_seed} -> keeping {len(results)} completed trial(s)")
                break
            except Exception:
                logger.exception(f"trial seed {trial_seed} raised -> skipping it")
                result = None
            if result is not None:
                results.append(result)
                if checkpoint_file is not None:
                    import json
                    checkpoint_file.write(json.dumps(result) + "\n")
                    checkpoint_file.flush()
            logger.info(f"trial seed {trial_seed}: {'ok' if result is not None else 'skipped'}")
    finally:
        if checkpoint_file is not None:
            checkpoint_file.close()

    return results


def summarize(results: list[dict]) -> str:
    import statistics

    if not results:
        return "No trials produced a result."

    def mean(key: str) -> float:
        return statistics.mean(r[key] for r in results)

    lines = [
        f"trials: {len(results)}",
        "",
        "-- repair (R_repairing = skip/retry/substitute) --",
        f"  mean time:              {mean('repair_time_seconds'):.6f} s",
        f"  mean steps considered:  {mean('repair_steps_considered'):.1f}",
        f"  mean edit distance:     {mean('repair_edit_distance'):.2f}",
        f"  mean Stability() after: {mean('repair_probability_stability_after'):.3f}",
        "",
        "-- replan (full pool re-rank) --",
        f"  mean time:              {mean('replan_time_seconds'):.6f} s",
        f"  mean steps considered:  {mean('replan_steps_considered'):.1f}",
        f"  mean edit distance:     {mean('replan_edit_distance'):.2f}",
        f"  mean Stability() after: {mean('replan_probability_stability_after'):.3f}",
        "",
        f"mean original Stability() (before either branch): {mean('original_probability_stability'):.3f}",
        "",
        "-- single-task replan, WITH evaluation (isolates pool-size confound: SAME task, fresh "
        "re-derivation, no pool re-rank) --",
        f"  mean time:              {mean('single_task_replan_time_seconds'):.6f} s",
        f"  mean steps considered:  {mean('single_task_replan_steps_considered'):.1f}",
        f"  mean edit distance:     {mean('single_task_replan_edit_distance'):.2f}",
        f"  mean Stability() after: {mean('single_task_replan_probability_stability_after'):.3f}",
        "",
        "-- single-task replan, WITHOUT evaluation (grounds the original template directly, no "
        "estimate() call) --",
        f"  mean time:              {mean('single_task_replan_no_eval_time_seconds'):.6f} s",
        f"  mean steps considered:  {mean('single_task_replan_no_eval_steps_considered'):.1f}",
        f"  mean edit distance:     {mean('single_task_replan_no_eval_edit_distance'):.2f}",
        f"  mean Stability() after: {mean('single_task_replan_no_eval_probability_stability_after'):.3f}"
        f"  (NOT meaningful -- always ~1.0, action_probability was never estimated; see docstring)",
        "",
        "-- Hypothesis 3 verdict (repair should cost less and preserve more) --",
        f"  time ratio (repair/replan):            {mean('repair_time_seconds') / mean('replan_time_seconds'):.4f}"
        if mean('replan_time_seconds') else "  time ratio: n/a (replan_time is 0)",
        f"  steps-considered ratio (repair/replan): "
        f"{mean('repair_steps_considered') / mean('replan_steps_considered'):.4f}"
        if mean('replan_steps_considered') else "  steps-considered ratio: n/a",
        f"  edit-distance ratio (repair/replan):    "
        f"{mean('repair_edit_distance') / mean('replan_edit_distance'):.4f}"
        if mean('replan_edit_distance') else "  edit-distance ratio: n/a (replan preserved everything)",
        "",
        "-- confound isolation: repair vs. single-task replan (equal scope: one task) --",
        f"  time ratio (repair/single-task-replan):            "
        f"{mean('repair_time_seconds') / mean('single_task_replan_time_seconds'):.4f}"
        if mean('single_task_replan_time_seconds') else "  n/a",
        f"  steps-considered ratio (repair/single-task-replan): "
        f"{mean('repair_steps_considered') / mean('single_task_replan_steps_considered'):.4f}"
        if mean('single_task_replan_steps_considered') else "  n/a",
        "-- confound isolation: single-task replan vs. whole-pool replan (how much cost is pool size) --",
        f"  time ratio (single-task-replan/whole-pool-replan):            "
        f"{mean('single_task_replan_time_seconds') / mean('replan_time_seconds'):.4f}"
        if mean('replan_time_seconds') else "  n/a",
        f"  steps-considered ratio (single-task-replan/whole-pool-replan): "
        f"{mean('single_task_replan_steps_considered') / mean('replan_steps_considered'):.4f}"
        if mean('replan_steps_considered') else "  n/a",
        "",
        "-- confound isolation: single-task replan WITH eval vs. WITHOUT eval (cost of the "
        "estimate() call itself, at equal single-task scope) --",
        f"  time ratio (with-eval/without-eval):            "
        f"{mean('single_task_replan_time_seconds') / mean('single_task_replan_no_eval_time_seconds'):.4f}"
        if mean('single_task_replan_no_eval_time_seconds') else "  n/a",
        f"  steps-considered ratio (with-eval/without-eval): "
        f"{mean('single_task_replan_steps_considered') / mean('single_task_replan_no_eval_steps_considered'):.4f}"
        if mean('single_task_replan_no_eval_steps_considered') else "  n/a",
    ]
    return "\n".join(lines)


def export_csv(results: list[dict], csv_path: Path) -> None:
    import csv
    if not results:
        csv_path.write_text("")
        return
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        writer.writeheader()
        writer.writerows(results)


def export_json(results: list[dict], json_path: Path) -> None:
    import json
    import statistics

    summary = {"n_trials": len(results)}
    if results:
        numeric_keys = [k for k, v in results[0].items() if isinstance(v, (int, float)) and k != "trial_seed"]
        summary["means"] = {k: statistics.mean(r[k] for r in results) for k in numeric_keys}
        if len(results) > 1:
            summary["stdevs"] = {k: statistics.stdev(r[k] for r in results) for k in numeric_keys}
    with open(json_path, "w") as f:
        json.dump({"summary": summary, "trials": results}, f, indent=2)


def plot_results(results: list[dict], png_path: Path) -> None:
    """
    One PNG, four panels: mean time and mean steps-considered (repair vs. replan, log-scale bars
    since replan touches an order of magnitude more steps), the edit-distance comparison, and a
    per-trial time trace showing how consistent the gap is across trials, not just its average.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import statistics

    if not results:
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.text(0.5, 0.5, "No trials produced a result.", ha="center", va="center")
        ax.axis("off")
        fig.savefig(png_path, dpi=150)
        plt.close(fig)
        return

    branches = ["repair", "replan"]
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    ax = axes[0, 0]
    ax.bar(branches, [statistics.mean(r["repair_time_seconds"] for r in results),
                      statistics.mean(r["replan_time_seconds"] for r in results)],
          color=["tab:green", "tab:red"])
    ax.set_title("Mean planning time")
    ax.set_ylabel("seconds")

    ax = axes[0, 1]
    ax.bar(branches, [statistics.mean(r["repair_steps_considered"] for r in results),
                      statistics.mean(r["replan_steps_considered"] for r in results)],
          color=["tab:green", "tab:red"])
    ax.set_title("Mean steps considered\n(planning effort)")

    ax = axes[1, 0]
    ax.bar(branches, [statistics.mean(r["repair_edit_distance"] for r in results),
                      statistics.mean(r["replan_edit_distance"] for r in results)],
          color=["tab:green", "tab:red"])
    ax.set_title("Mean plan-stability edit distance\n(Fox et al., vs. the failed plan)")

    ax = axes[1, 1]
    xs = list(range(len(results)))
    ax.plot(xs, [r["repair_time_seconds"] for r in results], marker="o", label="repair", color="tab:green")
    ax.plot(xs, [r["replan_time_seconds"] for r in results], marker="o", label="replan", color="tab:red")
    ax.set_title("Planning time per trial")
    ax.set_xlabel("trial")
    ax.set_ylabel("seconds")
    ax.legend()

    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)


_NAVY = {
    "repair": "#0B2545",
    "single_task_replan_no_eval": "#1B7A6E",
    "single_task_replan": "#3D6FA6",
    "replan": "#8FB8DE",
    "baseline": "#9AA5B1",
}
_LINESTYLE = {
    "repair": "-",
    "single_task_replan_no_eval": "--",
    "single_task_replan": "-",
    "replan": "-",
}


def plot_grounded_results(results: list[dict], png_path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker
    import statistics

    if not results:
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.text(0.5, 0.5, "No trials produced a result.", ha="center", va="center")
        ax.axis("off")
        fig.savefig(png_path, dpi=150)
        plt.close(fig)
        return

    branches = ["repair", "single_task_replan_no_eval", "single_task_replan", "replan"]
    branch_labels = ["repair", "single-task replan\n(no eval)", "single-task replan\n(with eval)",
                     "whole-pool\nreplan"]
    colors = [_NAVY[b] for b in branches]

    def mean(key: str) -> float:
        return statistics.mean(r[key] for r in results)

    def _plain_decimal_log_ticks(axis) -> None:
        axis.yaxis.set_major_formatter(matplotlib.ticker.ScalarFormatter())
        axis.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        axis.ticklabel_format(axis="y", style="plain")

    fig, axes = plt.subplots(2, 3, figsize=(19, 10))

    ax = axes[0, 0]
    ax.bar(branch_labels, [mean(f"{b}_time_seconds") for b in branches], color=colors)
    ax.set_title("Mean planning time")
    ax.set_ylabel("seconds")
    ax.set_yscale("log")
    _plain_decimal_log_ticks(ax)

    ax = axes[0, 1]
    ax.bar(branch_labels, [mean(f"{b}_steps_considered") for b in branches], color=colors)
    ax.set_title("Mean steps considered\n(planning effort, not directly comparable across\n"
                 "branches -- see time-per-step panel)")
    ax.set_yscale("log")
    _plain_decimal_log_ticks(ax)

    ax = axes[0, 2]
    ax.bar(branch_labels, [mean(f"{b}_time_per_step") for b in branches], color=colors)
    ax.set_title("Mean time per step considered\n(cost efficiency of each branch's own machinery)")
    ax.set_ylabel("seconds / step")

    ax = axes[1, 0]
    deltas = [mean(f"{b}_stability_delta") for b in branches]
    bar_colors = []
    hatches = []
    for i, b in enumerate(branches):
        if b == "single_task_replan_no_eval":
            bar_colors.append("0.75")
            hatches.append("////")
        else:
            bar_colors.append(colors[i] if deltas[i] >= 0 else "#B23A48")
            hatches.append(None)
    bars = ax.bar(branch_labels, deltas, color=bar_colors)
    for bar_patch, hatch in zip(bars, hatches):
        if hatch:
            bar_patch.set_hatch(hatch)
            bar_patch.set_edgecolor("0.4")
    ax.axhline(0, color="0.3", linewidth=1)
    ax.set_title("Mean Stability() delta (after - before)\n"
                 "positive = branch made the plan more reliable\n"
                 "(hatched = not meaningful, see caption)")
    no_eval_idx = branches.index("single_task_replan_no_eval")
    ax.annotate("not real --\nStability()\nalways 1.0", (no_eval_idx, deltas[no_eval_idx]),
               textcoords="offset points", xytext=(0, -14), ha="center", va="top",
               fontsize=7.5, color="0.3")

    ax = axes[1, 1]
    xs = list(range(len(results)))
    for branch, label in zip(branches, branch_labels):
        ax.plot(xs, [r[f"{branch}_time_seconds"] for r in results], marker="o", markersize=3,
                linewidth=1.2, linestyle=_LINESTYLE[branch], label=label.replace("\n", " "),
                color=_NAVY[branch])
    ax.set_title("Planning time per trial")
    ax.set_xlabel("trial")
    ax.set_ylabel("seconds")
    ax.set_yscale("log")
    _plain_decimal_log_ticks(ax)
    ax.legend(fontsize=8, framealpha=0.6)

    ax = axes[1, 2]
    pool_sizes = [r["fresh_pool_size"] for r in results]
    replan_costs = [r["replan_steps_considered"] for r in results]
    ax.scatter(pool_sizes, replan_costs, color=_NAVY["replan"], edgecolor=_NAVY["repair"],
              linewidth=0.5, s=28, zorder=3)
    if len(set(pool_sizes)) > 1:
        slope, intercept = statistics.linear_regression(pool_sizes, replan_costs)
        xs_fit = [min(pool_sizes), max(pool_sizes)]
        ax.plot(xs_fit, [slope * x + intercept for x in xs_fit], color=_NAVY["repair"],
               linewidth=1.5, linestyle="--",
               label=f"steps considered ≈ {slope:.1f} × pool size")
        ax.legend(fontsize=8, framealpha=0.6)
    ax.set_title("Whole-pool replan cost vs. outstanding pool size\n(the direct evidence for the pool-size confound)")
    ax.set_xlabel("tasks still outstanding (fresh_pool_size)")
    ax.set_ylabel("replan steps considered")

    fig.suptitle("H3 repair vs. replan (grounded): cost and stability at equal scope",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(png_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


_ALL_BRANCH_LABELS = {
    "repair": "repair",
    "single_task_replan_no_eval": "single-task replan\n(no eval)",
    "single_task_replan": "single-task replan\n(with eval)",
    "replan": "whole-pool\nreplan",
}


def _plot_single_grounded_boxplot(results: list[dict], png_path: Path, key: str, ylabel: str,
                                  branches: list[str], branch_labels: list[str],
                                  colors: list[str]) -> None:
    import statistics
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker
    import matplotlib.patches
    import matplotlib.lines
    from reporting import _style_axis, _NAVY_AXIS

    def _nice_step(data_min: float, data_max: float, target_ticks: int = 12) -> float:
        span = data_max - data_min
        if span <= 0:
            return 1.0
        raw_step = span / target_ticks
        exponent = math.floor(math.log10(raw_step))
        fraction = raw_step / (10 ** exponent)
        for nice in (1, 2, 2.5, 5, 10):
            if fraction <= nice:
                return nice * (10 ** exponent)
        return 10 * (10 ** exponent)

    fig, ax = plt.subplots(figsize=(13, 9))
    fig.subplots_adjust(left=0.13, right=0.96, top=0.96, bottom=0.22)

    data = [[r[f"{b}_{key}"] for r in results] for b in branches]
    bplot = ax.boxplot(data, tick_labels=branch_labels, patch_artist=True, showmeans=True,
                       widths=0.55,
                       medianprops=dict(color="black", linewidth=2),
                       meanprops=dict(marker="D", markerfacecolor="white",
                                    markeredgecolor="black", markersize=9),
                       flierprops=dict(marker="o", markersize=6, markerfacecolor="0.5",
                                      markeredgecolor="none", alpha=0.5))
    for patch, color in zip(bplot["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.85)

    ax.set_ylabel(ylabel, fontsize=28)
    ax.tick_params(axis="y", labelsize=24)
    ax.tick_params(axis="x", labelsize=20)
    flat = [v for series in data for v in series]
    data_min, data_max = min(flat), max(flat)
    step = _nice_step(data_min, data_max)
    ax.yaxis.set_major_locator(matplotlib.ticker.MultipleLocator(step))
    decimals = max(0, -int(math.floor(math.log10(step))) + 1) if step < 1 else 0
    value_fmt = f"%.{decimals}f"
    ax.yaxis.set_major_formatter(matplotlib.ticker.FormatStrFormatter(value_fmt))
    _style_axis(ax)

    headroom = (data_max - data_min) * 0.12 or data_max * 0.12 or 1.0
    for i, series in enumerate(data):
        top_of_box = max(series)
        mean_value = statistics.mean(series)
        median_value = statistics.median(series)
        ax.text(i + 1, top_of_box + headroom,
                f"mean {value_fmt % mean_value}\nmed {value_fmt % median_value}",
                ha="center", va="bottom", fontsize=20, color="0.2", linespacing=1.3)
    ax.set_ylim(top=data_max + headroom * 2.8)

    branch_handles = [matplotlib.patches.Patch(facecolor=colors[i], alpha=0.85, edgecolor="0.3",
                                               label=branch_labels[i].replace("\n", " "))
                      for i in range(len(branches))]
    mark_handles = [
        matplotlib.lines.Line2D([0], [0], color="black", linewidth=2, label="median"),
        matplotlib.lines.Line2D([0], [0], marker="D", markerfacecolor="white",
                                markeredgecolor="black", linestyle="none", markersize=10, label="mean"),
        matplotlib.lines.Line2D([0], [0], marker="o", markerfacecolor="0.5", markeredgecolor="none",
                                linestyle="none", markersize=9, alpha=0.7, label="outlier trial"),
    ]
    fig.legend(handles=branch_handles + mark_handles, loc="lower center", ncol=4,
              fontsize=16, framealpha=0.7, bbox_to_anchor=(0.5, -0.04),
              handlelength=1.6, handleheight=1.2, columnspacing=1.4)

    fig.savefig(png_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_grounded_boxplots(results: list[dict], png_path: Path,
                           branches: Optional[list[str]] = None) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not results:
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.text(0.5, 0.5, "No trials produced a result.", ha="center", va="center")
        ax.axis("off")
        fig.savefig(png_path, dpi=150)
        plt.close(fig)
        return

    if branches is None:
        branches = ["repair", "single_task_replan_no_eval", "single_task_replan", "replan"]
    branch_labels = [_ALL_BRANCH_LABELS[b] for b in branches]
    colors = [_NAVY[b] for b in branches]

    time_path = png_path.with_name(f"{png_path.stem}_time{png_path.suffix}")
    steps_path = png_path.with_name(f"{png_path.stem}_steps{png_path.suffix}")
    _plot_single_grounded_boxplot(results, time_path, "time_seconds", "seconds",
                                  branches, branch_labels, colors)
    _plot_single_grounded_boxplot(results, steps_path, "steps_considered", "steps",
                                  branches, branch_labels, colors)


def plot_grounded_time_trace_and_mean(results: list[dict], png_path: Path,
                                      branches: Optional[list[str]] = None) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker
    import matplotlib.patches
    import matplotlib.lines
    import statistics
    from reporting import _style_axis, _NAVY_AXIS

    if not results:
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.text(0.5, 0.5, "No trials produced a result.", ha="center", va="center")
        ax.axis("off")
        fig.savefig(png_path, dpi=150)
        plt.close(fig)
        return

    if branches is None:
        branches = ["repair", "single_task_replan_no_eval", "single_task_replan", "replan"]
    branch_labels = [_ALL_BRANCH_LABELS[b] for b in branches]
    colors = [_NAVY[b] for b in branches]

    import math
    fig = plt.figure(figsize=(20, 11.5))
    grouped_pair = ("repair", "single_task_replan_no_eval")
    if all(b in branches for b in grouped_pair):
        row_groups = [list(grouped_pair)] + [[b] for b in branches if b not in grouped_pair]
    else:
        row_groups = [[b] for b in branches]

    gs = fig.add_gridspec(len(row_groups), 2, width_ratios=[1, 1], hspace=0.9, wspace=0.35)
    trace_axes = [fig.add_subplot(gs[i, 0]) for i in range(len(row_groups))]
    axE = fig.add_subplot(gs[:, 1])

    def _seconds_label(value, _pos):
        return f"{value:g} s"

    for i, group in enumerate(row_groups):
        ax = trace_axes[i]
        group_values = []
        for branch in group:
            color = colors[branches.index(branch)]
            ys = [r[f"{branch}_time_seconds"] for r in results]
            group_values.extend(ys)
            ax.plot(range(len(ys)), ys, color=color, linewidth=0.8, alpha=0.9,
                    label=_ALL_BRANCH_LABELS[branch].replace("\n", " "))
        data_min, data_max = min(group_values), max(group_values)
        margin = (data_max - data_min) * 0.1 or data_max * 0.1 or 1e-6
        ax.set_ylim(max(0, data_min - margin), data_max + margin)
        ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(nbins=4))
        ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(_seconds_label))
        row_title = " vs. ".join(_ALL_BRANCH_LABELS[b].replace("\n", " ") for b in group)
        ax.set_title(row_title, fontsize=13, color=_NAVY_AXIS, pad=6)
        ax.set_ylabel("planning time\n(seconds)", fontsize=10, linespacing=1.3)
        ax.tick_params(axis="both", labelsize=10)
        if len(group) > 1:
            ax.legend(fontsize=8, loc="upper right", framealpha=0.85)
        if i < len(row_groups) - 1:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel("trial index", fontsize=11)
        _style_axis(ax)
    trace_axes[0].text(0.01, 1.65, "A", transform=trace_axes[0].transAxes, ha="left", va="top",
                       fontsize=20, fontweight="bold", color=_NAVY_AXIS)

    means, errs = [], []
    for branch in branches:
        ys = [r[f"{branch}_time_seconds"] for r in results]
        means.append(statistics.mean(ys))
        if len(ys) > 1:
            sd = statistics.stdev(ys)
            errs.append(1.96 * sd / (len(ys) ** 0.5))
        else:
            errs.append(0.0)
    axE.bar(branch_labels, means, color=colors)
    axE.errorbar(branch_labels, means, yerr=errs, fmt="none",
                ecolor=_NAVY_AXIS, elinewidth=2, capsize=6, zorder=3)
    for i, m in enumerate(means):
        axE.text(i, m + max(errs) * 0.3 + max(means) * 0.02, f"{m:.4f} s",
                 ha="center", va="bottom", fontsize=11, color="0.2", fontweight="bold")
    axE.set_ylabel("mean planning time (seconds)", fontsize=12)
    axE.tick_params(axis="both", labelsize=11)
    axE.text(0.02, 0.98, "B", transform=axE.transAxes, ha="left", va="top",
             fontsize=20, fontweight="bold", color=_NAVY_AXIS)
    _style_axis(axE)

    branch_handles = [matplotlib.patches.Patch(facecolor=colors[i], alpha=0.85, edgecolor="0.3",
                                               label=branch_labels[i].replace("\n", " "))
                      for i in range(len(branches))]
    fig.legend(handles=branch_handles, loc="lower center", ncol=len(branch_handles),
              fontsize=13, framealpha=0.7, bbox_to_anchor=(0.5, -0.02))

    fig.tight_layout(rect=[0, 0.08, 1, 1])
    fig.savefig(png_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_mean_planning_time(results: list[dict], png_path: Path,
                            branches: Optional[list[str]] = None) -> None:
    import statistics
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches
    import matplotlib.lines
    from reporting import _style_axis, _NAVY_AXIS

    if not results:
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.text(0.5, 0.5, "No trials produced a result.", ha="center", va="center")
        ax.axis("off")
        fig.savefig(png_path, dpi=150)
        plt.close(fig)
        return

    if branches is None:
        branches = ["repair", "single_task_replan_no_eval", "single_task_replan", "replan"]
    branch_labels = [_ALL_BRANCH_LABELS[b] for b in branches]
    colors = [_NAVY[b] for b in branches]

    fig, ax = plt.subplots(figsize=(13, 9.8))
    fig.subplots_adjust(left=0.14, right=0.96, top=0.96, bottom=0.24)

    means, errs = [], []
    for branch in branches:
        ys = [r[f"{branch}_time_seconds"] for r in results]
        means.append(statistics.mean(ys))
        if len(ys) > 1:
            sd = statistics.stdev(ys)
            errs.append(1.96 * sd / (len(ys) ** 0.5))
        else:
            errs.append(0.0)

    ax.bar(branch_labels, means, color=colors, width=0.6)
    ax.errorbar(branch_labels, means, yerr=errs, fmt="none",
               ecolor=_NAVY_AXIS, elinewidth=2.5, capsize=8, zorder=3)
    for i, (m, e) in enumerate(zip(means, errs)):
        y = m + max(errs) * 0.3 + max(means) * 0.09
        ax.text(i, y, f"{m:.4f} s", ha="center", va="bottom", fontsize=20,
               color="0.2", fontweight="bold")
        ax.text(i, y + max(means) * 0.06, f"95% CI [{m - e:.4f}, {m + e:.4f}]",
               ha="center", va="bottom", fontsize=13, color="0.45")
    ax.set_ylim(top=max(means) * 1.32)
    ax.set_ylabel("mean planning time (seconds)", fontsize=26)
    ax.tick_params(axis="y", labelsize=22)
    ax.tick_params(axis="x", labelsize=18)
    _style_axis(ax)

    branch_handles = [matplotlib.patches.Patch(facecolor=colors[i], alpha=0.85, edgecolor="0.3",
                                               label=branch_labels[i].replace("\n", " "))
                      for i in range(len(branches))]
    error_handle = matplotlib.lines.Line2D([0], [0], color=_NAVY_AXIS, linewidth=2.5,
                                           label="95% CI")
    fig.legend(handles=branch_handles + [error_handle], loc="lower center", ncol=len(branch_handles) + 1,
              fontsize=15, framealpha=0.7, bbox_to_anchor=(0.5, 0.0),
              handlelength=1.6, handleheight=1.2, columnspacing=1.4)

    fig.savefig(png_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_time_zoom(results: list[dict], png_path: Path,
                   branches: Optional[list[str]] = None) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker
    import matplotlib.patches
    import matplotlib.lines
    import statistics

    if not results:
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.text(0.5, 0.5, "No trials produced a result.", ha="center", va="center")
        ax.axis("off")
        fig.savefig(png_path, dpi=150)
        plt.close(fig)
        return

    if branches is None:
        branches = ["repair", "single_task_replan_no_eval", "single_task_replan"]
    branch_labels = [_ALL_BRANCH_LABELS[b] for b in branches]
    colors = [_NAVY[b] for b in branches]
    data = [[r[f"{b}_time_seconds"] for r in results] for b in branches]

    replan_values = [v for b, series in zip(branches, data) if b != "repair" for v in series]
    q1 = statistics.quantiles(replan_values, n=4)[0]
    q3 = statistics.quantiles(replan_values, n=4)[2]
    iqr = q3 - q1
    zoom_low = max(0.0, q1 - 1.5 * iqr)
    zoom_high = q3 + 1.5 * iqr

    fig, (ax_full, ax_zoom) = plt.subplots(1, 2, figsize=(15, 7))

    def _draw(ax, log: bool) -> None:
        bplot = ax.boxplot(data, tick_labels=branch_labels, patch_artist=True, showmeans=True,
                           medianprops=dict(color="black", linewidth=1.4),
                           meanprops=dict(marker="D", markerfacecolor="white",
                                        markeredgecolor="black", markersize=5),
                           flierprops=dict(marker="o", markersize=3, markerfacecolor="0.5",
                                          markeredgecolor="none", alpha=0.5))
        for patch, color in zip(bplot["boxes"], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.85)
        ax.grid(True, axis="y", color="0.9", linewidth=0.8)
        ax.set_axisbelow(True)
        ax.set_ylabel("seconds")
        if log:
            ax.set_yscale("log")
            ax.yaxis.set_major_formatter(matplotlib.ticker.ScalarFormatter())
            ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
            ax.ticklabel_format(axis="y", style="plain")

    branches_text = " vs. ".join(label.replace("\n", " ") for label in branch_labels)
    excluded_low_end_note = (
        f" ({branch_labels[0].splitlines()[0]}'s low end runs off the bottom -- see left panel "
        f"for its full range)" if "repair" in branches else ""
    )

    _draw(ax_full, log=True)
    ax_full.set_title("Full distribution\n(log scale, all trials visible)")

    _draw(ax_zoom, log=False)
    ax_zoom.set_ylim(zoom_low, zoom_high)
    ax_zoom.set_title(f"Zoomed on the overlap\n(anchored on the IQR of {branches_text}{excluded_low_end_note})")

    branch_handles = [matplotlib.patches.Patch(facecolor=colors[i], alpha=0.85, edgecolor="0.3",
                                               label=branch_labels[i].replace("\n", " "))
                      for i in range(len(branches))]
    mark_handles = [
        matplotlib.lines.Line2D([0], [0], color="black", linewidth=1.4, label="median"),
        matplotlib.lines.Line2D([0], [0], marker="D", markerfacecolor="white",
                                markeredgecolor="black", linestyle="none", markersize=6, label="mean"),
        matplotlib.lines.Line2D([0], [0], marker="o", markerfacecolor="0.5", markeredgecolor="none",
                                linestyle="none", markersize=5, alpha=0.7, label="outlier trial"),
    ]
    fig.legend(handles=branch_handles + mark_handles, loc="lower center", ncol=len(branch_handles) + 3,
              fontsize=9, framealpha=0.7, bbox_to_anchor=(0.5, -0.02))

    fig.suptitle(f"Planning time: {branches_text} (n={len(results)} trials)",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0.07, 1, 0.94])
    fig.savefig(png_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _timestamped_results_dir(base_dir: Path) -> Path:
    from datetime import datetime
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir = base_dir / f"h3_repair_vs_replan_grounded_{stamp}"
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


def run_and_report(
    n_trials: int = 20,
    challenge_mode: ChallengeMode = ChallengeMode.PP,
    out_dir: Optional[Path] = None,
    seed_offset: int = 0,
) -> Path:
    if out_dir is None:
        out_dir = _timestamped_results_dir(Path(__file__).resolve().parent / "results")
    else:
        out_dir.mkdir(parents=True, exist_ok=True)

    seeds = list(range(seed_offset, seed_offset + n_trials))
    results = run_h3_experiment(n_trials, challenge_mode, seeds=seeds,
                                checkpoint_path=out_dir / "checkpoint.jsonl")
    summary_text = summarize(results)
    print(summary_text)

    export_csv(results, out_dir / "trials.csv")
    export_json(results, out_dir / "summary.json")
    plot_results(results, out_dir / "results.png")
    plot_grounded_results(results, out_dir / "grounded_analysis.png")
    (out_dir / "summary.txt").write_text(summary_text)

    print(f"\nresults written to {out_dir}")
    return out_dir


def run_seed_blocks(
    n_blocks: int = 3,
    trials_per_block: int = 20,
    challenge_mode: ChallengeMode = ChallengeMode.PP,
) -> list[Path]:
    base_dir = Path(__file__).resolve().parent / "results"
    out_dirs = []
    for block in range(n_blocks):
        out_dir = _timestamped_results_dir(base_dir)
        print(f"\n=== seed block {block} (seeds {block * trials_per_block}..{(block + 1) * trials_per_block - 1}) ===")
        run_and_report(n_trials=trials_per_block, challenge_mode=challenge_mode, out_dir=out_dir,
                       seed_offset=block * trials_per_block)
        out_dirs.append(out_dir)
    return out_dirs


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="RQ3: repair vs. full replan, plan-structure-level (no robot execution).")
    parser.add_argument("--trials", type=int, default=100, help="number of seeded trials")
    parser.add_argument("--seed-blocks", type=int, default=3,
                        help="number of disjoint seed blocks to run for a seed-sensitivity check "
                             "(each of size --trials); 1 reproduces the old single-batch behavior")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    if args.seed_blocks > 1:
        run_seed_blocks(n_blocks=args.seed_blocks, trials_per_block=args.trials)
    else:
        run_and_report(n_trials=args.trials)
