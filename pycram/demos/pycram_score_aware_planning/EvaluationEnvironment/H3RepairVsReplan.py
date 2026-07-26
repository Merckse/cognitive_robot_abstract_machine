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
from helper_methods import get_remaining_task_steps
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

    single_task_start = time.perf_counter()
    with contextlib.redirect_stdout(io.StringIO()):
        single_task_replanned = evaluator.estimate(
            context=context, task_list=[deepcopy(next(t for t in CHALLENGE_TASKS.get(challenge_mode, [])
                                                       if t.id == task_id))],
            found_objects={}, risk_aversion=1)
    single_task_time = time.perf_counter() - single_task_start
    single_task_steps = single_task_replanned[0].task_steps

    single_task_distance = stabilizer._stability_distance_to(original_identity, single_task_steps)
    single_task_probability_stability_after = stabilizer._probability_stability(single_task_steps)
    single_task_steps_considered = len(single_task_steps)

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
        "-- single-task replan (isolates pool-size confound: SAME task, fresh re-derivation, no pool re-rank) --",
        f"  mean time:              {mean('single_task_replan_time_seconds'):.6f} s",
        f"  mean steps considered:  {mean('single_task_replan_steps_considered'):.1f}",
        f"  mean edit distance:     {mean('single_task_replan_edit_distance'):.2f}",
        f"  mean Stability() after: {mean('single_task_replan_probability_stability_after'):.3f}",
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


def _timestamped_results_dir(base_dir: Path) -> Path:
    from datetime import datetime
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir = base_dir / f"h3_repair_vs_replan_{stamp}"
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
