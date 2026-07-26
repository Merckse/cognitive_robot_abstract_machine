import logging
import multiprocessing
import sys
from pathlib import Path
from typing import Optional


def _ensure_source_roots_on_path() -> None:
    """See StaticEvaluation.py's helper of the same name -- identical reasoning, kept local so
    this file's subprocess is self-sufficient regardless of how it was launched."""
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

from evaluation_metrics import AggregateMetrics, RunMetrics
from pycram.datastructures.enums import ChallengeMode

logger = logging.getLogger(__name__)

CONDITIONS = (
    "static",
    "H1 (ordering)",
    "H1-prior (ordering+prior guess)",
    "H1+H2 (ordering+uncertainty clearance)",
    "H1+H3 (ordering+repair)",
)

REPAIR_SPLIT_CONDITIONS = (
    "H1+H3 (reposition)",
    "H1+H3 (defer)",
    "H1+H3 (repair combined)",
)


CONDITION_INFO: dict[str, tuple[str, str]] = {
    "static":                                    ("BASELINE",        "fixed sweep order, no estimation, no repair"),
    "static-random (no designed order)":         ("BASELINE ctrl",   "sweep + object order shuffled per run: baseline with the designer's ordering removed"),
    "H1 (ordering)":                             ("RQ1 · H1",        "expected-value ordering; oracle localization (no search)"),
    "H1-prior (ordering+prior guess)":           ("H2 baseline",     "ordering; uncertain tasks GUESS the room's first surface, no search"),
    "H1+H2 (ordering+uncertainty clearance)":    ("RQ3 · H1+H2",     "ordering + belief-driven exploration (search to clear uncertainty)"),
    "H1+H3 (ordering+repair)":                   ("RQ2 · H1+H3",     "ordering + PlanStabilizer repair on failure (full operator set)"),
    "H1+H3 (reposition)":                        ("H3 split · reposition", "repair = reposition robot + retry (unassisted)"),
    "H1+H3 (defer)":                             ("H3 split · defer",      "repair = defer & re-estimate (unassisted)"),
    "H1+H3 (repair combined)":                   ("H3 split · combined",   "repair = reposition + defer (unassisted)"),
}


def _print_run_banner(condition: str, challenge_mode: ChallengeMode, seed: int) -> None:
    """Loud, unmistakable marker of which condition/version this run is, before the sim output."""
    tag, description = CONDITION_INFO.get(condition, ("?", "unknown condition"))
    line = "=" * 78
    print(f"\n{line}", flush=True)
    print(f"  RUN CONDITION : {condition}", flush=True)
    print(f"  VERSION       : {tag}", flush=True)
    print(f"  WHAT IT TESTS : {description}", flush=True)
    print(f"  challenge={challenge_mode.name if hasattr(challenge_mode, 'name') else challenge_mode}   seed={seed}", flush=True)
    print(f"{line}\n", flush=True)


def _run_single_condition(condition: str, challenge_mode: ChallengeMode, random_seed: int,
                          time_budget_seconds: Optional[float],
                          result_queue: multiprocessing.Queue) -> None:
    """
    Entry point of the per-run subprocess. Imported lazily so importing this file in the parent
    process never itself triggers a challenge_setup() call.
    """
    _ensure_source_roots_on_path()

    _print_run_banner(condition, challenge_mode, random_seed)

    import random
    from helper_methods import challenge_setup

    random.seed(random_seed)
    world, dispatcher, context = challenge_setup(challenge_mode)

    if condition == "static":
        from StaticExecution import static_execution
        run_metrics = static_execution(dispatcher, context,
                                       time_budget_seconds=time_budget_seconds)
    elif condition == "static-random (no designed order)":
        from StaticExecution import static_execution
        run_metrics = static_execution(dispatcher, context, randomize_order=True,
                                       time_budget_seconds=time_budget_seconds)
    elif condition == "H1 (ordering)":
        from EstimationExecutioner import estimation_execution
        _, _, run_metrics = estimation_execution(
            dispatcher, context, challenge_mode,
            use_ground_truth_belief=True, enable_exploration=False,
            time_budget_seconds=time_budget_seconds)
    elif condition == "H1-prior (ordering+prior guess)":
        from EstimationExecutioner import estimation_execution
        _, _, run_metrics = estimation_execution(
            dispatcher, context, challenge_mode,
            use_ground_truth_belief=False, enable_exploration=False, use_prior_guess=True,
            time_budget_seconds=time_budget_seconds)
    elif condition == "H1+H2 (ordering+uncertainty clearance)":
        from EstimationExecutioner import estimation_execution
        _, _, run_metrics = estimation_execution(
            dispatcher, context, challenge_mode,
            use_ground_truth_belief=False, enable_exploration=True,
            time_budget_seconds=time_budget_seconds)
    elif condition == "H1+H3 (ordering+repair)":
        from ScoreAwareExecutioner import score_aware_execution
        _, _, run_metrics = score_aware_execution(
            dispatcher, context, challenge_mode, enable_exploration=False,
            time_budget_seconds=time_budget_seconds)
    elif condition == "H1+H3 (reposition)":
        from ScoreAwareExecutioner import score_aware_execution
        from pycram.datastructures.enums import PlanTransformationOperator
        _, _, run_metrics = score_aware_execution(
            dispatcher, context, challenge_mode, enable_exploration=False,
            enabled_repair_operators={PlanTransformationOperator.REPOSITION_ROBOT,
                                      PlanTransformationOperator.RETRY},
            time_budget_seconds=time_budget_seconds)
    elif condition == "H1+H3 (defer)":
        from ScoreAwareExecutioner import score_aware_execution
        from pycram.datastructures.enums import PlanTransformationOperator
        _, _, run_metrics = score_aware_execution(
            dispatcher, context, challenge_mode, enable_exploration=False,
            enabled_repair_operators={PlanTransformationOperator.REORDER,
                                      PlanTransformationOperator.RETRY},
            time_budget_seconds=time_budget_seconds)
    elif condition == "H1+H3 (repair combined)":
        from ScoreAwareExecutioner import score_aware_execution
        from pycram.datastructures.enums import PlanTransformationOperator
        _, _, run_metrics = score_aware_execution(
            dispatcher, context, challenge_mode, enable_exploration=False,
            enabled_repair_operators={PlanTransformationOperator.REPOSITION_ROBOT,
                                      PlanTransformationOperator.REORDER,
                                      PlanTransformationOperator.RETRY},
            time_budget_seconds=time_budget_seconds)
    else:
        raise ValueError(f"unknown condition {condition!r}")

    tag, _ = CONDITION_INFO.get(condition, ("?", ""))
    _print_run_results(condition, tag, random_seed, run_metrics)

    result_queue.put(run_metrics)


def _fmt(value, suffix: str = "") -> str:
    return "n/a" if value is None else f"{value:.3f}{suffix}"


def _print_run_results(condition: str, tag: str, seed: int, run_metrics) -> None:
    """End-of-run dump of this single run's results: headline metrics plus one line per task
    attempt (object, pickup/place outcome, its net score), so each run is legible on its own
    without waiting for the cross-run aggregate."""
    line = "-" * 78
    print(f"\n{line}", flush=True)
    print(f"  RUN RESULTS: {condition}  [{tag}]  seed={seed}", flush=True)
    print(f"{line}", flush=True)
    started = getattr(run_metrics, "tasks_started", 0)
    print(f"  score={getattr(run_metrics, 'total_score', 'n/a')}   "
          f"tasks {getattr(run_metrics, 'tasks_succeeded', 0)}/{started} succeeded   "
          f"success_rate={_fmt(getattr(run_metrics, 'success_rate', None))}   "
          f"assist_rate={_fmt(getattr(run_metrics, 'assist_rate', None))}   "
          f"nav={_fmt(getattr(run_metrics, 'navigation_distance_meters', None), ' m')}", flush=True)
    for attempt in getattr(run_metrics, "task_attempts", []):
        pickup = getattr(attempt.pickup_status, "name", attempt.pickup_status)
        place = getattr(attempt.place_status, "name", attempt.place_status)
        net = attempt.score + attempt.penalty
        flags = " assisted" if getattr(attempt, "assisted", False) else ""
        reason = f"  ({attempt.failure_reason})" if getattr(attempt, "failure_reason", None) else ""
        print(f"    - {attempt.object_type or attempt.object_name:<14} "
              f"pickup={pickup:<11} place={place:<11} net={net:+d}{flags}{reason}", flush=True)
    print(f"{line}\n", flush=True)


def evaluate_condition(
    condition: str,
    n_runs: int,
    challenge_mode: ChallengeMode = ChallengeMode.PP,
    run_timeout_seconds: float = 600.0,
    seeds: Optional[list[int]] = None,
    time_budget_seconds: Optional[float] = None,
) -> AggregateMetrics:
    """Run one condition n_runs times and aggregate -- same subprocess-per-run pattern as
    StaticEvaluation.evaluate_static_execution()."""
    mp_ctx = multiprocessing.get_context("spawn")
    runs: list[RunMetrics] = []
    run_seeds = seeds if seeds is not None else list(range(n_runs))

    for i in range(n_runs):
        seed = run_seeds[i % len(run_seeds)]
        result_queue = mp_ctx.Queue()
        process = mp_ctx.Process(
            target=_run_single_condition,
            args=(condition, challenge_mode, seed, time_budget_seconds, result_queue),
        )
        process.start()
        process.join(timeout=run_timeout_seconds)

        if process.is_alive():
            logger.error(f"[{condition}] run {i} exceeded {run_timeout_seconds}s -> terminating it")
            process.terminate()
            process.join()
            run_metrics = RunMetrics(completed=False)
        elif not result_queue.empty():
            run_metrics = result_queue.get()
        else:
            logger.error(f"[{condition}] run {i} exited (code {process.exitcode}) without producing metrics -- crashed")
            run_metrics = RunMetrics(completed=False)

        result_queue.close()
        runs.append(run_metrics)
        logger.info(
            f"[{condition}] run {i} (seed {seed}): completed={run_metrics.completed} "
            f"score={run_metrics.total_score if run_metrics.completed else 'n/a'}"
        )

    return AggregateMetrics(n_runs_attempted=n_runs, runs=runs)


def evaluate_all_conditions(
    n_runs_per_condition: int = 3,
    challenge_mode: ChallengeMode = ChallengeMode.PP,
    run_timeout_seconds: float = 600.0,
    conditions: tuple = CONDITIONS,
    time_budget_seconds: Optional[float] = None,
) -> dict:
    seeds = list(range(n_runs_per_condition))
    return {
        condition: evaluate_condition(condition, n_runs_per_condition, challenge_mode,
                                      run_timeout_seconds, seeds=seeds,
                                      time_budget_seconds=time_budget_seconds)
        for condition in conditions
    }


def compare_conditions(results: dict) -> str:
    lines: list[str] = []
    lines.append("!" * 62)
    lines.append("  TIME CAVEAT: the simulation is not throttled to real time -- the robot's")
    lines.append("  motions complete as fast as the machine can compute them, so every")
    lines.append("  seconds/*_per_second figure below is simulation wall-clock, NOT real robot")
    lines.append("  execution time. Report score, success rate, assist rate, operator usage,")
    lines.append("  navigation distance and plan-stability; treat all time figures as diagnostic")
    lines.append("  only and state this limitation where any time number is used.")
    lines.append("!" * 62)
    lines.append("")
    for condition, aggregate in results.items():
        lines.append("=" * 62)
        lines.append(f"  {condition.upper()}")
        lines.append("=" * 62)
        lines.append(aggregate.summary())
        lines.append("")

    lines.append("=" * 62)
    lines.append("  SCORE OVER TIME  (mean net score accumulated by t, ±SEM)")
    lines.append("  checkpoints at 25 / 50 / 75 / 100% of each condition's run length")
    lines.append("=" * 62)
    for condition, aggregate in results.items():
        series = aggregate.score_over_time(num_buckets=20)
        if not series:
            lines.append(f"  {condition:<45} (no completed runs)")
            continue
        checkpoints = []
        for fraction in (0.25, 0.50, 0.75, 1.0):
            index = max(0, min(len(series) - 1, round(fraction * len(series)) - 1))
            elapsed, mean_score, sem = series[index]
            checkpoints.append(f"t={elapsed:6.1f}s: {mean_score:6.1f}±{sem:.1f}")
        lines.append(f"  {condition:<45} " + "  |  ".join(checkpoints))
    lines.append("")

    static = results.get("static")
    h1 = results.get("H1 (ordering)")
    h1_prior = results.get("H1-prior (ordering+prior guess)")
    h1_h2 = results.get("H1+H2 (ordering+uncertainty clearance)")
    h1_h3 = results.get("H1+H3 (ordering+repair)")

    def delta(a: Optional[AggregateMetrics], b: Optional[AggregateMetrics], attr: str) -> str:
        if a is None or b is None:
            return "n/a"
        a_value, b_value = getattr(a, attr), getattr(b, attr)
        if a_value is None or b_value is None:
            return "n/a"
        return f"{b_value - a_value:+.3f}"

    if static is not None and h1 is not None:
        lines.append("=" * 62)
        lines.append("  H1 minus STATIC  (what ordering alone buys -- RQ1)")
        lines.append("=" * 62)
        lines.append(f"score:        {delta(static, h1, 'average_score')}")
        lines.append(f"success rate: {delta(static, h1, 'average_success_rate')}")
        lines.append(f"score/second: {delta(static, h1, 'average_score_per_second')}")
        lines.append("")

    if h1 is not None and h1_h2 is not None:
        lines.append("=" * 62)
        lines.append("  H1+H2 minus H1 (oracle)  (search vs a baseline that already knows -- expected ~0)")
        lines.append("=" * 62)
        lines.append("  H1 uses oracle localization, so it pays no uncertainty cost; this delta is")
        lines.append("  near-zero BY DESIGN and is NOT the H2 result. See the next block.")
        lines.append(f"score:        {delta(h1, h1_h2, 'average_score')}")
        lines.append(f"success rate: {delta(h1, h1_h2, 'average_success_rate')}")
        lines.append(f"navigation distance: {delta(h1, h1_h2, 'average_navigation_distance_meters')}")
        lines.append("")

    if h1_prior is not None and h1_h2 is not None:
        lines.append("=" * 62)
        lines.append("  H1+H2 minus H1-prior  (THE H2 TEST: search vs guess, both without oracle)")
        lines.append("=" * 62)
        lines.append("  Both resolve uncertainty without ground truth; H1-prior guesses the room's")
        lines.append("  first surface, H1+H2 searches. The gap on the uncertain subset is the value")
        lines.append("  of clearing uncertainty. NOTE: needs many seeds -- the effect depends on")
        lines.append("  whether objects spawn on the guessed (primary) surface, which the seed sets;")
        lines.append("  and read pickup failures together with failures-by-reason, since H1+H2's")
        lines.append("  uncertain-task failures can be grasp failures, not wrong-surface ones.")
        lines.append(f"score:                              {delta(h1_prior, h1_h2, 'average_score')}")
        lines.append(f"success rate (all tasks):           {delta(h1_prior, h1_h2, 'average_success_rate')}")
        lines.append(f"pickup success rate (uncertain only): {delta(h1_prior, h1_h2, 'uncertain_task_pickup_success_rate')}")
        lines.append(f"full-task success (uncertain only):   {delta(h1_prior, h1_h2, 'uncertain_task_place_success_rate')}")
        prior_fails = h1_prior.uncertain_task_pickup_failures
        h2_fails = h1_h2.uncertain_task_pickup_failures
        lines.append(f"pickup failures (uncertain only):   H1-prior {prior_fails} -> H1+H2 {h2_fails} "
                     f"({h2_fails - prior_fails:+d})")
        lines.append("")

    if h1 is not None and h1_h3 is not None:
        lines.append("=" * 62)
        lines.append("  H1+H3 minus H1  (what repair alone buys on top of ordering -- RQ2/H3)")
        lines.append("=" * 62)
        lines.append(f"score:        {delta(h1, h1_h3, 'average_score')}")
        lines.append(f"success rate: {delta(h1, h1_h3, 'average_success_rate')}")
        lines.append(f"success-after-error rate: {delta(h1, h1_h3, 'average_success_after_error_rate')}")
        operator_counts = h1_h3.operator_usage_counts
        lines.append(f"restructuring operators used: {operator_counts or 'none'}")
        lines.append(f"  layer A (human support: reposition/substitute/replan-with-assistance): "
                     f"{h1_h3.human_support_operator_counts or 'none'}")
        lines.append(f"  layer B (autonomous repair: skip/retry/replan/reorder): "
                     f"{h1_h3.autonomous_repair_operator_counts or 'none'}")
        human_rate = h1_h3.human_support_repair_rate
        autonomous_rate = h1_h3.autonomous_repair_rate
        lines.append(f"  human-support repair rate: {human_rate:.3f}" if human_rate is not None else "  human-support repair rate: n/a")
        lines.append(f"  autonomous repair rate: {autonomous_rate:.3f}" if autonomous_rate is not None else "  autonomous repair rate: n/a")
        lines.append(f"average plan-stability distance per repair: "
                     f"{h1_h3.average_plan_stability_distance if h1_h3.average_plan_stability_distance is not None else 'n/a'}")

    return "\n".join(lines)


def run_and_report(
    n_runs_per_condition: int = 3,
    challenge_mode: ChallengeMode = ChallengeMode.PP,
    run_timeout_seconds: float = 600.0,
    conditions: tuple = CONDITIONS,
    out_dir: Optional[Path] = None,
    time_budget_seconds: Optional[float] = None,
) -> Path:
    from reporting import export_all, timestamped_results_dir

    if out_dir is None:
        out_dir = timestamped_results_dir(Path(__file__).resolve().parent / "results", "conditions_eval")
    else:
        out_dir.mkdir(parents=True, exist_ok=True)

    results = evaluate_all_conditions(n_runs_per_condition, challenge_mode, run_timeout_seconds,
                                      conditions, time_budget_seconds=time_budget_seconds)
    print(compare_conditions(results))

    export_all(results, out_dir)

    logger.info(f"results written to {out_dir}")
    print(f"\nresults written to {out_dir}")
    return out_dir


if __name__ == "__main__":
    import argparse


    parser = argparse.ArgumentParser(
        description="Run static / H1 / H1+H2 / H1+H3 and collect/plot the comparison.")
    parser.add_argument("--runs", type=int, default=5,
                        help="runs per condition (default 20 -- the thesis target; each run is its "
                             "own subprocess, so a full 4-condition sweep is ~4x20 runs)")
    parser.add_argument("--timeout", type=float, default=600.0, help="per-run timeout in seconds")
    parser.add_argument("--time-budget", type=float, default=None,
                        help="challenge deadline in MODELLED seconds (see time_budget.py); "
                             "omit for the unbudgeted behaviour. 420/600 match the thesis's "
                             "seven- and ten-minute challenges")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    run_and_report(n_runs_per_condition=args.runs, run_timeout_seconds=args.timeout,
                   time_budget_seconds=args.time_budget)
