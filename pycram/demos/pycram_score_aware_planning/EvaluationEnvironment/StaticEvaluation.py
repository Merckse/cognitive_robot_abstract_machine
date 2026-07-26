"""
Multi-run harness for the static baseline, see StaticExecution.py. Runs the challenge n_runs
times and aggregates the per-run RunMetrics into an AggregateMetrics.

Each run executes in its own subprocess. This is not optional: challenge_setup() creates a new
rclpy node, a VizMarkerPublisher and a giskard executor every time it is called, and nothing in
this codebase tears any of that down. Calling it repeatedly in one process leaks all of it, so
earlier runs keep driving their own stale robot while the current one sits idle at the origin.
A fresh OS process per run is the only reliable teardown available.
"""

import logging
import multiprocessing
import sys
from pathlib import Path
from typing import Optional


def _ensure_source_roots_on_path() -> None:
    evaluation_dir = Path(__file__).resolve().parent
    demo_dir = evaluation_dir.parent                  # pycram_score_aware_planning
    repo_root = demo_dir.parents[2]                   # <repo>/pycram/demos/<demo> -> <repo>

    roots = [source_root for source_root in sorted(repo_root.glob("*/src")) if source_root.is_dir()]
    roots += [repo_root / "pycram" / "demos", demo_dir, evaluation_dir]

    for root in roots:
        path = str(root)
        if root.is_dir() and path not in sys.path:
            sys.path.insert(0, path)


_ensure_source_roots_on_path()

from evaluation_metrics import AggregateMetrics, RunMetrics
from pycram.datastructures.enums import ChallengeMode

logger = logging.getLogger(__name__)


def _run_single_evaluation(challenge_mode: ChallengeMode, random_seed: int,
                           time_budget_seconds: Optional[float],
                           result_queue: multiprocessing.Queue) -> None:
    _ensure_source_roots_on_path()

    import random

    from helper_methods import challenge_setup
    from StaticExecution import static_execution

    try:
        from MultiConditionEvaluation import _print_run_banner, _print_run_results
    except Exception:
        _print_run_banner = _print_run_results = None

    if _print_run_banner is not None:
        _print_run_banner("static", challenge_mode, random_seed)

    random.seed(random_seed)

    world, dispatcher, context = challenge_setup(challenge_mode)
    run_metrics = static_execution(dispatcher, context,
                                   time_budget_seconds=time_budget_seconds)

    if _print_run_results is not None:
        _print_run_results("static", "BASELINE", random_seed, run_metrics)

    result_queue.put(run_metrics)


def evaluate_static_execution(
    n_runs: int,
    challenge_mode: ChallengeMode = ChallengeMode.PP,
    run_timeout_seconds: float = 600.0,
    seeds: Optional[list[int]] = None,
    time_budget_seconds: Optional[float] = None,
) -> AggregateMetrics:

    mp_ctx = multiprocessing.get_context("spawn")
    runs: list[RunMetrics] = []
    run_seeds = seeds if seeds is not None else list(range(n_runs))
    variant = "static"

    for i in range(n_runs):
        seed = run_seeds[i % len(run_seeds)]
        result_queue = mp_ctx.Queue()
        process = mp_ctx.Process(
            target=_run_single_evaluation,
            args=(challenge_mode, seed, time_budget_seconds, result_queue),
        )
        process.start()
        process.join(timeout=run_timeout_seconds)

        if process.is_alive():
            logger.error(f"[{variant}] run {i} exceeded {run_timeout_seconds}s -> terminating it")
            process.terminate()
            process.join()
            run_metrics = RunMetrics(completed=False)
        elif not result_queue.empty():
            run_metrics = result_queue.get()
        else:
            logger.error(f"[{variant}] run {i} exited (code {process.exitcode}) without producing metrics -- crashed")
            run_metrics = RunMetrics(completed=False)

        result_queue.close()
        runs.append(run_metrics)
        logger.info(
            f"[{variant}] run {i} (seed {seed}): completed={run_metrics.completed} "
            f"score={run_metrics.total_score if run_metrics.completed else 'n/a'}"
        )

    return AggregateMetrics(n_runs_attempted=n_runs, runs=runs)


def report_static(aggregate: AggregateMetrics) -> str:

    lines: list[str] = []
    lines.append("!" * 62)
    lines.append("  TIME CAVEAT: the simulation is not throttled to real time -- the robot's")
    lines.append("  motions complete as fast as the machine can compute them, so every")
    lines.append("  seconds/*_per_second figure below is simulation wall-clock, NOT real robot")
    lines.append("  execution time. Report score, success rate, assist rate and navigation")
    lines.append("  distance; treat all time figures as diagnostic only and state this")
    lines.append("  limitation where any time number is used.")
    lines.append("!" * 62)
    lines.append("")
    lines.append("=" * 62)
    lines.append("  STATIC BASELINE")
    lines.append("=" * 62)
    lines.append(aggregate.summary())
    lines.append("")

    lines.append("=" * 62)
    lines.append("  SCORE OVER TIME  (mean net score accumulated by t, +/-SEM)")
    lines.append("  checkpoints at 25 / 50 / 75 / 100% of the run length")
    lines.append("=" * 62)
    series = aggregate.score_over_time(num_buckets=20)
    if not series:
        lines.append("  (no completed runs)")
    else:
        checkpoints = []
        for fraction in (0.25, 0.50, 0.75, 1.0):
            index = max(0, min(len(series) - 1, round(fraction * len(series)) - 1))
            elapsed, mean_score, sem = series[index]
            checkpoints.append(f"t={elapsed:6.1f}s: {mean_score:6.1f}+/-{sem:.1f}")
        lines.append("  " + "  |  ".join(checkpoints))

    return "\n".join(lines)


def run_and_report(
    n_runs: int = 3,
    challenge_mode: ChallengeMode = ChallengeMode.PP,
    run_timeout_seconds: float = 600.0,
    out_dir: Optional[Path] = None,
    time_budget_seconds: Optional[float] = None,
) -> Path:
    from reporting import export_all, timestamped_results_dir

    if out_dir is None:
        out_dir = timestamped_results_dir(Path(__file__).resolve().parent / "results", "static_eval")
    else:
        out_dir.mkdir(parents=True, exist_ok=True)

    aggregate = evaluate_static_execution(n_runs, challenge_mode, run_timeout_seconds,
                                          time_budget_seconds=time_budget_seconds)
    print(report_static(aggregate))

    export_all({"static": aggregate}, out_dir)

    logger.info(f"results written to {out_dir}")
    print(f"\nresults written to {out_dir}")
    return out_dir


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Run the static baseline and collect/plot results.")
    parser.add_argument("--runs", type=int, default=20, help="number of runs")
    parser.add_argument("--timeout", type=float, default=600.0, help="per-run timeout in seconds")
    parser.add_argument("--time-budget", type=float, default=None,
                        help="challenge deadline in MODELLED seconds (see time_budget.py); omit "
                             "for unbudgeted. 420/600 match the thesis's challenge lengths")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    run_and_report(n_runs=args.runs, run_timeout_seconds=args.timeout,
                   time_budget_seconds=args.time_budget)
