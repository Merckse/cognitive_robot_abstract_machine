import argparse
import logging
import multiprocessing
import sys
from pathlib import Path
from typing import Optional


def _ensure_source_roots_on_path() -> None:
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

DEFAULT_RISK_AVERSIONS = (1.0, 1.5, 2.0, 3.0)


def _label(risk_aversion: float, decline: bool, min_pickup_probability: float = 0.0) -> str:
    suffix = " +decline-neg" if decline else ""
    if min_pickup_probability > 0.0:
        suffix += f" +p-gate>={min_pickup_probability:g}"
    return f"H1 a={risk_aversion:g}" + suffix


def _run_variant(risk_aversion: float, decline: bool, challenge_mode: ChallengeMode,
                 seed: int, result_queue: multiprocessing.Queue,
                 min_pickup_probability: float = 0.0) -> None:
    _ensure_source_roots_on_path()
    import random
    from helper_methods import challenge_setup
    from EstimationExecutioner import estimation_execution

    label = _label(risk_aversion, decline, min_pickup_probability)
    print(f"\n{'=' * 78}\n  RISK-AVERSION RUN  {label}   seed={seed}\n{'=' * 78}\n", flush=True)

    random.seed(seed)
    world, dispatcher, context = challenge_setup(challenge_mode)
    _, _, run_metrics = estimation_execution(
        dispatcher, context, challenge_mode,
        use_ground_truth_belief=True, enable_exploration=False,
        risk_aversion=risk_aversion, decline_negative_ev=decline,
        min_pickup_probability=min_pickup_probability)

    try:
        from MultiConditionEvaluation import _print_run_results
        _print_run_results(label, f"a={risk_aversion:g}", seed, run_metrics)
    except Exception:
        pass

    result_queue.put(run_metrics)


def evaluate_variant(risk_aversion: float, decline: bool, n_runs: int,
                     challenge_mode: ChallengeMode = ChallengeMode.PP,
                     run_timeout_seconds: float = 600.0,
                     seeds: Optional[list[int]] = None,
                     min_pickup_probability: float = 0.0) -> AggregateMetrics:
    """One variant (a value, decline flag, probability-gate) run n_runs times, one subprocess per run."""
    mp_ctx = multiprocessing.get_context("spawn")
    runs: list[RunMetrics] = []
    run_seeds = seeds if seeds is not None else list(range(n_runs))
    for i in range(n_runs):
        seed = run_seeds[i % len(run_seeds)]
        result_queue = mp_ctx.Queue()
        process = mp_ctx.Process(target=_run_variant,
                                 args=(risk_aversion, decline, challenge_mode, seed, result_queue,
                                       min_pickup_probability))
        process.start()
        process.join(timeout=run_timeout_seconds)
        if process.is_alive():
            logger.error(f"[a={risk_aversion:g} decline={decline}] run {i} exceeded "
                         f"{run_timeout_seconds}s -> terminating")
            process.terminate()
            process.join()
            run_metrics = RunMetrics(completed=False)
        elif not result_queue.empty():
            run_metrics = result_queue.get()
        else:
            logger.error(f"[a={risk_aversion:g} decline={decline}] run {i} crashed "
                         f"(exit {process.exitcode})")
            run_metrics = RunMetrics(completed=False)
        result_queue.close()
        runs.append(run_metrics)
    return AggregateMetrics(n_runs_attempted=n_runs, runs=runs)


def _compare(results: dict) -> str:
    """Console summary: final average score plus 25/50/75/100% score-over-time checkpoints."""
    lines = ["", "=" * 62, "  RISK-AVERSION TRIAL  (final score + score-over-time)", "=" * 62,
             "  (time is diagnostic sim-seconds -- see the harness time caveat)"]
    for label, aggregate in results.items():
        avg = aggregate.average_score
        series = aggregate.score_over_time(num_buckets=20)
        checkpoints = []
        for fraction in (0.25, 0.50, 0.75, 1.0):
            if not series:
                break
            index = max(0, min(len(series) - 1, round(fraction * len(series)) - 1))
            elapsed, mean_score, sem = series[index]
            checkpoints.append(f"t={elapsed:5.1f}s:{mean_score:6.1f}")
        avg_str = "n/a" if avg is None else f"{avg:.1f}"
        lines.append(f"  {label:<24} final={avg_str:>7}   " + "  ".join(checkpoints))
    return "\n".join(lines)


_OKABE_ITO = ["#194678", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#F0E442", "#000000"]


def plot_risk_aversion(variants: list, png_path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from reporting import _style_axis, _style_figure, _NAVY_AXIS

    sweep = sorted((v for v in variants if not v[1] and "p-gate" not in v[2]), key=lambda v: v[0])
    declines = [v for v in variants if v[1] or "p-gate" in v[2]]

    fig, (ax_final, ax_time) = plt.subplots(1, 2, figsize=(16, 6.5))

    xs = [a for a, _, _, _ in sweep]
    means = [agg.score_dispersion["mean"] for _, _, _, agg in sweep]
    errs = [agg.score_dispersion["ci95_halfwidth"] or 0.0 for _, _, _, agg in sweep]
    plotted_pts = [(x, m) for x, m in zip(xs, means) if m is not None]
    if plotted_pts:
        ax_final.errorbar(xs, means, yerr=errs, marker="o", markersize=8, linewidth=2,
                          color=_NAVY_AXIS, capsize=4, zorder=3, label="H1 sweep")
        best_x, best_m = max(plotted_pts, key=lambda xm: xm[1])
        ax_final.annotate(f"best  a={best_x:g}\n{best_m:.0f}", (best_x, best_m),
                          textcoords="offset points", xytext=(8, 10), fontsize=11,
                          fontweight="bold", color=_NAVY_AXIS)
    for a, decline_flag, variant_label, agg in declines:
        d = agg.score_dispersion
        if d["mean"] is None:
            continue
        tag = "decline\nnegative-EV" if decline_flag else "pickup-p\ngate"
        ax_final.errorbar([a], [d["mean"]], yerr=[d["ci95_halfwidth"] or 0.0], marker="D",
                          markersize=10, linewidth=0, color=_OKABE_ITO[3], capsize=4, zorder=4,
                          label=variant_label)
        ax_final.annotate(tag, (a, d["mean"]), textcoords="offset points",
                          xytext=(8, -22), fontsize=9, color=_OKABE_ITO[3])
    ax_final.axhline(0, color=_NAVY_AXIS, linewidth=0.8)
    ax_final.set_title("Final score vs risk aversion  (mean ± 95% CI)", fontsize=12)
    ax_final.set_xlabel("risk aversion  a    (ranking gain = score · pᵃ)")
    ax_final.set_ylabel("final net score")
    _style_axis(ax_final)
    ax_final.legend(fontsize=9, framealpha=0.6)

    # ---- RIGHT: accumulated score over time ----
    for idx, (a, decline, label, agg) in enumerate(variants):
        series = agg.score_over_time(num_buckets=20)
        if not series:
            continue
        times = [t for t, _, _ in series]
        ys = [m for _, m, _ in series]
        sems = [s for _, _, s in series]
        color = _OKABE_ITO[idx % len(_OKABE_ITO)]
        ax_time.plot(times, ys, "--" if decline else "-", color=color, linewidth=2,
                     marker="o", markersize=3, label=label)
        ax_time.fill_between(times, [y - s for y, s in zip(ys, sems)],
                             [y + s for y, s in zip(ys, sems)], color=color, alpha=0.12)
    ax_time.axhline(0, color=_NAVY_AXIS, linewidth=0.8)
    ax_time.set_title("Accumulated score over time  (mean ± SEM)", fontsize=12)
    ax_time.set_xlabel("elapsed sim-seconds  (diagnostic, not wall clock)")
    ax_time.set_ylabel("mean net score accumulated by t")
    _style_axis(ax_time)
    ax_time.legend(fontsize=8, framealpha=0.6, loc="upper left")

    _style_figure(fig, suptitle="Risk-aversion trial (H1 ordering): which a earns most, and how score accrues")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(png_path, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def run_and_export(n_runs: int, challenge_mode: ChallengeMode, run_timeout_seconds: float,
                   risk_aversions, decline_at: Optional[float], seeds: Optional[list[int]],
                   out_dir: Optional[Path] = None, gate_at: Optional[float] = None,
                   gate_threshold: float = 0.15) -> Path:
    from reporting import export_all, timestamped_results_dir

    results: dict = {}
    variants: list = []   # (risk_aversion, decline, label, aggregate) for the dedicated analysis plot
    for risk_aversion in risk_aversions:
        label = _label(risk_aversion, False)
        aggregate = evaluate_variant(risk_aversion, False, n_runs, challenge_mode, run_timeout_seconds, seeds)
        results[label] = aggregate
        variants.append((risk_aversion, False, label, aggregate))
    if decline_at is not None:
        label = _label(decline_at, True)
        aggregate = evaluate_variant(decline_at, True, n_runs, challenge_mode, run_timeout_seconds, seeds)
        results[label] = aggregate
        variants.append((decline_at, True, label, aggregate))
    if gate_at is not None:
        label = _label(gate_at, False, gate_threshold)
        aggregate = evaluate_variant(gate_at, False, n_runs, challenge_mode, run_timeout_seconds, seeds,
                                     min_pickup_probability=gate_threshold)
        results[label] = aggregate
        variants.append((gate_at, False, label, aggregate))

    if out_dir is None:
        out_dir = timestamped_results_dir(Path(__file__).resolve().parent / "results", "risk_aversion_trial")

    export_all(results, out_dir)
    plot_risk_aversion(variants, out_dir / "risk_aversion_analysis.png")
    print(_compare(results))
    print(f"\nresults written to {out_dir}")
    print("  headline analysis: risk_aversion_analysis.png  (final-score-vs-a  +  score-over-time)")
    print("  raw trajectories:  score_over_time.png / score_over_time.csv")
    return out_dir


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Sweep risk-aversion for the H1 ordering arm and plot accumulated score over time.")
    parser.add_argument("--runs", type=int, default=3, help="runs per risk-aversion value")
    parser.add_argument("--risk-aversions", type=float, nargs="+", default=list(DEFAULT_RISK_AVERSIONS),
                        help="risk-aversion values a to sweep (clamped to (0.5, 4) by the evaluator)")
    parser.add_argument("--decline-at", type=float, default=2.0,
                        help="also run a decline-negative-EV variant at this a; pass a negative number to skip")
    parser.add_argument("--gate-at", type=float, default=None,
                        help="also run a pickup-probability-gate variant at this a (declines only tasks "
                             "whose pickup step is below --gate-threshold, unlike --decline-at's EV cutoff)")
    parser.add_argument("--gate-threshold", type=float, default=0.15,
                        help="pickup probability below which a task is declined by --gate-at")
    parser.add_argument("--seeds", type=int, nargs="+", default=None,
                        help="explicit seeds (default: 0..runs-1); same across all variants for a fair layout")
    args = parser.parse_args()

    decline_at = None if args.decline_at is not None and args.decline_at < 0 else args.decline_at
    run_and_export(args.runs, ChallengeMode.PP, 600.0,
                   args.risk_aversions, decline_at, args.seeds,
                   gate_at=args.gate_at, gate_threshold=args.gate_threshold)
