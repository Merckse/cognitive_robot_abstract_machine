"""
Shared export and plot layer for the multi-run evaluation harnesses. Pulled out of the static
harness so a second one does not have to duplicate the same CSV/JSON/plot code. Every condition
is a dict[str, AggregateMetrics] keyed by condition name, and everything here is written
generically against that shape.
"""

from pathlib import Path
from typing import Optional


def timestamped_results_dir(base_dir: Path, prefix: str) -> Path:
    from datetime import datetime
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir = base_dir / f"{prefix}_{stamp}"
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


def export_task_attempts_csv(results: dict, csv_path: Path) -> None:
    """One row per task attempt across every run of every condition -- the raw data behind the summaries."""
    import csv
    # object_type, was_uncertain, is_trash and error_during_execution are the fields the derived
    # aggregates actually group by (sheet categories, the H2 uncertain subset, trash vs place,
    # success-after-error). Leaving them out of the raw export made every one of those numbers
    # impossible to re-derive or check from the CSV that is supposed to be the raw data behind it.
    fieldnames = [
        "condition", "run_index", "completed", "object_name", "object_type",
        "pickup_location", "place_location", "pickup_status", "place_status",
        "assisted", "was_uncertain", "is_trash", "error_during_execution",
        "score", "penalty", "duration_seconds", "failure_reason",
    ]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for condition, aggregate in results.items():
            for run_index, run in enumerate(aggregate.runs):
                for attempt in run.task_attempts:
                    writer.writerow({
                        "condition": condition,
                        "run_index": run_index,
                        "completed": run.completed,
                        "object_name": attempt.object_name,
                        "object_type": attempt.object_type,
                        "pickup_location": attempt.pickup_location,
                        "place_location": attempt.place_location,
                        "pickup_status": attempt.pickup_status.name,
                        "place_status": attempt.place_status.name,
                        "assisted": attempt.assisted,
                        "was_uncertain": attempt.was_uncertain,
                        "is_trash": attempt.is_trash,
                        "error_during_execution": attempt.error_during_execution,
                        "score": attempt.score,
                        "penalty": attempt.penalty,
                        "duration_seconds": attempt.duration_seconds,
                        "failure_reason": attempt.failure_reason or "",
                    })


def export_run_summary_csv(results: dict, csv_path: Path) -> None:
    """One row per run: enough to plot score/success-rate/navigation trends without re-parsing
    the attempt CSV."""
    import csv
    fieldnames = ["condition", "run_index", "completed", "total_score",
                  "gross_score", "total_penalty",
                  "tasks_started", "tasks_succeeded", "success_rate", "success_after_error_rate",
                  "pickup_failures", "place_failures",
                  "run_duration_seconds", "modelled_seconds_spent", "time_budget_seconds",
                  "tasks_skipped_over_budget",
                  "score_per_second", "success_rate_per_second",
                  "navigation_distance_meters", "navigation_distance_per_task",
                  "average_task_duration_seconds", "assist_rate"]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for condition, aggregate in results.items():
            for run_index, run in enumerate(aggregate.runs):
                writer.writerow({
                    "condition": condition,
                    "run_index": run_index,
                    "completed": run.completed,
                    "total_score": run.total_score if run.completed else "",
                    "gross_score": run.gross_score if run.completed else "",
                    "total_penalty": run.total_penalty if run.completed else "",
                    "tasks_started": run.tasks_started if run.completed else "",
                    "tasks_succeeded": run.tasks_succeeded if run.completed else "",
                    "success_rate": run.success_rate if run.completed else "",
                    "success_after_error_rate": run.success_after_error_rate if run.completed else "",
                    "pickup_failures": run.pickup_failures if run.completed else "",
                    "place_failures": run.place_failures if run.completed else "",
                    "run_duration_seconds": run.run_duration_seconds if run.completed else "",
                    "modelled_seconds_spent": run.modelled_seconds_spent if run.completed else "",
                    "time_budget_seconds": run.time_budget_seconds if run.completed else "",
                    "tasks_skipped_over_budget": run.tasks_skipped_over_budget if run.completed else "",
                    "score_per_second": run.score_per_second if run.completed else "",
                    "success_rate_per_second": run.success_rate_per_second if run.completed else "",
                    "navigation_distance_meters": run.navigation_distance_meters if run.completed else "",
                    "navigation_distance_per_task": run.navigation_distance_per_task if run.completed else "",
                    "average_task_duration_seconds": run.average_task_duration_seconds if run.completed else "",
                    "assist_rate": run.assist_rate if run.completed else "",
                })


_STATS_FIELDS = ["attempts", "pickup_attempts", "pickup_succeeded", "pickup_success_rate",
                 "place_attempts", "place_succeeded", "place_success_rate",
                 "total_score", "total_penalty"]


def export_per_object_csv(results: dict, csv_path: Path) -> None:
    """
    One row per (condition, object) -- pickup and place success tracked separately, since a
    place never even runs when its pickup failed. Derived from AggregateMetrics.per_object_stats.
    """
    import csv
    fieldnames = ["condition", "object_name"] + _STATS_FIELDS
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for condition, aggregate in results.items():
            for object_name, entry in aggregate.per_object_stats.items():
                writer.writerow({"condition": condition, "object_name": object_name, **entry})


def export_category_csv(results: dict, csv_path: Path) -> None:
    """
    One row per (condition, phase, sheet category) -- phase is "pickup" or "place". Derived from
    AggregateMetrics.pickup_category_stats / place_category_stats.
    """
    import csv
    fieldnames = ["condition", "phase", "category"] + _STATS_FIELDS
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for condition, aggregate in results.items():
            for category, entry in aggregate.pickup_category_stats.items():
                writer.writerow({"condition": condition, "phase": "pickup", "category": category, **entry})
            for category, entry in aggregate.place_category_stats.items():
                writer.writerow({"condition": condition, "phase": "place", "category": category, **entry})


def export_operator_usage_csv(results: dict, csv_path: Path) -> None:
    """
    One row per (condition, operator) -- how often each restructuring operator won stabilize()'s
    selection. Empty for conditions with no repair layer (static baseline, H1-only).
    """
    import csv
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["condition", "operator", "count"])
        writer.writeheader()
        for condition, aggregate in results.items():
            for operator_name, count in aggregate.operator_usage_counts.items():
                writer.writerow({"condition": condition, "operator": operator_name, "count": count})


def export_score_over_time_csv(results: dict, csv_path: Path, num_buckets: int = 20) -> None:
    """
    One row per (condition, elapsed_seconds) giving the average net score accumulated across that
    condition's completed runs by that time, with its standard error. Lets you compare score
    *trajectories* -- whether a condition front-loads points then stalls -- not just final totals.
    Times are fake sim seconds (see RunMetrics.score_timeline).
    """
    import csv
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["condition", "elapsed_seconds", "mean_score", "sem"])
        writer.writeheader()
        for condition, aggregate in results.items():
            for elapsed, mean_score, sem in aggregate.score_over_time(num_buckets=num_buckets):
                writer.writerow({
                    "condition": condition,
                    "elapsed_seconds": round(elapsed, 3),
                    "mean_score": round(mean_score, 3),
                    "sem": round(sem, 3),
                })


_NAVY_AXIS = "#194678"
_THESIS_FIG_BG = "#EDF0F4"   # ~thesisblue!8!white
_NAVY_PALETTE = [_NAVY_AXIS, "#1B7A6E", "#6D93BE", "#CC79A7", "#E69F00", "#D55E00", "#3D6FA6", "#9AA5B1"]


def _condition_colors(conditions: list) -> dict:
    return {condition: _NAVY_PALETTE[i % len(_NAVY_PALETTE)] for i, condition in enumerate(conditions)}


def _style_axis(ax) -> None:
    """Thesis figure-frame look: light-grey y-grid behind the data, axis spines/ticks and the
    title recoloured thesisblue instead of matplotlib's defaults, white plot area."""
    ax.set_facecolor("white")
    ax.grid(True, axis="y", color="0.9", linewidth=0.8)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_color(_NAVY_AXIS)
    ax.tick_params(colors=_NAVY_AXIS)
    ax.title.set_color(_NAVY_AXIS)
    ax.title.set_fontweight("bold")


def _style_figure(fig, suptitle: Optional[str] = None) -> None:
    """Approximates the thesisFigBox card these PNGs get embedded into (Thesis/main_end (1).tex):
    a thesisblue!8!white wash behind the panels and a bold thesisblue rounded border, so the
    PNG itself already carries the frame instead of relying on the LaTeX wrapper around it."""
    fig.patch.set_facecolor(_THESIS_FIG_BG)
    fig.patch.set_edgecolor(_NAVY_AXIS)
    fig.patch.set_linewidth(2.5)
    if suptitle:
        fig.suptitle(suptitle, fontsize=14, fontweight="bold", color=_NAVY_AXIS)


def plot_score_over_time(results: dict, png_path: Path, num_buckets: int = 20) -> None:
    """One line per condition: average accumulated net score vs elapsed sim-time, with a ±SEM band."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = _condition_colors(list(results.keys()))
    fig, ax = plt.subplots(figsize=(11, 7))
    plotted = False
    for condition, aggregate in results.items():
        series = aggregate.score_over_time(num_buckets=num_buckets)
        if not series:
            continue
        xs = [elapsed for elapsed, _, _ in series]
        ys = [mean_score for _, mean_score, _ in series]
        sems = [sem for _, _, sem in series]
        color = colors[condition]
        line, = ax.plot(xs, ys, marker="o", markersize=3, color=color, label=condition)
        lower = [y - s for y, s in zip(ys, sems)]
        upper = [y + s for y, s in zip(ys, sems)]
        ax.fill_between(xs, lower, upper, alpha=0.15, color=color)
        plotted = True

    ax.set_title("Accumulated score over time (avg across runs, ±SEM)")
    ax.set_xlabel("elapsed sim-seconds (diagnostic, not wall clock)")
    ax.set_ylabel("mean net score accumulated by t")
    ax.axhline(0, color=_NAVY_AXIS, linewidth=0.8)
    _style_axis(ax)
    if plotted:
        # Outside the axes on the right so it never covers the curves.
        ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), fontsize=8, framealpha=0.9)
    _style_figure(fig)
    fig.tight_layout()
    fig.savefig(png_path, dpi=120, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def export_summary_json(results: dict, json_path: Path) -> None:
    import json
    summary = {}
    for condition, aggregate in results.items():
        summary[condition] = {
            "n_runs_attempted": aggregate.n_runs_attempted,
            "finished_executions": aggregate.number_of_finished_executions,
            "average_score": aggregate.average_score,
            "average_gross_score": aggregate.average_gross_score,
            "average_penalty": aggregate.average_penalty,
            "score_dispersion": aggregate.score_dispersion,
            "penalty_dispersion": aggregate.penalty_dispersion,
            "success_rate_dispersion": aggregate.success_rate_dispersion,
            "average_navigation_distance_meters": aggregate.average_navigation_distance_meters,
            "average_success_rate": aggregate.average_success_rate,
            "average_success_after_error_rate": aggregate.average_success_after_error_rate,
            "average_time_spent_per_task_seconds": aggregate.average_time_spent_per_task_seconds,
            "average_score_per_second": aggregate.average_score_per_second,
            "average_success_rate_per_second": aggregate.average_success_rate_per_second,
            "average_navigation_distance_per_task": aggregate.average_navigation_distance_per_task,
            "average_assist_rate": aggregate.average_assist_rate,
            "total_pickup_failures": aggregate.total_pickup_failures,
            "total_place_failures": aggregate.total_place_failures,
            "uncertain_task_count": len(aggregate.uncertain_task_attempts),
            "uncertain_task_pickup_success_rate": aggregate.uncertain_task_pickup_success_rate,
            "uncertain_task_place_success_rate": aggregate.uncertain_task_place_success_rate,
            "uncertain_task_pickup_failures": aggregate.uncertain_task_pickup_failures,
            "failure_reason_counts": aggregate.failure_reason_counts,
            "per_object_stats": aggregate.per_object_stats,
            "pickup_category_stats": aggregate.pickup_category_stats,
            "place_category_stats": aggregate.place_category_stats,
            "operator_usage_counts": aggregate.operator_usage_counts,
            "total_repairs_applied": aggregate.total_repairs_applied,
            "human_support_operator_counts": aggregate.human_support_operator_counts,
            "autonomous_repair_operator_counts": aggregate.autonomous_repair_operator_counts,
            "human_support_repair_rate": aggregate.human_support_repair_rate,
            "autonomous_repair_rate": aggregate.autonomous_repair_rate,
            "average_plan_stability_distance": aggregate.average_plan_stability_distance,
            "average_probability_stability_before": aggregate.average_probability_stability_before,
            "average_probability_stability_after": aggregate.average_probability_stability_after,
        }
    with open(json_path, "w") as f:
        json.dump(summary, f, indent=2)


def _grouped_bar(ax, categories: list, results: dict, conditions: list, value_of) -> None:
    """Shared grouped-bar-chart layout: one cluster per category, one bar per condition inside it."""
    colors = _condition_colors(conditions)
    bar_width = 0.8 / max(len(conditions), 1)
    for condition_index, condition in enumerate(conditions):
        values = [value_of(results[condition], name) for name in categories]
        positions = [i + condition_index * bar_width for i in range(len(categories))]
        ax.bar(positions, values, width=bar_width, color=colors[condition], label=condition)
    ax.set_xticks([i + bar_width * (len(conditions) - 1) / 2 for i in range(len(categories))])
    ax.set_xticklabels(categories, rotation=45, ha="right")
    ax.legend(fontsize=7, framealpha=0.5, loc="best")
    _style_axis(ax)


def plot_results(results: dict, png_path: Path) -> None:
    """
    One PNG, fourteen panels covering every headline/additional metric plus the per-task,
    per-category, and repair-attribution breakdowns:

      row 0: average score / average success rate / average assist rate / average navigation
             distance -- one bar per condition.
      row 1: score-per-run and navigation-distance-per-run trends (one line per condition,
             x-axis is run index -- a proxy for progression across the trials, not wall-clock
             time, since runs are sequential subprocesses), plus average score/second and
             average success-rate/second (throughput over execution time, distinct from the
             plain success rate in row 0).
      row 2: per-object pickup success rate, per-object place success rate (tracked separately --
             a place never even runs when its pickup failed), pickup-by-sheet-category net score,
             place-by-sheet-category net score.
      row 3: restructuring-operator usage counts and average plan-stability distance per repair
             -- both empty for conditions with no repair layer (static baseline, H1-only); the
             metrics that attribute credit to H3 specifically, distinct from H1's ordering.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    conditions = list(results.keys())
    colors = _condition_colors(conditions)
    bar_colors = [colors[c] for c in conditions]
    fig, axes = plt.subplots(5, 4, figsize=(24, 22))

    ax = axes[0, 0]
    means = [results[c].average_score for c in conditions]
    errs = [results[c].score_dispersion["ci95_halfwidth"] or 0.0 for c in conditions]
    ax.bar(conditions, [m or 0 for m in means], color=bar_colors)
    ax.errorbar(conditions, [m or 0 for m in means], yerr=errs, fmt="none",
                ecolor=_NAVY_AXIS, elinewidth=1.5, capsize=4, zorder=3)
    plotted_means = [(c, m) for c, m in zip(conditions, means) if m is not None]
    if plotted_means:
        best_condition, best_mean = max(plotted_means, key=lambda cm: cm[1])
        ax.annotate(f"best: {best_mean:.0f}", (best_condition, best_mean), textcoords="offset points",
                    xytext=(0, 10), ha="center", fontsize=9, fontweight="bold", color=_NAVY_AXIS)
    ax.set_title("Average score  (mean ± 95% CI)")

    ax = axes[0, 1]
    ax.bar(conditions, [results[c].average_success_rate or 0 for c in conditions], color=bar_colors)
    ax.set_title("Average success rate")
    ax.set_ylim(0, 1)

    ax = axes[0, 2]
    ax.bar(conditions, [results[c].average_assist_rate or 0 for c in conditions], color=bar_colors)
    ax.set_title("Average assist rate")
    ax.set_ylim(0, 1)

    ax = axes[0, 3]
    ax.bar(conditions, [results[c].average_navigation_distance_meters or 0 for c in conditions], color=bar_colors)
    ax.set_title("Average navigation distance")
    ax.set_ylabel("m")

    ax = axes[1, 0]
    for condition in conditions:
        run_scores = [r.total_score if r.completed else None for r in results[condition].runs]
        xs = [i for i, s in enumerate(run_scores) if s is not None]
        ys = [s for s in run_scores if s is not None]
        ax.plot(xs, ys, marker="o", color=colors[condition], label=condition)
    ax.set_title("Score per run")
    ax.set_xlabel("run index")
    ax.legend(fontsize=7, framealpha=0.5, loc="best")

    ax = axes[1, 1]
    for condition in conditions:
        run_distances = [r.navigation_distance_meters if r.completed else None for r in results[condition].runs]
        xs = [i for i, d in enumerate(run_distances) if d is not None]
        ys = [d for d in run_distances if d is not None]
        ax.plot(xs, ys, marker="o", color=colors[condition], label=condition)
    ax.set_title("Navigation distance per run")
    ax.set_xlabel("run index")
    ax.set_ylabel("m")
    ax.legend(fontsize=7, framealpha=0.5, loc="best")

    ax = axes[1, 2]
    ax.bar(conditions, [results[c].average_score_per_second or 0 for c in conditions], color=bar_colors)
    ax.set_title("Average score / second")

    ax = axes[1, 3]
    ax.bar(conditions, [results[c].average_success_rate_per_second or 0 for c in conditions], color=bar_colors)
    ax.set_title("Average successes / second\n(throughput, not success rate)")

    all_objects = sorted({name for aggregate in results.values() for name in aggregate.per_object_stats})

    ax = axes[2, 0]
    _grouped_bar(ax, all_objects, results, conditions,
                lambda agg, name: agg.per_object_stats.get(name, {}).get("pickup_success_rate") or 0)
    ax.set_title("Per-object pickup success rate")
    ax.set_ylim(0, 1)

    ax = axes[2, 1]
    _grouped_bar(ax, all_objects, results, conditions,
                lambda agg, name: agg.per_object_stats.get(name, {}).get("place_success_rate") or 0)
    ax.set_title("Per-object place success rate")
    ax.set_ylim(0, 1)

    all_pickup_categories = sorted({name for aggregate in results.values() for name in aggregate.pickup_category_stats})
    ax = axes[2, 2]
    _grouped_bar(ax, all_pickup_categories, results, conditions,
                lambda agg, name: agg.pickup_category_stats.get(name, {}).get("total_score", 0)
                                  + agg.pickup_category_stats.get(name, {}).get("total_penalty", 0))
    ax.set_title("Net score by pickup category")

    all_place_categories = sorted({name for aggregate in results.values() for name in aggregate.place_category_stats})
    ax = axes[2, 3]
    _grouped_bar(ax, all_place_categories, results, conditions,
                lambda agg, name: agg.place_category_stats.get(name, {}).get("total_score", 0)
                                  + agg.place_category_stats.get(name, {}).get("total_penalty", 0))
    ax.set_title("Net score by place category")

    all_operators = sorted({name for aggregate in results.values() for name in aggregate.operator_usage_counts})
    ax = axes[3, 0]
    if all_operators:
        _grouped_bar(ax, all_operators, results, conditions,
                    lambda agg, name: agg.operator_usage_counts.get(name, 0))
    ax.set_title("Restructuring operator usage (H3)")

    ax = axes[3, 1]
    ax.bar(conditions, [results[c].average_plan_stability_distance or 0 for c in conditions], color=bar_colors)
    ax.set_title("Average plan-stability distance\nper repair (H3)")

    # Two-layer transformation split: A) human support (REQUEST_OBJECT_REPOSITION,
    # SUBSTITUTE_WITH_ASSISTANCE, REPLAN_WITH_ASSISTANCE) vs. B) autonomous repair (SKIP, RETRY,
    # REPLAN, REORDER) -- what fraction of every repair actually applied needed a human, versus
    # what the robot recovered on its own. Both stay n/a (0-height bar) wherever no repair ran.
    ax = axes[3, 2]
    ax.bar(conditions, [results[c].human_support_repair_rate or 0 for c in conditions], color=bar_colors)
    ax.set_title("Layer A: human-support\nrepair rate")
    ax.set_ylim(0, 1)

    ax = axes[3, 3]
    ax.bar(conditions, [results[c].autonomous_repair_rate or 0 for c in conditions], color=bar_colors)
    ax.set_title("Layer B: autonomous\nrepair rate")
    ax.set_ylim(0, 1)

    # Thesis eq. 5.24's Stability(Pi_task) (product of remaining steps' success probability),
    # before vs. after each repair -- a different question from the Fox et al. edit-distance
    # panel above: not how much a repair changed the plan's structure, but whether the repaired
    # plan is itself more or less likely to succeed than the plan it replaced.
    ax = axes[4, 0]
    ax.bar(conditions, [results[c].average_probability_stability_before or 0 for c in conditions], color=bar_colors)
    ax.set_title("Stability() before repair\n(thesis eq. 5.24)")
    ax.set_ylim(0, 1)

    ax = axes[4, 1]
    ax.bar(conditions, [results[c].average_probability_stability_after or 0 for c in conditions], color=bar_colors)
    ax.set_title("Stability() after repair\n(thesis eq. 5.24)")
    ax.set_ylim(0, 1)

    # Gross earned vs penalty lost, side by side, with the net drawn on top. The average-score bar
    # alone reports only the net, which hides the gain-cost relation the methodology's score/penalty
    # pair exists to expose: two conditions can net the same points while one of them earned and
    # lost far more, and that difference IS the risk the risk-aversion exponent is steering.
    ax = axes[4, 2]
    bar_width = 0.35
    positions = list(range(len(conditions)))
    gross = [results[c].average_gross_score or 0 for c in conditions]
    penalties = [results[c].average_penalty or 0 for c in conditions]
    nets = [results[c].average_score or 0 for c in conditions]
    ax.bar([p - bar_width / 2 for p in positions], gross, width=bar_width,
           color=_NAVY_PALETTE[1], label="gross earned")
    ax.bar([p + bar_width / 2 for p in positions], penalties, width=bar_width,
           color=_NAVY_PALETTE[5], label="penalty lost")
    ax.plot(positions, nets, marker="D", linestyle="none", color=_NAVY_AXIS, label="net")
    ax.axhline(0, color=_NAVY_AXIS, linewidth=0.8)
    ax.set_xticks(positions)
    ax.set_xticklabels(conditions, rotation=45, ha="right")
    ax.legend(fontsize=7, framealpha=0.5, loc="best")
    ax.set_title("Gross earned vs penalty lost\n(net = gross + penalty)")

    axes[4, 3].axis("off")

    for row in axes:
        for ax in row:
            if ax.axison:
                _style_axis(ax)

    _style_figure(fig)
    fig.tight_layout()
    fig.savefig(png_path, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def export_all(results: dict, out_dir: Path) -> None:
    """Every CSV/JSON/plot export, in one call, into out_dir."""
    export_task_attempts_csv(results, out_dir / "task_attempts.csv")
    export_run_summary_csv(results, out_dir / "run_summary.csv")
    export_per_object_csv(results, out_dir / "per_object.csv")
    export_category_csv(results, out_dir / "per_category.csv")
    export_operator_usage_csv(results, out_dir / "operator_usage.csv")
    export_score_over_time_csv(results, out_dir / "score_over_time.csv")
    export_summary_json(results, out_dir / "summary.json")
    plot_results(results, out_dir / "results.png")
    plot_score_over_time(results, out_dir / "score_over_time.png")


def plot_baseline_figure(aggregate, png_path: Path, condition_label: str = "static") -> None:
    """
    One publication-sized figure for the baseline chapter, instead of the 20-panel comparison grid.

    plot_results() is built to compare conditions: with a single condition most of its panels are
    one bar wide and six are blank (they hold H3 repair metrics the baseline has none of). The four
    panels here are the ones that actually say something about a lone baseline:

      (a) per-object pickup success -- the capability gap, the section's central finding;
      (b) gross earned vs penalty lost vs net -- the methodology's gain-cost pair, which a single
          net-score bar hides;
      (c) score per run against the mean and its 95% CI -- why the score needs a CI at all;
      (d) net score by sheet category -- which categories are net-negative contributors.

    Panel (a) also fixes a misreading in the comparison grid: there, an object whose pickup never
    succeeded shows an empty bar in the *place* panel, which reads as "placing failed" when placing
    was never attempted at all. Here placing is not plotted per object, and the never-succeeded
    pickups are drawn in the penalty colour and labelled, so a zero reads as what it is.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))

    # (a) per-object pickup success rate, worst first so the zero block is impossible to miss.
    ax = axes[0, 0]
    per_object = aggregate.per_object_stats
    names = sorted(per_object, key=lambda n: per_object[n]["pickup_success_rate"] or 0)
    rates = [per_object[n]["pickup_success_rate"] or 0 for n in names]
    colors = [_NAVY_PALETTE[5] if r == 0 else _NAVY_AXIS for r in rates]
    ax.bar(names, rates, color=colors)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=45, ha="right")
    ax.set_ylim(0, 1)
    ax.set_ylabel("pickup success rate")
    never = sum(1 for r in rates if r == 0)
    ax.set_title(f"Per-object pickup success\n({never} of {len(names)} classes never succeeded)")

    # (b) the gain-cost pair.
    ax = axes[0, 1]
    gross = aggregate.average_gross_score or 0
    penalty = aggregate.average_penalty or 0
    net = aggregate.average_score or 0
    ax.bar(["gross earned", "penalty lost", "net"], [gross, penalty, net],
           color=[_NAVY_PALETTE[1], _NAVY_PALETTE[5], _NAVY_AXIS])
    for index, value in enumerate([gross, penalty, net]):
        ax.annotate(f"{value:+.1f}", (index, value), textcoords="offset points",
                    xytext=(0, 8 if value >= 0 else -16), ha="center", fontweight="bold",
                    color=_NAVY_AXIS)
    ax.axhline(0, color=_NAVY_AXIS, linewidth=0.8)
    ax.set_ylabel("points per run")
    ax.set_title("Where the score comes from\n(net = gross + penalty)")

    # (c) per-run score against mean +/- 95% CI -- the justification for reporting a CI.
    ax = axes[1, 0]
    scores = [r.total_score for r in aggregate.completed_runs]
    dispersion = aggregate.score_dispersion
    half = dispersion["ci95_halfwidth"] or 0
    ax.plot(range(len(scores)), scores, marker="o", color=_NAVY_AXIS, label="run score")
    ax.axhline(net, color=_NAVY_PALETTE[1], linewidth=1.5, label=f"mean {net:.1f}")
    ax.fill_between(range(len(scores)), net - half, net + half, alpha=0.15,
                    color=_NAVY_PALETTE[1], label=f"95% CI +/-{half:.1f}")
    ax.axhline(0, color=_NAVY_PALETTE[5], linewidth=0.8)
    negative = sum(1 for s in scores if s < 0)
    ax.set_xlabel("run index")
    ax.set_ylabel("net score")
    ax.legend(fontsize=8, loc="best", framealpha=0.9)
    ax.set_title(f"Score per run ({negative} of {len(scores)} runs net negative)")

    # (d) which sheet categories actually pay.
    ax = axes[1, 1]
    category_stats = {f"pickup: {k}": v for k, v in aggregate.pickup_category_stats.items()}
    category_stats.update({f"place: {k}": v for k, v in aggregate.place_category_stats.items()})
    labels = list(category_stats)
    nets = [category_stats[k]["total_score"] + category_stats[k]["total_penalty"] for k in labels]
    ax.bar(labels, nets, color=[_NAVY_PALETTE[5] if n < 0 else _NAVY_AXIS for n in nets])
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.axhline(0, color=_NAVY_AXIS, linewidth=0.8)
    ax.set_ylabel("net points (all runs)")
    ax.set_title("Net score by sheet category")

    for row in axes:
        for axis in row:
            _style_axis(axis)
    _style_figure(fig, suptitle=f"Baseline ({condition_label}): {aggregate.number_of_finished_executions} runs")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(png_path, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_baseline_panels(aggregate, out_dir: Path, prefix: str = "baseline") -> list:
    """
    The same four baseline panels as plot_baseline_figure(), but one PNG per panel, with no title
    and a legend on each.

    For embedding individually in the thesis: the caption carries what the title said, and dropping
    the in-image title avoids the duplicate heading a \\caption would otherwise create. Each panel
    gains a legend so it still reads on its own once the shared figure heading is gone -- in
    particular the colour split (never-succeeded classes, net-negative categories) is only
    self-evident if it is spelled out.

    plot_baseline_figure() is deliberately left untouched; this writes alongside it.

    :return: the paths written, in panel order (a)-(d).
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D

    out_dir.mkdir(parents=True, exist_ok=True)
    written = []

    def finish(fig, ax, name):
        _style_axis(ax)
        ax.set_title("")
        _style_figure(fig)
        fig.tight_layout()
        path = out_dir / f"{prefix}_{name}.png"
        fig.savefig(path, dpi=200, bbox_inches="tight", facecolor=fig.get_facecolor())
        plt.close(fig)
        written.append(path)

    # (a) per-object pickup success
    fig, ax = plt.subplots(figsize=(7.5, 5))
    per_object = aggregate.per_object_stats
    names = sorted(per_object, key=lambda n: per_object[n]["pickup_success_rate"] or 0)
    rates = [per_object[n]["pickup_success_rate"] or 0 for n in names]
    ax.bar(names, rates, color=_NAVY_AXIS)
    # A rate of exactly 0 draws a bar of zero height, i.e. nothing at all -- which reads as
    # "missing data" rather than "failed every time", and leaves the legend promising a colour that
    # never appears. Mark those classes explicitly at y=0 instead of faking a small bar height.
    zero_indices = [i for i, r in enumerate(rates) if r == 0]
    ax.scatter(zero_indices, [0] * len(zero_indices), marker="x", s=90, linewidths=2.5,
               color=_NAVY_PALETTE[5], zorder=5, clip_on=False)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=45, ha="right")
    ax.set_ylim(0, 1)
    ax.set_ylabel("pickup success rate")
    ax.legend(handles=[Patch(color=_NAVY_AXIS, label="succeeded at least once"),
                       Line2D([], [], marker="x", linestyle="none", markersize=9,
                              markeredgewidth=2.5, color=_NAVY_PALETTE[5],
                              label=f"never succeeded ({len(zero_indices)} classes)")],
              fontsize=9, loc="upper left", framealpha=0.9)
    finish(fig, ax, "pickup_success")

    # (b) gross vs penalty vs net
    fig, ax = plt.subplots(figsize=(6, 5))
    gross = aggregate.average_gross_score or 0
    penalty = aggregate.average_penalty or 0
    net = aggregate.average_score or 0
    labels = ["gross earned", "penalty lost", "net"]
    values = [gross, penalty, net]
    bar_colors = [_NAVY_PALETTE[1], _NAVY_PALETTE[5], _NAVY_AXIS]
    ax.bar(labels, values, color=bar_colors)
    for index, value in enumerate(values):
        ax.annotate(f"{value:+.1f}", (index, value), textcoords="offset points",
                    xytext=(0, 8 if value >= 0 else -16), ha="center", fontweight="bold",
                    color=_NAVY_AXIS)
    ax.axhline(0, color=_NAVY_AXIS, linewidth=0.8)
    ax.set_ylabel("points per run")
    ax.legend(handles=[Patch(color=c, label=l) for c, l in zip(bar_colors, labels)],
              fontsize=9, loc="lower left", framealpha=0.9)
    finish(fig, ax, "score_composition")

    # (c) per-run score with mean and CI
    fig, ax = plt.subplots(figsize=(7.5, 5))
    scores = [r.total_score for r in aggregate.completed_runs]
    half = aggregate.score_dispersion["ci95_halfwidth"] or 0
    negative = sum(1 for s in scores if s < 0)
    ax.plot(range(len(scores)), scores, marker="o", color=_NAVY_AXIS, label="net score of one run")
    ax.axhline(net, color=_NAVY_PALETTE[1], linewidth=1.5, label=f"mean {net:.1f}")
    ax.fill_between(range(len(scores)), net - half, net + half, alpha=0.15,
                    color=_NAVY_PALETTE[1], label=f"95% CI +/-{half:.1f}")
    ax.axhline(0, color=_NAVY_PALETTE[5], linewidth=0.8,
               label=f"zero ({negative} of {len(scores)} runs below)")
    ax.set_xlabel("run index")
    ax.set_ylabel("net score")
    ax.legend(fontsize=9, loc="best", framealpha=0.9)
    finish(fig, ax, "score_per_run")

    # (d) net score by sheet category
    fig, ax = plt.subplots(figsize=(7.5, 5))
    category_stats = {f"pickup: {k}": v for k, v in aggregate.pickup_category_stats.items()}
    category_stats.update({f"place: {k}": v for k, v in aggregate.place_category_stats.items()})
    labels = list(category_stats)
    nets = [category_stats[k]["total_score"] + category_stats[k]["total_penalty"] for k in labels]
    ax.bar(labels, nets, color=[_NAVY_PALETTE[5] if n < 0 else _NAVY_AXIS for n in nets])
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.axhline(0, color=_NAVY_AXIS, linewidth=0.8)
    ax.set_ylabel("net points (all runs)")
    ax.legend(handles=[Patch(color=_NAVY_AXIS, label="net positive contribution"),
                       Patch(color=_NAVY_PALETTE[5], label="net negative contribution")],
              fontsize=9, loc="best", framealpha=0.9)
    finish(fig, ax, "category_net")

    return written
