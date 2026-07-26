import statistics
from dataclasses import dataclass, field
from typing import Optional

from pycram.datastructures.enums import PlanTransformationOperator, TaskStatus

HUMAN_SUPPORT_OPERATORS = frozenset({
    PlanTransformationOperator.REPLAN_WITH_ASSISTANCE.name,
    PlanTransformationOperator.SUBSTITUTE_WITH_ASSISTANCE.name,
    PlanTransformationOperator.REQUEST_OBJECT_REPOSITION.name,
})
AUTONOMOUS_REPAIR_OPERATORS = frozenset({
    PlanTransformationOperator.SKIP.name,
    PlanTransformationOperator.RETRY.name,
    PlanTransformationOperator.REPLAN.name,
    PlanTransformationOperator.REORDER.name,
})


@dataclass
class TaskAttempt:
    """One object's pickup+place handling within a single challenge run."""

    object_name: str
    pickup_location: str
    place_location: str
    pickup_status: TaskStatus
    place_status: TaskStatus
    assisted: bool
    score: int
    penalty: int
    duration_seconds: float
    failure_reason: Optional[str] = None
    """
    Exception type of whatever went wrong, or None if nothing did.
    """

    object_type: str = ""
    """
    The semantic annotation class name (e.g. "Knife", "Plate", "DishwasherTab"), NOT
    object_name 
    """

    is_trash: bool = False
    """Whether this attempt's destination is THROW_AWAY rather than PLACE."""

    was_uncertain: bool = False
    """
    True if this task's location was unknown up front and resolved at runtime.
    """

    error_during_execution: bool = False
    """
    True if this task hit a FAILED/INTERRUPTED sub-plan at any point, even one a later repair
    recovered from..
    """

    ERROR_STATUSES = (TaskStatus.FAILED, TaskStatus.INTERRUPTED)

    @property
    def had_error(self) -> bool:
        return (self.error_during_execution
                or self.pickup_status in self.ERROR_STATUSES
                or self.place_status in self.ERROR_STATUSES)

    @property
    def pickup_succeeded(self) -> bool:
        return self.pickup_status == TaskStatus.SUCCEEDED

    @property
    def place_attempted(self) -> bool:
        return self.pickup_succeeded

    @property
    def succeeded(self) -> bool:
        return self.place_status == TaskStatus.SUCCEEDED

    @property
    def place_failed(self) -> bool:
        return self.place_attempted and self.place_status in self.ERROR_STATUSES


@dataclass
class RunMetrics:
    """Everything recorded during a single execution of the static-environment challenge."""

    completed: bool = False
    """False if the run raised out of static_execution() itself (crash, not a per-task FAILED)."""

    run_duration_seconds: float = 0.0
    navigation_distance_meters: float = 0.0
    task_attempts: list[TaskAttempt] = field(default_factory=list)

    time_budget_seconds: Optional[float] = None
    """The modelled-time deadline this run was given, or None if it ran unbudgeted."""

    modelled_seconds_spent: float = 0.0
    """
    Modelled seconds the run actually consumed (see time_budget.py). 
    """

    tasks_skipped_over_budget: int = 0
    """Tasks passed over because they no longer fitted."""

    operator_usage: dict[str, int] = field(default_factory=dict)
    """
    How many times each PlanTransformationOperator (by .value) won stabilize()'s selection in
    this run. .
    """

    plan_stability_distances: list[int] = field(default_factory=list)
    """
    One entry per repair actually applied, from PlanStabilizer's own Fox-et-al..
    """

    probability_stability_before: list[float] = field(default_factory=list)
    probability_stability_after: list[float] = field(default_factory=list)
    """
    One entry per repair actually applied, from PlanStabilizer._probability_stability .
    """

    @property
    def total_score(self) -> int:
        """
        Points earned minus points lost.
        """
        return sum(t.score + t.penalty for t in self.task_attempts)

    @property
    def gross_score(self) -> int:
        """
        Points earned, before penalties are deducted.
        """
        return sum(t.score for t in self.task_attempts)

    @property
    def total_penalty(self) -> int:
        """Points lost, carried **negative** (so gross_score + total_penalty == total_score)."""
        return sum(t.penalty for t in self.task_attempts)

    def score_timeline(self) -> list[tuple[float, float]]:
        """
        Cumulative (elapsed_seconds, net_score) after each task, in execution order, starting at
        (0.0, 0.0).
        """
        timeline: list[tuple[float, float]] = [(0.0, 0.0)]
        cumulative_time = 0.0
        cumulative_score = 0.0
        for attempt in self.task_attempts:
            cumulative_time += attempt.duration_seconds
            cumulative_score += attempt.score + attempt.penalty
            timeline.append((cumulative_time, cumulative_score))
        return timeline

    def score_at_time(self, elapsed_seconds: float) -> float:
        """
        Net score this run had accumulated by `elapsed_seconds`, as a step function: the sum over
        every task that had completed at or before that time.
        meaningful.
        """
        score = 0.0
        cumulative_time = 0.0
        for attempt in self.task_attempts:
            cumulative_time += attempt.duration_seconds
            if cumulative_time > elapsed_seconds:
                break
            score += attempt.score + attempt.penalty
        return score

    @property
    def tasks_started(self) -> int:
        return len(self.task_attempts)

    @property
    def tasks_succeeded(self) -> int:
        return sum(1 for t in self.task_attempts if t.succeeded)

    @property
    def success_rate(self) -> Optional[float]:
        if not self.task_attempts:
            return None
        return self.tasks_succeeded / self.tasks_started

    @property
    def tasks_with_error(self) -> list[TaskAttempt]:
        return [t for t in self.task_attempts if t.had_error]

    @property
    def success_after_error_rate(self) -> Optional[float]:
        """Of the tasks that hit a FAILED/INTERRUPTED sub-step, how many still ended up placed
        successfully
        """
        errored = self.tasks_with_error
        if not errored:
            return None
        return sum(1 for t in errored if t.succeeded) / len(errored)

    @property
    def average_task_duration_seconds(self) -> Optional[float]:
        if not self.task_attempts:
            return None
        return statistics.mean(t.duration_seconds for t in self.task_attempts)

    @property
    def assist_rate(self) -> Optional[float]:
        if not self.task_attempts:
            return None
        return sum(1 for t in self.task_attempts if t.assisted) / self.tasks_started

    @property
    def score_per_second(self) -> Optional[float]:
        if self.run_duration_seconds <= 0:
            return None
        return self.total_score / self.run_duration_seconds

    @property
    def success_rate_per_second(self) -> Optional[float]:
        """Successful tasks per second of run time ."""
        if self.run_duration_seconds <= 0:
            return None
        return self.tasks_succeeded / self.run_duration_seconds

    @property
    def navigation_distance_per_task(self) -> Optional[float]:
        if not self.task_attempts:
            return None
        return self.navigation_distance_meters / self.tasks_started

    @property
    def pickup_failures(self) -> int:
        return sum(1 for t in self.task_attempts if t.pickup_status in TaskAttempt.ERROR_STATUSES)

    @property
    def place_failures(self) -> int:
        return sum(1 for t in self.task_attempts
                   if t.place_attempted and t.place_status in TaskAttempt.ERROR_STATUSES)


@dataclass
class AggregateMetrics:
    """Mean of RunMetrics across n_runs executions of the same challenge."""

    n_runs_attempted: int
    runs: list[RunMetrics]

    @property
    def number_of_finished_executions(self) -> int:
        return sum(1 for r in self.runs if r.completed)

    @property
    def completed_runs(self) -> list[RunMetrics]:
        return [r for r in self.runs if r.completed]

    @property
    def average_score(self) -> Optional[float]:
        runs = self.completed_runs
        return statistics.mean(r.total_score for r in runs) if runs else None

    @property
    def average_gross_score(self) -> Optional[float]:
        """
        Mean points earned per run, before penalties.
        """
        runs = self.completed_runs
        return statistics.mean(r.gross_score for r in runs) if runs else None

    @property
    def average_penalty(self) -> Optional[float]:
        runs = self.completed_runs
        return statistics.mean(r.total_penalty for r in runs) if runs else None

    @property
    def penalty_dispersion(self) -> dict:
        return self._dispersion([r.total_penalty for r in self.completed_runs])

    def score_over_time(self, num_buckets: int = 20) -> list[tuple[float, float, float]]:
        """
        Average net score across completed runs at a grid of elapsed-time points, so conditions can
        be compared on *how* they accumulate score, not just the final total
        """
        runs = [r for r in self.completed_runs if r.task_attempts]
        if not runs:
            return []
        max_time = max(r.score_timeline()[-1][0] for r in runs)
        if max_time <= 0:
            return []
        grid = [max_time * i / num_buckets for i in range(1, num_buckets + 1)]
        series: list[tuple[float, float, float]] = []
        for elapsed in grid:
            scores = [r.score_at_time(elapsed) for r in runs]
            mean = statistics.mean(scores)
            sem = statistics.stdev(scores) / (len(scores) ** 0.5) if len(scores) > 1 else 0.0
            series.append((elapsed, mean, sem))
        return series

    @staticmethod
    def _dispersion(values: list) -> dict:
        """Sample mean / stdev / standard error / 95%-CI half-width of per-run values.


        """
        n = len(values)
        if n == 0:
            return {"n": 0, "mean": None, "stdev": None, "sem": None, "ci95_halfwidth": None}
        mean = statistics.mean(values)
        stdev = statistics.stdev(values) if n > 1 else 0.0
        sem = stdev / (n ** 0.5)
        return {"n": n, "mean": mean, "stdev": stdev, "sem": sem, "ci95_halfwidth": 1.96 * sem}

    @property
    def score_dispersion(self) -> dict:
        """mean/stdev/sem/95%-CI."""
        return self._dispersion([r.total_score for r in self.completed_runs])

    @property
    def success_rate_dispersion(self) -> dict:
        """Per-run success_rate dispersion across completed runs."""
        return self._dispersion([r.success_rate for r in self.completed_runs if r.success_rate is not None])

    @property
    def average_navigation_distance_meters(self) -> Optional[float]:
        runs = self.completed_runs
        return statistics.mean(r.navigation_distance_meters for r in runs) if runs else None

    @property
    def average_success_rate(self) -> Optional[float]:
        all_tasks = []
        for run in self.completed_runs:
            for attempt in run.task_attempts:
                all_tasks.append(attempt)
        if not all_tasks:
            return None
        succeeded = 0
        for attempt in all_tasks:
            if attempt.succeeded:
                succeeded += 1
        return succeeded / len(all_tasks)

    @property
    def average_success_after_error_rate(self) -> Optional[float]:
        """Pooled the same way as average_success_rate, over tasks that hit a FAILED sub-step."""
        errored = []
        for run in self.completed_runs:
            for attempt in run.tasks_with_error:
                errored.append(attempt)
        if not errored:
            return None
        return sum(1 for t in errored if t.succeeded) / len(errored)

    @property
    def average_time_spent_per_task_seconds(self) -> Optional[float]:
        all_tasks = []
        for run in self.completed_runs:
            for attempt in run.task_attempts:
                all_tasks.append(attempt)
        if not all_tasks:
            return None
        return statistics.mean(t.duration_seconds for t in all_tasks)

    @property
    def average_score_per_second(self) -> Optional[float]:
        runs = [r for r in self.completed_runs if r.run_duration_seconds > 0]
        return statistics.mean(r.score_per_second for r in runs) if runs else None

    @property
    def average_success_rate_per_second(self) -> Optional[float]:
        runs = [r for r in self.completed_runs if r.run_duration_seconds > 0]
        return statistics.mean(r.success_rate_per_second for r in runs) if runs else None

    @property
    def average_navigation_distance_per_task(self) -> Optional[float]:
        runs = [r for r in self.completed_runs if r.task_attempts]
        return statistics.mean(r.navigation_distance_per_task for r in runs) if runs else None

    @property
    def average_assist_rate(self) -> Optional[float]:
        runs = [r for r in self.completed_runs if r.task_attempts]
        return statistics.mean(r.assist_rate for r in runs) if runs else None

    @property
    def total_pickup_failures(self) -> int:
        return sum(r.pickup_failures for r in self.completed_runs)

    @property
    def total_place_failures(self) -> int:
        return sum(r.place_failures for r in self.completed_runs)

    @property
    def uncertain_task_attempts(self) -> list:
        uncertain = []
        for run in self.completed_runs:
            for attempt in run.task_attempts:
                if attempt.was_uncertain:
                    uncertain.append(attempt)
        return uncertain

    @property
    def uncertain_task_pickup_success_rate(self) -> Optional[float]:
        """Pickup success rate among tasks that needed localization (was_uncertain).
        """
        uncertain = self.uncertain_task_attempts
        if not uncertain:
            return None
        return sum(1 for t in uncertain if t.pickup_succeeded) / len(uncertain)

    @property
    def uncertain_task_place_success_rate(self) -> Optional[float]:
        """Full-task (placed) success rate among tasks that needed localization """
        uncertain = self.uncertain_task_attempts
        if not uncertain:
            return None
        return sum(1 for t in uncertain if t.succeeded) / len(uncertain)

    @property
    def uncertain_task_pickup_failures(self) -> int:
        """Pickup failures among tasks that needed localization, pooled across completed runs.
        """
        return sum(1 for t in self.uncertain_task_attempts if not t.pickup_succeeded)

    @property
    def operator_usage_counts(self) -> dict:
        """
        How often each restructuring operator won stabilize()'s selection, pooled across every
        completed run, worst-used first.
        """
        counts: dict = {}
        for run in self.completed_runs:
            for operator_name, count in run.operator_usage.items():
                counts[operator_name] = counts.get(operator_name, 0) + count
        return dict(sorted(counts.items(), key=lambda item: item[1], reverse=True))

    @property
    def average_plan_stability_distance(self) -> Optional[float]:
        """Mean Fox-et-al. plan-stability distance across every repair actually applied, pooled
        across every completed run."""
        distances = []
        for run in self.completed_runs:
            for distance in run.plan_stability_distances:
                distances.append(distance)
        return statistics.mean(distances) if distances else None

    @property
    def average_probability_stability_before(self) -> Optional[float]:
        """Mean thesis-eq.-5.24 Stability() of the plan just before each repair actually applied,
        pooled across every completed run. Complementary to average_plan_stability_distance --
        see RunMetrics.probability_stability_before for why both are tracked."""
        values = []
        for run in self.completed_runs:
            for value in run.probability_stability_before:
                values.append(value)
        return statistics.mean(values) if values else None

    @property
    def average_probability_stability_after(self) -> Optional[float]:
        """Mean thesis-eq.-5.24 Stability() of the plan just after each repair actually applied.
        Compare against average_probability_stability_before to see whether repairs make the
        remaining plan more or less likely to succeed, on average."""
        values = []
        for run in self.completed_runs:
            for value in run.probability_stability_after:
                values.append(value)
        return statistics.mean(values) if values else None

    @property
    def human_support_operator_counts(self) -> dict:
        """Layer A of operator_usage_counts: only REQUEST_OBJECT_REPOSITION,
        SUBSTITUTE_WITH_ASSISTANCE, REPLAN_WITH_ASSISTANCE -- repairs that required a human."""
        counts = self.operator_usage_counts
        return {name: count for name, count in counts.items() if name in HUMAN_SUPPORT_OPERATORS}

    @property
    def autonomous_repair_operator_counts(self) -> dict:
        """Layer B of operator_usage_counts: SKIP, RETRY, REPLAN, REORDER -- repairs the robot
        performed without asking a human for anything."""
        counts = self.operator_usage_counts
        return {name: count for name, count in counts.items() if name in AUTONOMOUS_REPAIR_OPERATORS}

    @property
    def total_repairs_applied(self) -> int:
        """Every repair actually applied, either layer, pooled across completed runs. The
        denominator human_support_repair_rate and autonomous_repair_rate are fractions of."""
        return sum(self.operator_usage_counts.values())

    @property
    def human_support_repair_rate(self) -> Optional[float]:
        """Of every repair actually applied, what fraction required human support (layer A).
        None if no repair ran at all"""
        total = self.total_repairs_applied
        if total == 0:
            return None
        return sum(self.human_support_operator_counts.values()) / total

    @property
    def autonomous_repair_rate(self) -> Optional[float]:
        total = self.total_repairs_applied
        if total == 0:
            return None
        return sum(self.autonomous_repair_operator_counts.values()) / total

    @staticmethod
    def _new_stats_entry() -> dict:
        return {
            "attempts": 0, "pickup_attempts": 0, "pickup_succeeded": 0,
            "place_attempts": 0, "place_succeeded": 0,
            "total_score": 0, "total_penalty": 0,
        }

    @staticmethod
    def _fold_attempt(entry: dict, attempt: "TaskAttempt") -> None:
        entry["attempts"] += 1
        entry["pickup_attempts"] += 1
        entry["pickup_succeeded"] += 1 if attempt.pickup_succeeded else 0
        if attempt.place_attempted:
            entry["place_attempts"] += 1
            entry["place_succeeded"] += 1 if attempt.succeeded else 0
        entry["total_score"] += attempt.score
        entry["total_penalty"] += attempt.penalty

    @staticmethod
    def _finalize_rates(entry: dict) -> None:
        entry["pickup_success_rate"] = (
            entry["pickup_succeeded"] / entry["pickup_attempts"] if entry["pickup_attempts"] else None)
        entry["place_success_rate"] = (
            entry["place_succeeded"] / entry["place_attempts"] if entry["place_attempts"] else None)

    @property
    def per_object_stats(self) -> dict:
        """
        Pickup and place attempts/successes/success_rate tracked *separately* (a place never
        even ran if the pickup failed
        """
        stats: dict = {}
        for run in self.completed_runs:
            for attempt in run.task_attempts:
                key = attempt.object_type or attempt.object_name
                entry = stats.setdefault(key, self._new_stats_entry())
                self._fold_attempt(entry, attempt)
        for entry in stats.values():
            self._finalize_rates(entry)
        return dict(sorted(stats.items(), key=lambda kv: kv[1]["place_success_rate"] or 0, reverse=True))

    @staticmethod
    def _pickup_sheet_category(object_type: str) -> str:
        """
        Buckets a pickup the way the sheet's Picking section groups its line items: cutlery
        (2x+50), plate (+100), dishwasher tab (+100), everything else falls into the flat
        common-object/transportation rate.
        """
        if object_type in ("Knife", "Fork", "Spoon"):
            return "cutlery"
        if object_type == "Plate":
            return "plate"
        if object_type == "DishwasherTab":
            return "dishwasher_tab"
        return "common_object"

    @staticmethod
    def _place_sheet_category(place_location: str, is_trash: bool) -> str:
        """
        Buckets a place/throw-away the way the sheet's Placing section groups its line items:
        trash disposal is scored separately from placing at a designated location, dishwasher
        placements get their own bonus, everything else is the flat common-object rate.
        """
        if is_trash:
            return "trash"
        if place_location in ("dishwasher", "dishwasher_rack"):
            return "dishwasher_placement"
        return "common_object"

    @property
    def pickup_category_stats(self) -> dict:
        stats: dict = {}
        for run in self.completed_runs:
            for attempt in run.task_attempts:
                category = self._pickup_sheet_category(attempt.object_type)
                entry = stats.setdefault(category, self._new_stats_entry())
                self._fold_attempt(entry, attempt)
        for entry in stats.values():
            self._finalize_rates(entry)
        return dict(sorted(stats.items(), key=lambda kv: kv[1]["total_score"] + kv[1]["total_penalty"], reverse=True))

    @property
    def place_category_stats(self) -> dict:
        """Same shape as per_object_stats, grouped by _place_sheet_category."""
        stats: dict = {}
        for run in self.completed_runs:
            for attempt in run.task_attempts:
                category = self._place_sheet_category(attempt.place_location, attempt.is_trash)
                entry = stats.setdefault(category, self._new_stats_entry())
                self._fold_attempt(entry, attempt)
        for entry in stats.values():
            self._finalize_rates(entry)
        return dict(sorted(stats.items(), key=lambda kv: kv[1]["total_score"] + kv[1]["total_penalty"], reverse=True))

    @property
    def failure_reason_counts(self) -> dict:
        """
        How many task attempts failed for each reason, worst first.
        """
        counts: dict = {}
        for run in self.completed_runs:
            for attempt in run.task_attempts:
                if attempt.failure_reason is None:
                    continue
                counts[attempt.failure_reason] = counts.get(attempt.failure_reason, 0) + 1
        return dict(sorted(counts.items(), key=lambda item: item[1], reverse=True))

    def summary(self) -> str:
        def fmt(value, suffix=""):
            return f"{value:.3f}{suffix}" if value is not None else "n/a"

        score_dispersion = self.score_dispersion
        success_rate_dispersion = self.success_rate_dispersion
        lines = [
            f"finished executions: {self.number_of_finished_executions}/{self.n_runs_attempted}",
            f"average score: {fmt(self.average_score)} "
            f"(stdev {fmt(score_dispersion['stdev'])}, "
            f"95% CI +/-{fmt(score_dispersion['ci95_halfwidth'])}, n={score_dispersion['n']})",
            f"  = gross {fmt(self.average_gross_score)} earned "
            f"{fmt(self.average_penalty)} lost to penalties "
            f"(penalty 95% CI +/-{fmt(self.penalty_dispersion['ci95_halfwidth'])})",
            f"average navigation distance: {fmt(self.average_navigation_distance_meters, ' m')}",
            f"average success rate (per task started): {fmt(self.average_success_rate)}",
            f"per-run success rate: {fmt(success_rate_dispersion['mean'])} "
            f"(stdev {fmt(success_rate_dispersion['stdev'])}, "
            f"95% CI +/-{fmt(success_rate_dispersion['ci95_halfwidth'])})",
            f"average success-after-error rate: {fmt(self.average_success_after_error_rate)}",
            f"average time spent per task: {fmt(self.average_time_spent_per_task_seconds, ' s')}",
            f"-- additional metrics --",
            f"average score per second: {fmt(self.average_score_per_second)}",
            f"average success rate per second (throughput): {fmt(self.average_success_rate_per_second)}",
            f"average navigation distance per task: {fmt(self.average_navigation_distance_per_task, ' m')}",
            f"average assist rate: {fmt(self.average_assist_rate)}",
            f"pickup failures (total): {self.total_pickup_failures}",
            f"place failures (total): {self.total_place_failures}",
            f"-- localization-sensitive subset (H2 lives here) --",
            f"uncertain tasks (location resolved at runtime): {len(self.uncertain_task_attempts)}",
            f"  pickup success rate among them: {fmt(self.uncertain_task_pickup_success_rate)}",
            f"  full-task success rate among them: {fmt(self.uncertain_task_place_success_rate)}",
            f"  pickup failures among them: {self.uncertain_task_pickup_failures} "
            f"(see failures-by-reason for wrong-surface vs grasp)",
        ]
        reason_counts = self.failure_reason_counts
        if reason_counts:
            lines.append("-- failures by reason --")
            for reason, count in reason_counts.items():
                lines.append(f"  {reason}: {count}")

        def entry_line(name: str, entry: dict) -> str:
            return (
                f"  {name}: pickup {entry['pickup_succeeded']}/{entry['pickup_attempts']} "
                f"({fmt(entry['pickup_success_rate'])}) place {entry['place_succeeded']}/{entry['place_attempts']} "
                f"({fmt(entry['place_success_rate'])}) score={entry['total_score']:+d} "
                f"penalty={entry['total_penalty']:+d} net={entry['total_score'] + entry['total_penalty']:+d}"
            )

        per_object = self.per_object_stats
        if per_object:
            lines.append("-- per-task success (pickup vs. place, which objects actually land) --")
            for name, entry in per_object.items():
                lines.append(entry_line(name, entry))

        pickup_category = self.pickup_category_stats
        if pickup_category:
            lines.append("-- picking, by sheet category --")
            for name, entry in pickup_category.items():
                lines.append(entry_line(name, entry))

        place_category = self.place_category_stats
        if place_category:
            lines.append("-- placing, by sheet category --")
            for name, entry in place_category.items():
                lines.append(entry_line(name, entry))

        operator_counts = self.operator_usage_counts
        if operator_counts:
            lines.append("-- restructuring operators used (H3 repair only) --")
            for operator_name, count in operator_counts.items():
                layer = "A: human support" if operator_name in HUMAN_SUPPORT_OPERATORS else "B: autonomous repair"
                lines.append(f"  {operator_name} [{layer}]: {count}")
            lines.append(f"  layer A (human support) rate: {fmt(self.human_support_repair_rate)}")
            lines.append(f"  layer B (autonomous repair) rate: {fmt(self.autonomous_repair_rate)}")
            lines.append(f"  average plan-stability distance per repair (Fox et al., edit distance): "
                         f"{fmt(self.average_plan_stability_distance)}")
            lines.append(f"  average Stability() before repair (thesis eq. 5.24, success-probability): "
                         f"{fmt(self.average_probability_stability_before)}")
            lines.append(f"  average Stability() after repair (thesis eq. 5.24, success-probability): "
                         f"{fmt(self.average_probability_stability_after)}")
        return "\n".join(lines)
