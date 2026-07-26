import sys
from pathlib import Path


def _ensure_source_roots_on_path() -> None:
    """
    Puts the repository's source roots on sys.path, so this file can be run directly instead of
    only through an IDE run config that already adds them.
    """
    evaluation_dir = Path(__file__).resolve().parent
    demo_dir = evaluation_dir.parent                  # pycram_score_aware_planning
    repo_root = demo_dir.parents[2]                   # <repo>/pycram/demos/<demo> -> <repo>

    roots = [source_root for source_root in sorted(repo_root.glob("*/src")) if source_root.is_dir()]
    roots += [repo_root / "pycram", repo_root / "pycram" / "demos", demo_dir, evaluation_dir]

    for root in roots:
        path = str(root)
        if root.is_dir() and path not in sys.path:
            sys.path.insert(0, path)


_ensure_source_roots_on_path()

import datetime
from typing import Optional

from Evaluate.CompositeEvaluator import CompositeEvaluator
from ScoreAwareExecutioner import (
    closest_location,
    commit_to_prior,
    get_target_object_by_task,
    is_uncertain,
    opportunistic_exploration_target,
    scan_table,
    task_objects_exist,
    unscanned_locations_for,
    unscanned_search_locations,
    update_tasks_from_belief,
)
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.events.event_handler import EventDispatcher
from StaticExecution import navigation_distance
from common.cram_types import Task, Status, ActionType
from common.values import CHALLENGE_TASKS
from demos.pycram_score_aware_planning.Structurizer.structurizer import PlanStructurizer
from evaluation_metrics import RunMetrics
from helper_methods import (
    closest_navigation_location,
    find_object_annotation,
    generate_plan_taskstep_list,
    normalize_task_estimation,
    remove_object_from_world,
    reposition_task_list,
)
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import ChallengeMode, TaskStatus, Arms
from pycram.motion_executor import simulated_robot
from pycram.plans.factories import sequential
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction
from task_attempt_builder import build_task_attempt
from time_budget import TimeBudget, task_cost_seconds
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.world_description.world_entity import Body


def commit_to_ground_truth(context: Context, task: Task) -> None:

    target = get_target_object_by_task(task)
    location = None
    if target is not None:
        annotation = find_object_annotation(context.world, target)
        if annotation is not None:
            body = annotation.bodies[0]
            location = closest_navigation_location(context.challenge_mode, body.global_pose.to_position())
    for task_step in task.task_steps:
        if task_step.action_type in (ActionType.NAVIGATE, ActionType.PICKUP) and not task_step.location:
            task_step.location = location or ""
    task.uncertain = False


def ground_truth_found_objects(context: Context, task_list: list[Task]) -> dict:
    found: dict = {}
    for task in task_list:
        target = get_target_object_by_task(task)
        if target is None:
            continue
        annotation = find_object_annotation(context.world, target)
        if annotation is None:
            continue
        found[annotation] = closest_navigation_location(
            context.challenge_mode, annotation.bodies[0].global_pose.to_position())
    return found


def estimate_without_assistance(evaluator: CompositeEvaluator, context: Context, task_list: list[Task],
                                found_objects: dict, risk_aversion: float = 1) -> list[Task]:
    task_list_probability_evaluated = evaluator.probability_evaluator.estimate(
        task_list=task_list, context=context, found_objects=found_objects, probability_threshold=0.0)
    task_list_score_evaluated = evaluator.score_evaluator.estimate(
        task_list=task_list_probability_evaluated, context=context)

    min_risk_aversion, max_risk_aversion = 0.5, 4
    if not min_risk_aversion <= risk_aversion <= max_risk_aversion:
        risk_aversion = 2

    for task in task_list_score_evaluated:
        cum_expected_score = 0
        for ts in task.task_steps:
            if ts.action_assisted:
                ts.expected_score = -abs(ts.action_penalty) / ts.action_time if ts.action_time else 0.0
            else:
                safe_gain = ts.action_score * ts.action_probability ** risk_aversion
                expected_penalty = abs(ts.action_penalty) * (1 - ts.action_probability)
                ts.expected_score = (safe_gain - expected_penalty) / ts.action_time if ts.action_time else 0.0
            cum_expected_score += ts.expected_score
        task.cum_expected_score = cum_expected_score

    return normalize_task_estimation(task_list_score_evaluated)


def estimation_execution(dispatcher: EventDispatcher, context: Context, challenge_mode: ChallengeMode,
                          use_ground_truth_belief: bool = True, enable_exploration: bool = False,
                          use_prior_guess: bool = False,
                          risk_aversion: float = 1.5, decline_negative_ev: bool = False,
                          min_pickup_probability: float = 0.0,
                          time_budget_seconds: Optional[float] = None):
    hsrb = context.robot
    evaluator = CompositeEvaluator()
    structurizer = PlanStructurizer()
    scoretime_monitor = ScoreTimeMonitor(challenge_mode=challenge_mode)
    budget = TimeBudget(time_budget_seconds)

    found_objects: dict[Body, str] = {}
    explored_locations: list[str] = []
    plans = []

    run_start = datetime.datetime.now()
    run_metrics = RunMetrics()

    task_list: list[Task] = CHALLENGE_TASKS.get(challenge_mode)
    initially_uncertain_ids = {t.id for t in task_list if is_uncertain(t, {})}
    current_task = None
    while task_list != []:
        unachievable_tasks = []
        for task in task_list:
            if not task_objects_exist(task, context.world):
                unachievable_tasks.append(task)
        for unachievable_task in unachievable_tasks:
            print(f"[skip] task {unachievable_task.id}: object no longer in the world")
            unachievable_task.status = Status.SKIPPED
            task_list.remove(unachievable_task)
        if not task_list:
            break

        if current_task and current_task.status == Status.SUCCESS:
            scoretime_monitor.record_task_end(current_task)
        elif current_task and current_task.status == Status.RUNNING:
            scoretime_monitor.record_task_interrupt(current_task)

        with simulated_robot:
            sequential([ParkArmsAction(Arms.BOTH), MoveTorsoAction(TorsoState.LOW)], context).perform()

        if use_ground_truth_belief:
            found_objects = ground_truth_found_objects(context, task_list)

        evaluated_tasks: list[Task] = estimate_without_assistance(
            evaluator=evaluator, context=context, task_list=task_list, found_objects=found_objects,
            risk_aversion=risk_aversion)
        task_list: list[Task] = structurizer.structurize(task_list=evaluated_tasks)

        if min_pickup_probability > 0.0:
            declined = [
                t for t in task_list
                if any(step.action_type == ActionType.PICKUP
                       and step.action_probability < min_pickup_probability
                       for step in t.task_steps)
            ]
            for task in declined:
                print(f"[decline-p] task {task.id}: pickup probability below "
                      f"{min_pickup_probability} -> declining", flush=True)
                task.status = Status.SKIPPED
                task_list.remove(task)
            if not task_list:
                break

        if decline_negative_ev:
            best_ev = max((t.cum_expected_score or 0) for t in task_list)
            if best_ev <= 0:
                for remaining in task_list:
                    remaining.status = Status.SKIPPED
                print(f"[decline] best remaining expected score {best_ev:.2f} <= 0 -> "
                      f"declining {len(task_list)} task(s) and stopping", flush=True)
                break

        if not budget.unlimited:
            if budget.exhausted:
                for remaining in task_list:
                    remaining.status = Status.SKIPPED
                print(f"[budget] {budget.total_seconds:.0f}s modelled budget spent -> "
                      f"stopping with {len(task_list)} task(s) unattempted", flush=True)
                break
            candidate_cost = task_cost_seconds(task_list[0])
            if not budget.fits(candidate_cost):
                over_budget_task = task_list.pop(0)
                budget.note_skipped(f"task {over_budget_task.id}", candidate_cost)
                over_budget_task.status = Status.SKIPPED
                continue

        current_task = task_list[0]
        current_task.status = Status.RUNNING
        target = get_target_object_by_task(current_task)
        scoretime_monitor.record_task_start(current_task)

        if is_uncertain(current_task, found_objects):
            if enable_exploration:
                own_unscanned = unscanned_locations_for(challenge_mode, current_task, explored_locations)
                search_unscanned = unscanned_search_locations(
                    challenge_mode, task_list, explored_locations, found_objects)

                if not own_unscanned:
                    print(f"[explore] no location found for task {current_task.id} -> skipping")
                    task_list.remove(current_task)
                    continue

                scan_table(dispatcher, context,
                          closest_location(challenge_mode, search_unscanned or own_unscanned, hsrb),
                          hsrb, explored_locations, found_objects)
                update_tasks_from_belief(task_list, found_objects)
                continue  # re-rank now that the belief changed
            elif use_prior_guess:
                commit_to_prior(challenge_mode, current_task)
            else:
                commit_to_ground_truth(context, current_task)

        if enable_exploration and not scoretime_monitor.limited_time():
            detour_surface = opportunistic_exploration_target(
                challenge_mode=challenge_mode, task_list=task_list, current_task=current_task,
                robot=hsrb, explored=explored_locations, found=found_objects,
            )
            if detour_surface is not None:
                print(f"[explore] {detour_surface} is closer than the next step of task "
                      f"{current_task.id} -> scanning it on the way")
                scan_table(dispatcher, context, detour_surface, hsrb, explored_locations, found_objects)
                update_tasks_from_belief(task_list, found_objects)
                continue  # re-rank: what we just saw may promote a different task

        task_list.pop(0)
        budget.spend(task_cost_seconds(current_task))
        exec_steps = reposition_task_list(context=context, task_list=current_task.task_steps, robot=hsrb)
        plan = generate_plan_taskstep_list(taskstep_list=exec_steps, context=context)
        print("going for", target)

        task_wall_start = datetime.datetime.now()

        with simulated_robot:
            plans.append(plan)
            plan.perform()
            scoretime_monitor.record_plan_scores(task_steps=exec_steps, plan=plan)
            run_metrics.navigation_distance_meters += navigation_distance(plan)

            with simulated_robot:
                sequential([ParkArmsAction(Arms.BOTH), MoveTorsoAction(TorsoState.LOW)], context).perform()

            if plan.status == TaskStatus.SUCCEEDED:
                current_task.status = Status.SUCCESS

        with simulated_robot:
            sequential([ParkArmsAction(Arms.BOTH), MoveTorsoAction(TorsoState.LOW)], context).perform()

        task_duration = (datetime.datetime.now() - task_wall_start).total_seconds()
        attempt = build_task_attempt(
            exec_steps=exec_steps, plan=plan, context=context, duration_seconds=task_duration,
            was_uncertain=current_task.id in initially_uncertain_ids)
        if attempt is not None:
            run_metrics.task_attempts.append(attempt)
            if attempt.place_failed and remove_object_from_world(context.world, attempt.object_name):
                print(f"[cleanup] removed {attempt.object_name}: place failed, was blocking positions")

    # The loop books a task's measured time on the *next* iteration, so the task that emptied
    # the list would otherwise never be closed out.
    if current_task and current_task.status == Status.SUCCESS:
        scoretime_monitor.record_task_end(current_task)
    elif current_task and current_task.status == Status.RUNNING:
        scoretime_monitor.record_task_interrupt(current_task)

    print(f"[score] final score {scoretime_monitor.score} over "
          f"{len(scoretime_monitor.score_events)} recorded step(s), "
          f"{scoretime_monitor.time_taken_seconds_total()}s elapsed")

    run_metrics.completed = True
    run_metrics.run_duration_seconds = (datetime.datetime.now() - run_start).total_seconds()
    run_metrics.time_budget_seconds = budget.total_seconds
    run_metrics.modelled_seconds_spent = budget.spent_seconds
    run_metrics.tasks_skipped_over_budget = budget.skipped_over_budget
    return plans, scoretime_monitor, run_metrics


if __name__ == "__main__":
    from helper_methods import challenge_setup

    challenge_mode: ChallengeMode = ChallengeMode.PP
    world, dispatcher, context = challenge_setup(challenge_mode)
    estimation_execution(dispatcher, context, challenge_mode)