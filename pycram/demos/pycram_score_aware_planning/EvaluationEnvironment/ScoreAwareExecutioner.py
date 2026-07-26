
import datetime
import math
import time
from copy import deepcopy
from typing import Optional

import rclpy

from Evaluate.CompositeEvaluator import CompositeEvaluator
from SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.actions.simulate_perception import simulate_perception
from SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.events.event_handler import EventDispatcher
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from StaticExecution import navigation_distance
from Stabilizer.PlanStabilizer import PlanStabilizer
from common.cram_types import Task, TaskStep, ActionType, Status
from common.values import CHALLENGE_TASKS, ROOM_SURFACES, ROOM_SURFACES_PP, get_surfaces, get_rooms, lookup_operators
from demos.pycram_score_aware_planning.helper_methods import generic_object_spawner
from evaluation_metrics import RunMetrics
from time_budget import TimeBudget, task_cost_seconds
from helper_methods import generate_plan_taskstep_list, \
    NAVIGATION_POSES, at_location, get_navigation_poses, reposition_task_list, find_object_annotation, \
    perceived_instance, remove_object_from_world
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus, PlanTransformationOperator, Arms, ChallengeMode
from pycram.motion_executor import simulated_robot
from task_attempt_builder import build_task_attempt

from demos.pycram_score_aware_planning.common.hsrb_testing import setup_world
from demos.pycram_score_aware_planning.Structurizer.structurizer import PlanStructurizer
from pycram.plans.factories import sequential
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction
from semantic_digital_twin.datastructures.definitions import TorsoState, RoomEnum
from semantic_digital_twin.robots.abstract_robot import Manipulator, AbstractRobot
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.spatial_types import Point3, Quaternion
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.geometry import Color
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation, Body



def get_target_object_by_task(task: Task) -> SemanticAnnotation | None:
    """The object this task is about (first step that names one)."""
    for ts in task.task_steps:
        if ts.object_annotations is not None:
            return ts.object_annotations
    return None


def is_uncertain(task: Task, found: dict) -> bool:
    """
    Uncertain == we don't yet know where this task's object is.

    Uncertainty is a *question over the current belief* (found_objects)
    :param
    """
    target = get_target_object_by_task(task)
    if target is None:
        return False
    for ts in task.task_steps:
        if ts.uncertain:
            return True
        if not ts.uncertain:
            return False

    return perceived_instance(found, target) is None


def closest_location(challenge_mode: ChallengeMode, locations: list[str], robot) -> str:
    """Nearest navigable location to the robot's current base pose."""
    rx, ry = robot.root.global_pose.x, robot.root.global_pose.y
    best_location = locations[0]
    best_distance = math.dist((rx, ry), get_navigation_poses(challenge_mode, best_location)[:2])
    for loc in locations[1:]:
        distance = math.dist((rx, ry), get_navigation_poses(challenge_mode, loc)[:2])
        if distance < best_distance:
            best_distance = distance
            best_location = loc
    return best_location


def surfaces_in(challenge_mode : ChallengeMode, room: RoomEnum) -> list[str]:
    """The tables that can be scanned within a room (empty for an unknown room)."""
    return get_surfaces(challenge_mode, room)


def get_task_room(task: Task) -> Optional[RoomEnum]:
    """The coarse room prior of a task (first step that declares one)."""
    for ts in task.task_steps:
        if ts.room and ts.uncertain:
            return ts.room
    return None


def unscanned_locations_for(challenge_mode: ChallengeMode, task: Task,
                            explored: list[str]) -> list[str]:
    rooms = [get_task_room(task)] if get_task_room(task) else get_rooms(challenge_mode)
    locations = []
    for room in rooms:
        for surface in surfaces_in(challenge_mode, room) or []:
            if surface not in explored and surface not in locations:
                locations.append(surface)
    return locations


def unscanned_search_locations(challenge_mode: ChallengeMode, task_list: list[Task],
                               explored: list[str], found: dict) -> list[str]:
    locations = []
    for task in task_list:
        if not is_uncertain(task, found):
            continue
        for surface in unscanned_locations_for(challenge_mode, task, explored):
            if surface not in locations:
                locations.append(surface)
    return locations


OPPORTUNISTIC_MIN_TASK_PROBABILITY: float = 0.25



def first_planned_location(challenge_mode: ChallengeMode, task: Task) -> Optional[str]:
    for task_step in task.task_steps:
        if task_step.location and get_navigation_poses(challenge_mode, task_step.location) is not None:
            return task_step.location
    return None


def opportunistic_exploration_target(challenge_mode: ChallengeMode, task_list: list[Task],
                                     current_task: Task, robot: AbstractRobot,
                                     explored: list[str], found: dict) -> Optional[str]:
    planned_location = first_planned_location(challenge_mode, current_task)
    if planned_location is None:
        return None

    robot_x, robot_y = robot.root.global_pose.x, robot.root.global_pose.y
    best_distance = math.dist((robot_x, robot_y),
                              get_navigation_poses(challenge_mode, planned_location)[:2])
    best_surface = None

    for task in task_list:
        if task is current_task:
            continue
        if not is_uncertain(task, found):
            continue
        if task.probability < OPPORTUNISTIC_MIN_TASK_PROBABILITY:
            continue
        if task.cum_expected_score <= 0:
            continue

        for surface in surfaces_in(challenge_mode, get_task_room(task)) or []:
            if surface in explored:
                continue
            navigation_pose = get_navigation_poses(challenge_mode, surface)
            if navigation_pose is None:
                continue
            distance = math.dist((robot_x, robot_y), navigation_pose[:2])
            if distance < best_distance:
                best_distance = distance
                best_surface = surface

    return best_surface


def scan_table(dispatcher: EventDispatcher, context: Context, surface: str, robot: AbstractRobot,
               explored: list[str], found: dict) -> None:
    """
    Navigate to a single table and scan it.

    Records every perceived (non-furniture) object in `found`, keyed by the table it was seen on,
    and marks the table in `explored` so it is never rescanned across tasks. One table per call so
    the caller can re-evaluate the plan after each scan instead of sweeping a whole room blindly.
    """
    world = context.world
    explored.append(surface)
    print(f"[explore] scanning table {surface}")

    plan_explore = generate_plan_taskstep_list(
        taskstep_list=[TaskStep(action_type=ActionType.NAVIGATE, location=surface)], context=context)
    with simulated_robot:
        plan_explore.perform()

    for body in simulate_perception(world, dispatcher, context, robot) or []:
        if body not in dispatcher.known_furniture:
            found[world.get_semantic_annotation_by_name(body.name)] = surface


def task_objects_exist(task: Task, world) -> bool:
    for task_step in task.task_steps:
        if task_step.object_annotations is None:
            continue
        if find_object_annotation(world, task_step.object_annotations) is None:
            return False
    return True


def get_objects_of_interest(task_list: list[Task]) -> list[SemanticAnnotation]:
    objects_of_interest: list[SemanticAnnotation] = []
    for t in task_list:
        for ts in t.task_steps:
            if ts.object_annotations is not None:
                objects_of_interest.append(ts.object_annotations)
    return objects_of_interest

def update_tasks_from_belief(task_list: list[Task], found: dict) -> None:
    for task in task_list:
        target = get_target_object_by_task(task)
        if target is None:
            continue
        seen = perceived_instance(found, target)
        if seen is None:
            continue
        found_at = found[seen]
        if not found_at:
            continue
        for ts in task.task_steps:
            if ts.action_type in (ActionType.NAVIGATE, ActionType.PICKUP) and not ts.location:
                ts.location = found_at
            if ts.object_annotations is not None and isinstance(seen, ts.object_annotations):
                ts.perceived_object = seen
        task.uncertain = False

def commit_to_prior(challenge_mode: ChallengeMode, task: Task) -> None:
    room = get_task_room(task)
    surfaces = surfaces_in(challenge_mode, room) if room else []
    guess = surfaces[0] if surfaces else ""
    for task_step in task.task_steps:
        if task_step.action_type in (ActionType.NAVIGATE, ActionType.PICKUP) and not task_step.location:
            task_step.location = guess
    task.uncertain = False


def _reset_task_for_retry(task: Task) -> None:
    task.status = Status.NOT_ASSIGNED
    for step in task.task_steps:
        step.action_outcome = Status.NOT_ASSIGNED
        step.action_assisted = False
        step.nav_pose_override = None


def score_aware_execution(dispatcher : EventDispatcher, context: Context, challenge_mode : ChallengeMode,
                          enable_exploration: bool = True,
                          enabled_repair_operators: Optional[set] = None,
                          max_defers: int = 1,
                          time_budget_seconds: Optional[float] = None):

    hsrb = context.robot
    evaluator = CompositeEvaluator()
    structurizer = PlanStructurizer()
    budget = TimeBudget(time_budget_seconds)
    stabilizer = PlanStabilizer()
    # without the mode the challenge duration stays 0 and limited_time() is always True
    scoretime_monitor = ScoreTimeMonitor(challenge_mode=challenge_mode)

    explored_locations = []
    found_objects: dict[Body, str] = {}

    plans = [] # consisting of all plans ever generated
    repaired_plans = [] # consisting of (originalplan, repaired_plan)

    run_start = datetime.datetime.now()
    run_metrics = RunMetrics()

    task_list: list[Task] = CHALLENGE_TASKS.get(challenge_mode)
    defer_counts: dict = {}
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

        evaluated_tasks: list[Task] = evaluator.estimate(context=context, task_list=task_list,
                                                         found_objects=found_objects, risk_aversion=1)
        task_list: list[Task] = structurizer.structurize(task_list=evaluated_tasks)


        if not budget.unlimited:
            if budget.exhausted:
                for remaining in task_list:
                    remaining.status = Status.SKIPPED
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

                # searched everywhere we could and still don't know where it is -> give up on it
                if not own_unscanned:
                    print(
                        f"[explore] no location found for task {current_task.id} -> skipping")  # TODO: take assistance instead of skipping
                    task_list.remove(current_task)
                    continue

                # scan the single NEAREST unscanned table, fold what we see into the belief, then re-evaluate
                scan_table(dispatcher, context,
                           closest_location(challenge_mode, search_unscanned or own_unscanned, hsrb),
                           hsrb, explored_locations,
                           found_objects)
                # opportunistic: resolve EVERY task whose object we happened to perceive, not just this one
                update_tasks_from_belief(task_list, found_objects)
                print(
                    f"scanned and updated beliefstate")  # TODO: take assistance instead of skipping
                continue  # re-rank now that the belief changed -- a different task may now be best
            else:
                commit_to_prior(challenge_mode, current_task)

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
        final_plan, final_exec_steps = plan, exec_steps
        task_navigation_distance = 0.0

        with (simulated_robot):
            plans.append(plan)
            plan.perform()
            scoretime_monitor.record_plan_scores(task_steps=exec_steps, plan=plan)
            task_navigation_distance += navigation_distance(plan)

            with simulated_robot:
                sequential([ParkArmsAction(Arms.BOTH), MoveTorsoAction(TorsoState.LOW)], context).perform()

            task_hit_error = plan.status in (TaskStatus.INTERRUPTED, TaskStatus.FAILED)


            if (task_hit_error
                    and enabled_repair_operators is not None
                    and PlanTransformationOperator.REORDER in enabled_repair_operators
                    and defer_counts.get(current_task.id, 0) < max_defers):
                defer_counts[current_task.id] = defer_counts.get(current_task.id, 0) + 1
                _reset_task_for_retry(current_task)
                task_list.append(current_task)
                run_metrics.operator_usage[PlanTransformationOperator.REORDER.name] = (
                    run_metrics.operator_usage.get(PlanTransformationOperator.REORDER.name, 0) + 1)
                scoretime_monitor.record_task_interrupt(current_task)
                print(f"[defer] task {current_task.id} failed -> deferring "
                      f"(#{defer_counts[current_task.id]}) and re-estimating")
                continue

            if enabled_repair_operators is None:
                # default behaviour: the exception's own candidate list
                remaining_candidate_operators = lookup_operators(exception=plan.plan.root.reason)
            else:
                repair_priority = [
                    PlanTransformationOperator.REPOSITION_ROBOT,
                    PlanTransformationOperator.RETRY,
                    PlanTransformationOperator.SKIP,
                ]
                remaining_candidate_operators = [
                    op for op in repair_priority
                    if op == PlanTransformationOperator.SKIP or op in enabled_repair_operators
                ]
            while plan.status == TaskStatus.INTERRUPTED or plan.status == TaskStatus.FAILED:
                if not remaining_candidate_operators:
                    break

                repair = stabilizer.stabilize(plan=plan, task=current_task, exception=plan.plan.root.reason,
                                              scoretime_monitor=scoretime_monitor, context=context,
                                              exec_steps=exec_steps,
                                              remaining_candidate_operators=remaining_candidate_operators)
                if repair is None:
                    break

                operator = repair.operator
                repaired_plan = repair.plan
                repaired_task_list = repair.task_steps
                print(f"[stabilizer] {plan.reason} -> recover via {operator} "
                      f"(expected value {repair.expected_value:.1f}); "
                      f"re-performing {len(repaired_task_list)} step(s)")
                still_untried = []
                for candidate_operator in remaining_candidate_operators:
                    if candidate_operator != operator:
                        still_untried.append(candidate_operator)
                remaining_candidate_operators = still_untried
                run_metrics.operator_usage[operator.name] = run_metrics.operator_usage.get(operator.name, 0) + 1
                run_metrics.plan_stability_distances.append(repair.stability_distance)
                run_metrics.probability_stability_before.append(repair.probability_stability_before)
                run_metrics.probability_stability_after.append(repair.probability_stability_after)
                if operator == PlanTransformationOperator.SKIP:
                    current_task.status = Status.SKIPPED
                else:
                    repaired_plans.append(repaired_plan)
                    repaired_plan.perform()
                    scoretime_monitor.record_plan_scores(task_steps=repaired_task_list, plan=repaired_plan)
                    task_navigation_distance += navigation_distance(repaired_plan)
                    final_plan, final_exec_steps = repaired_plan, repaired_task_list
                    if repaired_plan.status == TaskStatus.SUCCEEDED:
                        plan.status = TaskStatus.SUCCEEDED
                        current_task.status = Status.SUCCESS


        with simulated_robot:
            sequential([ParkArmsAction(Arms.BOTH), MoveTorsoAction(TorsoState.LOW)], context).perform()

        run_metrics.navigation_distance_meters += task_navigation_distance
        task_duration = (datetime.datetime.now() - task_wall_start).total_seconds()
        attempt = build_task_attempt(
            exec_steps=final_exec_steps, plan=final_plan, context=context, duration_seconds=task_duration,
            error_during_execution=task_hit_error)
        if attempt is not None:
            run_metrics.task_attempts.append(attempt)
            if attempt.place_failed and remove_object_from_world(context.world, attempt.object_name):
                print(f"[cleanup] removed {attempt.object_name}: place failed, was blocking positions")

    # the loop books a task's time on the next iteration, so the last task needs closing here
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
    return plans, repaired_plans, run_metrics