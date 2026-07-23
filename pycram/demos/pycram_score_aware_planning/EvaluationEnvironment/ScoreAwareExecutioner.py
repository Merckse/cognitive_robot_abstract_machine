
import math
import time
from copy import deepcopy
from typing import Optional

import rclpy

from Evaluate.CompositeEvaluator import CompositeEvaluator
from SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.actions.simulate_perception import simulate_perception
from SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.events.event_handler import EventDispatcher
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from Stabilizer.PlanStabilizer import PlanStabilizer
from common.cram_types import Task, TaskStep, ActionType, Status
from common.values import CHALLENGE_TASKS, ROOM_SURFACES, ROOM_SURFACES_PP, get_surfaces, lookup_operators
from demos.pycram_score_aware_planning.helper_methods import generic_object_spawner
from helper_methods import generate_plan_taskstep_list, \
    NAVIGATION_POSES, at_location, get_navigation_poses
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus, PlanTransformationOperator, Arms, ChallengeMode
from pycram.motion_executor import simulated_robot

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
    # `target` is the annotation *class* (e.g. Bowl); `seen` keys are perceived *instances*.
    return not any(isinstance(seen, target) for seen in found)


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
        found_at = ""
        for seen, loc in found.items():
            if isinstance(seen, target):
                found_at = loc
                break
        if not found_at:
            continue
        for ts in task.task_steps:
            if ts.action_type in (ActionType.NAVIGATE, ActionType.PICKUP) and not ts.location:
                ts.location = found_at
        task.uncertain = False
def add_navigation(context: Context, explored ,task_list: list[TaskStep]) -> None:
    temp_robot = deepcopy(context.robot)
    exec_steps = []
    for ts in task_list:
        if ts.location != "" and not at_location(location=ts.location, robot=temp_robot):
            # location known -> place the robot at the approach pose so distance is measured from there
            if ts.location in NAVIGATION_POSES:
                x, y, _ = NAVIGATION_POSES[ts.location]
                temp_robot.root.global_pose.x = x
                temp_robot.root.global_pose.y = y
            exec_steps.append(TaskStep(action_type=ActionType.NAVIGATE, location=ts.location))
        exec_steps.append(ts)

def score_aware_execution(dispatcher : EventDispatcher, context: Context, challenge_mode : ChallengeMode):
    hsrb = context.robot
    evaluator = CompositeEvaluator()
    structurizer = PlanStructurizer()
    stabilizer = PlanStabilizer()
    scoretime_monitor = ScoreTimeMonitor()

    explored_locations = []
    found_objects: dict[Body, str] = {}

    plans = [] # consisting of all plans ever generated
    repaired_plans = [] # consisting of (originalplan, repaired_plan)

    task_list: list[Task] = CHALLENGE_TASKS.get(challenge_mode)
    current_task = None
    while task_list != []:

        if current_task and current_task.status == Status.SUCCESS:
            scoretime_monitor.record_task_end(current_task)
        elif current_task and current_task.status == Status.RUNNING:
            scoretime_monitor.record_task_interrupt(current_task)

        with simulated_robot:
            sequential([ParkArmsAction(Arms.BOTH), MoveTorsoAction(TorsoState.LOW)], context).perform()

        evaluated_tasks: list[Task] = evaluator.estimate(context=context, task_list=task_list,
                                                         found_objects=found_objects, risk_aversion=1)
        task_list: list[Task] = structurizer.structurize(task_list=evaluated_tasks)

        current_task = task_list[0]
        current_task.status = Status.RUNNING
        target = get_target_object_by_task(current_task)

        scoretime_monitor.record_task_start(current_task)

        # A room prior scopes the search to a few tables; with no prior we consider every room.
        if is_uncertain(current_task, found_objects):
            prior = get_task_room(current_task)
            try:
                candidate_rooms = [prior] if prior else list(get_surfaces(challenge_mode, prior))
                # every not-yet-scanned table across the candidate rooms
                unscanned_tables = []
                for r in candidate_rooms:
                    for s in surfaces_in(challenge_mode, r):
                        if s not in explored_locations:
                            unscanned_tables.append(s)
            except Exception as e:
                continue
            # searched everywhere we could and still don't know where it is -> give up on it
            if not unscanned_tables:
                print(
                    f"[explore] no location found for task {current_task.id} -> skipping")  # TODO: take assistance instead of skipping
                task_list.remove(current_task)
                continue

            # scan the single NEAREST unscanned table, fold what we see into the belief, then re-evaluate
            scan_table(dispatcher, context, closest_location(challenge_mode, unscanned_tables, hsrb), hsrb, explored_locations,
                       found_objects)
            # opportunistic: resolve EVERY task whose object we happened to perceive, not just this one
            update_tasks_from_belief(task_list, found_objects)
            print(
                f"scanned and updated beliefstate")  # TODO: take assistance instead of skipping
            continue  # re-rank now that the belief changed -- a different task may now be best

        task_list.pop(0)

        # TODO: make cleaner with nice method
        # Build navigation-inserted steps in a LOCAL list -- do NOT mutate current_task.task_steps,
        # otherwise re-processing the same task (re-rank / recovery) re-prepends navigation forever.
        exec_steps = []
        for ts in current_task.task_steps:
            if ts.location != "" and not at_location(context=context,location=ts.location, robot=hsrb):
                exec_steps.append(TaskStep(action_type=ActionType.NAVIGATE, location=ts.location))
            exec_steps.append(ts)

        plan = generate_plan_taskstep_list(taskstep_list=exec_steps, context=context)
        print("going for", target)

        with (simulated_robot):
            scoretime_monitor.record_task_start(task=current_task)
            plans.append(plan)
            plan.perform()

            with simulated_robot:
                sequential([ParkArmsAction(Arms.BOTH), MoveTorsoAction(TorsoState.LOW)], context).perform()

            remaining_candidate_operators = lookup_operators(exception=plan.plan.root.reason)
            while plan.status == TaskStatus.INTERRUPTED or plan.status == TaskStatus.FAILED:
                if not remaining_candidate_operators:
                    break

                winning = stabilizer.stabilize(plan=plan, task=current_task, exception=plan.plan.root.reason,
                                               scoretime_monitor=scoretime_monitor, context=context,
                                               exec_steps=exec_steps,
                                               remaining_candidate_operators=remaining_candidate_operators)
                if winning is None:
                    break

                operator, expected_value, repaired_plan, repaired_task_list = winning
                print(f"[stabilizer] {plan.reason} -> recover via {operator} "
                      f"(expected value {expected_value:.1f}); re-performing {len(repaired_task_list)} step(s)")
                remaining_candidate_operators = [op for op in remaining_candidate_operators if op != operator]
                if operator == PlanTransformationOperator.SKIP:
                    current_task.status = Status.SKIPPED
                else:
                    repaired_plans.append(repaired_plan)
                    repaired_plan.perform()
                    if repaired_plan.status == TaskStatus.SUCCEEDED:
                        plan.status = TaskStatus.SUCCEEDED
                        current_task.status = Status.SUCCESS


        with simulated_robot:
            sequential([ParkArmsAction(Arms.BOTH), MoveTorsoAction(TorsoState.LOW)], context).perform()

    return plans, repaired_plans