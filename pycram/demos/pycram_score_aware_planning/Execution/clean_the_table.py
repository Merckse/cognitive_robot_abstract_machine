import math

import rclpy

from Evaluate.CompositeEvaluator import CompositeEvaluator
from SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.actions.simulate_perception import simulate_perception
from SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.events.event_handler import EventDispatcher
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from Stabilizer.PlanStabilizer import PlanStabilizer
from common.cram_types import Task, TaskStep, ActionType, Status
from common.values import CHALLENGE_TASKS, ROOM_SURFACES
from demos.pycram_score_aware_planning.helper_methods import generic_object_spawner
from helper_methods import generate_plan_taskstep_list, \
    NAVIGATION_POSES
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus, PlanTransformationOperator
from pycram.motion_executor import simulated_robot

from demos.pycram_score_aware_planning.common.hsrb_testing import setup_world
from demos.pycram_score_aware_planning.common.cram_types import ChallengeMode
from demos.pycram_score_aware_planning.Structurizer.structurizer import PlanStructurizer
from semantic_digital_twin.robots.abstract_robot import Manipulator, AbstractRobot
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.spatial_types import Point3, Quaternion
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.geometry import Color
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation, Body

"""
# ------------- TODO ------------- #
STILL TODO: 
- [] calculate NAVIGATION, dynamically and implement it into the net of shit
- [] calculate the navigation time into the explore time, so it gets considered
- [] implement the Navigation at the proper time, so it actually gets considered
- [] implement more sophisticated plan-stability model
- [] 
- [] 



TASK completion under:
- [] Plan stability evaluation - basic

    CERTAINTY:
    [] - Task rescheduling
    [] - post-iori changes
    [] - Navigation time estimation
    
DOs:
    [] - navigation duration estimate for spm

EXECUTION:
    [] - outsourcing the execution to the Executionor

# ------------- to implement today ------------- #
 [] - Distance navigation time
 [] - estimated time for action based on constraints
 [] - calculating taken time and time left
 [] - check for failure after action?
 [] - Base checks for possibility of action (height, misc.)
"""

"""
implement and actually use hannas contents.
Meaning, I want to now only work with objects, that have been found and located. If a object hasnt been located, then there is uncertainty 
and the object, needs to be found first.
Meaning - Object uncertainty = True - Explore

Resulting in smth like this: If a task a is selected and object a.) hasnt been found - go explore all locations.

Imagine you have two simulations, for actual simulation and you act towards the bot, as if it doesnt know the knowledge of sem-dt until it detected objects.
Means. I calculate the tasks broadly

Means: I first have to adjust the probability and score to work with uncertainty, since I dont know anything about the objects.
"""

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
    # `target` is the annotation *class* (e.g. Bowl); `seen` keys are perceived *instances*.
    return not any(isinstance(seen, target) for seen in found)


def closest_location(locations: list[str], robot) -> str:
    """Nearest navigable location to the robot's current base pose."""
    rx, ry = robot.root.global_pose.x, robot.root.global_pose.y
    return min(locations, key=lambda loc: math.dist(
        (rx, ry), (NAVIGATION_POSES[loc][0], NAVIGATION_POSES[loc][1])))

def surfaces_in(room: str) -> list[str]:
    """The tables that can be scanned within a room (empty for an unknown room)."""
    return ROOM_SURFACES.get(room, [])


def get_task_room(task: Task) -> str:
    """The coarse room prior of a task (first step that declares one)."""
    for ts in task.task_steps:
        if ts.room and ts.uncertain:
            return ts.room
    return ""

def at_location(location : str, robot : AbstractRobot, threshold: float = 0.5)-> bool:
    coords_goal = NAVIGATION_POSES.get(location)
    pose_goal : Pose= Pose(Point3(x=coords_goal[0], y=coords_goal[1], z=0), Quaternion(coords_goal[2][0], coords_goal[2][1], coords_goal[2][2], coords_goal[2][3]))
    point3_goal : Point3 = pose_goal.to_position()
    robot_pose : Point3= robot.root.global_pose.to_position()

    is_at_location : bool = True if float(robot_pose.euclidean_distance(point3_goal)) < threshold else False
    return is_at_location

def scan_table(dispatcher : EventDispatcher, context : Context, surface: str, robot: AbstractRobot,
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
    """
    Propagate the shared belief onto *every* task, not just the current one.

    For each task whose target object has been perceived somewhere (it appears in `found`),
    write that location onto the task's still-unlocated NAVIGATE/PICKUP steps and clear its
    uncertainty. This is what makes perception opportunistic: spotting task B's object while
    scanning for task A immediately resolves task B, so the next structurize() ranks it as
    known instead of re-exploring for it later.
    """
    for task in task_list:
        target = get_target_object_by_task(task)
        if target is None:
            continue
        found_at = next((loc for seen, loc in found.items() if isinstance(seen, target)), "")
        if not found_at:
            continue
        for ts in task.task_steps:
            if ts.action_type in (ActionType.NAVIGATE, ActionType.PICKUP) and not ts.location:
                ts.location = found_at
        task.uncertain = False

def main():
    # HSRB specified local world setup
    world, dispatcher = setup_world()
    evaluator = CompositeEvaluator()
    structurizer = PlanStructurizer()
    stabilizer = PlanStabilizer()
    scoretime_monitor = ScoreTimeMonitor()

    hsrb = HSRB.from_world(world)

    explored_locations = []
    found_objects: dict[Body, str] = {}

    context = Context(world=world, robot=hsrb)
    context.evaluate_conditions = False
    dispatcher.known_furniture = world.bodies

    challenge_mode: ChallengeMode = ChallengeMode.PP
    task_list: list[Task] = CHALLENGE_TASKS.get(challenge_mode)

    # Spawning objects
    generic_object_spawner(["Bowl"], [(1.325, 6.23, 0.81)], world, color=Color.GREEN())
    generic_object_spawner(["Plate"], [(-0.15, 0.88, 0.85)], world, color=Color.ORANGE())
    # generic_object_spawner(["Milk"], [(2.42, 0.128, 0.945)], world, color=Color.RED())
    generic_object_spawner(["Milk"], [(1.037, -2.31, 0.645)], world, color=Color.RED())
    generic_object_spawner(["Knife"], [(4.65, 4.84, 1.62)], world, color=Color.CYAN())
    generic_object_spawner(["Apple"], [(4.135, 1.865, 0.54)], world, color=Color.WHITE())
    # generic_object_spawner(["Cereal"], [(1.037, -2.31, 0.645)], world, color=Color.BLUE())
    generic_object_spawner(["Cereal"], [(2.42, 0.128, 0.945)], world, color=Color.BLUE())
    generic_object_spawner(["Cup"], [(2.33475, 5.215, 0.83)], world, color=Color.BEIGE())

    while task_list != []:
        evaluated_tasks: list[Task] = evaluator.estimate(context=context, task_list=task_list, found_objects=found_objects, risk_aversion=1)
        task_list: list[Task] = structurizer.structurize(task_list=evaluated_tasks)

        current_task = task_list[0]
        current_task.status = Status.RUNNING
        target = get_target_object_by_task(current_task)

        # ---- TOP TASK UNKNOWN -> scan ONE table, then loop back to RE-RANK (don't execute yet) ----
        # A room prior scopes the search to a few tables; with no prior we consider every room.
        if is_uncertain(current_task, found_objects):
            prior = get_task_room(current_task)
            candidate_rooms = [prior] if prior else list(ROOM_SURFACES.keys())
            # every not-yet-scanned table across the candidate rooms
            unscanned_tables = [s for r in candidate_rooms for s in surfaces_in(r) if s not in explored_locations]

            # searched everywhere we could and still don't know where it is -> give up on it
            if not unscanned_tables:
                print(f"[explore] no location found for task {current_task.id} -> skipping")  # TODO: take assistance instead of skipping
                task_list.remove(current_task)
                continue

            # scan the single NEAREST unscanned table, fold what we see into the belief, then re-evaluate
            scan_table(dispatcher, context, closest_location(unscanned_tables, hsrb), hsrb, explored_locations, found_objects)
            # opportunistic: resolve EVERY task whose object we happened to perceive, not just this one
            update_tasks_from_belief(task_list, found_objects)
            print(
                f"scanned and updated beliefstate")  # TODO: take assistance instead of skipping
            continue  # re-rank now that the belief changed -- a different task may now be best

        # ---- TOP TASK KNOWN -> execute it ----
        task_list.pop(0)

        # Build navigation-inserted steps in a LOCAL list -- do NOT mutate current_task.task_steps,
        # otherwise re-processing the same task (re-rank / recovery) re-prepends navigation forever.
        exec_steps = []
        for ts in current_task.task_steps:
            if ts.location != "" and not at_location(location=ts.location, robot=hsrb):
                exec_steps.append(TaskStep(action_type=ActionType.NAVIGATE, location=ts.location))
            exec_steps.append(ts)


        plan = generate_plan_taskstep_list(taskstep_list=exec_steps, context=context)
        print("going for", target)

        with (simulated_robot):
            scoretime_monitor.record_task_start(task=current_task)
            plan.perform()
            if plan.status == TaskStatus.INTERRUPTED or plan.status == TaskStatus.FAILED:
                winning = stabilizer.stabilize(plan=plan, task=current_task, exception=plan.reason,
                                               scoretime_monitor=scoretime_monitor, context=context)
                if winning is not None:
                    operator, expected_value, repaired_plan, repaired_task_list = winning
                    print(f"[stabilizer] {plan.reason} -> recover via {operator} "
                          f"(expected value {expected_value:.1f}); re-performing {len(repaired_task_list)} step(s)")
                    if operator == PlanTransformationOperator.SKIP:
                        # SKIP = give up and move on; do NOT re-append (that retries the same failure forever)
                        current_task.status = Status.SKIPPED
                    else:
                        # splice in the winning repair as a fresh plan and re-perform from the failure onward
                        recovery_plan = stabilizer.build_recovery_plan(repaired_task_list, context)
                        recovery_plan.perform()

if __name__ == "__main__":
    main()