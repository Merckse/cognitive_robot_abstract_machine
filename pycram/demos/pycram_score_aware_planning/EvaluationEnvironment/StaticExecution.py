import datetime
import logging
import random
from typing import Optional

from semantic_digital_twin.robots.hsrb import HSRB

from SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.actions.simulate_perception import simulate_perception
from SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.events.event_handler import EventDispatcher
from common.cram_types import ActionType
from common.values import STATIC_CHALLENGE_TASKS, StaticTask
from evaluation_metrics import AggregateMetrics, RunMetrics, TaskAttempt
from time_budget import TimeBudget
from helper_methods import challenge_setup, get_navigation_poses, compute_surface_spaces, \
    find_free_placement_pose, get_values, nav_time_between, origin_to_bottom_offset, remove_object_from_world
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import ChallengeMode, Arms, TaskStatus
from pycram.motion_executor import simulated_robot
from pycram.plans.factories import sequential, make_node
from pycram.plans.plan_node import ActionNode, PlanNode
from pycram.robot_plans.actions.core.navigation import NavigateAction
from pycram.robot_plans.actions.core.pick_up import PickUpAction
from pycram.robot_plans.actions.core.placing import PlaceAction, PlaceInTrashAction
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.spatial_types import Point3, Quaternion
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation

logger = logging.getLogger(__name__)

challenge_mode: ChallengeMode = ChallengeMode.PP

TORSO_LIFT_PLACEMENT_HEIGHT: float = 0.90

def navigate_to(location: str, context: Context):
    world = context.world
    x, y, (qx, qy, qz, qw) = get_navigation_poses(challenge_mode, location)
    pose = Pose(
        position=Point3(x, y, 0.0, reference_frame=world.root),
        orientation=Quaternion(qx, qy, qz, qw, reference_frame=world.root),
        reference_frame=world.root,
    )
    return NavigateAction(target_location=pose, keep_joint_states=True)


def surface_target_pose(location: str, object_body, world, robot) -> Optional[Pose]:
    surface_spaces = compute_surface_spaces(world=world)
    matched_surface = next((s for s in surface_spaces if s.name == location), None)
    if matched_surface is None:
        logger.warning(f"no surface named '{location}' in the world -> cannot place there")
        return None

    robot_position = robot.root.global_pose.to_position()
    free_spot = find_free_placement_pose(
        surface=matched_surface, object_body=object_body, world=world,
        reference_point=(float(robot_position.x), float(robot_position.y)),
    )
    if free_spot is None:
        logger.warning(f"no free placement spot left on '{location}'")
        return None
    x, y = free_spot

    origin_to_bottom = origin_to_bottom_offset(object_body)
    z = matched_surface.z_surface - origin_to_bottom + 0.002
    logger.debug(
        f"placing {object_body.name} on '{location}': z_surface={matched_surface.z_surface:.4f} "
        f"origin_to_bottom={origin_to_bottom:.4f} -> z={z:.4f}"
    )

    return Pose(
        position=Point3(x, y, z, reference_frame=world.root),
        orientation=robot.root.global_pose.to_quaternion(),
        reference_frame=world.root,
    )


def navigation_distance(plan_node: PlanNode) -> float:
    total = 0.0
    for node in plan_node.plan.nodes:
        if isinstance(node, ActionNode) and isinstance(node.designator, NavigateAction):
            data = node.execution_data
            if data is None or data.execution_end_pose is None:
                continue
            start = data.execution_start_pose.to_position()
            end = data.execution_end_pose.to_position()
            total += float(start.euclidean_distance(end))
    return total


def static_object_cost_seconds(annotation, pickup_location: str, place_location: str,
                               challenge_mode) -> float:
    _, _, pickup_time, _ = get_values(action_type=ActionType.PICKUP,
                                      semantic_annotation=type(annotation), location=pickup_location)
    _, _, place_time, _ = get_values(action_type=ActionType.PLACE,
                                     semantic_annotation=type(annotation), location=place_location)
    travel_time = 0
    from_pose = get_navigation_poses(challenge_mode, pickup_location)
    to_pose = get_navigation_poses(challenge_mode, place_location)
    if from_pose is not None and to_pose is not None:
        travel_time = round(nav_time_between(from_pose[0], from_pose[1], to_pose[0], to_pose[1]))
    return float(pickup_time + place_time + travel_time)


def static_execution(dispatcher: EventDispatcher, context: Context,
                     randomize_order: bool = False,
                     time_budget_seconds: Optional[float] = None) -> RunMetrics:
    robot : HSRB = context.robot
    world = context.world
    # the fixed sweep: a hardcoded order, walked once, no re-ranking and no revisiting
    challenge_relevant_places = ["trash_can", "dining_table", "extra_space", "cabinet", "lowerTable"]
    if randomize_order:
        random.shuffle(challenge_relevant_places)
        logger.info(f"[static] randomized sweep order: {challenge_relevant_places}")

    static_tasks: list[StaticTask] = STATIC_CHALLENGE_TASKS.get(challenge_mode, [])

    def task_for(annotation) -> Optional[StaticTask]:
        """The task this perceived object belongs to, exact type before subclass."""
        for static_task in static_tasks:
            if type(annotation) is static_task.object_annotation:
                return static_task
        for static_task in static_tasks:
            if isinstance(annotation, static_task.object_annotation):
                return static_task
        return None

    run_start = datetime.datetime.now()
    metrics = RunMetrics()
    handled_bodies: set = set()
    budget = TimeBudget(time_budget_seconds)
    budget_exhausted = False

    sweep_position = (float(robot.root.global_pose.to_position().x),
                      float(robot.root.global_pose.to_position().y))

    for place in challenge_relevant_places:
        if budget_exhausted:
            break

        # Deadline gate. Placed after ranking/declining and before the task is started, so the
        # ranker has already had its say and the budget only decides whether its top pick still
        # fits. A task that does not fit is skipped and the loop re-ranks rather than ending the
        # run -- see time_budget.py for why stopping on the first oversized task would measure the
        # stopping rule instead of the ordering.
        if not budget.unlimited:
            surface_pose = get_navigation_poses(challenge_mode, place)
            if surface_pose is not None:
                sweep_cost = float(round(nav_time_between(sweep_position[0], sweep_position[1],
                                                          surface_pose[0], surface_pose[1])))
                if budget.exhausted or not budget.fits(sweep_cost):
                    budget.note_skipped(f"sweep leg to {place}", sweep_cost)
                    budget_exhausted = True
                    break
                budget.spend(sweep_cost)
                sweep_position = (surface_pose[0], surface_pose[1])
        with simulated_robot:
            sequential([ParkArmsAction(Arms.BOTH), MoveTorsoAction(TorsoState.LOW)], context).perform()

        scan_plan = sequential(children=[make_node(navigate_to(place, context))], context=context)
        with simulated_robot:
            scan_plan.perform()
        metrics.navigation_distance_meters += navigation_distance(scan_plan)

        found: list[SemanticAnnotation] = []
        for body in simulate_perception(world, dispatcher, context, robot) or []:
            if body not in dispatcher.known_furniture:
                found.append(world.get_semantic_annotation_by_name(body.name))

        if randomize_order:
            random.shuffle(found)

        for annotation in found:
            object_body = annotation.bodies[0]

            # An object is handled once per run. Without this the baseline double-counts itself:
            # the destinations are themselves scanned later (dining_table objects are stored on
            # shelf_4, and "cabinet" -- scanned afterwards -- is exactly where shelf_4 lives), so
            # an object moved earlier is perceived again at its new home and carried back. That
            # inflates tasks_started and navigation_distance with work the baseline was never
            # asked to do, which flatters nothing but does distort every per-task average.
            if object_body in handled_bodies:
                logger.debug(f"{object_body.name} already handled this run -> not repeating it")
                continue
            handled_bodies.add(object_body)

            static_task = task_for(annotation)
            if static_task is None:
                pass
            elif not budget.unlimited:
                object_cost = static_object_cost_seconds(annotation, place,
                                                         static_task.place_location, challenge_mode)
                if budget.exhausted:
                    logger.info(f"[budget] {budget.total_seconds:.0f}s modelled budget spent "
                                f"-> ending the sweep")
                    budget_exhausted = True
                    break
                if not budget.fits(object_cost):
                    budget.note_skipped(str(object_body.name), object_cost)
                    continue
                budget.spend(object_cost)

            if static_task is None:
                logger.debug(f"{object_body.name} is not part of the task universe -> ignoring it")
                continue
            store = static_task.place_location

            try:
                attempt = handle_object(annotation, object_body, place, static_task,
                                        context, robot, world, metrics)
            except Exception:
                logger.exception(f"unhandled error while handling {object_body.name} -> recording it as a failure")
                attempt = TaskAttempt(
                    object_name=str(object_body.name),
                    pickup_location=place,
                    place_location=store,
                    pickup_status=TaskStatus.INTERRUPTED,
                    place_status=TaskStatus.INTERRUPTED,
                    assisted=False,
                    score=0,
                    penalty=crash_penalty(annotation, place, store),
                    duration_seconds=0.0,
                    failure_reason="unhandled exception in handle_object",
                    object_type=type(annotation).__name__,
                    is_trash=static_task.action_type == ActionType.THROW_AWAY,
                )
            metrics.task_attempts.append(attempt)

    with simulated_robot:
        sequential([ParkArmsAction(Arms.BOTH), MoveTorsoAction(TorsoState.LOW)], context).perform()

    metrics.completed = True
    metrics.run_duration_seconds = (datetime.datetime.now() - run_start).total_seconds()
    metrics.time_budget_seconds = budget.total_seconds
    metrics.modelled_seconds_spent = budget.spent_seconds
    metrics.tasks_skipped_over_budget = budget.skipped_over_budget
    return metrics


def failure_reason_of(*plans) -> Optional[str]:
    for plan in plans:
        if plan is None:
            continue
        if plan.status not in TaskAttempt.ERROR_STATUSES:
            continue
        reason = getattr(plan.plan.root, "reason", None) or getattr(plan, "reason", None)
        if reason is not None:
            return type(reason).__name__
        return f"unknown ({plan.status.name})"
    return None


def report_placement(object_body, store: str, target_location, place_plan, robot) -> None:
    print(f"\n[place] ---- {object_body.name} -> '{store}' ----")
    if target_location is None:
        print("[place] no target pose was produced (no surface match, or no free spot)")
    else:
        target_position = target_location.to_position() if hasattr(target_location, "to_position") else target_location
        print(f"[place] target   x={float(target_position.x):+.4f} y={float(target_position.y):+.4f} "
              f"z={float(target_position.z):+.4f}")

    try:
        actual = object_body.global_pose.to_position()
        print(f"[place] actual   x={float(actual.x):+.4f} y={float(actual.y):+.4f} z={float(actual.z):+.4f}")
        if target_location is not None:
            target_position = target_location.to_position() if hasattr(target_location, "to_position") else target_location
            print(f"[place] delta    dx={float(actual.x) - float(target_position.x):+.4f} "
                  f"dy={float(actual.y) - float(target_position.y):+.4f} "
                  f"dz={float(actual.z) - float(target_position.z):+.4f}   "
                  f"(dz < 0 means it ended up lower than asked for)")
    except Exception as error:
        print(f"[place] could not read the object's pose afterwards: {error}")

    print(f"[place] status   {place_plan.status}")
    reason = getattr(place_plan.plan.root, "reason", None) or getattr(place_plan, "reason", None)
    if reason is not None:
        print(f"[place] reason   {type(reason).__name__}: {str(reason)[:300]}")
    robot_position = robot.root.global_pose.to_position()
    print(f"[place] robot at x={float(robot_position.x):+.4f} y={float(robot_position.y):+.4f}")


def crash_penalty(annotation: SemanticAnnotation, place: str, store: str) -> int:
    """Both sub-steps' penalties, for a task that blew up before it could report its own."""
    _, pickup_penalty, _, _ = get_values(
        action_type=ActionType.PICKUP, semantic_annotation=type(annotation), location=place)
    _, place_penalty, _, _ = get_values(
        action_type=ActionType.PLACE, semantic_annotation=type(annotation), location=store)
    return pickup_penalty + place_penalty


def handle_object(annotation: SemanticAnnotation, object_body, place: str, static_task: StaticTask,
                  context: Context, robot, world, metrics: RunMetrics) -> TaskAttempt:
    store = static_task.place_location
    _, _, _, pickup_success_probability = get_values(
        action_type=ActionType.PICKUP, semantic_annotation=type(annotation), location=place)
    pickup_plan = sequential(children=[
        make_node(navigate_to(place, context)),
        make_node(PickUpAction(object_geometry=object_body, arm=Arms.LEFT,
                               success_probability=pickup_success_probability)),
        make_node(ParkArmsAction(Arms.LEFT)),
        make_node(navigate_to(store, context)),
    ], context=context)
    with simulated_robot:
        pickup_plan.perform()
    metrics.navigation_distance_meters += navigation_distance(pickup_plan)

    place_action = None
    place_plan = None
    place_status = TaskStatus.CREATED
    if pickup_plan.status == TaskStatus.SUCCEEDED:
        if static_task.action_type == ActionType.THROW_AWAY:
            place_action = PlaceInTrashAction(object_designator=object_body, arm=Arms.LEFT)
        else:
            target_location = surface_target_pose(store, object_body, world, robot)
            if target_location is None:
                place_status = TaskStatus.INTERRUPTED
            else:
                _, _, _, place_success_probability = get_values(
                    action_type=ActionType.PLACE, semantic_annotation=type(annotation), location=store)
                place_action = PlaceAction(object_designator=object_body, arm=Arms.LEFT,
                                           target_location=target_location,
                                           success_probability=place_success_probability)
        if place_action is not None:
            place_children = []
            target_position = getattr(place_action, "target_location", None)
            if target_position is not None and hasattr(target_position, "to_position"):
                if float(target_position.to_position().z) >= TORSO_LIFT_PLACEMENT_HEIGHT:
                    place_children.append(make_node(MoveTorsoAction(TorsoState.HIGH)))
            place_children += [
                make_node(place_action),
                make_node(ParkArmsAction(Arms.LEFT)),
            ]
            place_plan = sequential(children=place_children, context=context)
            with simulated_robot:
                place_plan.perform()
            metrics.navigation_distance_meters += navigation_distance(place_plan)
            place_status = place_plan.status
            report_placement(object_body, store, getattr(place_action, "target_location", None),
                             place_plan, robot)

    pickup_score, pickup_penalty, _, _ = get_values(
        action_type=ActionType.PICKUP, semantic_annotation=type(annotation), location=place)
    place_score, place_penalty, _, _ = get_values(
        action_type=ActionType.PLACE, semantic_annotation=type(annotation), location=store)
    task_score = (pickup_score if pickup_plan.status == TaskStatus.SUCCEEDED else 0) + \
                 (place_score if place_status == TaskStatus.SUCCEEDED else 0)
    task_penalty = (pickup_penalty if pickup_plan.status in TaskAttempt.ERROR_STATUSES else 0) + \
                   (place_penalty if place_status in TaskAttempt.ERROR_STATUSES else 0)

    task_end_time = None
    if place_plan is not None and place_plan.end_time is not None:
        task_end_time = place_plan.end_time
    elif pickup_plan.end_time is not None:
        task_end_time = pickup_plan.end_time
    duration_seconds = (task_end_time - pickup_plan.start_time).total_seconds() if task_end_time else 0.0

    attempt = TaskAttempt(
        object_name=str(object_body.name),
        pickup_location=place,
        place_location=store,
        pickup_status=pickup_plan.status,
        place_status=place_status,
        assisted=bool(getattr(place_action, "assisted", False)
                      or getattr(pickup_plan, "assisted", False)),
        score=task_score,
        penalty=task_penalty,
        duration_seconds=duration_seconds,
        failure_reason=failure_reason_of(pickup_plan, place_plan),
        object_type=type(annotation).__name__,
        is_trash=static_task.action_type == ActionType.THROW_AWAY,
    )
    if attempt.place_failed and remove_object_from_world(world, attempt.object_name):
        print(f"[cleanup] removed {attempt.object_name}: place failed, was blocking positions")
    return attempt


if __name__ == "__main__":
    world, dispatcher, context = challenge_setup(challenge_mode)
    run_metrics = static_execution(dispatcher, context)
    print(AggregateMetrics(n_runs_attempted=1, runs=[run_metrics]).summary())
