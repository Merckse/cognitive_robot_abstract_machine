import logging
import math
import os
import random
from copy import deepcopy
from random import randint
from time import sleep
from typing import Optional

from plotly.graph_objs.indicator.gauge import step

from common.cram_types import Status
from common.values import evaluation
from probabilistic_model.bayesian_network.bayesian_network import Root

from giskardpy.motion_statechart.goals.place import PlacementNotReachableException
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment, ChallengeMode
from pycram.datastructures.grasp import GraspDescription
from pycram.language import SequentialNode
from pycram.plans.factories import sequential
from pycram.plans.plan_node import PlanNode
from pycram.robot_plans.actions.base import ActionDescription
from pycram.robot_plans.actions.core.container import OpenAction
from pycram.robot_plans.actions.core.navigation import NavigateAction
from pycram.robot_plans.actions.core.pick_up import PickUpAction, RequestObjectRepositionAction
from pycram.robot_plans.actions.core.placing import PlaceAction, PlaceInTrashAction
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction
from demos.pycram_score_aware_planning.common.cram_types import Task, ActionType, SurfaceSpace, TaskStep
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.exceptions import WorldEntityNotFoundError
from semantic_digital_twin.reasoning.predicates import is_supported_by
from semantic_digital_twin.reasoning.queries import annotation_class_by_label
from semantic_digital_twin.robots.abstract_robot import Manipulator, AbstractRobot
from semantic_digital_twin.semantic_annotations.mixins import HasRootBody, HasSupportingSurface, HasDoors, HasDrawers, \
    IsPerceivable
from semantic_digital_twin.semantic_annotations.semantic_annotations import Knife, Fork, Spoon, DishwasherTab, Milk, \
    Bowl, Plate, Table, Mug, Cup, Cereal, Bottle, Pringles, Apple, Sofa, Dishwasher
from semantic_digital_twin.spatial_types import (
    Point3,
    Quaternion,
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection, ActiveConnection1DOF, Connection6DoF
from semantic_digital_twin.world_description.geometry import Scale, Color
from semantic_digital_twin.world_description.world_entity import (
    Body, SemanticAnnotation,
)

import numpy as np


from pycram.datastructures.dataclasses import Context

from demos.pycram_score_aware_planning.common.hsrb_testing import setup_world
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.world_description.geometry import Color


logger = logging.getLogger(__name__)


def move_object_to_new_pose(
    semantic_annotation: HasRootBody, new_transform: HomogeneousTransformationMatrix
):
    world = semantic_annotation._world
    new_transform_world = world.transform(new_transform, world.root)
    parent_connection = semantic_annotation.root.parent_connection
    parent_connection_parent = parent_connection.parent
    parent_connection_child = parent_connection.child
    new_transform_world.reference_frame = parent_connection_parent
    new_transform_world.child_frame = parent_connection_child
    new_parent_connection = FixedConnection(
        parent=parent_connection_parent,
        child=parent_connection_child,
        parent_T_connection_expression=new_transform_world,
    )
    world.remove_connection(parent_connection)
    world.add_connection(new_parent_connection)


def spawn_semantic_with_body(
    semantic_type: HasRootBody | str,
    name: str,
    scale: Scale,
    pose: Pose,
    world: World,
    color: Optional[Color] = Color.WHITE(),
):
    """
    Spawns a semantic annotation with a body in the world based on the provided information.
    If an annotation with the same name already exists, it is removed before spawning the new one.

    :param semantic_type: The type of the semantic annotation to spawn
    :param name: The name of the semantic annotation to spawn
    :param scale: The scale of the object to spawn
    :param pose: The pose of the object to spawn
    :param world: The world in which to spawn the object
    :param color: The color of the object to spawn
    :return: The spawned semantic annotation
    """

    semantic_type = annotation_class_by_label(semantic_type)

    pose.z -= 0.015  # To avoid spawning objects in the air due to small inaccuracies in the pose estimation.

    if pose.reference_frame is not None and pose.reference_frame != world.root:
        world_root_T_self = world.transform(pose, world.root).to_homogeneous_matrix()
    else:
        world_root_T_self = pose.to_homogeneous_matrix()
        world_root_T_self.reference_frame = world.root

    try:
        object_to_spawn: HasRootBody = world.get_semantic_annotation_by_name(name)
        with world.modify_world():
            move_object_to_new_pose(object_to_spawn, world_root_T_self)
    except WorldEntityNotFoundError:
        with world.modify_world():
            object_to_spawn = semantic_type.create_with_new_body_in_world(
                name=PrefixedName(name),
                world=world,
                scale=scale,
                world_root_T_self=world_root_T_self,
            )
            object_to_spawn.bodies[0].visual.shapes[0].color = color
    return object_to_spawn


def generic_object_spawner(
        names: list[str] ,
        pose : list[Pose | tuple[float, float, float]],
        world : World,
        custom_name : list[str] = None,
        color: list[Color] = Color.WHITE(),
):
    i = 0
    for name in names:
        curr_pose = pose[i]
        if isinstance(curr_pose, (tuple, list)):
            x, y, z = curr_pose[0], curr_pose[1], curr_pose[2]
        else:
            x, y, z = curr_pose.x, curr_pose.y, curr_pose.z
        scale_: Scale = Scale(x=0.05, y=0.05, z=0.2)
        try:
            if name.lower() == "bottle":
                raise ValueError("bottle mesh disabled (ungraspable size)")
            object = STLParser(
                os.path.join(
                    os.path.dirname(__file__), "..", "..", "resources", "objects", name.lower()+".stl"
                )
            ).parse()

            object.root.name = PrefixedName(name.lower())
            if custom_name:
                object.root.name = PrefixedName(custom_name.lower())

            # meshes are modeled around their own origin, so lift the body until the mesh bottom rests on z
            mesh_bottom_z = object.root.combined_mesh.bounds[0][2]
            with world.modify_world():
                world.merge_world_at_pose(
                    object,
                    HomogeneousTransformationMatrix.from_xyz_quaternion(
                        x, y, z - mesh_bottom_z, reference_frame=world.root
                    ),
                )
            body = world.get_body_by_name(name.lower())
            semantic_cls = annotation_class_by_label(name)
            with world.modify_world():
                world.add_semantic_annotation(semantic_cls(name=body.name, root=body))
            shapes = body.visual.shapes
            for s in shapes:
                s.color = color
        except:
            # fallback boxes have their origin in the center -> lift by half the box height
            pose_point3_: Point3 = Point3(x=x, y=y, z=z + scale_.z / 2, reference_frame=world.root)
            orientation_quaternion_: Quaternion = Quaternion(x=0, y=0, z=0, w=1, reference_frame=world.root)
            pose_: Pose = Pose(position=pose_point3_, orientation=orientation_quaternion_, reference_frame=world.root)
            object_to_spawn = spawn_semantic_with_body(semantic_type=name,name=name.lower(),pose=pose_, scale=scale_,world=world, color = color)

        i+=1

def normalize(value, min_value, max_value):
    if min_value == max_value:
        return value
    return (value - min_value) / (max_value - min_value)

def normalize_list_values(values : list[float]) -> list[float]:

    high, low = max(values), min(values)
    if low == high:
        return [1.0] * len(values)

    return [normalize(v, low, high) for v in values]

def normalize_task_estimation(task_list : list[Task]) -> list[Task]:
    scores : list[float] = []
    score_per_seconds : list[float] = []
    probabilities : list[float] = []
    for task in task_list:
        scores.append(task.score)
        score_per_seconds.append(task.score_per_seconds)
        probabilities.append(task.probability)
    normalized_score = normalize_list_values(scores)
    normalized_score_per_seconds = normalize_list_values(score_per_seconds)
    normalized_probabilities = normalize_list_values(probabilities)
    for i,task_estimate in enumerate(task_list):
        task_estimate.normalized_score = normalized_score[i]
        task_estimate.normalized_probability = normalized_probabilities[i]
        task_estimate.normalized_score_per_seconds = normalized_score_per_seconds[i]
    return task_list


def navigation_subplan(challenge_mode : ChallengeMode, target_location: str, world, pose_override=None):
    if pose_override is not None:
        x, y, (qx, qy, qz, qw) = pose_override
    else:
        x, y, (qx, qy, qz, qw) = get_navigation_poses(challenge_mode, target_location)
    pose = Pose(
        position=Point3(x, y, 0.0, reference_frame=world.root),
        orientation=Quaternion(qx, qy, qz, qw, reference_frame=world.root),
        reference_frame=world.root,
    )
    return NavigateAction(target_location=pose, keep_joint_states=True)

class ObjectNotInWorldError(Exception):
    """
    Raised when a task names an object type that has no instance in the world at all.
    """


def find_object_annotation(world: World, object_annotation: SemanticAnnotation):
    if not isinstance(object_annotation, type):
        return None
    annotations = world.get_semantic_annotations_by_type(object_annotation)
    return annotations[0] if annotations else None


def remove_object_from_world(world, object_name: str) -> bool:
    try:
        annotation = world.get_semantic_annotation_by_name(object_name)
    except WorldEntityNotFoundError:
        return False
    if annotation is None:
        return False

    body = annotation.root
    with world.modify_world():
        world.remove_semantic_annotation(annotation)
        parent_connection = getattr(body, "parent_connection", None)
        if parent_connection is not None:
            world.remove_connection(parent_connection)
        world.remove_kinematic_structure_entity(body)
    return True


def perceived_instance(found: dict, object_annotation: SemanticAnnotation):
    for seen in found:
        if type(seen) is object_annotation:
            return seen
    for seen in found:
        if isinstance(seen, object_annotation):
            return seen
    return None


def body_of(perceived_object, world: World, object_annotation: SemanticAnnotation) -> Body:
    if perceived_object is not None:
        return perceived_object.bodies[0]
    name = getattr(object_annotation, "__name__", object_annotation)
    logger.warning(
        f"no perceived instance recorded for {name}; falling back to a by-class world lookup, "
        f"which may return an object the robot has never actually seen"
    )
    return find_object_body(world, object_annotation)


def find_object_body(world: World, object_annotation: SemanticAnnotation) -> Body:
    annotation = find_object_annotation(world, object_annotation)
    if annotation is None:
        name = getattr(object_annotation, "__name__", object_annotation)
        raise ObjectNotInWorldError(f"no {name} exists in the world")
    return annotation.bodies[0]


def pickup_subplan(object_annotation: SemanticAnnotation, arm: Arms, world: World, assisted : bool=False,
                   perceived_object=None, location: str = ""):
    object_body : Body = body_of(perceived_object, world, object_annotation)
    success_probability = evaluation(ActionType.PICKUP, object_annotation, location).probability
    return PickUpAction(object_geometry=object_body, arm=arm, assisted=assisted,
                        success_probability=success_probability)


def request_reposition_subplan(object_annotation: SemanticAnnotation, arm: Arms, world: World,
                               perceived_object=None):
    object_body : Body = body_of(perceived_object, world, object_annotation)
    return RequestObjectRepositionAction(object_designator=object_body, arm=arm)

def openable_body(annotation) -> Body:
    if isinstance(annotation, HasDoors) and annotation.doors:
        return annotation.doors[0].root
    if isinstance(annotation, HasDrawers) and annotation.drawers:
        return annotation.drawers[0].root
    return annotation.bodies[0]

def open_subplan(object_annotation, arm, world, assisted):
    annotation = find_object_annotation(world, object_annotation)
    if annotation is None:
        name = getattr(object_annotation, "__name__", object_annotation)
        raise ObjectNotInWorldError(f"no {name} exists in the world")
    object_body: Body = openable_body(annotation)
    return OpenAction(object_designator=object_body, arm=arm, assisted=assisted)


def origin_to_bottom_offset(object_body: Body) -> float:
    local_bbs = object_body.collision.as_bounding_box_collection_in_frame(object_body).bounding_boxes
    if local_bbs:
        return float(min(bb.z_interval.lower for bb in local_bbs))

    combined_mesh = getattr(object_body, "combined_mesh", None)
    mesh_bounds = getattr(combined_mesh, "bounds", None) if combined_mesh is not None else None
    if mesh_bounds is not None and len(mesh_bounds):
        mesh_offset = float(mesh_bounds[0][2])
        logger.warning(
            f"{object_body.name} exposes no collision bounding boxes (every collision shape has "
            f"an origin without a reference_frame, so they were all skipped); falling back to the "
            f"visual mesh bottom {mesh_offset:.4f} to seat it"
        )
        return mesh_offset

    logger.warning(
        f"{object_body.name} has neither collision bounding boxes nor a visual mesh; assuming its "
        f"origin sits at its bottom. If the object is modelled around its centre it will be "
        f"placed half its own height too low."
    )
    return 0.0


def body_is_upright(body: Body, upward_threshold: float = 0.95) -> bool:
    world_T_body = body.global_pose.to_np()
    # third column of the rotation block = the body's local Z axis expressed in world coordinates
    return float(world_T_body[2, 2]) > upward_threshold


def compute_surface_spaces(world) -> list[SurfaceSpace]:
    surfaces = world.get_semantic_annotations_by_type(HasSupportingSurface)
    result = []

    for surface in surfaces:
        if surface.root is None or surface.root._world is None:
            # dangling annotation whose body was removed from the world (e.g. the sofa)
            continue

        if surface.supporting_surface is not None and body_is_upright(surface.root):
            bb_collection = surface.supporting_surface.area.as_bounding_box_collection_in_frame(world.root)
        else:
            bb_collection = surface.as_bounding_box_collection_in_frame(world.root)

        if not bb_collection.bounding_boxes:
            continue

        x_min = min(bb.x_interval.lower for bb in bb_collection)
        x_max = max(bb.x_interval.upper for bb in bb_collection)
        y_min = min(bb.y_interval.lower for bb in bb_collection)
        y_max = max(bb.y_interval.upper for bb in bb_collection)
        z_surface = max(bb.z_interval.upper for bb in bb_collection)

        result.append(SurfaceSpace(
            name=str(surface.name),
            x_min=round(x_min, 3),
            x_max=round(x_max, 3),
            y_min=round(y_min, 3),
            y_max=round(y_max, 3),
            z_surface=round(z_surface, 3),
        ))

    return result


def objects_on_surface(
    surface: SurfaceSpace,
    world,
    z_tolerance: float = 0.05,
    stack_tolerance: float = 0.15,
) -> list[tuple[Body, list]]:
    structural_bodies: set[Body] = set()
    for annotation in world.get_semantic_annotations_by_type(HasSupportingSurface):
        if isinstance(annotation, IsPerceivable):
            continue
        for body in annotation.bodies:
            structural_bodies.add(body)

    result = []
    for body in world.bodies_with_collision:
        if body in structural_bodies:
            continue

        world_T_body = world.compute_forward_kinematics(world.root, body)
        bb_col = body.collision.as_bounding_box_collection_at_origin(world_T_body)
        bbs = bb_col.bounding_boxes
        if not bbs:
            continue
        z_bot = min(bb.z_interval.lower for bb in bbs)
        cx = (min(bb.x_interval.lower for bb in bbs) + max(bb.x_interval.upper for bb in bbs)) / 2
        cy = (min(bb.y_interval.lower for bb in bbs) + max(bb.y_interval.upper for bb in bbs)) / 2
        if (
            surface.x_min <= cx <= surface.x_max
            and surface.y_min <= cy <= surface.y_max
            and surface.z_surface - z_tolerance <= z_bot <= surface.z_surface + stack_tolerance
        ):
            result.append((body, bbs))
    return result


def blocks_reach_to(
    candidate: tuple[float, float],
    candidate_half: tuple[float, float],
    occupied_box: tuple[float, float, float, float],
    reference_point: tuple[float, float],
) -> bool:
    reference_x, reference_y = reference_point
    box_x_min, box_x_max, box_y_min, box_y_max = occupied_box
    object_x = (box_x_min + box_x_max) / 2
    object_y = (box_y_min + box_y_max) / 2

    to_object_x = object_x - reference_x
    to_object_y = object_y - reference_y
    distance_to_object = math.hypot(to_object_x, to_object_y)
    if distance_to_object < 1e-6:
        return False
    direction_x = to_object_x / distance_to_object
    direction_y = to_object_y / distance_to_object

    to_candidate_x = candidate[0] - reference_x
    to_candidate_y = candidate[1] - reference_y
    along = to_candidate_x * direction_x + to_candidate_y * direction_y
    if not 0.0 < along < distance_to_object:
        # behind the other object, or behind the robot -- either way it is not in the way
        return False

    lateral = abs(to_candidate_x * direction_y - to_candidate_y * direction_x)
    corridor = (
        max(candidate_half)
        + max(box_x_max - box_x_min, box_y_max - box_y_min) / 2
    )
    return lateral < corridor

def find_free_placement_pose(
        surface: SurfaceSpace,
        object_body: Body,
        world: World,
        grid_step: float = 0.05,
        edge_margin: float = 0.10,
        object_clearance: float = 0.08,
        reference_point: Optional[tuple[float, float]] = None,
) -> Optional[tuple[float, float]]:
    # Object half-extents in its local frame (upright assumption)
    local_bbs = object_body.collision.as_bounding_box_collection_in_frame(object_body).bounding_boxes
    if local_bbs:
        half_x = (max(bb.x_interval.upper for bb in local_bbs) - min(bb.x_interval.lower for bb in local_bbs)) / 2
        half_y = (max(bb.y_interval.upper for bb in local_bbs) - min(bb.y_interval.lower for bb in local_bbs)) / 2
    else:
        half_x = 0.1
        half_y = 0.05

    # Collect footprints of all objects already on the surface, excluding the object itself
    occupied: list[tuple[float, float, float, float]] = []
    for body, bbs in objects_on_surface(surface, world):
        if body is object_body:
            continue
        occupied.append((
            min(bb.x_interval.lower for bb in bbs),
            max(bb.x_interval.upper for bb in bbs),
            min(bb.y_interval.lower for bb in bbs),
            max(bb.y_interval.upper for bb in bbs),
        ))

    if reference_point is not None:
        x_centre, y_centre = reference_point
    else:
        x_centre = (surface.x_min + surface.x_max) / 2
        y_centre = (surface.y_min + surface.y_max) / 2

    def search(margin: float, clearance: float, avoid_blocking: bool) -> Optional[tuple[float, float]]:
        # Candidate region: inset so the whole footprint plus the margin stays on the surface
        x_lo = surface.x_min + half_x + margin
        x_hi = surface.x_max - half_x - margin
        y_lo = surface.y_min + half_y + margin
        y_hi = surface.y_max - half_y - margin
        if x_lo > x_hi or y_lo > y_hi:
            return None

        # Candidates ordered by distance to the reference point, not raster-scan order --
        # a raster scan always returns the first free cell it hits, which is always the
        # same (x_lo, y_lo) corner whenever that corner happens to be free. That biases
        # every placement into one corner of the surface instead of looking natural.
        candidates = [
            (float(x), float(y))
            for x in np.arange(x_lo, x_hi + grid_step, grid_step)
            for y in np.arange(y_lo, y_hi + grid_step, grid_step)
        ]
        candidates.sort(key=lambda xy: (xy[0] - x_centre) ** 2 + (xy[1] - y_centre) ** 2)

        for x, y in candidates:
            obj_x_min, obj_x_max = x - half_x - clearance, x + half_x + clearance
            obj_y_min, obj_y_max = y - half_y - clearance, y + half_y + clearance
            free = True
            for occupied_box in occupied:
                ox_min, ox_max, oy_min, oy_max = occupied_box
                if (obj_x_min < ox_max and obj_x_max > ox_min
                        and obj_y_min < oy_max and obj_y_max > oy_min):
                    free = False
                    break
                if avoid_blocking and reference_point is not None and blocks_reach_to(
                    (x, y), (half_x, half_y), occupied_box, reference_point
                ):
                    free = False
                    break
            if free:
                return x, y
        return None

    attempts = [
        (edge_margin, object_clearance, True),   # everything we want
        (edge_margin, object_clearance, False),  # crowded surface: accept blocking a neighbour
        (0.0, object_clearance, False),          # narrow surface: accept sitting near the rim
        (0.0, 0.0, False),                       # last resort: only refuse actual overlap
    ]
    for attempt_margin, attempt_clearance, attempt_avoid_blocking in attempts:
        spot = search(
            margin=attempt_margin,
            clearance=attempt_clearance,
            avoid_blocking=attempt_avoid_blocking,
        )
        if spot is not None:
            return spot

    return None



def place_subplan(challenge_mode : ChallengeMode, object_annotation: SemanticAnnotation, arm: Arms, target_location: str, world, assisted : bool, robot: AbstractRobot, perceived_object=None):
    object_body : Body = body_of(perceived_object, world, object_annotation)

    def resolve_placement_pose() -> tuple[Optional[Pose], bool]:
        surface_spaces = compute_surface_spaces(world=world)
        matched_surface = next((s for s in surface_spaces if s.name == target_location), None)
        if matched_surface is None:
            logger.warning(ValueError(f"No surface named '{target_location}' found in the world."))
            return None, True

        robot_position = robot.root.global_pose.to_position()
        free_spot = find_free_placement_pose(
            surface=matched_surface, world=world, object_body=object_body,
            reference_point=(float(robot_position.x), float(robot_position.y)),
        )
        if free_spot is None:
            logger.warning(f"No free placement spot found on '{target_location}'.")
            return None, True

        origin_to_bottom = origin_to_bottom_offset(object_body)
        x, y = free_spot
        z = matched_surface.z_surface - origin_to_bottom + 0.002

        _, _, (qx, qy, qz, qw) = get_navigation_poses(challenge_mode, target_location)
        pose = Pose(
            position=Point3(x, y, z, reference_frame=world.root),
            orientation=Quaternion(qx, qy, qz, qw, reference_frame=world.root),
            reference_frame=world.root,
        )
        return pose, z > 1.1

    success_probability = evaluation(ActionType.PLACE, object_annotation, target_location).probability
    return PlaceAction(object_designator=object_body, arm=arm, assisted=assisted, pose_resolver=resolve_placement_pose,
                       success_probability=success_probability)

def throw_away_subplan(object_annotation: SemanticAnnotation, arm: Arms, world: World, assisted : bool=False,
                       perceived_object=None):
    object_body : Body = body_of(perceived_object, world, object_annotation)
    return PlaceInTrashAction(object_designator=object_body, arm=arm, assisted=assisted)

def place_in_trash_subplan(object_annotation: SemanticAnnotation, arm: Arms, target_location: str, world, assisted : bool,
                           perceived_object=None):
    object_body : Body = body_of(perceived_object, world, object_annotation)

    return PlaceInTrashAction(object_designator=object_body, arm=arm, assisted=assisted)

def generate_plan(tasks: list[Task], context: Context):
    from pycram.plans.factories import make_node
    plan = sequential(children=[], context=context)
    arm = Arms.LEFT
    for task in tasks:
        last_pickup_object: Optional[str] = None
        for task_steps in task.task_steps:
            action = None
            match task_steps.action_type:
                case ActionType.NAVIGATE:
                    action = navigation_subplan(target_location=task_steps.location, world=context.world)
                case ActionType.PICKUP:
                    last_pickup_object = task_steps.object_annotations
                    action = pickup_subplan(object_name=task_steps.object_annotations, arm=arm, world=context.world)
                case ActionType.PLACE:
                    action = place_in_trash_subplan(object_name=last_pickup_object, arm=arm, target_location=task_steps.object_placement, world=context.world, object_annotation=task_steps.object_annotations)
                case ActionType.PLACE:
                    action = place_in_trash_subplan(object_name=last_pickup_object, arm=arm,
                                           target_location=task_steps.object_placement, world=context.world, object_annotation=task_steps.object_annotations)
                case ActionType.PARK:
                    action = ParkArmsAction(Arms.LEFT)
                case ActionType.DETECT:
                    pass
                case _:
                    raise NotImplementedError(f"Action type not implemented: {task_steps.action_type}")
            if action is not None:
                plan.add_child(make_node(action))

    return plan

def at_location(context : Context, location : str, robot: AbstractRobot, threshold: float = 0.5, yaw_threshold: float = math.radians(20)) -> bool:
    challenge_mode = context.challenge_mode
    navigation_pose = get_navigation_poses(challenge_mode=challenge_mode, location=location)
    if navigation_pose is None:
        # unknown location -> we cannot claim the robot is standing there
        return False
    x_goal, y_goal, (qx, qy, qz, qw) = navigation_pose
    point3_goal : Point3 = Point3(x=x_goal, y=y_goal, z=0)
    robot_pose : Pose = robot.root.global_pose
    robot_position : Point3 = robot_pose.to_position()

    if float(robot_position.euclidean_distance(point3_goal)) >= threshold:
        return False

    goal_yaw = 2.0 * math.atan2(qz, qw)
    pose_matrix = robot_pose.to_np()

    robot_yaw = math.atan2(pose_matrix[1, 0], pose_matrix[0, 0])
    yaw_error = math.atan2(math.sin(robot_yaw - goal_yaw), math.cos(robot_yaw - goal_yaw))
    return abs(yaw_error) < yaw_threshold

def resolve_step_objects(task_steps: list[TaskStep]) -> None:
    last_pickup_object: Optional[SemanticAnnotation] = None
    for task_step in task_steps:
        if task_step.object_annotations is not None:
            if task_step.action_type == ActionType.PICKUP:
                last_pickup_object = task_step.object_annotations
            continue
        if task_step.action_type in (ActionType.PLACE, ActionType.THROW_AWAY):
            task_step.object_annotations = last_pickup_object

def generate_plan_taskstep_list(taskstep_list: list[TaskStep], context: Context) -> PlanNode:
    from pycram.plans.factories import make_node
    challenge_mode: ChallengeMode = context.challenge_mode
    plan = sequential(children=[], context=context)
    arm = Arms.LEFT
    action = None
    resolve_step_objects(taskstep_list)
    last_pickup_object: Optional[SemanticAnnotation] = None
    action_list : list[ActionDescription] = []
    for task_step in taskstep_list:
        action = None
        # TODO: add assisted tasks
        match task_step.action_type:
            case ActionType.NAVIGATE:
                action = navigation_subplan(challenge_mode=challenge_mode, target_location=task_step.location, world=context.world,
                                            pose_override=task_step.nav_pose_override)
            case ActionType.PICKUP:
                last_pickup_object : SemanticAnnotation= task_step.object_annotations
                action = pickup_subplan(object_annotation=last_pickup_object, arm=arm, world=context.world, assisted=task_step.action_assisted,
                                        perceived_object=task_step.perceived_object, location=task_step.location)
            case ActionType.PLACE:
                action = place_subplan(challenge_mode=challenge_mode, object_annotation=last_pickup_object, arm=arm, target_location=task_step.location, world=context.world, assisted=task_step.action_assisted, robot=context.robot,
                                       perceived_object=task_step.perceived_object)
            case ActionType.THROW_AWAY:
                action = place_in_trash_subplan(object_annotation=last_pickup_object, arm=arm,
                                       target_location=task_step.location, world=context.world,
                                       assisted=task_step.action_assisted,
                                       perceived_object=task_step.perceived_object)
            case ActionType.OPEN:
                action = open_subplan(object_annotation=task_step.object_annotations, arm=arm, world=context.world, assisted=task_step.action_assisted)
            case ActionType.PARK:
                action = ParkArmsAction(Arms.LEFT)
            case ActionType.REQUEST_REPOSITION:
                action = request_reposition_subplan(object_annotation=task_step.object_annotations,
                                                   arm=arm, world=context.world,
                                                   perceived_object=task_step.perceived_object)

            case ActionType.DETECT:
                pass
            case _:
                raise NotImplementedError(f"Action type not implemented: {task_step.action_type}")
        if action is not None:
            # action_list.append(action)
            plan.add_child(make_node(action))
    return plan

def _quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))

def find_surface_of_object(context:Context, body: Body) -> HasSupportingSurface | None:
    world = context.world
    surfaces = world.get_semantic_annotations_by_type(HasSupportingSurface)
    for surface in surfaces:
        if surface.bodies[0]._world is None:
            # dangling annotation whose body was removed from the world (e.g. the sofa)
            continue
        if surface.bodies[0] is body:
            # an object (bowl, plate, ...) is itself a HasSupportingSurface but never its own surface
            continue
        if is_supported_by(body, surface.bodies[0]):
            return surface
    return None

def navigation_location_names(challenge_mode: ChallengeMode) -> list[str]:
    """Every location name this challenge mode knows an approach pose for."""
    match challenge_mode:
        case ChallengeMode.PP:
            return list(NAVIGATION_POSES_PP.keys())
        case _:
            return list(NAVIGATION_POSES.keys())


def closest_navigation_location(challenge_mode: ChallengeMode, position: Point3) -> Optional[str]:
    best_location = None
    best_distance = math.inf
    for location in navigation_location_names(challenge_mode):
        if location == "":
            continue
        navigation_pose = get_navigation_poses(challenge_mode, location)
        if navigation_pose is None:
            continue
        x, y, _ = navigation_pose
        distance = math.dist((float(position.x), float(position.y)), (x, y))
        if distance < best_distance:
            best_distance = distance
            best_location = location
    return best_location


def ground_step_location(context: Context, task_step: TaskStep) -> Optional[str]:
    if task_step.object_annotations is None:
        return None
    world = context.world
    body = body_of(task_step.perceived_object, world, task_step.object_annotations)
    if body is None:
        return None
    return closest_navigation_location(context.challenge_mode, body.global_pose.to_position())


def reposition_task_list(context: Context, task_list: list[TaskStep], robot: AbstractRobot) -> list[TaskStep]:
    challenge_mode = context.challenge_mode
    exec_steps: list[TaskStep] = []
    predicted_pose = None

    for task_step in task_list:
        location = task_step.location

        override = task_step.nav_pose_override
        if override is not None:
            if predicted_pose != override:
                exec_steps.append(TaskStep(action_type=ActionType.NAVIGATE, location=location,
                                           nav_pose_override=override))
            predicted_pose = override
            exec_steps.append(task_step)
            continue

        navigation_pose = get_navigation_poses(challenge_mode, location) if location else None

        if navigation_pose is None:
            grounded_location = ground_step_location(context=context, task_step=task_step)
            if grounded_location is None:
                exec_steps.append(task_step)
                continue
            logger.warning(
                f"{task_step.action_type} step names location '{location}', which this challenge "
                f"mode has no approach pose for -> navigating to '{grounded_location}' instead, "
                f"the closest known location to the step's own object"
            )
            location = grounded_location
            task_step.location = grounded_location
            navigation_pose = get_navigation_poses(challenge_mode, location)

        if predicted_pose is None:
            already_there = False
        else:
            already_there = predicted_pose == navigation_pose

        if not already_there:
            exec_steps.append(TaskStep(action_type=ActionType.NAVIGATE, location=location))
        predicted_pose = navigation_pose
        exec_steps.append(task_step)

    return exec_steps


def compute_reposition_pose(context: Context, task_step: TaskStep, standoff: float = 0.55):
    world = context.world
    challenge_mode = context.challenge_mode

    default_pose = get_navigation_poses(challenge_mode, task_step.location) if task_step.location else None
    if default_pose is None:
        return None

    candidate = task_step.perceived_object or task_step.object_annotations
    if isinstance(candidate, type):
        annotation = find_object_annotation(world, candidate)
    elif candidate is not None and getattr(candidate, "bodies", None):
        annotation = candidate
    else:
        annotation = None
    if annotation is None or not annotation.bodies:
        return None

    object_pose = annotation.bodies[0].global_pose
    object_x, object_y = float(object_pose.x), float(object_pose.y)

    base_x, base_y = default_pose[0], default_pose[1]
    direction_x, direction_y = object_x - base_x, object_y - base_y
    distance = math.hypot(direction_x, direction_y)
    if distance < 1e-3:
        # already essentially on top of the object -- no meaningful reposition to make
        return None
    unit_x, unit_y = direction_x / distance, direction_y / distance

    new_x = object_x - unit_x * standoff
    new_y = object_y - unit_y * standoff
    yaw = math.atan2(direction_y, direction_x)
    return (new_x, new_y, _quat(yaw))

# TODO : TO POSE

def get_navigation_poses(challenge_mode : ChallengeMode, location : str):
    navigation_poses = []
    match challenge_mode:
        case ChallengeMode.FD:
            navigation_poses = NAVIGATION_POSES.get(location)
        case ChallengeMode.PP:
            navigation_poses = NAVIGATION_POSES_PP.get(location)
        case ChallengeMode.GPSR:
            navigation_poses = NAVIGATION_POSES.get(location)

    return navigation_poses



NAVIGATION_POSES: dict[str,
tuple[float, float,
tuple[float, float, float, float]]] = {
    "cooking_table": (1.3, 4.6, _quat( math.pi / 2)),   # south of table, facing north
    "dining_table":  (3.4, 5.7, _quat( math.pi)),       # east of table (wide side), facing west
    "table":         (3.5, 1.8, _quat(-math.pi / 2)),   # north of table, facing south
    "lowerTable":    (3.0, 2.2, _quat( 0.0)),           # west of table,  facing east
    "desk":          (1.3, 1.2, _quat( math.pi)),        # east of desk,   facing west
    "shelf_1":       (3.3,  4.7,  _quat( 0.0)),          # west of cupboard, facing east
    "shelf_2":       (3.3,  4.7,  _quat( 0.0)),          # same approach as shelf_1
    "counterTop": (1.859, -0.852, _quat(-math.pi / 2)),  # north of counter, facing south
    "": (0, 0, _quat(-math.pi / 2)),  # going to 0,0 if unknown
}

NAVIGATION_POSES_PP: dict[str,
tuple[float, float,
tuple[float, float, float, float]]] = {
    "cooking_table": (1.3, 4.6, _quat( math.pi / 2)),   # south of table, facing north
    "dining_table":  (3.4, 5.7, _quat( math.pi)),       # east of table (wide side), facing west
    "table":         (3.5, 1.8, _quat(-math.pi / 2)),   # north of table, facing south
    "dishwasher":    (3.40, 2.63, _quat(-math.pi / 2)), # north of the moved dishwasher
    "dishwasher_rack": (3.40, 3.05, _quat(-math.pi / 2)), # same approach pose as "dishwasher"
    "lowerTable":    (3.05, 4.05, _quat( math.pi)),     # east of the moved table
    "extra_space":   (3.95, 3.70, _quat( 0.1)),         # west of extra_space table
    "trash_can":     (2.1975, 3, _quat(-math.pi / 2)),  # north of the (moved) can, facing south
    "trash":         (2.1975, 3, _quat(-math.pi / 2)),  # alias used by place-in-trash task steps
    "desk":          (1.3, 1.2, _quat( math.pi)),        # east of desk,   facing west
    "cabinet":       (3.55, 4.72, _quat( 0.1)),          # same approach for every shelf layer
    "shelf_1":       (3.55, 4.72, _quat( 0.1)),          # west of cupboard, facing east, clear of the opened doors
    "shelf_2":       (3.55, 4.72, _quat( 0.1)),          # same approach for every shelf layer
    "shelf_3":       (3.55, 4.72, _quat( 0.1)),
    "shelf_4":       (3.55, 4.72, _quat( 0.1)),
    "counterTop": (1.859, -0.852, _quat(-math.pi / 2)),  # north of counter, facing south
    "": (0, 0, _quat(-math.pi / 2)),  # going to 0,0 if unknown
}

def get_values(action_type: ActionType, semantic_annotation: SemanticAnnotation = None, location: str = ""
               ) -> tuple[int, int, int, float]:
    p = evaluation(action_type, semantic_annotation, location)
    return p.score, p.penalty, p.time, p.probability

def get_remaining_task_steps(task: Task) -> list[TaskStep]:
    failed_taskstep : list[TaskStep] = []
    for t in task.task_steps:
        if t.action_outcome in (
                Status.FAILURE,
                Status.FAILURE_RECOVERABLE,
                Status.FAILURE_UNRECOVERABLE) or failed_taskstep != []:
            failed_taskstep.append(t)

    return failed_taskstep

def nav_time_between(x1: float, y1: float, x2: float, y2: float, speed: float = 0.5) -> float:
    return math.dist((x1, y1), (x2, y2)) / speed


def nav_time(robot_pose: Point3, to_loc: str, speed: float = 0.55 / 3.6) -> float:
    (x1, y1) = robot_pose.x, robot_pose.y
    (x2, y2) = NAVIGATION_POSES[to_loc][:2]
    return nav_time_between(x1, y1, x2, y2, speed)

def challenge_setup(challenge_mode : ChallengeMode, doors_open: bool = True):
    match challenge_mode:
        case ChallengeMode.PP:
            return setup_pp(challenge_mode, doors_open=doors_open)
        case ChallengeMode.GPSR:
            setup_gpsr()
        case ChallengeMode.FD:
            setup_fd()


def setup_pp_all_doors_open(challenge_mode : ChallengeMode = ChallengeMode.PP):
    return setup_pp(challenge_mode, doors_open=True)


def setup_pp_all_doors_closed(challenge_mode : ChallengeMode = ChallengeMode.PP):
    return setup_pp(challenge_mode, doors_open=False)

def randomized_placement(supporting_surface : HasSupportingSurface, object_scales : list[Scale], max_attempts: int = 100,
                         front_clearance: float = 0.0, approach_xy: Optional[tuple[float, float]] = None) -> list[Pose]:
    world = supporting_surface._world # retrieving world for later transformation.
    region = supporting_surface.supporting_surface

    bb = region.area.as_bounding_box_collection_in_frame(region).bounding_box() # get bounding box to use upper and lower bound

    reserve_front_edge = front_clearance > 0.0 and approach_xy is not None
    front_unit_x = front_unit_y = 0.0
    surface_center_x = surface_center_y = 0.0
    front_edge_limit = 0.0
    if reserve_front_edge:
        corner_specs = [(bb.min_x, bb.min_y), (bb.min_x, bb.max_y),
                        (bb.max_x, bb.min_y), (bb.max_x, bb.max_y)]
        corners_world_xy: list[tuple[float, float]] = []
        for corner_x, corner_y in corner_specs:
            corner_pose = Pose(position=Point3(corner_x, corner_y, bb.max_z), reference_frame=region)
            corner_world = world.transform(corner_pose, world.root)
            corners_world_xy.append((float(corner_world.x), float(corner_world.y)))
        surface_center_x = sum(cx for cx, _ in corners_world_xy) / 4
        surface_center_y = sum(cy for _, cy in corners_world_xy) / 4
        front_dx = approach_xy[0] - surface_center_x
        front_dy = approach_xy[1] - surface_center_y
        front_len = math.hypot(front_dx, front_dy)
        front_unit_x, front_unit_y = front_dx / front_len, front_dy / front_len
        # how far the surface itself reaches toward the robot from its centre, along the approach
        front_extent = 0.0
        for corner_x_w, corner_y_w in corners_world_xy:
            reach = (corner_x_w - surface_center_x) * front_unit_x + (corner_y_w - surface_center_y) * front_unit_y
            if reach > front_extent:
                front_extent = reach
        front_edge_limit = front_extent - front_clearance

    pose_temp = [] # used to store the temp_poses for easier collision comparasion.
    poses : list[Pose]= []
    threshold = 0.05
    for scale in object_scales:
        overlap = False
        for _ in range(max_attempts):
            # highest-x, highest-y
            hx, hy = scale.x / 2, scale.y / 2

            # random x and y to place at
            rand_x, rand_y = random.uniform(bb.min_x + hx, bb.max_x - hx), random.uniform(bb.min_y + hy, bb.max_y - hy)

            # current-low-y, current-high-y [...], having the corner coordinates of the pose.
            cly, chy = rand_y - hy, rand_y + hy
            clx, chx = rand_x - hx, rand_x + hx

            for pose in pose_temp:
                x,y,ux,uy = pose[0], pose[1], pose[2], pose[3] # Retrieving the x,y, half x, half y, to have a scale and then compare it to the objects.

                # object-low-y, object-high-y [...], having the corner coordinates of the pose.
                oly,ohy = y - uy-threshold, y + uy+threshold
                olx,ohx = x - ux-threshold, x + ux+threshold

                # check for overlap of any kind
                overlap = True if clx < ohx and chx > olx and cly < ohy and chy > oly else False
                if overlap:
                    break

            # If not overlapped at any pose, then append, else retry.
            if not overlap:
                # z is the surface top; generic_object_spawner lifts each object by its own geometry
                pose = Pose(position=Point3(rand_x, rand_y, bb.max_z), reference_frame=region)
                world_pose = world.transform(pose, world.root) # Transforming into the world root, since we are still in region frame.

                if reserve_front_edge:
                    toward_robot = ((float(world_pose.x) - surface_center_x) * front_unit_x
                                    + (float(world_pose.y) - surface_center_y) * front_unit_y)
                    if toward_robot > front_edge_limit:
                        continue

                pose_temp.append((rand_x,rand_y,hx,hy))
                poses.append(world_pose)
                break
        else:
            # a silent skip here would make the caller crash later with a confusing IndexError
            raise RuntimeError(
                f"could not place object {len(poses)} of {len(object_scales)} "
                f"(scale {scale.x}x{scale.y}) on {supporting_surface.name} "
                f"after {max_attempts} attempts - surface too small or too crowded"
            )
    return poses




def set_container_openings(world: World, opened: bool) -> list[str]:
    containers = []
    for annotation in list(world.get_semantic_annotations_by_type(HasDoors)) + \
                      list(world.get_semantic_annotations_by_type(HasDrawers)):
        # a Dishwasher is both HasDoors and HasDrawers, so it turns up twice
        if annotation not in containers:
            containers.append(annotation)

    moved: list[str] = []
    for annotation in containers:
        movable_parts = []
        if isinstance(annotation, HasDoors):
            movable_parts += [door.root for door in annotation.doors]
        if isinstance(annotation, HasDrawers):
            movable_parts += [drawer.root for drawer in annotation.drawers]

        for part in movable_parts:
            try:
                connection = part.get_first_parent_connection_of_type(ActiveConnection1DOF)
            except Exception as error:
                logger.warning(f"cannot open/close {part.name}: {error}")
                continue
            limits = connection.dof.limits
            connection.position = limits.upper.position if opened else limits.lower.position
            moved.append(str(part.name))

    logger.info(f"containers set to {'open' if opened else 'closed'}: {moved}")
    return moved


def setup_pp(challenge_mode : ChallengeMode, doors_open: bool = True):
    world, dispatcher = setup_world()

    hsrb = HSRB.from_world(world)

    context = Context(world=world, robot=hsrb, challenge_mode=challenge_mode)
    context.evaluate_conditions = False

    set_container_openings(world, opened=doors_open)

    # Removing sofa -------------------------------------------------
    sofa: Body = world.get_body_by_name("sofa")
    sofa_annotation = world.get_semantic_annotation_by_name("sofa")
    with world.modify_world():
        # the annotation has to go too, otherwise surface sweeps hit a body without a world
        world.remove_semantic_annotation(sofa_annotation)
        world.remove_connection(sofa.parent_connection)
        world.remove_kinematic_structure_entity(sofa)

    # Moving dishwasher -------------------------------------------------
    body = world.get_body_by_name("dishwasher")
    conn = body.parent_connection
    with world.modify_world():
        world.remove_connection(conn)
        new_conn = Connection6DoF.create_with_dofs(parent=world.root, child=body, world=world)
        world.add_connection(new_conn)

    new_conn.origin = HomogeneousTransformationMatrix.from_xyz_quaternion(
        3.39975, 1.705,0.365, reference_frame=world.root
    )


    # Moving lowerTable -------------------------------------------------
    body = world.get_body_by_name("lowerTable")
    conn = body.parent_connection
    with world.modify_world():
        world.remove_connection(conn)
        new_conn = Connection6DoF.create_with_dofs(parent=world.root, child=body, world=world)
        world.add_connection(new_conn)

    # the table is 0.44 tall and the origin is its center -> z=0.22 puts the legs on the floor
    new_conn.origin = HomogeneousTransformationMatrix.from_xyz_quaternion(
        2.39975, 4.05,0.22, reference_frame=world.root
    )


    # Moving trashcan -------------------------------------------------
    trashcan = world.get_body_by_name("trash_can")
    conn = trashcan.parent_connection
    with world.modify_world():
        world.remove_connection(conn)
        new_conn = Connection6DoF.create_with_dofs(parent=world.root, child=trashcan, world=world)
        world.add_connection(new_conn)

    new_conn.origin = HomogeneousTransformationMatrix.from_xyz_quaternion(
        2.1975, 1.705,0.2, reference_frame=world.root
    )


    # Placing extra_table -------------------------------------------------
    # in room dining_room TODO: update that shit in the table list
    with world.modify_world():
        table = Table.create_with_new_body_in_world(
            world=world,
            name=PrefixedName("extra_space"),
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=4.59975, y=3.705, z=0.22),
            scale=Scale(x=0.37, y=0.91, z=0.44),
        )
        for body in table.bodies:
            for s in body.visual.shapes:
                s.color = Color.BEIGE()

    # calculate supporting surfaces -------------------------------------------------
    ss: list[SemanticAnnotation] = world.get_semantic_annotations_by_type(HasSupportingSurface)

    with world.modify_world():
        for s in ss:
            try:
                s.calculate_supporting_surface()
            except Exception as e:
                print(f"ignoring {s.name}: {e}")

    dispatcher.known_furniture = world.bodies

    cutlery = ["Knife", "Fork"]
    tableware = ["Cup"]
    common_objects = ["Pringles", "Apple", "Bottle", "MustardBottle", "TomatoSoup"]
    breakfast_objects = ["Bowl", "Spoon"]

    cabinet_objects = ["Milk", "Cereal"]

    # Autogenerating object lists ----------------------------------
    trash_obj2 = "Bottle"
    trash_obj1 = "Banana"
    # both trash picks get spawned elsewhere, so the table picks must not duplicate them
    common_obj1, common_obj2 = random.sample(
        [o for o in common_objects if o not in (trash_obj1, trash_obj2)], 2
    )
    cuttlery_obj = random.choice(cutlery)
    table_objects = tableware + [trash_obj1, common_obj1, common_obj2, cuttlery_obj]

    extra_space_objects = random.sample(
        [obj for obj in common_objects if obj not in table_objects and obj != trash_obj2], 2
    )
    extra_space_objects.append("DishwasherTab")

    scales_table_obj = [Scale(x=0.1, y=0.1, z=0.2) for _ in range(len(table_objects))]
    scales_extra_obj = [Scale(x=0.1, y=0.1, z=0.2) for _ in range(len(extra_space_objects))]
    scales_lower_table = [Scale(x=0.1, y=0.1, z=0), Scale(x=0.1, y=0.1, z=0.2)]
    # generate random placements ----------------------------------
    dining_table : SemanticAnnotation = world.get_semantic_annotation_by_name("dining_table")
    extra_space : SemanticAnnotation = world.get_semantic_annotation_by_name("extra_space")
    lower_table : SemanticAnnotation = world.get_semantic_annotation_by_name("lowerTable")
    dining_nav_x, dining_nav_y, _ = get_navigation_poses(challenge_mode, "dining_table")
    milk_front_clearance = 0.28
    placement_dining_table = randomized_placement(
        dining_table, scales_table_obj,
        front_clearance=milk_front_clearance, approach_xy=(dining_nav_x, dining_nav_y),
    )
    placement_extra_space = randomized_placement(extra_space, scales_extra_obj)
    placement_lower_table = randomized_placement(lower_table, scales_lower_table)

    # overlap and placement trash objects ----------------------------------
    threshold = 0.35
    # Scale is the full bounding-box extent -> halve it for the can's half-widths
    can_hx, can_hy = trashcan.visual.scale.x / 2, trashcan.visual.scale.y / 2
    # spawned trash objects are 0.1 x 0.1 fallback boxes
    obj_hx = obj_hy = 0.05

    trashcan_x, trashcan_y = trashcan.global_pose.x, trashcan.global_pose.y
    trashcan_lx, trashcan_ly = trashcan_x - can_hx, trashcan_y - can_hy
    trashcan_ux, trashcan_uy = trashcan_x + can_hx, trashcan_y + can_hy

    nav_x, nav_y, _ = get_navigation_poses(challenge_mode, "trash_can")
    approach_dx, approach_dy = nav_x - trashcan_x, nav_y - trashcan_y
    approach_distance = math.hypot(approach_dx, approach_dy)
    facing_x, facing_y = approach_dx / approach_distance, approach_dy / approach_distance
    lateral_x, lateral_y = -facing_y, facing_x

    overlap = True
    while overlap:
        standoff = max(can_hx, can_hy) + obj_hx + random.uniform(0.05, threshold)
        lateral = random.uniform(-0.15, 0.15)
        rand_x = trashcan_x + facing_x * standoff + lateral_x * lateral
        rand_y = trashcan_y + facing_y * standoff + lateral_y * lateral

        place_lx, place_ly = rand_x - obj_hx, rand_y - obj_hy
        place_ux, place_uy = rand_x + obj_hx, rand_y + obj_hy

        overlap = place_lx <= trashcan_ux and place_ux >= trashcan_lx and place_ly <= trashcan_uy and place_uy >= trashcan_ly

    # z=0: the trash object stands on the floor
    trash_placement = Pose(position=Point3(x=rand_x,y=rand_y,z=0.0),reference_frame=world.root)
    pose_1  = Pose(Point3(x=4.5,y=4.74,z=0.2), reference_frame=world.root)
    pose_2 =  Pose(Point3(x=4.5,y=4.74,z=0.5), reference_frame=world.root)
    poses_shelf = [pose_1, pose_2]

    # spawn objects ----------------------------------
    generic_object_spawner(names=[trash_obj2], pose=[trash_placement], world=world, color=Color.BLUE())
    generic_object_spawner(names=table_objects, pose=placement_dining_table, world=world, color=Color.RED())
    generic_object_spawner(names=extra_space_objects, pose=placement_extra_space, world=world, color=Color.ORANGE())
    generic_object_spawner(names=breakfast_objects, pose=placement_lower_table, world=world, color=Color.GREEN())
    generic_object_spawner(names=cabinet_objects, pose=poses_shelf, world=world, color=Color.GREEN())
    return world, dispatcher, context


def setup_gpsr():
    task_list: list[Task] = []

    world, dispatcher = setup_world()

    hsrb = HSRB.from_world(world)

    context = Context(world=world, robot=hsrb)
    context.evaluate_conditions = False
    dispatcher.known_furniture = world.bodies

    left_door: Body = world.get_body_by_name("cupboard_left_door")
    right_door: Body = world.get_body_by_name("cupboard_right_door")

    left_door_connection = left_door.get_semantic_annotations_by_type(ActiveConnection1DOF)
    right_door_connection = right_door.get_semantic_annotations_by_type(ActiveConnection1DOF)

    left_door_connection.position = left_door_connection.dof.limits.upper.position
    right_door_connection.position = right_door_connection.dof.limits.upper.position

    object_list = ["Plate", "Knife", "Fork", "Spoon", "DishwasherTab", "ReferenceError", "Milk", "Bowl", "Table"]
    # Spawning objects
    generic_object_spawner(["Bowl"], [(1.325, 6.23, 0.81)], world, color=Color.GREEN())
    generic_object_spawner(["Plate"], [(0.2, 1, 0.85)], world, color=Color.ORANGE())
    # generic_object_spawner(["Milk"], [(2.42, 0.128, 0.945)], world, color=Color.RED())
    generic_object_spawner(["Milk"], [(1.037, -2.31, 0.645)], world, color=Color.RED())
    generic_object_spawner(["Knife"], [(4.65, 4.84, 1.62)], world, color=Color.CYAN())
    generic_object_spawner(["Apple"], [(4.135, 1.865, 0.54)], world, color=Color.WHITE())
    # generic_object_spawner(["Cereal"], [(1.037, -2.31, 0.645)], world, color=Color.BLUE())
    generic_object_spawner(["Cereal"], [(2.42, 0.128, 0.945)], world, color=Color.BLUE())
    generic_object_spawner(["Cup"], [(2.33475, 5.215, 0.83)], world, color=Color.BEIGE())
    pass

def setup_fd():
    task_list: list[Task] = []

    world, dispatcher = setup_world()

    hsrb = HSRB.from_world(world)

    context = Context(world=world, robot=hsrb)
    context.evaluate_conditions = False
    dispatcher.known_furniture = world.bodies

    left_door: Body = world.get_body_by_name("cupboard_left_door")
    right_door: Body = world.get_body_by_name("cupboard_right_door")

    left_door_connection = left_door.get_semantic_annotations_by_type(ActiveConnection1DOF)
    right_door_connection = right_door.get_semantic_annotations_by_type(ActiveConnection1DOF)

    left_door_connection.position = left_door_connection.dof.limits.upper.position
    right_door_connection.position = right_door_connection.dof.limits.upper.position

    object_list = [Plate, Knife, Fork, Spoon, DishwasherTab, ReferenceError, Milk, Bowl, Table]

    # Spawning objects
    generic_object_spawner(["Bowl"], [(1.325, 6.23, 0.81)], world, color=Color.GREEN())
    generic_object_spawner(["Plate"], [(0.2, 1, 0.85)], world, color=Color.ORANGE())
    # generic_object_spawner(["Milk"], [(2.42, 0.128, 0.945)], world, color=Color.RED())
    generic_object_spawner(["Milk"], [(1.037, -2.31, 0.645)], world, color=Color.RED())
    generic_object_spawner(["Knife"], [(4.65, 4.84, 1.62)], world, color=Color.CYAN())
    generic_object_spawner(["Apple"], [(4.135, 1.865, 0.54)], world, color=Color.WHITE())
    # generic_object_spawner(["Cereal"], [(1.037, -2.31, 0.645)], world, color=Color.BLUE())
    generic_object_spawner(["Cereal"], [(2.42, 0.128, 0.945)], world, color=Color.BLUE())
    generic_object_spawner(["Cup"], [(2.33475, 5.215, 0.83)], world, color=Color.BEIGE())
    pass