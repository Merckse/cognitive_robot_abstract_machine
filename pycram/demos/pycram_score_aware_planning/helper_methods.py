import logging
import math
from time import sleep
from typing import Optional

from common.types import Status
from common.values import evaluation
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from pycram.datastructures.grasp import GraspDescription
from pycram.language import SequentialNode
from pycram.plans.factories import sequential
from pycram.plans.plan_node import PlanNode
from pycram.robot_plans.actions.base import ActionDescription
from pycram.robot_plans.actions.core.navigation import NavigateAction
from pycram.robot_plans.actions.core.pick_up import PickUpAction
from pycram.robot_plans.actions.core.placing import PlaceAction
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction
from demos.pycram_score_aware_planning.common.types import Task, ActionType, SurfaceSpace, TaskStep
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.exceptions import WorldEntityNotFoundError
from semantic_digital_twin.reasoning.predicates import is_supported_by
from semantic_digital_twin.reasoning.queries import annotation_class_by_label
from semantic_digital_twin.robots.abstract_robot import Manipulator
from semantic_digital_twin.semantic_annotations.mixins import HasRootBody, HasSupportingSurface
from semantic_digital_twin.spatial_types import (
    Point3,
    Quaternion,
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Scale, Color
from semantic_digital_twin.world_description.world_entity import (
    Body, SemanticAnnotation,
)

import numpy as np


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

    # If the pose has a frame_id, we need to transform it to the world root frame.
    # Otherwise, we assume it is already in the world root frame.
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
        pose : list[tuple[float,float,float]] ,
        world : World,
        color: Optional[Color] = None,
):
    i = 0
    for name in names:
        pose_xyz = pose[i]
        pose_point3_ : Point3= Point3(x=pose_xyz[0], y=pose_xyz[1], z=pose_xyz[2], reference_frame=world.root)
        orientation_quaternion_ : Quaternion = Quaternion(x=0, y=0, z=0, w=1, reference_frame=world.root)
        pose_ : Pose= Pose(position=pose_point3_, orientation=orientation_quaternion_, reference_frame=world.root)
        scale_ : Scale= Scale(x=0.2, y=0.2, z=0.2)

        object_to_spawn = spawn_semantic_with_body(semantic_type=name,name=name.lower(),pose=pose_, scale=scale_,world=world, color = color)
        i+=1
    return object_to_spawn


def perceive_and_spawn_all_objects(world: World):
    """
    Query all perceived objects via the robokudo interface, extracts the relevant information for each object,
    and spawns them in the world using the spawn_semantic_with_body method.

    :param world: The world in which to spawn the perceived objects
    """

    try:
        from pycram.external_interfaces import robokudo
    except ImportError:
        raise ImportError()

    # Fix because perception output is always one query behind
    robokudo.send_query()
    sleep(2)

    perceived_objects_result = robokudo.query_all_objects().res
    for perceived_object in perceived_objects_result:

        object_dimensions = perceived_object.shape_size[0].dimensions
        object_scale = Scale(
            object_dimensions.x, object_dimensions.y, object_dimensions.z
        )

        object_pose_stamped = perceived_object.pose[0]
        object_pose = Pose(
            position=Point3(
                object_pose_stamped.pose.position.x,
                object_pose_stamped.pose.position.y,
                object_pose_stamped.pose.position.z,
            ),
            orientation=Quaternion(
                object_pose_stamped.pose.orientation.x,
                object_pose_stamped.pose.orientation.y,
                object_pose_stamped.pose.orientation.z,
                object_pose_stamped.pose.orientation.w,
            ),
            reference_frame=world.get_kinematic_structure_entity_by_name(
                object_pose_stamped.header.frame_id
            ),
        )

        object_name = extract_name_from_json_string(perceived_object.attribute[0])
        object_type = perceived_object.type

        spawn_semantic_with_body(
            semantic_type=object_type,
            name=object_name,
            scale=object_scale,
            pose=object_pose,
            world=world,
        )

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


def navigation_subplan(target_location: str, world):
    x, y, (qx, qy, qz, qw) = NAVIGATION_POSES[target_location]
    pose = Pose(
        position=Point3(x, y, 0.0, reference_frame=world.root),
        orientation=Quaternion(qx, qy, qz, qw, reference_frame=world.root),
        reference_frame=world.root,
    )
    return NavigateAction(target_location=pose, keep_joint_states=True)

def pickup_subplan(object_annotation: SemanticAnnotation, arm: Arms, world: World):
    manipulator: Manipulator = world.get_semantic_annotations_by_type(Manipulator)[0]
    object_body : Body= world.get_semantic_annotations_by_type(object_annotation)[0].bodies[0]
    grasp = GraspDescription(approach_direction=ApproachDirection.FRONT, vertical_alignment=VerticalAlignment.NoAlignment, manipulator=manipulator)
    return PickUpAction(grasp_description=grasp, object_designator=object_body, arm=arm)

def compute_surface_spaces(world) -> list[SurfaceSpace]:
    surfaces = world.get_semantic_annotations_by_type(HasSupportingSurface)
    result = []

    for surface in surfaces:
        # Prefer the pre-calculated supporting_surface Region (upward-facing geometry only)
        # over the full structural bounding box, which includes legs and support frames.
        if surface.supporting_surface is not None:
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
) -> list[tuple[Body, list]]:
    """
    Return (body, bounding_boxes) for every body currently resting on the given surface.

    A body is considered "on" the surface when its bottom face is within z_tolerance
    above the surface's top (z_surface) and its x/y centre falls within the surface footprint.
    Furniture bodies that make up the supporting surfaces themselves are excluded.
    """
    furniture_bodies: set[Body] = {
        b
        for ann in world.get_semantic_annotations_by_type(HasSupportingSurface)
        for b in ann.bodies
    }

    result = []
    for body in world.bodies_with_collision:
        if body in furniture_bodies:
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
            and surface.z_surface - z_tolerance <= z_bot
        ):
            result.append((body, bbs))
    return result

def find_free_placement_pose(
    surface: SurfaceSpace,
    object_body: Body,
    world,
    grid_step: float = 0.05,
    padding: float = 0.02,
) -> Optional[tuple[float, float]]:
    """
    Find a free (x, y) position on the surface where object_body can be placed
    without overlapping any object already on that surface.

    Returns the (x, y) centre of the object's footprint in world frame, or None if
    no free spot exists. The caller must compute z independently:
        z = surface.z_surface + object_half_z
    """
    # Object half-extents in its local frame (upright assumption)
    local_bbs = object_body.collision.as_bounding_box_collection_in_frame(object_body).bounding_boxes
    if local_bbs:
        half_x = (max(bb.x_interval.upper for bb in local_bbs) - min(bb.x_interval.lower for bb in local_bbs)) / 2 + padding
        half_y = (max(bb.y_interval.upper for bb in local_bbs) - min(bb.y_interval.lower for bb in local_bbs)) / 2 + padding
    else:
        half_x = 0.1 + padding
        half_y = 0.1 + padding

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

    # Candidate region: inset by the object's half-extents so the object stays on the surface
    x_lo = surface.x_min + half_x
    x_hi = surface.x_max - half_x
    y_lo = surface.y_min + half_y
    y_hi = surface.y_max - half_y

    if x_lo > x_hi or y_lo > y_hi:
        return None

    for x in np.arange(x_lo, x_hi + grid_step, grid_step):
        for y in np.arange(y_lo, y_hi + grid_step, grid_step):
            obj_x_min, obj_x_max = x - half_x, x + half_x
            obj_y_min, obj_y_max = y - half_y, y + half_y
            overlaps = any(
                obj_x_min < ox_max and obj_x_max > ox_min
                and obj_y_min < oy_max and obj_y_max > oy_min
                for ox_min, ox_max, oy_min, oy_max in occupied
            )
            if not overlaps:
                return float(x), float(y)

    return None



def place_subplan(object_annotation: SemanticAnnotation, arm: Arms, target_location: str, world):
    object_body : Body= world.get_semantic_annotations_by_type(object_annotation)[0].bodies[0]

    surface_spaces = compute_surface_spaces(world=world)

    free_spot = None
    matched_surface = None
    for surface_space in surface_spaces:
        if surface_space.name == target_location:
            matched_surface = surface_space
            free_spot = find_free_placement_pose(surface=surface_space, world=world, object_body=object_body)
            break

    if matched_surface is None:
        logger.warning(ValueError(f"No surface named '{target_location}' found in the world."))
        # TODO: add human assisted place
        return None
    if free_spot is None:
        logger.warning(f"No free placement spot found on '{target_location}'.")
        # TODO: add human assisted place
        return None


    x, y = free_spot
    z = matched_surface.z_surface
    if z > 1:
        z = 1

    _, _, (qx, qy, qz, qw) = NAVIGATION_POSES[target_location]
    pose = Pose(
        position=Point3(x, y, z, reference_frame=world.root),
        orientation=Quaternion(qx, qy, qz, qw, reference_frame=world.root),
        reference_frame=world.root,
    )
    return PlaceAction(object_designator=object_body, arm=arm, target_location=pose)

def assisted_place_subplan(object_name: str, arm: Arms, target_location: str, world):
    object_body : Body= world.get_body_by_name(object_name)

    surface_spaces = compute_surface_spaces(world=world)

    free_spot = None
    matched_surface = None
    for surface_space in surface_spaces:
        if surface_space.name == target_location:
            matched_surface = surface_space
            free_spot = find_free_placement_pose(surface=surface_space, world=world, object_body=object_body)
            break

    if matched_surface is None:
        logger.warning(ValueError(f"No surface named '{target_location}' found in the world."))
        # TODO: add human assisted place
        return None
    if free_spot is None:
        logger.warning(f"No free placement spot found on '{target_location}'.")
        # TODO: add human assisted place
        return None


    x, y = free_spot
    z = matched_surface.z_surface
    if z > 1:
        z = 1

    _, _, (qx, qy, qz, qw) = NAVIGATION_POSES[target_location]
    pose = Pose(
        position=Point3(x, y, z, reference_frame=world.root),
        orientation=Quaternion(qx, qy, qz, qw, reference_frame=world.root),
        reference_frame=world.root,
    )
    return PlaceAction(object_designator=object_body, arm=arm, target_location=pose)
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
                    action = place_subplan(object_name=last_pickup_object, arm=arm, target_location=task_steps.object_placement, world=context.world)
                case ActionType.PARK:
                    action = ParkArmsAction(Arms.LEFT)
                case ActionType.DETECT:
                    pass
                case _:
                    raise NotImplementedError(f"Action type not implemented: {task_steps.action_type}")
            if action is not None:
                plan.add_child(make_node(action))

    return plan

def generate_plan_task(task: Task, context: Context) -> PlanNode:
    from pycram.plans.factories import make_node
    plan = sequential(children=[], context=context)
    arm = Arms.LEFT
    action = None
    last_pickup_object: Optional[SemanticAnnotation] = None
    action_list : list[ActionDescription] = []
    for task_steps in task.task_steps:
        # TODO: add assisted tasks
        if task_steps.action_assisted:
            match task_steps.action_type:
                case ActionType.NAVIGATE:
                    action = navigation_subplan(target_location=task_steps.location, world=context.world)
                case ActionType.PICKUP:
                    last_pickup_object = task_steps.object_annotations
                    action = pickup_subplan(object_annotation=task_steps.object_annotations, arm=arm, world=context.world)
                case ActionType.PLACE:
                    action = place_subplan(object_annotation=last_pickup_object, arm=arm,
                                           target_location=task_steps.location, world=context.world)
                case ActionType.PARK:
                    action = ParkArmsAction(Arms.LEFT)
                case ActionType.DETECT:
                    pass
                case _:
                    raise NotImplementedError(f"Action type not implemented: {task_steps.action_type}")
        else:
            match task_steps.action_type:
                case ActionType.NAVIGATE:
                    action = navigation_subplan(target_location=task_steps.location, world=context.world)
                case ActionType.PICKUP:
                    last_pickup_object : SemanticAnnotation= task_steps.object_annotations
                    action = pickup_subplan(object_annotation=last_pickup_object, arm=arm, world=context.world)
                case ActionType.PLACE:
                    action = place_subplan(object_annotation=last_pickup_object, arm=arm, target_location=task_steps.location, world=context.world)
                case ActionType.PARK:
                    action = ParkArmsAction(Arms.LEFT)
                case ActionType.DETECT:
                    pass
                case _:
                    raise NotImplementedError(f"Action type not implemented: {task_steps.action_type}")
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
        if is_supported_by(body, surface.bodies[0]):
            return surface
    return None

# name → (x, y, quaternion(x,y,z,w))
NAVIGATION_POSES: dict[str, tuple[float, float, tuple[float, float, float, float]]] = {
    "cooking_table": (1.3, 4.6, _quat( math.pi / 2)),   # south of table, facing north
    "dining_table":  (2.6, 4.1, _quat( math.pi / 2)),   # south of table, facing north
    "table":         (3.5, 1.8, _quat(-math.pi / 2)),   # north of table, facing south
    "lowerTable":    (3.0, 2.2, _quat( 0.0)),           # west of table,  facing east
    "desk":          (1.3, 1.2, _quat( math.pi)),        # east of desk,   facing west
    "shelf_1":       (3.3,  4.7,  _quat( 0.0)),          # west of cupboard, facing east
    "shelf_2":       (3.3,  4.7,  _quat( 0.0)),          # same approach as shelf_1
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


def plan_to_TaskSteps(plan: SequentialNode) -> list[TaskStep]:
    sub_nodes = plan.plan.root.children
    return []
    # task_steps = []
    # for sub_node in sub_nodes:
    #     match sub_node.action_type:
    #         case ActionType.NAVIGATE:
    #     isinstance(sub_node.action, MoveTorsoAction)
