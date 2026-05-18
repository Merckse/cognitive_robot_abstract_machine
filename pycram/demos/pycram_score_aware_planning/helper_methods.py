from time import sleep
from typing import Optional

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from pycram.datastructures.grasp import GraspDescription
from pycram.plans.factories import sequential
from pycram.robot_plans.actions.core.navigation import NavigateAction
from pycram.robot_plans.actions.core.pick_up import PickUpAction
from pycram.robot_plans.actions.core.placing import PlaceAction
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction
from pycram_score_aware_planning.Evaluate.types import TaskEstimation, Task, ActionType
from pycram_score_aware_planning.Evaluate.values import NAVIGATION_POSES
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.exceptions import WorldEntityNotFoundError
from semantic_digital_twin.reasoning.queries import annotation_class_by_label
from semantic_digital_twin.robots.abstract_robot import Manipulator
from semantic_digital_twin.semantic_annotations.mixins import HasRootBody
from semantic_digital_twin.spatial_types import (
    Point3,
    Quaternion,
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Scale, Color


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
        i=+1
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

def normalize_task_estimation(task_estimation : list[TaskEstimation]) -> list[TaskEstimation]:
    scores : list[float] = []
    score_per_seconds : list[float] = []
    probabilities : list[float] = []
    for task_estimate in task_estimation:
        scores.append(task_estimate.task_score)
        score_per_seconds.append(task_estimate.task_score_per_seconds)
        probabilities.append(task_estimate.task_probability)
    normalized_score = normalize_list_values(scores)
    normalized_score_per_seconds = normalize_list_values(score_per_seconds)
    normalized_probabilities = normalize_list_values(probabilities)
    for i,task_estimate in enumerate(task_estimation):
        task_estimate.task_score = normalized_score[i]
        task_estimate.task_score_per_seconds = normalized_score_per_seconds[i]
        task_estimate.task_probability = normalized_probabilities[i]
    return task_estimation


def navigation_subplan(target_location: str, world):
    x, y, (qx, qy, qz, qw) = NAVIGATION_POSES[target_location]
    pose = Pose(
        position=Point3(x, y, 0.0, reference_frame=world.root),
        orientation=Quaternion(qx, qy, qz, qw, reference_frame=world.root),
        reference_frame=world.root,
    )
    return NavigateAction(target_location=pose, keep_joint_states=True)

def pickup_subplan(object_name: str, arm: Arms, world: World):
    manipulator: Manipulator = world.get_semantic_annotations_by_type(Manipulator)[0]
    object_body = world.get_body_by_name(object_name)
    grasp = GraspDescription(approach_direction=ApproachDirection.FRONT, vertical_alignment=VerticalAlignment.NoAlignment, manipulator=manipulator)
    return PickUpAction(grasp_description=grasp, object_designator=object_body, arm=arm)

def place_subplan(object_name: str, arm: Arms, target_location: str, world):
    object_body = world.get_body_by_name(object_name)
    x, y, (qx, qy, qz, qw) = NAVIGATION_POSES[target_location]
    pose = Pose(
        position=Point3(x, y, 0.8, reference_frame=world.root),
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
                    last_pickup_object = task_steps.object_name
                    action = pickup_subplan(object_name=task_steps.object_name, arm=arm, world=context.world)
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

