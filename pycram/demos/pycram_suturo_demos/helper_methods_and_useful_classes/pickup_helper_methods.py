import math
import time
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot
from pycram.ros.ros2.ros_tools import wait_for_message
from pycram.datastructures.enums import PickUpMode
from pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
    perceive_and_spawn_all_objects,
)
from semantic_digital_twin.reasoning.predicates import compute_euclidean_distance_2d
from semantic_digital_twin.robots.abstract_robot import ParallelGripper
from semantic_digital_twin.semantic_annotations.mixins import HasSupportingSurface
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Color
from semantic_digital_twin.world_description.world_entity import (
    Body,
    SemanticAnnotation,
)
from suturo_resources.queries import (
    query_get_next_object_euclidean_x_y,
    query_semantic_annotations_on_surfaces,
    query_annotations_by_color,
)

import semantic_digital_twin
import logging

logger = logging.getLogger(__name__)


def try_get_object_to_pickup(world: World, object_name_method: str) -> Body | None:
    """
    Tries to retrieve a body from the world by name.
    Raises an exception if the object is not found.

    :param world: The world to search in.
    :param object_name_method: The name of the object to retrieve.
    :return: The Body with the given name.
    :raises Exception: If no object with the given name exists in the world.
    """
    try:
        object_to_pickup_method = world.get_body_by_name(object_name_method)
        logger.info(f"picking up object with name '{object_name_method}'")
        return object_to_pickup_method
    except semantic_digital_twin.exceptions.WorldEntityNotFoundError:
        raise Exception(f"object with name '{object_name_method}' not found")


def attach_object_to_hsrb(world: World, object_designator: Body):
    """
    Attaches the given object to the HSR-B's end effector tool frame.
    This is a workaround since attaching within Actions does not work with motions.

    :param world: The world in which to attach the object.
    :param object_designator: The body to attach to the gripper.
    """
    manipulator = world.get_semantic_annotations_by_type(ParallelGripper)[0]
    with world.modify_world():
        world.move_branch_with_fixed_connection(
            object_designator, manipulator.tool_frame
        )


def detach_object_from_hsrb(world: World, object_designator: Body):
    """
    Detaches the given object from the HSR-B by re-parenting it to the world root.
    This is a workaround since detaching within Actions does not work with motions.

    :param world: The world in which to detach the object.
    :param object_designator: The body to detach from the gripper.
    """
    with world.modify_world():
        world.move_branch_with_fixed_connection(object_designator, world.root)


def try_perceiving_and_spawning_and_find_object(
    world: World, object_name: str
) -> Body | None:
    """
    Attempts to perceive and spawn all objects via RoboKudo, then retrieves
    the object matching the given name. Gracefully handles missing RoboKudo import.

    :param world: The world in which to spawn perceived objects.
    :param object_name: The name of the object to find after spawning.
    :return: The Body matching the given name, or None if not found.
    """
    try:
        from demos.pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
            perceive_and_spawn_all_objects,
        )

        perceived_objects: dict[Any, Any] = perceive_and_spawn_all_objects(world)
        logger.info(f"perceived following objects: '{perceived_objects}'")
    except ImportError:
        logger.info("Could not import robokudo")
        perceived_objects = {}
    object_to_pickup = try_get_object_to_pickup(world, object_name)
    logger.info(f"object_to_Pickup: '{object_to_pickup}'")
    return object_to_pickup


def query_get_next_surface_euclidean_x_y(
    main_body: Body,
) -> SemanticAnnotation:
    """
    Queries the next surface based on Euclidean distance in x and y coordinates
    relative to the given main body. This function utilizes
    semantic annotations of objects and orders them by their Euclidean distances
    to the main body.

    :param main_body: The main body to which the Euclidean distance is computed.
    :return: The closest SemanticAnnotation with a supporting surface.
    """
    world = main_body._world
    supporting_surfaces = world.get_semantic_annotations_by_type(HasSupportingSurface)
    closest_supporting_surface_distance = (
        supporting_surfaces[0],
        compute_euclidean_distance_2d(
            body1=supporting_surfaces[0].root,
            body2=main_body,
        ),
    )  # default value

    for supporting_surface in supporting_surfaces:
        current_surface_distance = compute_euclidean_distance_2d(
            body1=supporting_surface.root,
            body2=main_body,
        )
        if current_surface_distance < closest_supporting_surface_distance[1]:
            closest_supporting_surface_distance = (
                supporting_surface,
                current_surface_distance,
            )

    return closest_supporting_surface_distance[0]


def get_nearest_object(
    world: World, supporting_surface_name: Optional[str] = None
) -> Body | None:
    """
    Returns the nearest object to the robot on a supporting surface, if no surface is named, then it falls back,
    based on Euclidean distance in the X-Y plane.

    :param world: The world to query.
    :param supporting_surface_name: Optional surface name; falls back to nearest surface if None.
    :return: The nearest Body on the surface, or None if the surface is empty.
    """
    robot_view = Context.from_world(world).robot

    if supporting_surface_name is not None:
        supporting_surface = world.get_semantic_annotations_by_name(
            supporting_surface_name
        )
    else:
        supporting_surface = query_get_next_surface_euclidean_x_y(robot_view.bodies[0])

    nearest_objects_list = query_get_next_object_euclidean_x_y(
        robot_view.bodies[0], supporting_surface
    ).tolist()
    return nearest_objects_list[0].bodies[0] if nearest_objects_list else None


def get_object_with_color(
    world: World, color: Color, supporting_surface_name: Optional[str] = None
) -> Body | None:
    """
    Returns the first object on the given surface that matches the color.
    Falls back to the nearest surface if supporting_surface_name is None.

    :param world: The world to query.
    :param color: The color to filter objects by.
    :param supporting_surface_name: Optional surface name; falls back to nearest surface if None.
    :return: The first matching Body, or None if no match is found.
    """
    robot_view = Context.from_world(world).robot
    if supporting_surface_name is not None:
        supporting_surface = world.get_semantic_annotations_by_name(
            supporting_surface_name
        )
    else:
        supporting_surface = query_get_next_surface_euclidean_x_y(robot_view.bodies[0])
    cooking_table_annotation = world.get_semantic_annotation_by_name("cooking_table")
    objects_on_table = query_semantic_annotations_on_surfaces(
        [supporting_surface], world
    ).tolist()
    colored_objects = query_annotations_by_color(color, objects_on_table)
    return colored_objects[0].bodies[0] if colored_objects else None


def parse_color(color_str: str) -> Color:
    """
    Converts a color name string to a Color object.
    Falls back to white if the color string is not recognized.

    :param color_str: A color name string (e.g. 'red', 'blue', 'green').
    :return: The corresponding Color object, or Color.WHITE() if unrecognized.
    """
    color_map = {
        "red": Color.RED(),
        "yellow": Color.YELLOW(),
        "green": Color.GREEN(),
        "cyan": Color.CYAN(),
        "blue": Color.BLUE(),
        "magenta": Color.MAGENTA(),
        "white": Color.WHITE(),
        "black": Color.BLACK(),
        "gray": Color.GRAY(),
        "grey": Color.GRAY(),
        "beige": Color.BEIGE(),
        "orange": Color.ORANGE(),
    }
    return color_map.get(color_str.strip().lower(), Color.WHITE())


# TODO: fix the static cooking_table bias
def get_pickup_mode() -> tuple[PickUpMode, str, Color]:
    """
    Prompts the user to select a pickup mode and any required parameters.
    Returns the selected mode along with the object name or color if applicable.

    :return: A tuple of (PickUpType, object_name, object_color).
             object_name is empty string if not applicable.
             object_color defaults to Color.WHITE() if not applicable.
    """
    object_color = Color.WHITE()
    object_name = ""
    mode_map = {
        "nearest object": PickUpMode.PICK_UP_OBJECT_BY_NEAREST,
        "object by color": PickUpMode.PICK_UP_OBJECT_BY_COLOR,
        "object by name": PickUpMode.PICK_UP_OBJECT_SEARCH,
    }

    mode_str = input(
        "Pick up modes: object by name, object by color, nearest object? (e.g: nearest object)"
    )

    mode = mode_map.get(mode_str.strip().lower(), PickUpMode.PICK_UP_OBJECT_BY_NEAREST)

    match mode:
        case PickUpMode.PICK_UP_OBJECT_SEARCH:
            object_name = input("Which object do you want to pick up? ")
            logger.info(f"Looking for object: {object_name}")
        case PickUpMode.PICK_UP_OBJECT_BY_COLOR:
            color_str = input(
                "Which color should the object be? (e.g: red, blue, green) "
            )
            object_color = parse_color(color_str)
            logger.info(f"Object color: {object_color}")

    return mode, object_name, object_color


def object_to_pickup_by_mode(
    world: World,
    mode: PickUpMode,
    supporting_surface_name: Optional[str] = None,
    object_name: str = "",
    color: Color = Color.WHITE(),
) -> Body | None:
    """
    Resolves which object to pick up based on the given pickup mode. This is best performed RIGHT before pickup, since the nearest_object method uses the robots position

    :param world: The world to search for objects in.
    :param mode: The pickup strategy to use.
    :param supporting_surface_name: The name of the supporting surface to use.
    :param object_name: The name of the object (used for PICK_UP_OBJECT_SEARCH mode).
    :param color: The color to filter by (used for PICK_UP_OBJECT_BY_COLOR mode).
    :return: The resolved Body to pick up, or None if no suitable object was found.
    """
    match mode:
        case PickUpMode.PICK_UP_OBJECT_SEARCH:
            object_to_pickup = try_perceiving_and_spawning_and_find_object(
                world=world, object_name=object_name
            )
            logger.info(f"object_to_pickup by name: {object_to_pickup}")
        case PickUpMode.PICK_UP_OBJECT_BY_COLOR:
            object_to_pickup = get_object_with_color(
                world=world,
                color=color,
                supporting_surface_name=supporting_surface_name,
            )
            logger.info(f"object_to_pickup by color: {object_to_pickup}")
        case _:
            object_to_pickup = get_nearest_object(
                world=world, supporting_surface_name=supporting_surface_name
            )
            logger.info(f"object_to_pickup by nearest: {object_to_pickup}")
    return object_to_pickup


def item_between_fingertips(
    fingertip_distance: float,
    closed_value: float = -0.1007,
    open_value: float = 0.0538,
    threshhold: float = 0.05,
) -> bool:
    """
    Returns True if the gripper is not fully closed and not fully open,
    which can indicate that an item is between the fingertips.

    Args:
        fingertip_distance: Current value from /gripper_command/fingertip_distance
        closed_value: Typical fully closed value
        open_value: Typical fully open value
        threshhold: Tolerance around the reference values

    Returns:
        True if the distance suggests an object is between the fingertips.
    """
    closed_min = closed_value - threshhold
    closed_max = closed_value + threshhold
    open_min = open_value - threshhold
    open_max = open_value + threshhold

    is_closed = closed_min <= fingertip_distance <= closed_max
    is_open = open_min <= fingertip_distance <= open_max

    # Object likely present if it is neither clearly open nor clearly closed
    return not is_closed and not is_open


def validate_grasped() -> bool:
    """
    Subscribes to /gripper_command/fingertip_distance once, then checks whether
    the distance indicates an object is held. Creates and destroys a temporary node.
    """
    node = rclpy.create_node("gripper_distance_subscriber")

    msg = wait_for_message(
        msg_type=float, node=node, topic_name="/gripper_command/fingertip_distance"
    )
    success = msg is not None
    if success:
        logger.info(f"Gripper fingertip distance: {msg.data}")
    else:
        logger.warning("Timed out waiting for gripper fingertip distance")
    node.destroy_node()

    is_object_between_fingertips = item_between_fingertips(fingertip_distance=msg)
    return is_object_between_fingertips


@dataclass
class PickupDeadzone:
    """Defines the spatial limits within which the robot can successfully grasp an object."""

    min_distance: float = 0.3  # too close, robot can't reach down
    max_distance: float = 0.8  # too far to reach
    max_angle_deg: float = 45.0  # cone in front of robot (±45°)
    max_height_diff: float = 0.2  # object must be near floor level


def is_in_pickup_zone(self, object_position: tuple[float, float, float]) -> bool:
    """
    Returns True if the object is within the robot's reachable pickup envelope.

    :param object_position: (x, y, z) in the robot's base frame — x forward, y left, z up.
    """
    ox, oy, oz = object_position

    # Horizontal distance from robot base
    distance = math.sqrt(ox**2 + oy**2)

    # Too close or too far
    if distance < self.deadzone.min_distance:
        logger.debug(f"Object too close: {distance:.2f}m")
        return False
    if distance > self.deadzone.max_distance:
        logger.debug(f"Object too far: {distance:.2f}m")
        return False

    # Outside forward cone
    angle_deg = math.degrees(math.atan2(abs(oy), ox))
    if angle_deg > self.deadzone.max_angle_deg:
        logger.debug(f"Object outside reach cone: {angle_deg:.1f}°")
        return False

    # Object too high or too low
    if abs(oz) > self.deadzone.max_height_diff:
        logger.debug(f"Object height out of range: {oz:.2f}m")
        return False

    return True


def look_at_point(context: Context, point: Point3):
    """Turns the robot head to face the given 3-D point (simulated motion only)."""
    from pycram.robot_plans import LookAtActionDescription

    with simulated_robot:
        SequentialPlan(
            context,
            LookAtActionDescription(
                PoseStamped.from_spatial_type(
                    HomogeneousTransformationMatrix.from_point_rotation_matrix(
                        point=point, reference_frame=point.reference_frame
                    )
                )
            ),
        ).perform()


# TODO: put into proper file
def try_percieve_and_retrieve(
    simulated: bool = False,
    context: Context = None,
    angle: int = 1,
    talking_node: Any = None,
    object_name: str = None,
) -> Body | None:
    """
    Moves the robot to a perception pose, runs object detection, then returns
    the named object. Returns None if the object is not found after detection.
    """
    from pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo

    world = context.world
    robot_view = context.robot
    standard_delay = 2
    table = world.get_body_by_name("cooking_table")
    look_at = HomogeneousTransformationMatrix.to_position(table.global_pose)

    talking_node.pub(
        text=f"Trying to position, to perceive object.", delay=standard_delay
    )
    move_demo(
        simulated=simulated,
        world=world,
        context=context,
        target_pose="PERCEPTION_ANGLE_" + str(angle),
    )
    look_at_point(context, look_at)
    perceive_and_spawn_all_objects(world)

    try:
        object_to_pickup: Body | None = world.get_body_by_name(object_name)
        talking_node.pub(
            text=f"Found object {object_to_pickup.name}.", delay=standard_delay
        )
        return object_to_pickup
    except Exception:
        object_to_pickup: Body | None = None
        talking_node.pub(
            text=f"Could not find object {object_name}.",
            delay=standard_delay,
        )
        time.sleep(2)
        return object_to_pickup
