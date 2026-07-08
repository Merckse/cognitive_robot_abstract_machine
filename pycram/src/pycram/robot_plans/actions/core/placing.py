from __future__ import annotations

import time
from dataclasses import dataclass, field

import geometry_msgs
import numpy as np
from geometry_msgs.msg import PoseStamped
from typing_extensions import Any, Dict

from giskardpy.utils.math import point_to_caster_angles
from krrood.entity_query_language.core.base_expressions import SymbolicExpression
from krrood.entity_query_language.factories import or_, not_, and_
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import (
    Arms,
    ApproachDirection,
    VerticalAlignment,
)
from pycram.datastructures.grasp import GraspDescription
from pycram.plans.factories import sequential, execute_single, code
from pycram.querying.predicates import GripperIsFree
from pycram.robot_plans import RetractMotion, GiskardMoveGripperMotion, PointGripperForwardMotion
from pycram.robot_plans.actions.base import ActionDescription, DescriptionType
from pycram.robot_plans.actions.core.pick_up import PickUpAction, ReachAction
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction
from pycram.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)
from pycram.robot_plans.motions.transportation import ApproachPlacementMotion, GiskardReachMotion
from pycram.view_manager import ViewManager
from semantic_digital_twin.datastructures.definitions import GripperState, TorsoState
from semantic_digital_twin.reasoning.predicates import allclose
from semantic_digital_twin.reasoning.robot_predicates import is_body_in_gripper
from semantic_digital_twin.robots.abstract_robot import Manipulator
from semantic_digital_twin.semantic_annotations.semantic_annotations import TrashCan
from semantic_digital_twin.spatial_types import Point3, HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.world_entity import Body


@dataclass
class PlaceAction(ActionDescription):
    """
    Places an Object at a position using an arm.
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be place
    """
    target_location: Pose
    """
    Pose in the world at which the object should be placed
    """
    arm: Arms
    """
    Arm that is currently holding the object
    """

    assisted: bool = False
    """
    Decides if the robot takes assistance or not
    """

    def execute(self) -> None:
        arm = ViewManager.get_arm_view(self.arm, self.robot)
        manipulator = arm.manipulator

        if self.assisted:
            self.add_subplan(sequential([HandedOverAction(self.arm)])).perform()
            time.sleep(1) # TODO: for eval make higher
            self.add_subplan(
                sequential([GiskardMoveGripperMotion(gripper_state=GripperState.OPEN, arm=self.arm)])).perform()

            self.place_target_location(manipulator)

        else:
            self.add_subplan(
                sequential(
                    [
                    GiskardPlaceAction(
                        object_designator=self.object_designator,
                        arm=Arms.LEFT,
                        target_location=self.target_location,
                    ),
                    ]
                )
            ).perform()
            # Detaches the object from the robot
            world_root = self.world.root
            obj_transform = self.world.compute_forward_kinematics(
                world_root, self.object_designator
            )
            with self.world.modify_world():
                self.world.remove_connection(self.object_designator.parent_connection)
                connection = Connection6DoF.create_with_dofs(
                    parent=world_root, child=self.object_designator, world=self.world
                )
                self.world.add_connection(connection)
                connection.origin = obj_transform


        self.add_subplan(
            execute_single(RetractMotion(gripper=manipulator)
        )).perform()

    def place_target_location(self, manipulator: Manipulator):
        print(f"Into gripper with {manipulator.name}")
        tool_frame_pose = manipulator.tool_frame.global_pose

        obj = self.object_designator
        obj_rot_matrix = obj.global_pose.to_rotation_matrix()
        # tool_frame_pose = manipulator.tool_frame.global_pose

        print("placing at", self.target_location)

        world_root = self.world.root
        obj_transform = self.world.compute_forward_kinematics(
            world_root, self.object_designator
        )

        print(self.target_location)

        with self.world.modify_world():
            self.world.remove_connection(self.object_designator.parent_connection)
            connection = Connection6DoF.create_with_dofs(
                parent=world_root, child=self.object_designator, world=self.world
            )
            world_transf_obj_goal = HomogeneousTransformationMatrix.from_point_rotation_matrix(
                point=self.target_location.to_position(),
                rotation_matrix=obj_rot_matrix,
                reference_frame=self.world.root,
            )
            self.world.add_connection(connection)
            connection.origin = world_transf_obj_goal


            # old = obj.parent_connection
            # self.world.remove_connection(old)
            # self.world.add_connection(Connection6DoF(
            #     parent=old.parent, child=obj,
            #     parent_T_connection_expression=self.world.transform(world_transf_obj_goal, old.parent),
            # ))

    @staticmethod
    def pre_condition(
        variables, context: Context, kwargs: Dict[str, Any]
    ) -> SymbolicExpression:
        """
        The object needs to be in the gripper frame
        """
        manipulator = ViewManager.get_end_effector_view(variables["arm"], context.robot)
        return or_(
            not_(GripperIsFree(manipulator)),
            is_body_in_gripper(kwargs["object_designator"], manipulator) > 0.9,
        )

    @staticmethod
    def post_condition(
        variables, context: Context, kwargs: Dict[str, Any]
    ) -> SymbolicExpression:
        """
        the gripper must be free again and the object needs to be at the target location
        """
        manipulator = ViewManager.get_end_effector_view(variables["arm"], context.robot)
        return and_(
            GripperIsFree(manipulator),
            is_body_in_gripper(kwargs["object_designator"], manipulator) < 0.1,
            allclose(
                kwargs["object_designator"].global_pose,
                kwargs["target_location"].to_spatial_type(),
                atol=0.03,
            ),
        )

@dataclass
class PlaceInTrashAction(ActionDescription):
    """
    Places an Object at a position using an arm.
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be place
    """

    arm: Arms
    """
    Arm that is currently holding the object
    """

    assisted: bool = False
    """
    Decides if the robot takes assistance or not
    """

    def execute(self) -> None:
        arm = ViewManager.get_arm_view(self.arm, self.robot)
        manipulator = arm.manipulator

        trash_can = self.world.get_semantic_annotations_by_type(TrashCan)[0]
        trash_can_pose = trash_can.root.global_pose
        trash_can_size = trash_can.scale.y

        target_location = trash_can_pose

        target_location.z = target_location.z + trash_can_size + (self.object_designator.visual.scale.y/2) + 0.1

        if self.assisted:
            self.add_subplan(sequential([HandedOverAction(self.arm)])).perform()
            time.sleep(1) # TODO: for eval make higher
            self.add_subplan(
                sequential([GiskardMoveGripperMotion(gripper_state=GripperState.OPEN, arm=self.arm)])).perform()
        else:
            self.add_subplan(
                sequential(
                    [
                    GiskardPlaceAction(
                        object_designator=self.object_designator,
                        arm=Arms.LEFT,
                        target_location=target_location,
                    ),
                    ]
                )
            ).perform()

        # Detaches the object from the robot
        world_root = self.world.root
        obj_transform = self.world.compute_forward_kinematics(
            world_root, self.object_designator
        )
        with self.world.modify_world():
            self.world.remove_connection(self.object_designator.parent_connection)
            connection = Connection6DoF.create_with_dofs(
                parent=world_root, child=self.object_designator, world=self.world
            )
            self.world.add_connection(connection)
            connection.origin = obj_transform

        self.add_subplan(
            execute_single(RetractMotion(gripper=manipulator)
        )).perform()

@dataclass
class HandedOverAction(ActionDescription):
    """
    Grasps an object described by the given Object Designator description
    """

    arm: Arms
    """
    The arm that should be used to grasp
    """


    def execute(self) -> None:
        self.add_subplan(sequential([PointGripperForwardMotion(arm=self.arm)])).perform()


@dataclass
class GiskardPlaceAction(ActionDescription):
    """
    Places an Object at a position using an arm. By directly called GiskardMotion
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be place
    """

    target_location: PoseStamped | Point3 | Pose
    """
    Pose in the world at which the object should be placed
    """

    arm: Arms
    """
    Arm that is currently holding the object
    """

    ignore_orientation: bool = field(default=False, kw_only=True)
    """
    If True, the orientation of the object will be ignored.
    """

    allow_gripper_collision : bool = field(default=False, kw_only=True)

    def execute(self) -> None:
        arm = ViewManager.get_arm_view(self.arm, self.robot)
        manipulator = arm.manipulator

        if isinstance(self.target_location, Pose):
            goal = self.target_location = self.pose_to_ros(self.target_location)
        elif self.ignore_orientation:
            goal = self.target_location.pose.to_spatial_type().to_position()
        else:
            goal = self.target_location.pose.to_spatial_type()

        self.add_subplan(
            sequential([
                PointGripperForwardMotion(self.arm),
                ApproachPlacementMotion(
                    gripper=manipulator,
                    object_designator=self.object_designator,
                    goal_pose=goal,
                    allow_gripper_collision=self.allow_gripper_collision
                ),
                GiskardMoveGripperMotion(GripperState.OPEN, arm=self.arm),])
        ).perform()

    def pose_to_ros(self, pose: Pose) -> Point3:
        pose = pose.to_position()
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.pose.position.x = float(pose.x)
        pose_stamped.pose.position.y = float(pose.y)
        pose_stamped.pose.position.z = float(pose.z)
        # pose_stamped.pose.orientation.x = float(pose.to_quaternion().x)
        # pose_stamped.pose.orientation.y = float(pose.to_quaternion().y)
        # pose_stamped.pose.orientation.z = float(pose.to_quaternion().z)
        # pose_stamped.pose.orientation.w = float(pose.to_quaternion().w)
        pose_stamped.header.frame_id = pose.reference_frame.name.name
        return pose

    def point3_to_ros(self, point: Point3) -> HomogeneousTransformationMatrix:
        print("[place] point3_to_ros got:", type(point), repr(point))
        homogenous_transformation = HomogeneousTransformationMatrix()
        homogenous_transformation.reference_frame = point.reference_frame.id
        homogenous_transformation.x = float(point.x)
        homogenous_transformation.y = float(point.y)
        homogenous_transformation.z = float(point.z)

        return homogenous_transformation

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        raise NotImplementedError
        # """
        # Check if the object is placed at the target location.
        # """
        # self.validate_loss_of_contact()
        # self.validate_placement_location()

    # adapted from Pickup (duplicate code)
    def item_between_fingertips(
        self,
        fingertip_distance: float,
        closed_value: float = -0.0607,
        open_value: float = 0.1342,
        threshhold: float = 0.005,
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

    # adapted from Pickup (duplicate code)
    def validate_grasped(self):
        node = rclpy.create_node("fingertip_distance_subscriber")
        msg = None

        def callback(data: None):
            nonlocal msg
            msg = data

        subscription = node.create_subscription(
            msg_type=Float32,
            topic="/gripper_command/fingertip_distance",
            callback=callback,
            qos_profile=10,
        )

        while msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        logger.info(f"Gripper fingertip distance: {msg.data}")
        node.destroy_node()

        is_object_between_fingertips: bool = self.item_between_fingertips(
            fingertip_distance=msg.data
        )
        if not is_object_between_fingertips:
            raise ObjectNotGraspedError(
                obj=self.object_geometry, robot=self.context.robot, arm=self.arm
            )

    def validate_loss_of_contact(self):
        raise NotImplementedError
        # """
        # Check if the object is still in contact with the robot after placing it.
        # """
        # manipulator = ViewManager.get_arm_view(
        #     self.arm, self.robot_view
        # ).manipulator.tool_frame
        # contact_links = self.object_designator.get_contact_points_with_body(
        #     self.robot_view
        # ).get_all_bodies()
        # if contact_links:
        #     raise ObjectStillInContact(
        #         self.object_designator,
        #         contact_links,
        #         self.target_location,
        #         World.robot,
        #         self.arm,
        #     )

    def validate_placement_location(self):
        raise NotImplementedError
        """
        Check if the object is placed at the target location.
        """
        # pose_error_checker = PoseErrorChecker(World.conf.get_pose_tolerance())
        # if not pose_error_checker.is_error_acceptable(
        #     self.object_designator.pose, self.target_location
        # ):
        #     raise ObjectNotPlacedAtTargetLocation(
        #         self.object_designator, self.target_location, World.robot, self.arm
        #     )

# @dataclass
# class GiskardPlaceAndDetachAction(ActionDescription):
#     """
#     Places an Object at a position using an arm. By directly called GiskardMotion
#     """
#
#     object_designator: Body
#     """
#     Object designator_description describing the object that should be place
#     """
#
#     target_location: PoseStamped
#     """
#     Pose in the world at which the object should be placed
#     """
#
#     arm: Arms
#     """
#     Arm that is currently holding the object
#     """
#
#     ignore_orientation: bool = field(default=False, kw_only=True)
#     """
#     If True, the orientation of the object will be ignored.
#     """
#
#     def __post_init__(self):
#         super().__post_init__()
#
#     def execute(self) -> None:
#
#         robot_pre_action_pose = PoseStamped.from_spatial_type(
#             self.robot_view.root.global_pose
#         )
#         manipulator = self.robot_view.manipulators[self.arm]
#         print("Performing PlaceAction")
#         SequentialPlan(
#             self.context,
#             GiskardPlaceActionDescription(
#                 object_designator=self.object_designator,
#                 arm=Arms.LEFT,
#                 target_location=self.target_location,
#                 ignore_orientation=self.ignore_orientation,
#             ),
#         ).perform()
#         print("Placed object")
#
#         print("Detach object")
#         detach_object_from_hsrb(
#             world=self.world, object_designator=self.object_designator
#         )
#         print("Detached object")
#
#         print("Retracting")
#         SequentialPlan(
#             self.context,
#             RetractMotion(gripper=manipulator),
#         ).perform()
#         print("Retracted")
#
#     @classmethod
#     def description(
#         cls,
#         object_designator: Union[Iterable[Body], Body],
#         target_location: Union[Iterable[PoseStamped], PoseStamped],
#         arm: Union[Iterable[Arms], Arms],
#         ignore_orientation: bool = False,
#     ) -> PartialDesignator[GiskardPlaceAndDetachAction]:
#         return PartialDesignator[GiskardPlaceAndDetachAction](
#             GiskardPlaceAndDetachAction,
#             object_designator=object_designator,
#             target_location=target_location,
#             arm=arm,
#             ignore_orientation=ignore_orientation,
#         )
#
#
# @dataclass
# class GiskardOpenRetractAction(ActionDescription):
#     """
#     Places an Object at a position using an arm. By directly called GiskardMotion
#     """
#
#     arm: Arms
#     """
#     Arm that is currently holding the object
#     """
#
#     back_off_pose: PoseStamped = field(default=None, kw_only=True)
#
#     _pre_perform_callbacks = []
#     """
#     List to save the callbacks which should be called before performing the action.
#     """
#
#     def __post_init__(self):
#         super().__post_init__()
#
#     def execute(self) -> None:
#
#         arm = ViewManager.get_arm_view(self.arm, self.robot_view)
#         manipulator = arm.manipulator
#         SequentialPlan(
#             self.context,
#             GiskardMoveGripperMotion(GripperState.OPEN, self.arm),
#             RetractMotion(
#                 gripper=manipulator,
#             ),
#         ).perform()
#
#     def validate(
#         self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
#     ):
#         raise NotImplementedError
#         # """
#         # Check if the object is placed at the target location.
#         # """
#         # self.validate_loss_of_contact()
#         # self.validate_placement_location()
#
#     def validate_loss_of_contact(self):
#         raise NotImplementedError
#         # """
#         # Check if the object is still in contact with the robot after placing it.
#         # """
#         # manipulator = ViewManager.get_arm_view(
#         #     self.arm, self.robot_view
#         # ).manipulator.tool_frame
#         # contact_links = self.object_designator.get_contact_points_with_body(
#         #     self.robot_view
#         # ).get_all_bodies()
#         # if contact_links:
#         #     raise ObjectStillInContact(
#         #         self.object_designator,
#         #         contact_links,
#         #         self.target_location,
#         #         World.robot,
#         #         self.arm,
#         #     )
#
#     def validate_placement_location(self):
#         raise NotImplementedError
#         # """
#         # Check if the object is placed at the target location.
#         # """
#         # pose_error_checker = PoseErrorChecker(World.conf.get_pose_tolerance())
#         # if not pose_error_checker.is_error_acceptable(
#         #     self.object_designator.pose, self.target_location
#         # ):
#         #     raise ObjectNotPlacedAtTargetLocation(
#         #         self.object_designator, self.target_location, World.robot, self.arm
#         #     )
#
#     @classmethod
#     def description(
#         cls,
#         arm: Union[Iterable[Arms], Arms],
#         back_off_pose: Union[Iterable[PoseStamped], PoseStamped] | None = None,
#     ) -> PartialDesignator[GiskardOpenRetractAction]:
#         return PartialDesignator[GiskardOpenRetractAction](
#             GiskardOpenRetractAction,
#             arm=arm,
#             back_off_pose=back_off_pose,
#         )
#
#
# @dataclass
# class HandoverAction(ActionDescription):
#
#     world: World = field(kw_only=True, default=None)
#
#     def execute(self) -> None:
#         SequentialPlan(
#             self.context,
#             HandoverMotion(world=self.world),
#         ).perform()
#
#     @classmethod
#     def description(
#         cls,
#         world: World | None = None,
#     ) -> PartialDesignator[HandoverAction]:
#         return PartialDesignator[HandoverAction](
#             HandoverAction,
#             world=world,
#         )