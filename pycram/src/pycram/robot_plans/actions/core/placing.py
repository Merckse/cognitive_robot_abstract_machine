from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field
from datetime import timedelta

import geometry_msgs.msg
import rclpy
from narwhals import Float32

from pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    detach_object_from_hsrb,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.spatial_types import Point3
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import Union, Optional, Any, Iterable

from .pick_up import PickUpAction
from pycram.robot_plans.motions.gripper import GiskardMoveGripperMotion, RetractMotion
from pycram.robot_plans.motions.gripper import MoveTCPMotion
from pycram.robot_plans.motions.hri_handover import HandoverMotion
from pycram.robot_plans.motions.transportation import ApproachPlacementMotion
from ....datastructures.enums import (
    Arms,
    ApproachDirection,
    VerticalAlignment,
)
from ....datastructures.grasp import GraspDescription
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....failures import (
    ObjectNotPlacedAtTargetLocation,
    ObjectStillInContact,
    ObjectNotGraspedError,
)
from ....language import SequentialPlan
from ....view_manager import ViewManager
from ....robot_plans.actions.base import ActionDescription
from ....validation.error_checkers import PoseErrorChecker

logger = logging.getLogger(__name__)


@dataclass
class PlaceAction(ActionDescription):
    """
    Places an Object at a position using an arm.
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be place
    """
    target_location: PoseStamped
    """
    Pose in the world at which the object should be placed
    """
    arm: Arms
    """
    Arm that is currently holding the object
    """
    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()

    def execute(self) -> None:
        arm = ViewManager.get_arm_view(self.arm, self.robot_view)
        manipulator = arm.manipulator

        previous_pick = self.plan.get_previous_node_by_designator_type(
            self.plan_node, PickUpAction
        )
        previous_grasp = (
            previous_pick.designator_ref.grasp_description
            if previous_pick
            else GraspDescription(
                ApproachDirection.FRONT, VerticalAlignment.NoAlignment, manipulator
            )
        )

        SequentialPlan(
            self.context,
            ReachActionDescription(
                self.target_location,
                self.arm,
                previous_grasp,
                self.object_designator,
                reverse_reach_order=True,
            ),
            MoveGripperMotion(GripperState.OPEN, self.arm),
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

        _, _, retract_pose = previous_grasp._pose_sequence(
            self.target_location, self.object_designator, reverse=True
        )

        SequentialPlan(self.context, MoveTCPMotion(retract_pose, self.arm)).perform()

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        """
        Check if the object is placed at the target location.
        """
        self.validate_loss_of_contact()
        self.validate_placement_location()

    def validate_loss_of_contact(self):
        """
        Check if the object is still in contact with the robot after placing it.
        """
        contact_links = self.object_designator.get_contact_points_with_body(
            World.robot
        ).get_all_bodies()
        if contact_links:
            raise ObjectStillInContact(
                self.object_designator,
                contact_links,
                self.target_location,
                World.robot,
                self.arm,
            )

    def validate_placement_location(self):
        """
        Check if the object is placed at the target location.
        """
        pose_error_checker = PoseErrorChecker(World.conf.get_pose_tolerance())
        if not pose_error_checker.is_error_acceptable(
            self.object_designator.pose, self.target_location
        ):
            raise ObjectNotPlacedAtTargetLocation(
                self.object_designator, self.target_location, World.robot, self.arm
            )

    @classmethod
    def description(
        cls,
        object_designator: Union[Iterable[Body], Body],
        target_location: Union[Iterable[PoseStamped], PoseStamped],
        arm: Union[Iterable[Arms], Arms],
    ) -> PartialDesignator[PlaceAction]:
        return PartialDesignator[PlaceAction](
            PlaceAction,
            object_designator=object_designator,
            target_location=target_location,
            arm=arm,
        )


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

    def __post_init__(self):
        super().__post_init__()

    def execute(self) -> None:
        arm = ViewManager.get_arm_view(self.arm, self.robot_view)
        manipulator = arm.manipulator

        if isinstance(self.target_location, Point3):
            self.point3_to_ros(self.target_location)

        if isinstance(self.target_location, Pose):
            self.target_location = self.pose_to_ros(self.target_location)

        if self.ignore_orientation:
            goal = self.target_location.pose.to_spatial_type().to_position()
        else:
            goal = self.target_location.pose.to_spatial_type()
        goal.reference_frame = self.target_location.frame_id

        SequentialPlan(
            self.context,
            ApproachPlacementMotion(
                gripper=manipulator,
                object_designator=self.object_designator,
                goal_pose=goal,
            ),
            GiskardMoveGripperMotion(GripperState.OPEN, arm=self.arm),
        ).perform()

        detach_object_from_hsrb(
            world=self.object_designator._world,
            object_designator=self.object_designator,
        )

        SequentialPlan(self.context, RetractMotion(gripper=manipulator)).perform()

    def pose_to_ros(self, pose: Pose) -> PoseStamped:
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.pose.position.x = float(pose.x)
        pose_stamped.pose.position.y = float(pose.y)
        pose_stamped.pose.position.z = float(pose.z)
        pose_stamped.pose.orientation.x = float(pose.to_quaternion().x)
        pose_stamped.pose.orientation.y = float(pose.to_quaternion().y)
        pose_stamped.pose.orientation.z = float(pose.to_quaternion().z)
        pose_stamped.pose.orientation.w = float(pose.to_quaternion().w)
        pose_stamped.header.frame_id = pose.reference_frame.name.name
        return pose_stamped

    def point3_to_ros(self, point: Point3) -> PoseStamped:
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.pose.position.x = float(point.x)
        pose_stamped.pose.position.y = float(point.y)
        pose_stamped.pose.position.z = float(point.z)
        pose_stamped.pose.orientation.x = float(0)
        pose_stamped.pose.orientation.y = float(0)
        pose_stamped.pose.orientation.z = float(0)
        pose_stamped.pose.orientation.w = float(1)
        pose_stamped.header.frame_id = point.reference_frame.id
        return pose_stamped

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

    @classmethod
    def description(
        cls,
        object_designator: Body,
        target_location: PoseStamped | Pose | Point3,
        arm: Arms,
        ignore_orientation: bool = False,
    ) -> PartialDesignator[GiskardPlaceAction]:
        return PartialDesignator[GiskardPlaceAction](
            GiskardPlaceAction,
            object_designator=object_designator,
            target_location=target_location,
            arm=arm,
            ignore_orientation=ignore_orientation,
        )


@dataclass
class GiskardPlaceAndDetachAction(ActionDescription):
    """
    Places an Object at a position using an arm. By directly called GiskardMotion
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be place
    """

    target_location: PoseStamped
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

    def __post_init__(self):
        super().__post_init__()

    def execute(self) -> None:

        robot_pre_action_pose = PoseStamped.from_spatial_type(
            self.robot_view.root.global_pose
        )
        manipulator = self.robot_view.manipulators[self.arm]
        print("Performing PlaceAction")
        SequentialPlan(
            self.context,
            GiskardPlaceActionDescription(
                object_designator=self.object_designator,
                arm=Arms.LEFT,
                target_location=self.target_location,
                ignore_orientation=self.ignore_orientation,
            ),
        ).perform()
        print("Placed object")

        print("Detach object")
        detach_object_from_hsrb(
            world=self.world, object_designator=self.object_designator
        )
        print("Detached object")

        print("Retracting")
        SequentialPlan(
            self.context,
            RetractMotion(gripper=manipulator),
        ).perform()
        print("Retracted")

    @classmethod
    def description(
        cls,
        object_designator: Union[Iterable[Body], Body],
        target_location: Union[Iterable[PoseStamped], PoseStamped],
        arm: Union[Iterable[Arms], Arms],
        ignore_orientation: bool = False,
    ) -> PartialDesignator[GiskardPlaceAndDetachAction]:
        return PartialDesignator[GiskardPlaceAndDetachAction](
            GiskardPlaceAndDetachAction,
            object_designator=object_designator,
            target_location=target_location,
            arm=arm,
            ignore_orientation=ignore_orientation,
        )


@dataclass
class GiskardOpenRetractAction(ActionDescription):
    """
    Places an Object at a position using an arm. By directly called GiskardMotion
    """

    arm: Arms
    """
    Arm that is currently holding the object
    """

    back_off_pose: PoseStamped = field(default=None, kw_only=True)

    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()

    def execute(self) -> None:

        arm = ViewManager.get_arm_view(self.arm, self.robot_view)
        manipulator = arm.manipulator
        SequentialPlan(
            self.context,
            GiskardMoveGripperMotion(GripperState.OPEN, self.arm),
            RetractMotion(
                gripper=manipulator,
            ),
        ).perform()

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        raise NotImplementedError
        # """
        # Check if the object is placed at the target location.
        # """
        # self.validate_loss_of_contact()
        # self.validate_placement_location()

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
        # """
        # Check if the object is placed at the target location.
        # """
        # pose_error_checker = PoseErrorChecker(World.conf.get_pose_tolerance())
        # if not pose_error_checker.is_error_acceptable(
        #     self.object_designator.pose, self.target_location
        # ):
        #     raise ObjectNotPlacedAtTargetLocation(
        #         self.object_designator, self.target_location, World.robot, self.arm
        #     )

    @classmethod
    def description(
        cls,
        arm: Union[Iterable[Arms], Arms],
        back_off_pose: Union[Iterable[PoseStamped], PoseStamped] | None = None,
    ) -> PartialDesignator[GiskardOpenRetractAction]:
        return PartialDesignator[GiskardOpenRetractAction](
            GiskardOpenRetractAction,
            arm=arm,
            back_off_pose=back_off_pose,
        )


@dataclass
class HandoverAction(ActionDescription):

    world: World = field(kw_only=True, default=None)

    def execute(self) -> None:
        SequentialPlan(
            self.context,
            HandoverMotion(world=self.world),
        ).perform()

    @classmethod
    def description(
        cls,
        world: World | None = None,
    ) -> PartialDesignator[HandoverAction]:
        return PartialDesignator[HandoverAction](
            HandoverAction,
            world=world,
        )


PlaceActionDescription = PlaceAction.description
GiskardPlaceActionDescription = GiskardPlaceAction.description
GiskardPlaceAndDetachActionDescription = GiskardPlaceAndDetachAction.description
GiskardOpenRetractActionDescription = GiskardOpenRetractAction.description
HandoverActionDescription = HandoverAction.description
