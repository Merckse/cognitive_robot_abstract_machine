from __future__ import annotations

import logging
from dataclasses import dataclass, field
from datetime import timedelta

import rclpy
from std_msgs.msg import Float32
from typing_extensions import Union, Optional, Any, Iterable

from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body
from pycram.robot_plans.motions.gripper import MoveTCPMotion, MoveGripperMotion
from ...motions.gripper import GiskardMoveGripperMotion, RetractMotion, PullUpMotion
from ...motions.transportation import GiskardReachMotion

from ....config.action_conf import ActionConfig
from ....datastructures.enums import (
    Arms,
    MovementType,
    FindBodyInRegionMethod,
)
from ....datastructures.grasp import GraspDescription
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....failures import ObjectNotGraspedError
from ....failures import ObjectNotInGraspingArea
from ....language import SequentialPlan

from ....view_manager import ViewManager
from ....robot_plans.actions.base import ActionDescription

logger = logging.getLogger(__name__)


@dataclass
class ReachAction(ActionDescription):
    """
    Let the robot reach a specific pose.
    """

    target_pose: PoseStamped
    """
    Pose that should be reached.
    """

    arm: Arms
    """
    The arm that should be used for pick up
    """

    grasp_description: GraspDescription
    """
    The grasp description that should be used for picking up the object
    """

    object_designator: Body = None
    """
    Object designator_description describing the object that should be picked up
    """

    reverse_reach_order: bool = False

    def __post_init__(self):
        super().__post_init__()

    def execute(self) -> None:

        target_pre_pose, target_pose, _ = self.grasp_description._pose_sequence(
            self.target_pose, self.object_designator, reverse=self.reverse_reach_order
        )

        SequentialPlan(
            self.context,
            MoveTCPMotion(target_pre_pose, self.arm, allow_gripper_collision=False),
            MoveTCPMotion(
                target_pose,
                self.arm,
                allow_gripper_collision=False,
                movement_type=MovementType.CARTESIAN,
            ),
        ).perform()

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        """
        Check if object is contained in the gripper such that it can be grasped and picked up.
        """
        fingers_link_names = self.arm_chain.end_effector.fingers_link_names
        if fingers_link_names:
            if not is_body_between_fingers(
                self.object_designator,
                fingers_link_names,
                method=FindBodyInRegionMethod.MultiRay,
            ):
                raise ObjectNotInGraspingArea(
                    self.object_designator,
                    World.robot,
                    self.arm,
                    self.grasp_description,
                )
        else:
            logger.warning(
                f"Cannot validate reaching to pick up action for arm {self.arm} as no finger links are defined."
            )

    @classmethod
    def description(
        cls,
        target_pose: Union[Iterable[PoseStamped], PoseStamped],
        arm: Union[Iterable[Arms], Arms] = None,
        grasp_description: Union[Iterable[GraspDescription], GraspDescription] = None,
        object_designator: Union[Iterable[Body], Body] = None,
        reverse_reach_order: Union[Iterable[bool], bool] = False,
    ) -> PartialDesignator[ReachAction]:
        return PartialDesignator[ReachAction](
            ReachAction,
            target_pose=target_pose,
            arm=arm,
            grasp_description=grasp_description,
            object_designator=object_designator,
            reverse_reach_order=reverse_reach_order,
        )


@dataclass
class PickUpAction(ActionDescription):
    """
    Let the robot pick up an object.
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be picked up
    """

    arm: Arms
    """
    The arm that should be used for pick up
    """

    grasp_description: GraspDescription
    """
    The GraspDescription that should be used for picking up the object
    """

    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()

    def execute(self) -> None:
        SequentialPlan(
            self.context,
            MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm),
            ReachActionDescription(
                target_pose=PoseStamped.from_spatial_type(
                    self.object_designator.global_pose
                ),
                object_designator=self.object_designator,
                arm=self.arm,
                grasp_description=self.grasp_description,
            ),
            MoveGripperMotion(motion=GripperState.CLOSE, gripper=self.arm),
        ).perform()

        end_effector = ViewManager.get_end_effector_view(self.arm, self.robot_view)

        # Attach the object to the end effector
        with self.world.modify_world():
            self.world.move_branch_with_fixed_connection(
                self.object_designator, end_effector.tool_frame
            )

        _, _, lift_to_pose = self.grasp_description.grasp_pose_sequence(
            self.object_designator
        )
        SequentialPlan(
            self.context,
            MoveTCPMotion(
                lift_to_pose,
                self.arm,
                allow_gripper_collision=True,
                movement_type=MovementType.TRANSLATION,
            ),
        ).perform()

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        """
        Check if picked up object is in contact with the gripper.
        """
        if not has_gripper_grasped_body(self.arm, self.object_designator):
            raise ObjectNotGraspedError(
                self.object_designator, World.robot, self.arm, self.grasp_description
            )

    @classmethod
    def description(
        cls,
        object_designator: Union[Iterable[Body], Body],
        arm: Union[Iterable[Arms], Arms] = None,
        grasp_description: Union[Iterable[GraspDescription], GraspDescription] = None,
    ) -> PartialDesignator[PickUpAction]:
        return PartialDesignator[PickUpAction](
            PickUpAction,
            object_designator=object_designator,
            arm=arm,
            grasp_description=grasp_description,
        )


@dataclass
class GraspingAction(ActionDescription):
    """
    Grasps an object described by the given Object Designator description
    """

    object_designator: Body
    """
    Object Designator for the object that should be grasped
    """
    arm: Arms
    """
    The arm that should be used to grasp
    """
    grasp_description: GraspDescription
    """
    The distance in meters the gripper should be at before grasping the object
    """

    def execute(self) -> None:
        pre_pose, grasp_pose, _ = self.grasp_description.grasp_pose_sequence(
            self.object_designator
        )

        SequentialPlan(
            self.context,
            MoveTCPMotion(pre_pose, self.arm),
            MoveGripperMotion(GripperState.OPEN, self.arm),
            MoveTCPMotion(grasp_pose, self.arm, allow_gripper_collision=True),
            MoveGripperMotion(
                GripperState.CLOSE, self.arm, allow_gripper_collision=True
            ),
        ).perform()

    @classmethod
    def description(
        cls,
        object_designator: Union[Iterable[Body], Body],
        arm: Union[Iterable[Arms], Arms] = None,
        grasp_description: Union[
            Iterable[GraspDescription], GraspDescription
        ] = ActionConfig.grasping_prepose_distance,
    ) -> PartialDesignator[GraspingAction]:
        return PartialDesignator[GraspingAction](
            GraspingAction,
            object_designator=object_designator,
            arm=arm,
            grasp_description=grasp_description,
        )


@dataclass
class GiskardPickUpAction(ActionDescription):
    """
    Let the robot pick up an object.
    """

    object_geometry: Body = field(default=None, kw_only=True)
    """
    Object designator_description describing the object that should be picked up
    """

    arm: Arms = field(default=Arms.LEFT, kw_only=True)
    """
    arms that should be used for pick up
    """

    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)
    """
    If True, the gripper is kept vertically aligned during the grasp
    kw_only=True forces this to be passed as a keyword argument
    """

    def execute(self) -> bool:
        # Register attach as a post-perform callback BEFORE queuing the motion
        robot_pose_pre_manipulation = PoseStamped.from_spatial_type(
            self.context.robot.root.global_pose
        )

        grasped: bool = False

        if self.arm == Arms.LEFT:
            manipulator = self.robot_view.manipulators[0]
        else:
            manipulator = self.robot_view.manipulators[1]

        # try to grasp the object, if it is not grasped, throw an ObjectNotGraspedError so one can react within the demo
        SequentialPlan(
            self.context,
            GiskardMoveGripperMotion(gripper_state=GripperState.OPEN, arm=self.arm),
            GiskardReachMotion(
                manipulator=manipulator,
                object_geometry=self.object_geometry,
                gripper_vertical=self.gripper_vertical,
            ),
            GiskardMoveGripperMotion(gripper_state=GripperState.CLOSE, arm=self.arm),
        ).perform()

        # # not largely tested yet
        # if not self.validate_grasped():
        #     print("object has not been grasped")
        #     raise ObjectNotGraspedError(
        #         obj=self.object_designator, robot=self.context.robot, arm=self.arm
        #     )
        grasped = True

        return grasped

    # implement sometime, currently not implemented, since Motions have weird heirachys
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

    @classmethod
    def description(
        cls,
        object_geometry: Body,
        arm: Arms,
        gripper_vertical: bool = True,
    ) -> PartialDesignator[GiskardPickUpAction]:
        return PartialDesignator[GiskardPickUpAction](
            GiskardPickUpAction,
            object_geometry=object_geometry,
            arm=arm,
            gripper_vertical=gripper_vertical,
        )


ReachActionDescription = ReachAction.description
PickUpActionDescription = PickUpAction.description
GraspingActionDescription = GraspingAction.description
GiskardPickUpActionDescription = GiskardPickUpAction.description
