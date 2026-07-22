from __future__ import annotations

import logging
import random
import time
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Optional

from sqlalchemy.util import arm
from typing_extensions import Any, Dict

from krrood.entity_query_language.core.base_expressions import SymbolicExpression
from krrood.entity_query_language.factories import and_, or_, not_, variable_from
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import (
    Arms,
    MovementType,
)
from pycram.datastructures.grasp import GraspDescription
from pycram.plans.factories import sequential, execute_single, code
from pycram.plans.failures import ObjectNotGrasped
from pycram.pose_validator import (
    pose_sequence_reachability_validator,
)
from pycram.querying.predicates import GripperIsFree
from pycram.robot_plans import GiskardMoveGripperMotion, PullUpMotion, PointGripperForwardMotion
from pycram.robot_plans.actions.base import ActionDescription
from pycram.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)
from pycram.robot_plans.motions.hri_handover import HandoverMotion
from pycram.robot_plans.motions.transportation import GiskardReachMotion
from pycram.view_manager import ViewManager
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.reasoning.predicates import allclose
from semantic_digital_twin.reasoning.robot_predicates import is_body_in_gripper
from semantic_digital_twin.robots.abstract_robot import ParallelGripper, Manipulator
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.connections import FixedConnection, ActiveConnection1DOF
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


@dataclass
class ReachAction(ActionDescription):
    """
    Let the robot reach a specific pose.
    """

    target_pose: Pose
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

    def execute(self) -> None:

        target_pre_pose, target_pose, _ = self.grasp_description._pose_sequence(
            self.target_pose, self.object_designator, reverse=self.reverse_reach_order
        )
        self.add_subplan(
            sequential(
                children=[
                    MoveToolCenterPointMotion(
                        target_pre_pose, self.arm, allow_gripper_collision=False
                    ),
                    MoveToolCenterPointMotion(
                        target_pose,
                        self.arm,
                        allow_gripper_collision=False,
                        movement_type=MovementType.CARTESIAN,
                    ),
                ]
            )
        ).perform()

    @staticmethod
    def pre_condition(
        variables, context: Context, kwargs: Dict[str, Any]
    ) -> SymbolicExpression:
        """
        The sequence in which the robot would reach the target pose needs to be achiveable
        """
        manipulator = ViewManager.get_end_effector_view(variables["arm"], context.robot)
        test_world = deepcopy(context.world)
        grasp_pose_sequence = kwargs["grasp_description"]._pose_sequence(
            kwargs["target_pose"],
            kwargs["object_designator"],
            reverse=kwargs["reverse_reach_order"],
        )
        return and_(
            pose_sequence_reachability_validator(
                grasp_pose_sequence,
                manipulator.tool_frame,
                context.robot.from_world(test_world),
                test_world,
                context.robot.full_body_controlled,
            ),
        )

    @staticmethod
    def post_condition(
        variables, context: Context, kwargs: Dict[str, Any]
    ) -> SymbolicExpression | bool:
        """
        The end effector needs to be close to the target pose
        """
        manipulator = ViewManager.get_end_effector_view(kwargs["arm"], context.robot)
        return or_(
            is_body_in_gripper(variable_from(kwargs["object_designator"]), manipulator)
            > 0.9,
            allclose(
                variable_from(kwargs["object_designator"].global_pose.to_position()),
                ViewManager.get_end_effector_view(
                    kwargs["arm"], context.robot
                ).tool_frame.global_pose.to_position(),
                atol=3e-2,
            ),
        )

@dataclass
class PullUpAction(ActionDescription):
    """
    High-level motion for pulling up an object with a parallel gripper.

    This motion wraps the Giskard PullUp goal and exposes it to the
    motion framework via the `_motion_chart` property.
    """

    # The gripper that will execute the pullUp (must be a ParallelGripper)
    arm: Arms = field(default=None, kw_only=True)

    # The world object that should be pulledUp
    object_geometry: Body = field(default=None, kw_only=True)

    # If True, the gripper is kept vertically aligned during the grasp
    # kw_only=True forces this to be passed as a keyword argument
    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)

    assisted : bool = field(default=False, kw_only=True)
    """
    If assisted then the pull up puts the object into hand, this is made since pycram not really allows a pretty solution.
    """

    def execute(self) -> None:
        """
        Creates and returns the underlying Giskard PickUp goal.

        The motion framework queries this property to insert the task
        into the MotionStatechart.
        """
        print(f"Creating PullUp motion with {self.object_geometry}")
        # manipulators[0] = LEFT, manipulators[1] = RIGHT
        if self.arm == Arms.LEFT:
            manipulator = self.robot.manipulators[0]
        else:
            manipulator = self.robot.manipulators[1]

        if self.assisted:
            self.into_gripper(manipulator)

        self.attach_object()

        PullUpMotion(
            manipulator=manipulator, object_geometry=self.object_geometry
        ).perform()


    def into_gripper(self, manipulator : Manipulator) -> None:
        print(f"Into gripper with {manipulator.name}")
        tool_frame_pose = manipulator.tool_frame.global_pose

        obj = self.object_geometry
        obj_rot_matrix = obj.global_pose.to_rotation_matrix()
        tool_frame_pose = manipulator.tool_frame.global_pose


        world_transf_obj_goal = HomogeneousTransformationMatrix.from_point_rotation_matrix(
            point=tool_frame_pose.to_position(),
            rotation_matrix=obj_rot_matrix,
            reference_frame=self.world.root,
        )

        old = obj.parent_connection
        new_connection = FixedConnection(
            parent=old.parent, child=obj,
            parent_T_connection_expression=self.world.transform(world_transf_obj_goal, old.parent),
        )
        with self.world.modify_world():
            self.world.remove_connection(old)
            self.world.add_connection(new_connection)

    def attach_object(self) -> None:
        end_effector = ViewManager.get_end_effector_view(self.arm, self.robot)

        # Attach the object to the end effector
        with self.world.modify_world():
            self.world.move_branch_with_fixed_connection(
                self.object_geometry, end_effector.tool_frame)


@dataclass
class GiskardGraspAction(ActionDescription):
    """
    Opens the gripper, reaches the object and closes the gripper around it.
    Runs as its own action so the motions execute before the parent action
    continues, e.g. before PickUpAction attaches the object to the gripper.
    """

    object_geometry: Body
    """
    The object that should be grasped.
    """

    arm: Arms = field(default=Arms.LEFT, kw_only=True)
    """
    Which arm to use for the grasp.
    """

    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)
    """
    If True, constrains the gripper to stay vertically aligned during the reach.
    """

    def execute(self) -> None:
        # manipulators[0] = LEFT, manipulators[1] = RIGHT
        if self.arm == Arms.LEFT:
            manipulator = self.robot.manipulators[0]
        else:
            manipulator = self.robot.manipulators[1]

        self.add_subplan(
            sequential(
                children=[
                    GiskardMoveGripperMotion(gripper_state=GripperState.OPEN, arm=self.arm),
                    GiskardReachMotion(
                        manipulator=manipulator,
                        object_geometry=self.object_geometry,
                        gripper_vertical=self.gripper_vertical,
                    ),
                    GiskardMoveGripperMotion(gripper_state=GripperState.CLOSE, arm=self.arm),
                ]
            )
        ).perform()


@dataclass
class PickUpAction(ActionDescription):
    """
    Let the robot pick up an object.
    """

    object_geometry: Body
    """
    Object designator_description describing the object that should be picked up
    """

    arm: Arms = field(default=Arms.LEFT, kw_only=True)
    """
    Which arm to use for the pickup.
    """

    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)
    """
    If True, constrains the gripper to stay vertically aligned during the reach.
    """

    probability : float= 0
    """
    The simulated probability of a event failing
    """

    assisted : bool = False

    """
    If there is a need the object will be placed in the robots hand at price of penalty
    """

    def execute(self) -> None:
        # if not self.assisted:
        #     errors = [ObjectNotGrasped, CollisionViolatedError, ObjectDoesntFitException, TimeoutError, ObjectNotReachableException]
        #     n = 100
        #     failure = random.randint(0,n)
        #     error_type = random.randint(0,3)
        #     failure_range = n * self.probability
        #     if failure < failure_range:
        #         raise errors[error_type]

        # manipulators[0] = LEFT, manipulators[1] = RIGHT

        if self.arm == Arms.LEFT:
            manipulator = self.robot.manipulators[0]
        else:
            manipulator = self.robot.manipulators[1]


        if self.assisted:
            self.add_subplan(sequential([HandedOverAction(arm=self.arm)])).perform()
            time.sleep(1) # TODO: for eval make higher
            self.add_subplan(
                sequential([GiskardMoveGripperMotion(gripper_state=GripperState.CLOSE, arm=self.arm)])).perform()

        else:
            # The grasp runs as a sub-action so its motion state chart executes here,
            # before the attach. Motions added directly to this action are merged into
            # one motion state chart that only runs after execute() returns.
            self.add_subplan(
                sequential(
                    children=[
                        GiskardGraspAction(
                            self.object_geometry,
                            arm=self.arm,
                            gripper_vertical=self.gripper_vertical,
                        )
                    ]
                )
            ).perform()


        self.add_subplan(
            sequential(
                children=[
                    PullUpAction(arm=self.arm, object_geometry=self.object_geometry, assisted=self.assisted)
                ]
            )
        ).perform()


    @staticmethod
    def pre_condition(
        variables: Dict, context: Context, kwargs: Dict[str, Any]
    ) -> SymbolicExpression:
        """
        The gripper with which to grasp the object needs to be free and the object needs to be reachable
        """
        manipulator = ViewManager.get_end_effector_view(variables["arm"], context.robot)
        test_world = deepcopy(context.world)
        grasp_pose_sequence = kwargs["grasp_description"].grasp_pose_sequence(
            kwargs["object_designator"]
        )
        return and_(
            GripperIsFree(manipulator),
            pose_sequence_reachability_validator(
                grasp_pose_sequence,
                manipulator.tool_frame,
                context.robot.from_world(test_world),
                test_world,
                context.robot.full_body_controlled,
            ),
        )

    @staticmethod
    def post_condition(
        variables: Dict, context: Context, kwargs: Dict[str, Any]
    ) -> SymbolicExpression:
        """
        The object needs to be in the griper frame
        """
        manipulator = ViewManager.get_end_effector_view(variables["arm"], context.robot)
        return or_(
            not_(GripperIsFree(manipulator)),
            is_body_in_gripper(kwargs["object_designator"], manipulator) > 0.9,
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

        self.add_subplan(
            sequential(
                [
                    MoveToolCenterPointMotion(pre_pose, self.arm),
                    MoveGripperMotion(GripperState.OPEN, self.arm),
                    MoveToolCenterPointMotion(
                        grasp_pose, self.arm, allow_gripper_collision=True
                    ),
                    MoveGripperMotion(
                        GripperState.CLOSE, self.arm, allow_gripper_collision=True
                    ),
                ]
            )
        ).perform()

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
        self.add_subplan(sequential([PointGripperForwardMotion(arm=self.arm),
                                     GiskardMoveGripperMotion(gripper_state=GripperState.OPEN,
                                                              arm=self.arm)])).perform()

# @dataclass
# class GiskardPickUpAction(ActionDescription):
#     """
#     Picks up an object using Giskard motions: open gripper → reach → close gripper.
#     Uses GiskardReachMotion internally; does not rely on grasp descriptions.
#     """
#
#     object_geometry: Body = field(default=None, kw_only=True)
#     """
#     The world object to be grasped.
#     """
#
#     arm: Arms = field(default=Arms.LEFT, kw_only=True)
#     """
#     Which arm to use for the pickup.
#     """
#
#     gripper_vertical: Optional[bool] = field(default=True, kw_only=True)
#     """
#     If True, constrains the gripper to stay vertically aligned during the reach.
#     """
#
#     def execute(self) -> bool:
#         """Open gripper → reach to object → close gripper. Returns True if sequence completes."""
#         robot_pose_pre_manipulation = PoseStamped.from_spatial_type(
#             self.context.robot.root.global_pose
#         )
#
#         grasped: bool = False
#
#         # manipulators[0] = LEFT, manipulators[1] = RIGHT
#         if self.arm == Arms.LEFT:
#             manipulator = self.robot_view.manipulators[0]
#         else:
#             manipulator = self.robot_view.manipulators[1]
#
#         # ObjectNotGraspedError is raised by the commented-out validation below if re-enabled
#         SequentialPlan(
#             self.context,
#             GiskardMoveGripperMotion(gripper_state=GripperState.OPEN, arm=self.arm),
#             GiskardReachMotion(
#                 manipulator=manipulator,
#                 object_geometry=self.object_geometry,
#                 gripper_vertical=self.gripper_vertical,
#             ),
#             GiskardMoveGripperMotion(gripper_state=GripperState.CLOSE, arm=self.arm),
#         ).perform()
#
#         # # not largely tested yet
#         # if not self.validate_grasped():
#         #     print("object has not been grasped")
#         #     raise ObjectNotGraspedError(
#         #         obj=self.object_designator, robot=self.context.robot, arm=self.arm
#         #     )
#         grasped = True
#
#         return grasped
#
#     # implement sometime, currently not implemented, since Motions have weird heirachys
#     def item_between_fingertips(
#         self,
#         fingertip_distance: float,
#         closed_value: float = -0.0607,
#         open_value: float = 0.1342,
#         threshhold: float = 0.005,
#     ) -> bool:
#         """
#         Returns True if the gripper is not fully closed and not fully open,
#         which can indicate that an item is between the fingertips.
#
#         Args:
#             fingertip_distance: Current value from /gripper_command/fingertip_distance
#             closed_value: Typical fully closed value
#             open_value: Typical fully open value
#             threshhold: Tolerance around the reference values
#
#         Returns:
#             True if the distance suggests an object is between the fingertips.
#         """
#         closed_min = closed_value - threshhold
#         closed_max = closed_value + threshhold
#         open_min = open_value - threshhold
#         open_max = open_value + threshhold
#
#         is_closed = closed_min <= fingertip_distance <= closed_max
#         is_open = open_min <= fingertip_distance <= open_max
#
#         # Object likely present if it is neither clearly open nor clearly closed
#         return not is_closed and not is_open
#
#     def validate_grasped(self):
#         node = rclpy.create_node("fingertip_distance_subscriber")
#         msg = None
#
#         def callback(data: None):
#             nonlocal msg
#             msg = data
#
#         subscription = node.create_subscription(
#             msg_type=Float32,
#             topic="/gripper_command/fingertip_distance",
#             callback=callback,
#             qos_profile=10,
#         )
#
#         while msg is None:
#             rclpy.spin_once(node, timeout_sec=0.1)
#
#         logger.info(f"Gripper fingertip distance: {msg.data}")
#         node.destroy_node()
#
#         is_object_between_fingertips: bool = self.item_between_fingertips(
#             fingertip_distance=msg.data
#         )
#         if not is_object_between_fingertips:
#             raise ObjectNotGraspedError(
#                 obj=self.object_geometry, robot=self.context.robot, arm=self.arm
#             )
#
#     @classmethod
#     def description(
#         cls,
#         object_geometry: Body,
#         arm: Arms,
#         gripper_vertical: bool = True,
#     ) -> PartialDesignator[GiskardPickUpAction]:
#         return PartialDesignator[GiskardPickUpAction](
#             GiskardPickUpAction,
#             object_geometry=object_geometry,
#             arm=arm,
#             gripper_vertical=gripper_vertical,
#         )
