from dataclasses import dataclass, field
from typing import Optional

from giskardpy.motion_statechart.goals.place import ApproachPlacement
from semantic_digital_twin.robots.abstract_robot import ParallelGripper, Manipulator
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.world_description.world_entity import Body
from .base import BaseMotion
from giskardpy.motion_statechart.goals.pick_up import BoxGraspMagic, GraspingSequence


@dataclass
class GiskardReachMotion(BaseMotion):
    """
    High-level motion for picking up an object with a parallel gripper.

    This motion wraps the Giskard PickUp goal and exposes it to the
    motion framework via the `_motion_chart` property.
    """

    # The gripper that will execute the pickup (must be a ParallelGripper)
    manipulator: ParallelGripper = field(default=None, kw_only=True)

    # The world object that should be picked up
    object_geometry: Body = field(default=None, kw_only=True)

    # If True, the gripper is kept vertically aligned during the grasp
    # kw_only=True forces this to be passed as a keyword argument
    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)

    def perform(self):
        return

    @property
    def _motion_chart(self):
        """
        Creates and returns the underlying Giskard PickUp goal.

        The motion framework queries this property to insert the task
        into the MotionStatechart.
        """
        print(f"Creating PickUp motion with {self.object_geometry}")
        grasp_magic = BoxGraspMagic(
            manipulator=self.manipulator,
            object_geometry=self.object_geometry,
            gripper_vertical=self.gripper_vertical,
        )
        grasp = GraspingSequence(
            grasp_magic=grasp_magic,
        )
        return grasp


@dataclass
class ApproachPlacementMotion(BaseMotion):
    """
    Motion for placing an object, i.e., moving the gripper to a certain pose
    It creates a _motion_chart that is used by the motion framework
    It directly calls the implemented PickUp of Giskard.
    """

    gripper: Manipulator = field(kw_only=True)
    """
    Name of the gripper that should be moved
    """

    object_designator: Body = field(kw_only=True)
    """
    Object designator_description describing the object that should be placed
    """
    goal_pose: HomogeneousTransformationMatrix | Point3 = field(kw_only=True)
    """
    The goal_pose at which the object should be placed
    """

    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper is allowed to collide with something
    """

    def perform(self):
        return

    @property
    def _motion_chart(self):
        goal_pose = self.goal_pose

        return ApproachPlacement(
            manipulator=self.gripper,
            object_geometry=self.object_designator,
            goal=goal_pose,
        )
