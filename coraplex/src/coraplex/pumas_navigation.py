"""
Driving the HSR's base with the Pumas navigation of the lab, which plans its path on the
laser map.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from datetime import timedelta
from enum import StrEnum

import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.publisher import Publisher
from typing_extensions import Self

from coraplex.exceptions import NavigationGoalNotHeard, NavigationTimedOut
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.adapters.ros.tfwrapper import TFWrapper
from semantic_digital_twin.robots.robot_parts import AbstractRobot

# %% the Pumas interface


class PumasTopic(StrEnum):
    """
    The topics the Pumas navigation of the lab listens on.
    """

    GOAL = "/move_base_simple/goal"
    """
    Where to drive to, a pose in the map frame; RViz's *2D Goal Pose* tool sends there.
    """


# %% comparing base poses


@dataclass
class BasePoseDifference:
    """
    How far apart two poses of the base are on the floor, ignoring height, roll and
    pitch.
    """

    distance: float
    """
    The distance between the two positions on the floor, in meters.
    """

    yaw_error: float
    """
    The absolute angle between the two headings, between 0 and pi, in radians.
    """

    @classmethod
    def from_poses(cls, first: Pose, second: Pose) -> Self:
        """
        :param first: One pose of the base.
        :param second: The other pose of the base, in the same frame as ``first``.
        :return: The difference between ``first`` and ``second``.
        """
        yaw_difference = float(first.yaw) - float(second.yaw)
        return cls(
            distance=float(
                np.hypot(
                    float(first.x) - float(second.x), float(first.y) - float(second.y)
                )
            ),
            yaw_error=float(
                abs(np.arctan2(np.sin(yaw_difference), np.cos(yaw_difference)))
            ),
        )

    def is_within(self, distance_tolerance: float, yaw_tolerance: float) -> bool:
        """
        :param distance_tolerance: The largest :attr:`distance` allowed, in meters.
        :param yaw_tolerance: The largest :attr:`yaw_error` allowed, in radians.
        :return: Whether both parts of this difference stay within their tolerance.
        """
        return self.distance <= distance_tolerance and self.yaw_error <= yaw_tolerance


# %% driving with Pumas


@dataclass
class PumasNavigation:
    """
    Sends a pose to the running Pumas navigation and waits until the robot's base stands
    still there, as tf reports it.

    Pumas reports no outcome, so arriving is judged by :attr:`position_threshold`,
    :attr:`orientation_threshold` and the base having come to rest, see
    :meth:`wait_until_standing_at`.
    """

    node: Node
    """
    The spun node the goal is published and tf is listened to through.
    """

    robot: AbstractRobot
    """
    The robot whose base is driven.
    """

    position_threshold: float = 0.3
    """
    How far the base may stand from the goal, in meters, to count as arrived.
    """

    orientation_threshold: float = 0.1
    """
    How far the base may be turned away from the goal, in radians, to count as arrived.
    """

    timeout: timedelta = timedelta(minutes=3)
    """
    How long the base may take to arrive, and the goal topic to be listened to.
    """

    stillness_duration: timedelta = timedelta(seconds=0.5)
    """
    How long the base has to stay at rest within the thresholds to count as arrived.
    """

    stillness_position_tolerance: float = 0.01
    """
    How far the base may drift, in meters, and still count as at rest.
    """

    stillness_orientation_tolerance: float = 0.01
    """
    How far the base may turn, in radians, and still count as at rest.
    """

    polling_period: timedelta = timedelta(seconds=0.1)
    """
    How often the base pose is looked up while waiting.
    """

    goal_publisher: Publisher = field(init=False)
    """
    Publishes the goals on :attr:`PumasTopic.GOAL`.
    """

    tf: TFWrapper = field(init=False)
    """
    Looks up where the base stands.
    """

    def __post_init__(self) -> None:
        """
        Create :attr:`goal_publisher` and :attr:`tf` on :attr:`node`.
        """
        self.goal_publisher = self.node.create_publisher(
            PoseStamped, PumasTopic.GOAL, 1
        )
        self.tf = TFWrapper(node=self.node)

    def drive_to(self, goal: Pose) -> None:
        """
        Have Pumas drive the base to ``goal`` and wait until it stands still there.

        :param goal: Where the base should stand, in the root frame of the robot's
            world, which is the map frame Pumas plans in.
        :raises NavigationGoalNotHeard: If Pumas does not listen within :attr:`timeout`.
        :raises NavigationTimedOut: If the base does not arrive within :attr:`timeout`.
        """
        self.wait_until_pumas_listens()
        world = self.robot._world
        map_T_goal = world.transform(goal, world.root)
        message = PoseStamped()
        message.header.frame_id = str(world.root.name.name)
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.position.x = float(map_T_goal.x)
        message.pose.position.y = float(map_T_goal.y)
        x, y, z, w = map_T_goal.quaternion.to_np()
        message.pose.orientation.x = float(x)
        message.pose.orientation.y = float(y)
        message.pose.orientation.z = float(z)
        message.pose.orientation.w = float(w)
        self.goal_publisher.publish(message)
        self.wait_until_standing_at(map_T_goal)

    def wait_until_pumas_listens(self) -> None:
        """
        :raises NavigationGoalNotHeard: If nothing subscribes to :attr:`PumasTopic.GOAL`
            within :attr:`timeout`.
        """
        deadline = time.monotonic() + self.timeout.total_seconds()
        while self.goal_publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise NavigationGoalNotHeard(topic=PumasTopic.GOAL)
            time.sleep(self.polling_period.total_seconds())

    def wait_until_standing_at(self, map_T_goal: Pose) -> None:
        """
        Wait until the base stands within :attr:`position_threshold` and
        :attr:`orientation_threshold` of ``map_T_goal`` and has stayed within the
        stillness tolerances of one pose for :attr:`stillness_duration`.

        :param map_T_goal: The goal sent to Pumas, in the map frame.
        :raises NavigationTimedOut: If the base does not arrive within :attr:`timeout`.
        """
        deadline = time.monotonic() + self.timeout.total_seconds()
        resting_pose = self.current_base_pose()
        resting_since = time.monotonic()
        while True:
            time.sleep(self.polling_period.total_seconds())
            now = time.monotonic()
            base_pose = self.current_base_pose()
            if not BasePoseDifference.from_poses(base_pose, resting_pose).is_within(
                self.stillness_position_tolerance, self.stillness_orientation_tolerance
            ):
                resting_pose = base_pose
                resting_since = now
            is_at_goal = BasePoseDifference.from_poses(base_pose, map_T_goal).is_within(
                self.position_threshold, self.orientation_threshold
            )
            if is_at_goal and now - resting_since >= (
                self.stillness_duration.total_seconds()
            ):
                return
            if now >= deadline:
                raise NavigationTimedOut(goal=map_T_goal, timeout=self.timeout)

    def current_base_pose(self) -> Pose:
        """
        :return: Where the base stands in the map frame, as tf reports it now.
        """
        world = self.robot._world
        map_T_base = self.tf.lookup_transform(
            str(world.root.name.name), str(self.robot.root.name.name)
        ).transform
        return Pose.from_xyz_quaternion(
            map_T_base.translation.x,
            map_T_base.translation.y,
            map_T_base.translation.z,
            map_T_base.rotation.x,
            map_T_base.rotation.y,
            map_T_base.rotation.z,
            map_T_base.rotation.w,
            reference_frame=world.root,
        )
