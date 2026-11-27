"""
Tests the path preview of NavigateActionServerTask with in-process mock action servers for
navigation (nav2_msgs/NavigateToPose) and planning (nav2_msgs/ComputePathToPose).
"""

from __future__ import annotations

import threading
import time
import uuid
from dataclasses import dataclass, field

import pytest

pytest.importorskip("nav2_msgs")

from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from cramph.context import StatechartContext
from cramph.data_types import ObservationStateValues
from cramph.executor import RealTimePacer, StatechartExecutor
from cramph.statechart import Statechart
from giskardpy.motion_control import MotionControl
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.ros2_nodes.action_servers import (
    NavigateActionServerTask,
)
from giskardpy.motion_statechart.ros_context import RosNodeAccess
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.world import World


def unique_name(prefix: str) -> str:
    """
    :return: A name that is unique per test, to prevent DDS crosstalk.
    """
    return f"{prefix}_{uuid.uuid4().hex[:8]}"


@dataclass
class MockNavigation:
    """
    Navigation server that reaches every goal at once.
    """

    received_goals: list[ROSPoseStamped] = field(default_factory=list)

    def execute(self, goal_handle: ServerGoalHandle) -> NavigateToPose.Result:
        self.received_goals.append(goal_handle.request.pose)
        goal_handle.succeed()
        return NavigateToPose.Result()


@dataclass
class MockPlanner:
    """
    Planner that answers with a straight two-pose path to the requested goal.
    """

    received_goals: list[ComputePathToPose.Goal] = field(default_factory=list)

    def execute(self, goal_handle: ServerGoalHandle) -> ComputePathToPose.Result:
        request = goal_handle.request
        self.received_goals.append(request)
        start = ROSPoseStamped()
        start.header.frame_id = "map"
        result = ComputePathToPose.Result()
        result.path.header.frame_id = "map"
        result.path.poses = [start, request.goal]
        goal_handle.succeed()
        return result


def latched_path_subscription(node: Node, topic: str) -> tuple[list[Path], threading.Event]:
    """
    :return: The list received paths are appended to and an event set on the first one.
    """
    received: list[Path] = []
    arrived = threading.Event()

    def on_path(msg: Path) -> None:
        received.append(msg)
        arrived.set()

    node.create_subscription(
        Path,
        topic,
        on_path,
        QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        ),
    )
    return received, arrived


def run_navigation(task: NavigateActionServerTask, world: World, node: Node) -> None:
    """
    Executes the task in a statechart until the navigation succeeded.
    """
    statechart = Statechart(context=StatechartContext(world=world))
    statechart.add_node(node=task)
    statechart.add_node(node=EndMotion.when_true(node=task))
    executor = StatechartExecutor(
        context=statechart.context,
        extensions=[RosNodeAccess(ros_node=node), MotionControl()],
        pacer=RealTimePacer(),
    )
    executor.compile(statechart=statechart)
    executor.tick_until_end()


def navigation_task(world: World, navigation_topic: str, **kwargs) -> NavigateActionServerTask:
    return NavigateActionServerTask(
        action_topic=navigation_topic,
        message_type=NavigateToPose,
        target_pose=Pose.from_xyz_rpy(x=1.5, y=-0.5, reference_frame=world.root),
        base_link=world.root,
        **kwargs,
    )


def test_preview_publishes_the_planned_path(rclpy_node: Node) -> None:
    world = World.create_with_root_body("map")
    navigation_topic, planner_topic = unique_name("navigate"), unique_name("compute_path")
    path_topic = unique_name("planned_path_preview")
    navigation, planner = MockNavigation(), MockPlanner()
    servers = [
        ActionServer(rclpy_node, NavigateToPose, navigation_topic, navigation.execute),
        ActionServer(rclpy_node, ComputePathToPose, planner_topic, planner.execute),
    ]
    received, arrived = latched_path_subscription(rclpy_node, path_topic)
    task = navigation_task(
        world,
        navigation_topic,
        preview_path_action=planner_topic,
        preview_path_topic=path_topic,
    )
    try:
        run_navigation(task, world, rclpy_node)
        assert arrived.wait(timeout=5.0), "no path published"
        assert task.on_tick() == ObservationStateValues.TRUE
        assert len(planner.received_goals) == 1
        assert planner.received_goals[0].use_start is False
        assert planner.received_goals[0].goal.pose.position.x == pytest.approx(1.5)
        assert planner.received_goals[0].goal.pose.position.y == pytest.approx(-0.5)
        assert len(navigation.received_goals) == 1
        assert received[0].poses[-1].pose.position.x == pytest.approx(1.5)
        assert task._path_preview.latest_path is not None
    finally:
        for server in servers:
            server.destroy()


def test_navigation_works_without_a_planner(rclpy_node: Node) -> None:
    world = World.create_with_root_body("map")
    navigation_topic = unique_name("navigate")
    navigation = MockNavigation()
    server = ActionServer(rclpy_node, NavigateToPose, navigation_topic, navigation.execute)
    task = navigation_task(
        world, navigation_topic, preview_path_action=unique_name("no_planner_here")
    )
    try:
        start = time.time()
        run_navigation(task, world, rclpy_node)
        assert task.on_tick() == ObservationStateValues.TRUE
        assert len(navigation.received_goals) == 1
        assert task._path_preview.latest_path is None
        assert time.time() - start < 10.0
    finally:
        server.destroy()


def test_preview_is_off_by_default(rclpy_node: Node) -> None:
    world = World.create_with_root_body("map")
    navigation_topic = unique_name("navigate")
    navigation = MockNavigation()
    server = ActionServer(rclpy_node, NavigateToPose, navigation_topic, navigation.execute)
    task = navigation_task(world, navigation_topic)
    try:
        run_navigation(task, world, rclpy_node)
        assert task.on_tick() == ObservationStateValues.TRUE
        assert task._path_preview is None
    finally:
        server.destroy()
