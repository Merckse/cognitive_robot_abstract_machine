from __future__ import annotations

import logging
from dataclasses import dataclass, field

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from nav2_msgs.action import ComputePathToPose
except ModuleNotFoundError:
    ComputePathToPose = None

logger = logging.getLogger(__name__)


@dataclass(eq=False)
class NavigationPathPreview:
    """
    Asks the planner of a Nav2-compatible navigation stack for the path to a goal and
    publishes it as a latched ``nav_msgs/Path``, so it can be shown in RViz.

    The request is asynchronous and only reads from the planner: the robot does not move
    and a navigation goal sent at the same time is neither delayed nor changed.
    """

    ros_node: Node
    """
    Node that owns the action client and the publisher.
    """

    planner_action: str = "compute_path_to_pose"
    """
    Name of the planner's ``nav2_msgs/action/ComputePathToPose`` action.
    """

    path_topic: str = "planned_path_preview"
    """
    Topic the planned path is published on.
    """

    planner_id: str = ""
    """
    Planner plugin to use; empty selects the planner's default.
    """

    latest_path: Path | None = field(init=False, default=None)
    """
    The last path the planner returned, ``None`` before the first successful request.
    """

    _client: ActionClient = field(init=False)
    _publisher: Publisher = field(init=False)

    def __post_init__(self) -> None:
        if ComputePathToPose is None:
            raise ModuleNotFoundError(
                "nav2_msgs is required for the navigation path preview"
            )
        self._client = ActionClient(
            self.ros_node, ComputePathToPose, self.planner_action
        )
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.ros_node.create_publisher(Path, self.path_topic, latched)

    def wait_for_planner(self, timeout: float) -> bool:
        """
        Wait until the planner action is available.

        :param timeout: Maximum time to wait, in s.
        :return: Whether the planner is available.
        """
        available = self._client.wait_for_server(timeout_sec=timeout)
        if not available:
            logger.warning(
                f"Planner action '{self.planner_action}' not available, "
                f"no path preview on '{self.path_topic}'"
            )
        return available

    def request(self, goal: PoseStamped) -> bool:
        """
        Ask the planner for the path from the robot's current pose to the goal.

        The path is published on :attr:`path_topic` once the planner answers.

        :param goal: Goal pose, typically in the ``map`` frame.
        :return: Whether the request was sent (``False`` if the planner is not available).
        """
        if not self._client.server_is_ready():
            logger.warning(
                f"Planner action '{self.planner_action}' not ready, skipping path preview"
            )
            return False
        request = ComputePathToPose.Goal(
            goal=goal, planner_id=self.planner_id, use_start=False
        )
        self._client.send_goal_async(request).add_done_callback(
            self._on_goal_response
        )
        return True

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            logger.warning("Path preview request rejected by the planner")
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        if result.error_code != ComputePathToPose.Result.NONE or not result.path.poses:
            logger.warning(
                f"Planner found no path (error {result.error_code}: {result.error_msg})"
            )
            return
        self.latest_path = result.path
        self._publisher.publish(result.path)
        logger.info(
            f"Planned path with {len(result.path.poses)} poses published on "
            f"'{self.path_topic}'"
        )
