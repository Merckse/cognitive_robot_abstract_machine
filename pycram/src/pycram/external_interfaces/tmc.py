from typing_extensions import Optional
from typing import Callable

from rclpy.publisher import Publisher
from rclpy.node import Node

from tmc_voice_msgs.msg import Voice
from ..ros.ros2.publisher import create_publisher

import logging

logger = logging.getLogger(__name__)

import rclpy
from tmc_voice.tmc_voice_msgs.msg import *
from ..ros.ros2.publisher import create_publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy


class TextToSpeechPublisher:
    # Surprise its' initializations
    __is_init = False  # Private class variable

    __qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)  # Private
    __tts_node: Node  # Private
    __tts_pub: Publisher  # Private

    @staticmethod
    def __init_talk_interface(func: Callable) -> Callable:  # Private decorator
        """Ensures initialization of the talk interface before function execution."""

        def wrapper(*args, **kwargs):

            # Check if the interface is already initialized
            if TextToSpeechPublisher.__is_init:
                return func(*args, **kwargs)
            try:
                from tmc_voice_msgs.msg import Voice
            except ImportError:
                logger.warning("Failed to import tmc_voice_msgs - package may not be installed")
                return None
            TextToSpeechPublisher.__is_init = True

            rclpy.init()
            TextToSpeechPublisher.__tts_node = rclpy.create_node("tts")
            TextToSpeechPublisher.__tts_pub = create_publisher(
                "/talk_request",
                Voice,
                TextToSpeechPublisher.__tts_node,
                TextToSpeechPublisher.__qos
            )

            logger.info("Successfully initialized tmc interface")

            return func(*args, **kwargs)

        return wrapper

    @staticmethod
    def __get_tts_node(self) -> Node:  # Private method
        return TextToSpeechPublisher.__tts_node

    @__init_talk_interface  # Using private decorator
    def say(self, content: Optional[str] = ""):  # Public method
        """
        :param content: The message to be spoken
        :return:
        """
        msg = Voice()
        msg.language = 1
        msg.sentence = content

        TextToSpeechPublisher.__tts_pub.publish(msg)
        return
import rclpy

from rclpy.action import ActionClient
from rclpy.lifecycle import Node
from tmc_control_msgs.action import GripperApplyEffort


class GripperActionClient(Node):
    def __init__(self):
        super().__init__("gripper_action_client")
        self._action_client = ActionClient(
            self, GripperApplyEffort, "/gripper_controller/grasp"
        )

    def send_goal(self, effort):
        """
        Sendet ein Ziel (Goal) an den Gripper Action Server
        :param effort: Der Effort-Wert für den Gripper (positiv für öffnen, negativ für schließen)
        """
        goal_msg = GripperApplyEffort.Goal()
        goal_msg.effort = effort  # Setze den Effort-Wert

        # Sende das Ziel an den Action-Server und warte auf eine Antwort
        self.get_logger().info(f"Sending goal with effort: {effort}")
        self._action_client.wait_for_server()

        # Sende das Ziel und erhalte eine Antwort
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback, wenn der Action-Server eine Antwort auf das Ziel sendet.
        """
        result = future.result()
        if result.accepted:
            self.get_logger().info("Goal accepted by the action server.")
        else:
            self.get_logger().error("Goal rejected by the action server.")

    def feedback_callback(self, feedback):
        """
        Callback, um Feedback vom Action-Server zu erhalten.
        """
        self.get_logger().info(f"Feedback received: {feedback.feedback}")
