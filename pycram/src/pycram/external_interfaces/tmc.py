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
