# from typing_extensions import Optional
#
# from ..datastructures.enums import GripperState
# from ..robot_plans.motions import MoveGripperMotion
# from ..ros import  loginfo
# from ..ros import  create_publisher
# from ..ros import  Rate
#
# is_init = False
#
#
# def init_tmc_interface():
#     global is_init
#     if is_init:
#         return
#     from tmc_control_msgs.msg import GripperApplyEffortActionGoal
#     from tmc_msgs.msg import Voice
#     is_init = True
#     loginfo("Successfully initialized tmc interface")
#
#
#
# def tmc_gripper_control(designator: MoveGripperMotion, topic_name: Optional[str] = '/hsrb/gripper_controller/grasp/goal'):
#     """
#     Publishes a message to the gripper controller to open or close the gripper for the HSR.
#
#     :param designator: The designator containing the motion to be executed
#     :param topic_name: The topic name to publish the message to
#     """
#     if (designator.motion == GripperState.OPEN):
#         pub_gripper = create_publisher(topic_name, GripperApplyEffortActionGoal, 10)
#         rate = Rate(10)
#         msg = GripperApplyEffortActionGoal()
#         msg.goal.effort = 0.8
#         pub_gripper.publish(msg)
#
#     elif (designator.motion == GripperState.CLOSE):
#         pub_gripper = create_publisher(topic_name, GripperApplyEffortActionGoal, 10)
#         rate = Rate(10)
#         msg = GripperApplyEffortActionGoal()
#         msg.goal.effort = -0.8
#         pub_gripper.publish(msg)
#
#
# def tmc_talk(designator: TalkingMotion, topic_name: Optional[str] = '/talk_request'):
#     """
#     Publishes a sentence to the talk_request topic of the HSRB robot
#
#     :param designator: The designator containing the sentence to be spoken
#     :param topic_name: The topic name to publish the sentence to
#     """
#     pub = create_publisher(topic_name, Voice, 10)
#     texttospeech = Voice()
#     # language 1 = english (0 = japanese)
#     texttospeech.language = 1
#     texttospeech.sentence = designator.cmd
#
#     pub.publish(texttospeech)

from tmc_voice_msgs.msg import Voice
from ..ros.ros2.publisher import create_publisher

import logging
logger = logging.getLogger(__name__)

import rclpy
from tmc_voice_msgs.msg import Voice
from ..ros.ros2.publisher import create_publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy

class TextToSpeechPublisher():
    # Surprise its' initializations
    is_init = False

    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    tts_node: Node
    tts_pub: Publisher
    @staticmethod
    def init_talk_interface(func: Callable) -> Callable:


        """Ensures initialization of the talk interface before function execution."""
        def wrapper(*args, **kwargs):

            # Check if the interface is already initialized
            if TextToSpeechPublisher.is_init:
                return func(*args, **kwargs)
            try:
                from tmc_voice_msgs.msg import Voice        # This import is only needed if you intend to use voice
            except ImportError:
                logger.warning("Failed to import tmc_voice_msgs - package may not be installed")
                return None
            TextToSpeechPublisher.is_init = True

            rclpy.init()
            TextToSpeechPublisher.tts_node = rclpy.create_node("tts")
            TextToSpeechPublisher.tts_pub = create_publisher("/talk_request", Voice, TextToSpeechPublisher.tts_node, TextToSpeechPublisher.qos)

            logger.info("Successfully initialized tmc interface")

            return func(*args, **kwargs)
        return wrapper

    def get_tts_node(self)-> Node:
        return TextToSpeechPublisher.tts_node

    @init_talk_interface
    def say(self, content : Optional[str] = ""):
        """

        :param content: The message to be spoken
        :return:
        """

        msg = Voice()

        msg.language = 1
        msg.sentence = content

        TextToSpeechPublisher.tts_pub.publish(msg)
        return