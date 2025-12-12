import rclpy
import time
from cognitive_robot_abstract_machine.pycram.src.pycram.ros.ros2.subscriber import create_subscriber
import logging

from tmc_voice.tmc_voice_msgs.msg import *

logger = logging.getLogger()

try:

while True:
    time.sleep(2)

    subscriber = create_subscriber("/talk_request", Voice)
    rclpy