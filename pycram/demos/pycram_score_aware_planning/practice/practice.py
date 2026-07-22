import rclpy

from helper_methods import setup_pp
from pycram.datastructures.enums import ChallengeMode
from pycram.robot_plans.actions.core.container import OpenAction

rclpy.init()
world, dispatcher, context = setup_pp(ChallengeMode.PP)
d = world.get_body_by_name("dishwasher")
print(d)
