import rclpy

from pycram_suturo_demos.helper_methods_and_useful_classes.A_robot_setup import (
    robot_setup,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo
from pycram_suturo_demos.pycram_basic_hsr_demos.pickup_demo import pickup_demo
from pycram_suturo_demos.pycram_basic_hsr_demos.place_demo import place_demo
from semantic_digital_twin.spatial_types import Point3

rclpy.init()
result = robot_setup()

robot = result.robot_view
world = result.world
context = result.context

milk = world.get_body_by_name("milk.stl")
place_demo(
    simulation=True,
    context=context,
    place_pose=Point3(0, 0, 0),
    object_name="milk.stl",
    hsrb_world=world,
)
