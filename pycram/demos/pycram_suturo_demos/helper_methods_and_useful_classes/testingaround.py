import rclpy

from giskardpy.motion_statechart import context
from pycram_suturo_demos.helper_methods_and_useful_classes.A_robot_setup import (
    robot_setup,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo
from suturo_resources.queries import (
    query_get_next_surface_euclidean_x_y,
    query_get_next_surface_euclidean_x_y2,
)

rclpy.init()
result = robot_setup()

robot = result.robot_view
world = result.world
context = result.context

move_demo("CABINET", world, context)
