import time

import rclpy

from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from semantic_digital_twin.predetermined_maps.kitchen_environment import KitchenEnvironment

rclpy.init()
node = rclpy.create_node("kitchen_environment")
publisher = VizMarkerPublisher(_world=KitchenEnvironment().get_world(), node=node)
(publisher.with_tf_publisher())
time.sleep(100)