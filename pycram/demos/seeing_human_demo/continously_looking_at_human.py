from suturo_resources.suturo_map import load_environment

from pycram.datastructures.enums import TorsoState, Arms
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import (
    MoveTorsoActionDescription,
    ParkArmsActionDescription,
    LookAtActionDescription,
)
from ..pycram_suturo_demo.simulation_setup import setup_hsrb_in_environment
from pycram.external_interfaces import robokudo
import time
from time import sleep
from pycram.ros_utils.text_to_image import TextToImagePublisher

result = setup_hsrb_in_environment(load_environment=load_environment, with_viz=True)
world, robot_view, context, viz = (
    result.world,
    result.robot_view,
    result.context,
    result.viz,
)


with simulated_robot:
    """Testing"""
    text_pub = TextToImagePublisher()
    found_position = True
    timeout = time.time() + 30
    # While human is seen print location
    while found_position:
        # Send goal
        position = robokudo.query_current_human_position_in_continues()
        if position is not None and position.header.stamp.sec > time.time() - timeout:
            x = round(position.point.x)
            y = round(position.point.y)
            z = round(position.point.z)
            text_pub.publish_text(f"Point x: {x} y: {y} z: {z}")
            plan = SequentialPlan(
                context,
                ParkArmsActionDescription(Arms.BOTH),
                LookAtActionDescription(position),
            )
            plan.perform()
        else:
            text_pub.publish_text("No Human seen.")
            found_position = False
        sleep(0.5)

    # Close every think
    robokudo.shutdown_robokudo_interface()
