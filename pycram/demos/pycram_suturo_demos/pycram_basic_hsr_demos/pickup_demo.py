import time

from rclpy import logging

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, PickUpMode
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot, ExecutionEnvironment
from pycram.robot_plans import (
    GiskardPickUpActionDescription,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.nlp_human_robot_interaction import (
    TalkingNode,
)

from semantic_digital_twin.world_description.world_entity import Body


# ------------------------ BASE-DEFINITIONS
def pickup_demo(
    simulation: bool = True,
    context: Context = None,
    object_to_pickup: Body = None,
):
    """Runs GiskardPickUpAction for the given object. Skips silently if object_to_pickup is None."""
    # logger creation
    talking_node = TalkingNode()
    standard_delay = 2
    logger = logging.get_logger(__name__)

    # object_to_pickup is None when perception found nothing or the NLP parse failed
    if object_to_pickup == None:
        logger.warning("object_to_pickup is None, therefor pickup is skipped")
        return

    robot_type: ExecutionEnvironment = simulated_robot if simulation else real_robot

    # ------------------------ EXECUTION
    with robot_type:
        talking_node.pub(text="Stating pickup", delay=standard_delay)
        logger.info("Starting pickup demo")
        pickup_callback = SequentialPlan(
            context,
            GiskardPickUpActionDescription(
                object_geometry=object_to_pickup,
                arm=Arms.LEFT,
                gripper_vertical=True,
            ),
        ).perform()

        talking_node.pub("PickUp has is now finished", delay=standard_delay)


def pickup_with_callback_demo(
    simulation: bool = True,
    context: Context = None,
    object_to_pickup: Body = None,
):
    """Same as pickup_demo but returns the SequentialPlan result so callers can check success."""
    # logger creation
    talking_node = TalkingNode()
    standard_delay = 2
    logger = logging.get_logger(__name__)

    # object_to_pickup is None when perception found nothing or the NLP parse failed
    if object_to_pickup == None:
        logger.warning("object_to_pickup is None, therefor pickup is skipped")
        return

    robot_type: ExecutionEnvironment = simulated_robot if simulation else real_robot

    # ------------------------ EXECUTION
    with robot_type:
        talking_node.pub(text="Stating pickup", delay=standard_delay)
        logger.info("Starting pickup demo")
        pickup_callback = SequentialPlan(
            context,
            GiskardPickUpActionDescription(
                object_designator=object_to_pickup,
                arm=Arms.LEFT,
                gripper_vertical=True,
            ),
        ).perform()
        logger.info("PickUp has is now finished", delay=standard_delay)
        return pickup_callback
