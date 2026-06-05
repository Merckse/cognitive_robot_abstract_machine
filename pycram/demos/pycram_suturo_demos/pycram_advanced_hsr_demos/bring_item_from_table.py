from __future__ import annotations

import logging
from typing import Optional

import rclpy

from pycram.datastructures.dataclasses import Context
from pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    try_percieve_and_retrieve,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.hri_handover import (
    handover_robot_human_no_init,
    handover_human_robot_no_init,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo
from pycram_suturo_demos.pycram_basic_hsr_demos.pickup_demo import (
    pickup_demo,
)
from semantic_digital_twin.world_description.world_entity import Body

from demos.pycram_suturo_demos.helper_methods_and_useful_classes.nlp_human_robot_interaction import (
    TalkingNode,
)

logger = logging.getLogger(__name__)


def bring_item_from_table_to_human_demo(
    *,
    context: Context,
    object_name: str,
    supporting_surface_str: Optional[str] = None,
):
    """
    Full pick-and-handover sequence:
    1. Move to table
    2. Perceive from up to 3 angles; retry outer loop once if pickup fails
    3. Pick up object
    4. Move to start pose and hand over to human

    Falls back to human-to-robot handover if all pickup attempts fail.
    """
    rclpy.init()
    simulated = False
    world = context.world
    talking_node = TalkingNode()
    standard_delay = 2

    # TODO insert correct name here
    table = world.get_body_by_name(supporting_surface_str)

    # Move to table, on which the object is to be expected.
    move_to_table = move_demo(
        simulated=simulated,
        world=world,
        context=context,
        target_pose=supporting_surface_str,
    )
    move_to_starting_pose = move_demo(
        simulated=simulated,
        world=world,
        context=context,
        target_pose="ROBOT_START_POSE",
    )

    talking_node.pub(text="Moving to table.", delay=standard_delay)
    move_to_table()

    # Outer loop: retry the full perceive-and-pickup cycle up to 2 times.
    for j in range(2):

        # Inner loop: try up to 3 perception angles before giving up on this attempt.
        for i in range(3):
            object_to_pickup: Body | None = try_percieve_and_retrieve(
                context=context,
                object_name=object_name,
                talking_node=talking_node,
                simulated=simulated,
                angle=i,
            )
            if object_to_pickup:
                break

        if object_to_pickup is None:
            talking_node.pub(
                text="I couldnt find the object, driving back to start.",
                delay=standard_delay,
            )
            move_to_starting_pose()
            break

        pickup_callback = pickup_demo(
            simulation=simulated,
            context=context,
            object_to_pickup=object_to_pickup,
        )

        if pickup_callback:
            talking_node.pub(
                text="The object has been picked up, now moving to starting pose for handover",
                delay=standard_delay,
            )
            break
        else:
            talking_node.pub(
                text="object has not been picked up, retrying", delay=standard_delay
            )
            continue
    if pickup_callback is False:
        talking_node.pub(
            text="object has not been picked up, initiating human to hand-over",
            delay=standard_delay,
        )
        handover_human_robot_no_init(context=context, simulated=simulated)

    move_to_starting_pose()
    handover_robot_human_no_init(context=context, simulated=simulated)
