import time

from common.hsrb_testing import setup_world
from helper_methods import generic_object_spawner
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, TaskStatus
from pycram.motion_executor import simulated_robot
from pycram.plans.factories import sequential
from pycram.robot_plans.actions.core.navigation import NavigateAction
from pycram.robot_plans.actions.core.pick_up import PickUpAction
from pycram.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Bowl
from semantic_digital_twin.spatial_types.spatial_types import Pose

world, dispatcher = setup_world()
hsrb = HSRB.from_world(world)

context = Context(world , hsrb)


plan = sequential(
    [
        MoveTorsoAction(TorsoState.HIGH),
        ParkArmsAction(Arms.BOTH),
        MoveTorsoAction(TorsoState.LOW),

    ],
    context=context,
)

with simulated_robot:
    plan.perform()
    plan.perform()

    child = plan.children
    for c in child:
        print(c)
        print(c.status)
    #     if c.status == TaskStatus.INTERRUPTED:
    #         print(c)
    #         print("interrupted")
    # plan.resume()
    # plan.perform()
    # print(plan.status)

        # print(e)
        # print("status",plan.status)
        # print(plan.plan)
        # children = plan.children
        #
        # for c in plan.children:
        #     print("CHILD",c,c.status)
