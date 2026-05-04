import os
import time

from demos.pycram_score_aware_planning.eval import RobotScorer, ActionType, ActionOutcome
from demos.pycram_score_aware_planning.helper_methods import generic_object_spawner
from giskardpy.motion_statechart import context
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from pycram.datastructures.grasp import GraspDescription

from pycram.motion_executor import simulated_robot
from pycram.plans.factories import sequential
from pycram.plans.plan import Plan
from pycram.robot_plans.actions.composite.transporting import TransportAction
from pycram.robot_plans.actions.core.pick_up import PickUpAction
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction

from hsrb_testing import setup_world
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.collision_checking.collision_detector import CollisionCheckingResult
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.reasoning.world_reasoner import WorldReasoner
# from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Bowl,
    Spoon,
    Drawer,
    Handle,
)
from semantic_digital_twin.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Color

# HSRB specified local world setup
world = setup_world()

hsrb = HSRB.from_world(world)
context = Context(robot=hsrb,world=world)

generic_object_spawner(["Bowl"], [(4,1,4)], world, color=Color.GREEN())
generic_object_spawner(["bowl3"], [(4,2,4)], world, color=Color.GREEN())
bowl =  world.get_semantic_annotation_by_name("bowl3").bodies[0]
# print(world.get_semantic_annotation_by_name("cup").bodies[1])
manipulator = hsrb.arm.manipulator
pickup = sequential([PickUpAction(arm=Arms.LEFT, object_designator=bowl, grasp_description=GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, manipulator, rotate_gripper=True)
                                )], context=context)
plan_parkarm = sequential([ParkArmsAction(Arms.LEFT)], context=context)
robot_scoorer = RobotScorer()
with simulated_robot:
    print(pickup.status)
    try:
        pickup.perform()
    except Exception as e:
        pass
    outcome = pickup.status
    robot_scoorer.record(action_type=ActionType.PICKUP, object_name="bowl", outcome=outcome)
    print(robot_scoorer.summary())

    plan_parkarm.perform()
