import os
import time

from giskardpy.motion_statechart import context
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from pycram.datastructures.grasp import GraspDescription

from pycram.motion_executor import simulated_robot
from pycram.plans.factories import sequential
from pycram.plans.plan import Plan
from pycram.robot_plans.actions.composite.transporting import TransportAction
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction

from hsrb_testing import setup_world
from semantic_digital_twin.adapters.mesh import STLParser
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

# HSRB specified local world setup
world = setup_world()

hsrb = HSRB.from_world(world)
context = Context(robot=hsrb,world=world)
plan = sequential([ParkArmsAction(arm=Arms.LEFT)], context=context)

with simulated_robot:
    plan.perform()
