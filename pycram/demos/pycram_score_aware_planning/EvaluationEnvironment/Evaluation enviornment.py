import math
import time
from copy import deepcopy

import rclpy

from Evaluate.CompositeEvaluator import CompositeEvaluator
from SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.actions.simulate_perception import simulate_perception
from SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.events.event_handler import EventDispatcher
from ScoreAwareExecutioner import score_aware_execution
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from Stabilizer.PlanStabilizer import PlanStabilizer
from common.cram_types import Task, TaskStep, ActionType, Status
from common.values import CHALLENGE_TASKS, ROOM_SURFACES
from demos.pycram_score_aware_planning.helper_methods import generic_object_spawner
from helper_methods import generate_plan_taskstep_list, \
    NAVIGATION_POSES, at_location, print_base_yaw, challenge_setup
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus, PlanTransformationOperator, Arms
from pycram.motion_executor import simulated_robot

from demos.pycram_score_aware_planning.common.hsrb_testing import setup_world
from demos.pycram_score_aware_planning.common.cram_types import ChallengeMode
from demos.pycram_score_aware_planning.Structurizer.structurizer import PlanStructurizer
from pycram.plans.factories import sequential
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.robots.abstract_robot import Manipulator, AbstractRobot
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.spatial_types import Point3, Quaternion
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.geometry import Color
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation, Body
from EvaluationEnvironment import ScoreAwareExecutioner



def main():
    # HSRB specified local world setup
    world, dispatcher = setup_world()

    hsrb = HSRB.from_world(world)

    context = Context(world=world, robot=hsrb)
    context.evaluate_conditions = False
    dispatcher.known_furniture = world.bodies

    challenge_mode: ChallengeMode = ChallengeMode.PP
    challenge_setup(challenge_mode)

    # Spawning objects
    generic_object_spawner(["Bowl"], [(1.325, 6.23, 0.81)], world, color=Color.GREEN())
    generic_object_spawner(["Plate"], [(0.2, 1, 0.85)], world, color=Color.ORANGE())
    # generic_object_spawner(["Milk"], [(2.42, 0.128, 0.945)], world, color=Color.RED())
    generic_object_spawner(["Milk"], [(1.037, -2.31, 0.645)], world, color=Color.RED())
    generic_object_spawner(["Knife"], [(4.65, 4.84, 1.62)], world, color=Color.CYAN())
    generic_object_spawner(["Apple"], [(4.135, 1.865, 0.54)], world, color=Color.WHITE())
    # generic_object_spawner(["Cereal"], [(1.037, -2.31, 0.645)], world, color=Color.BLUE())
    generic_object_spawner(["Cereal"], [(2.42, 0.128, 0.945)], world, color=Color.BLUE())
    generic_object_spawner(["Cup"], [(2.33475, 5.215, 0.83)], world, color=Color.BEIGE())

    score_aware_execution(dispatcher, context, challenge_mode)

if __name__ == "__main__":
    main()