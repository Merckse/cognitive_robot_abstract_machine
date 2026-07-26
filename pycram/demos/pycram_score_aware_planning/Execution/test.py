import math
import time

import rclpy

from Evaluate.CompositeEvaluator import CompositeEvaluator
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from Stabilizer.PlanStabilizer import PlanStabilizer
from common.cram_types import Task, TaskStep, ActionType, Status
from common.values import CHALLENGE_TASKS, ROOM_SURFACES
from helper_methods import generic_object_spawner
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus, PlanTransformationOperator, Arms, ChallengeMode
from pycram.motion_executor import simulated_robot

from demos.pycram_score_aware_planning.common.hsrb_testing import setup_world
from demos.pycram_score_aware_planning.Structurizer.structurizer import PlanStructurizer
from pycram.plans.factories import sequential
from pycram.robot_plans.actions.core.container import OpenAction, CloseAction
from semantic_digital_twin.reasoning.queries import goal_surface_of_object
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.world_description.geometry import Color
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation, Body



# HSRB specified local world setup
rclpy.init()
world, dispatcher = setup_world()
evaluator = CompositeEvaluator()
structurizer = PlanStructurizer()
stabilizer = PlanStabilizer()
scoretime_monitor = ScoreTimeMonitor()

hsrb = HSRB.from_world(world)

explored_locations = []
found_objects: dict[Body, str] = {}

context = Context(world=world, robot=hsrb)
context.evaluate_conditions = False
dispatcher.known_furniture = world.bodies

challenge_mode: ChallengeMode = ChallengeMode.PP
task_list: list[Task] = CHALLENGE_TASKS.get(challenge_mode)


with simulated_robot:
    door = world.get_body_by_name("cupboard_left_door")
    # sequential([OpenAction(assisted=True, arm=Arms.LEFT, object_designator=door)],context).perform()
    #
    # door = world.get_body_by_name("dishwasher_rack")
    # sequential([OpenAction(assisted=True, arm=Arms.LEFT, object_designator=door)],context).perform()
    #
    #
    # door = world.get_body_by_name("dishwasher_rack")
    # sequential([CloseAction(assisted=True, arm=Arms.LEFT, object_designator=door)],context).perform()

    generic_object_spawner(["Bowl"], [(1.325, 6.23, 0.81)], world, color=Color.GREEN())
    generic_object_spawner(["Plate"], [(0.2, 1, 0.85)], world, color=Color.ORANGE())
    # generic_object_spawner(["Milk"], [(2.42, 0.128, 0.945)], world, color=Color.RED())
    generic_object_spawner(["Milk"], [(2.2, -2.11, 0.645)], world, color=Color.RED())



    generic_object_spawner(["Cereal"], [(4.65, 4.84, 1.62)], world, color=Color.CYAN())
    generic_object_spawner(["Apple"], [(4.135, 1.865, 0.54)], world, color=Color.WHITE())
    # generic_object_spawner(["Cereal"], [(1.037, -2.31, 0.645)], world, color=Color.BLUE())
    generic_object_spawner(["Knife"], [(2.42, 0.128, 0.945)], world, color=Color.BLUE())
    generic_object_spawner(["Cup"], [(2.33475, 5.215, 0.83)], world, color=Color.BEIGE())

    milk_anno = world.get_semantic_annotation_by_name("milk")
    cupboard = world.get_semantic_annotation_by_name("cupboard")

    target_layer = goal_surface_of_object(milk_anno, cupboard.shelf_layer)
    print(f"milk should go to: {target_layer.name if target_layer else 'no shelf found'}")