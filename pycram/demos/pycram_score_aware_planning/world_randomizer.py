# HSRB specified local world setup
import rclpy

from Evaluate.CompositeEvaluator import CompositeEvaluator
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from Stabilizer.PlanStabilizer import PlanStabilizer
from Structurizer.structurizer import PlanStructurizer
from common.hsrb_testing import setup_world
from common.types import ChallengeMode, Task
from common.values import CHALLENGE_TASKS
from pycram.datastructures.dataclasses import Context
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.mixins import HasSupportingSurface
from semantic_digital_twin.world_description.world_entity import Body

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

list_supporting_surfaces = world.get_semantic_annotations_by_type(HasSupportingSurface)
print(list_supporting_surfaces)