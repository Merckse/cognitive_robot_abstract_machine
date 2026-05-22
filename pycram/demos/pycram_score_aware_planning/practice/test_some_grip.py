from pycram.datastructures.dataclasses import Context
from pycram_score_aware_planning.common.hsrb_testing import setup_world
from pycram_score_aware_planning.helper_methods import generic_object_spawner
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Bowl
from semantic_digital_twin.world_description.geometry import Color

world, dispatcher = setup_world()
hsrb = HSRB.from_world(world)
context = Context(world=world, robot=hsrb)
context.evaluate_conditions = False
dispatcher.known_furniture = world.bodies

generic_object_spawner(["Bowl"], [(1.325, 5.99, 0.81)], world, color=Color.GREEN())




"""
SurfaceSpaces ->
"""
object_semantic_annotations = world.get_semantic_annotations_by_type(Bowl)[0]
object_name = object_semantic_annotations.name
object_pose = object_semantic_annotations.bodies[0].global_pose
robot = context.robot
robot_pose = robot.root.global_pose
gripper_width = robot.manipulators[0].tool_frame
print(gripper_width)