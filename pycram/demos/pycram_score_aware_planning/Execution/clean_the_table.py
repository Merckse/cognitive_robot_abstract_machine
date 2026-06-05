import math

from Evaluate.CompositeEvaluator import CompositeEvaluator
from common.types import Task
from common.values import TASKS
from demos.pycram_score_aware_planning.helper_methods import generic_object_spawner
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram.motion_executor import simulated_robot

from pycram.plans.factories import sequential
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction

from demos.pycram_score_aware_planning.common.hsrb_testing import setup_world
from demos.pycram_score_aware_planning.common.types import TaskMode
from demos.pycram_score_aware_planning.Structurizer.structurizer import PlanStructurizer
from demos.pycram_score_aware_planning.helper_methods import generate_plan
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.mixins import HasSupportingSurface
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.spatial_types import Point3, Quaternion
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.geometry import Color

"""
# ------------- TODO ------------- #
TASK completion under:
- [] Go to tables one by one after input
- [] Evaluate action based on object closeness

- [] Plan stability evaluation - basic

- Query where object is and go to closest location

    CERTAINTY:
    [] - object detection
    [] - Task rescheduling
    [] - post-iori changes
    [] - Navigation time estimation
    [] - sorting by tasks per room (I do have n tasks in room x, therefor I should go there)
    
    UNCERTAINTY:
    [] - Task space based task completion (e.g. I know there will be milk in the kitchen, but will there be x in the kitchen)

DOs:
    [] - navigation duration estimate for spm
    [] - object closeness for probability

EXECUTION:
    [] - outsourcing the execution to the Executionor
PROBABILITY:
    [ ] - Recognize the task space, what tasks are certainly calculateable under given taks-spaces
    [ ] - What are base probabilities
    [ ] - analyse the contexts, like object closeness and misc.
    [ ] - calculate probability from given positions (Orientation, Position)
[ ] - 
[ ] - 


# ------------- PARAMETER CONSTRAINTS ------------- #
\item object distances
\item object arrangement
\item object availability
\item pre-conditioned tasks

# ------------- to implement today ------------- #
 [] - Distance navigation time
 [] - estimated time for action based on constraints
 [] - calculating taken time and time left
 [] - List of how the tables are explored
 [] - check for failure after action?
 [] - Base checks for possibility of action (height, misc.)
"""


# HSRB specified local world setup
world, dispatcher = setup_world()

hsrb = HSRB.from_world(world)
context = Context(world=world, robot=hsrb)
task_mode = TaskMode.PP
context.evaluate_conditions = False
dispatcher.known_furniture = world.bodies

# COOKING TABLE [X]
# DINING_TABLE [X]
# table [X]
# lowerTable [X]
# desk [X]
# SHELF_1 [x]
# SHELF_2 [X]
explorable_locations = ["cooking_table","counterTop", "dining_table", "table", "lowerTable", "desk", "shelf_1", "shelf_2"]
kitchen_tables = ["table", "counterTop"]
livingroom_tables = ["lowerTable", "dining_table", "shelf_1", "shelf_2"]
office_tables = ["desk"]
some_room = ["cooking_table"]

generic_object_spawner(["Bowl"], [(1.325, 5.99, 0.81)], world, color=Color.GREEN())
generic_object_spawner(["Plate"], [(-0.15,0.88,0.85)], world, color=Color.ORANGE())
generic_object_spawner(["Milk"], [(1.037,-2.31,0.645)], world, color=Color.RED())
generic_object_spawner(["Knife"], [(4.65,4.84,1.62)], world, color=Color.GREEN())
generic_object_spawner(["Apple"], [(4.135,1.865,0.54)], world, color=Color.GREEN())
generic_object_spawner(["Cereal"], [(2.42,0.128,0.945)], world, color=Color.GREEN())
generic_object_spawner(["Cup"], [(2.33475,5.215,0.83)], world, color=Color.GREEN())

manipulator = hsrb.arm.manipulator
plan_parkarm = sequential([ParkArmsAction(Arms.LEFT)], context=context)
support = world.get_semantic_annotations_by_type(HasSupportingSurface)
print(support)

# tables = world.get_semantic_annotations_by_type(HasSupportingSurface)[0]
target_table_body = world.get_semantic_annotations_by_type(Table)[3].bodies[0]
# target_table_annotations = world.get_semantic_annotations_by_type(Table)[3]

# objects = semantic_annotations_on_surfaces(supporting_surfaces=[target_table_annotations], world=world)
pose_table = target_table_body.global_pose.to_homogeneous_matrix().to_pose()
pose_table.reference_frame = world.root
pose_table.z = 0

# Pre-table pose: 1 m in front of the table (negative x offset) at floor level
pre_table_pose = Pose(
    position=Point3(pose_table.x, pose_table.y - 1.0, 0.0),
    orientation=Quaternion(
        0.0, 0.0, math.sin((math.pi / 2) / 2), math.cos((math.pi / 2) / 2)
    ),
    reference_frame=world.root,
)
"""
Generate a structurized plan - based on standard evaluation 
"""
task_list : list[Task] = TASKS.get()
task_evaluator = CompositeEvaluator(context=context)
task_structurizer = PlanStructurizer()
task_evaluator.estimate()
task_plan = task_structurizer.structurize()
plan = generate_plan(tasks=task_plan, context=context)
print(plan)
"""
Generate the actual plan
# """

# Have to check before some long taking action
with simulated_robot:
    print(plan.plan)
    plan.perform()


# if visible_bodies is None:
#     visible_bodies = []
# for visible_body in visible_bodies:
#     if visible_body not in dispatcher.known_furniture:
#         perceived_objects.append(visible_body)
#
# print(perceived_objects)
