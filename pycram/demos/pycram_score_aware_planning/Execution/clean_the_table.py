import math

from Evaluate.CompositeEvaluator import CompositeEvaluator
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from Stabilizer.PlanStabilizer import PlanStabilizer
from common.cram_types import Task, TaskStep, Status
from common.values import CHALLENGE_TASKS
from demos.pycram_score_aware_planning.helper_methods import generic_object_spawner
from helper_methods import generate_plan_task, perceive_and_spawn_all_objects
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, TaskStatus
from pycram.motion_executor import simulated_robot

from pycram.plans.factories import sequential, make_node
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction

from demos.pycram_score_aware_planning.common.hsrb_testing import setup_world
from demos.pycram_score_aware_planning.common.cram_types import ChallengeMode
from demos.pycram_score_aware_planning.Structurizer.structurizer import PlanStructurizer
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
task_mode = ChallengeMode.PP
context.evaluate_conditions = False
dispatcher.known_furniture = world.bodies

explorable_locations = ["cooking_table","counterTop", "dining_table", "table", "lowerTable", "desk", "shelf_1", "shelf_2"]
kitchen_tables = ["table", "counterTop"]
livingroom_tables = ["lowerTable", "dining_table", "shelf_1", "shelf_2"]
office_tables = ["desk"]
some_room = ["cooking_table"]

generic_object_spawner(["Bowl"], [(1.325, 6.23, 0.81)], world, color=Color.GREEN())
generic_object_spawner(["Plate"], [(-0.15,0.88,0.85)], world, color=Color.ORANGE())
generic_object_spawner(["Milk"], [(1.037,-2.31,0.645)], world, color=Color.RED())
generic_object_spawner(["Knife"], [(4.65,4.84,1.62)], world, color=Color.CYAN())
generic_object_spawner(["Apple"], [(4.135,1.865,0.54)], world, color=Color.WHITE())
generic_object_spawner(["Cereal"], [(2.42,0.128,0.945)], world, color=Color.BLUE())
generic_object_spawner(["Cup"], [(2.33475,5.215,0.83)], world, color=Color.BEIGE())

manipulator = hsrb.arm.manipulator
plan_parkarm = sequential([ParkArmsAction(Arms.LEFT)], context=context)
support = world.get_semantic_annotations_by_type(HasSupportingSurface)

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
challengemode = ChallengeMode.PP
task_list : list[Task] = CHALLENGE_TASKS.get(challengemode)

evaluator = CompositeEvaluator()
structurizer = PlanStructurizer()
stabilizer = PlanStabilizer()
scoretime_monitor = ScoreTimeMonitor(task_mode)


while task_list != []:
    evaluated_tasks: list[Task] = evaluator.estimate(context=context, task_list=task_list)
    task_list: list[Task] = structurizer.structurize(task_list=evaluated_tasks)

    current_task = task_list[0]
    task_list.pop(0)

    plan = generate_plan_task(task=current_task, context=context)

    # Have to check before some long taking action
    # TODO: what happens, if the score event fails and the robot has to fallback, how to re-evaluated / stabilize
    # TODO: maybe add a on the fly monitor, that basically checks a action in the task_list as done, resulting in instant feedback, where to implement that?
    with simulated_robot:
        # for i in range(len(current_task.action_list)):
        #     task_step : TaskStep = current_task.task_steps[i]
        #     action = current_task.action_list[i]

            # if task_step.uncertain:
            #     try:
            #         perceive_and_spawn_all_objects(world=world)
            #     except:
            #         pass

            # plan = sequential([], context=context)
            # plan.add_child(make_node(action))


        plan.perform()
        if plan.status == TaskStatus.INTERRUPTED or plan.status == TaskStatus.FAILED:
            stabilizer.stabilize(plan, current_task, plan.reason, scoretime_monitor)

            # TODO: reimplement scoretime_monitor
            # scoretime_monitor.record_score(task_step=task_step, plan=plan)
            # if plan.status == TaskStatus.SUCCEEDED:
            #     if task_step.action_assisted:
            #         task_step.action_outcome = Status.SUCCESS_WITH_ASSIST
            #         continue
            #     task_step.action_outcome = Status.SUCCESS


            # if plan.status is TaskStatus.FAILED:
            #     plan = stabilizer.stabilize(plan)
