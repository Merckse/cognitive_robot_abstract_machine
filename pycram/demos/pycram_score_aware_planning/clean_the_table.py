from demos.pycram_score_aware_planning.eval import RobotScorer, ActionType, ActionOutcome
from demos.pycram_score_aware_planning.helper_methods import generic_object_spawner
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from pycram.datastructures.grasp import GraspDescription

from pycram.motion_executor import simulated_robot
from pycram.plans.factories import sequential
from pycram.robot_plans.actions.core.navigation import NavigateAction
from pycram.robot_plans.actions.core.pick_up import PickUpAction
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction

from hsrb_testing import setup_world
from semantic_digital_twin.reasoning.predicates import is_supported_by
from semantic_digital_twin.reasoning.queries import semantic_annotations_on_surfaces
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.mixins import HasSupportingSurface
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.spatial_types import Point3, Quaternion
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.geometry import Color

"""
PROBABILITY:
    [ ] - Recognize the task space, what tasks are certainly calculateable under given taks-spaces
    [ ] - What are base probabilities
    [ ] - analyse the contexts, like object closeness and misc.
    [ ] - calculate probability from given positions (Orientation, Position)
[ ] - 
[ ] - 

"""


# HSRB specified local world setup
world = setup_world()

hsrb = HSRB.from_world(world)
context = Context(robot=hsrb, world=world, evaluate_conditions=False)

generic_object_spawner(["Bowl"], [(1.325, 5.99, 0.81)], world, color=Color.GREEN())
generic_object_spawner(["bowl3"], [(4,2,4)], world, color=Color.GREEN())
bowl =  world.get_semantic_annotation_by_name("bowl3").bodies[0]

# print(world.get_semantic_annotation_by_name("cup").bodies[1])
manipulator = hsrb.arm.manipulator
plan_parkarm = sequential([ParkArmsAction(Arms.LEFT)], context=context)
robot_scoorer = RobotScorer()

tables = world.get_semantic_annotations_by_type(HasSupportingSurface)[0]
target_table_body = world.get_semantic_annotations_by_type(Table)[3].bodies[0]
target_table_annotations = world.get_semantic_annotations_by_type(Table)[3]

objects = semantic_annotations_on_surfaces(supporting_surfaces=[target_table_annotations], world=world)
print(objects)
pose_table = target_table_body.global_pose.to_homogeneous_matrix().to_pose()
pose_table.reference_frame = world.root
pose_table.z = 0

print(pose_table.x, pose_table.y)
# Pre-table pose: 1 m in front of the table (negative x offset) at floor level
pre_table_pose = Pose(
    position=Point3(pose_table.x , pose_table.y- 1.0, 0.0),
    orientation=Quaternion(0, 0, 1, 1),
    reference_frame=world.root,
)

navigate_to_pre_table = sequential([NavigateAction(pre_table_pose)], context=context)

with simulated_robot:
    navigate_to_pre_table.perform()