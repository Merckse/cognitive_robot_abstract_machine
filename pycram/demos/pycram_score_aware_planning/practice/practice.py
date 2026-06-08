from pycram.datastructures.dataclasses import Context
from pycram.motion_executor import simulated_robot
from pycram_score_aware_planning.common.hsrb_testing import setup_world
from pycram_score_aware_planning.common.types import TaskStep, ActionType, Task
from pycram_score_aware_planning.helper_methods import generic_object_spawner, compute_surface_spaces, generate_plan
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.mixins import IsPerceivable
from semantic_digital_twin.semantic_annotations.semantic_annotations import Bowl
from semantic_digital_twin.world_description.geometry import Color

world, dispatcher = setup_world()
hsrb = HSRB.from_world(world)
context = Context(world=world, robot=hsrb)
context.evaluate_conditions = False
dispatcher.known_furniture = world.bodies

generic_object_spawner(["Bowl"], [(1.325, 5.99, 0.80)], world, color=Color.GREEN())
generic_object_spawner(["Plate"], [(1.2, 5.75, 0.80)], world, color=Color.ORANGE())


with simulated_robot:
    generate_plan([Task(id=1, task_steps=[TaskStep(ActionType.NAVIGATE, location="cooking_table")])], context=context).perform()
"""
SurfaceSpaces ->
"""
object_semantic_annotations = world.get_semantic_annotations_by_type(Bowl)[0]
object_name = object_semantic_annotations.name
object_pose = object_semantic_annotations.bodies[0].global_pose
robot = context.robot
robot_pose = robot.root.global_pose
print()
print(f"object_pose : {object_pose}")
print(f"robot_pose : {robot_pose}")

import numpy as np

approach_vec = np.array([float(object_pose.x - robot_pose.x), float(object_pose.y - robot_pose.y)])  # raw vector from robot to target (direction + magnitude)
approach_dist = np.linalg.norm(approach_vec)  # straight-line distance robot → target in metres: √(x² + y²)
approach_dir = approach_vec / approach_dist   # unit vector: same direction, length = 1, so dot products give metres

# Determining the target service / on what surface the object is resting on
target_surface = None
for surface in compute_surface_spaces(world):
    if surface.x_min <= object_pose.x <= surface.x_max and surface.y_min <= object_pose.y <= surface.y_max:
        target_surface = surface
        break

gripper_half_width = 0.05  # half of HSRB open gripper width (~10 cm)
robot_pos = np.array([float(robot_pose.x), float(robot_pose.y)])
obj_pos   = np.array([float(object_pose.x), float(object_pose.y)])

print(f"target_surface: {target_surface}")
print(f"robot_pos: {robot_pos}, obj_pos: {obj_pos}, approach_dist: {approach_dist:.3f} m")

blocked = False
for ann in world.get_semantic_annotations_by_type(IsPerceivable):
    if ann is object_semantic_annotations:  # skip target
        continue

    pose = ann.root.global_pose
    ax, ay = float(pose.x), float(pose.y)
    print(f"  candidate {ann.name}: pos=({ax:.3f}, {ay:.3f})")

    if not (target_surface.x_min <= ax <= target_surface.x_max and
            target_surface.y_min <= ay <= target_surface.y_max):
        continue

    # object footprint radius from bounding box, so partial overlap into corridor counts
    try:
        bbs = ann.as_bounding_box_collection_in_frame(world.root).bounding_boxes
        if bbs:
            obj_half_x = (max(bb.x_interval.upper for bb in bbs) - min(bb.x_interval.lower for bb in bbs)) / 2
            obj_half_y = (max(bb.y_interval.upper for bb in bbs) - min(bb.y_interval.lower for bb in bbs)) / 2
            obj_radius = float(max(obj_half_x, obj_half_y))
        else:
            obj_radius = 0.1
    except Exception:
        obj_radius = 0.1

    other_pos  = np.array([ax, ay])
    to_other   = other_pos - robot_pos
    projection = np.dot(to_other, approach_dir)
    lateral    = np.linalg.norm(to_other - projection * approach_dir)
    effective_width = gripper_half_width + obj_radius
    print(f"    projection={projection:.3f} (need 0–{approach_dist:.3f}), lateral={lateral:.3f} (need <{effective_width:.3f}), obj_radius={obj_radius:.3f}")

    if 0 < projection < approach_dist and lateral < effective_width:
        print(f"BLOCKED by: {ann.name}")
        blocked = True
        break

print(f"Blocked: {blocked}")


print(f"approach_vec: {approach_vec}")
print(f"approach_dir: {approach_dir}")
