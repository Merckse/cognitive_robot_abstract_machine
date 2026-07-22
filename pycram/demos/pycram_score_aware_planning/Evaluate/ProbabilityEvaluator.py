import math
from copy import deepcopy
from dataclasses import dataclass, field

from numpy.ma.core import product

from common.cram_types import TaskStep, ActionType
from helper_methods import NAVIGATION_POSES
from pycram.datastructures.dataclasses import Context
from pycram.locations.costmaps import OccupancyCostmap
from demos.pycram_score_aware_planning.common.cram_types import Task, Status
from demos.pycram_score_aware_planning.common.values import evaluation
from demos.pycram_score_aware_planning.helper_methods import find_surface_of_object

from semantic_digital_twin.reasoning.predicates import compute_euclidean_planar_distance
from semantic_digital_twin.spatial_types import Vector3, HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation, Body


@dataclass(kw_only=True)
class RobotProbability:
    """
    Estimates execution success probabilities for robot tasks, factoring in
    per-action base rates, distance to target, and surrounding clutter.
    """

    """context containing the world and robot"""

    def p_pose(self):
        pass

    def p_robot_distance_obj(self, robot_body: Body, target_body: Body, distance_optimal=2.0, distance_max=5) -> float:
        """
        Calculates euclidean distance between robot and object
        :param context = context containing robot
        :param target_body = object for distance
        :param distance_optimal = optimal distance
        :param distance_max = max distance
        """
        # TODO: implement robots arm length to consider in optimality AND its dofs: Infos either from pose_validator, robot_predicates
        distance = float(compute_euclidean_planar_distance(body1=robot_body, body2=target_body, ignore_dimension=Vector3.Z()))
        # Full probability up to distance_optimal, sigmoid decay after distance_max
        if distance <= distance_optimal:
            return 1.0
        return 1.0 / (1.0 + math.exp(5 * (distance - distance_max)))

    def p_robot_distance_nav(self, robot_point: Point3, target_point: Point3, distance_optimal=3.5, distance_max=8.6) -> float:
        """
        Calculates euclidean distance between robot and object
        :param context = context containing robot
        :param target_body = object for distance
        :param distance_optimal = optimal distance
        :param distance_max = max distance
        """
        # TODO: implement robots arm length to consider in optimality AND its dofs: Infos either from pose_validator, robot_predicates
        distance = float(robot_point.euclidean_distance(target_point))
        # Full probability up to distance_optimal, then sigmoid decay: 50% at distance_max (0.6 at 8 m)
        if distance <= distance_optimal:
            return 1.0
        return 1.0 / (1.0 + math.exp(0.7 * (distance - distance_max)))

    def p_clutter_count(self, target_body: Body, radius=0.4) -> float:
        """
        Calculates how many objects are in radius and therefore potential risks for execution
        :param target_body = target body to which the objects should be counted to
        :param surface = the surface on which the object is
        :param radius = the radius in which the objects are expected to be a disruption
        """
        # TODO: implement way to retrieve surface of object, by itself
        world = target_body._world
        context = Context.from_world(world)
        surface = find_surface_of_object(body=target_body, context=context)
        nearby = []
        for obj in surface.objects:
            if obj.root != target_body and float(compute_euclidean_planar_distance(body1=obj.root, body2=target_body,ignore_dimension=Vector3.Z())) < radius:
                nearby.append(obj)
        n = len(nearby)
        return 1.0 / (1.0 + n)  # 0 objects - 1.0,  3 objects - 0.25

    def p_clutter_proximity(target_body: Body, nearby_objects: list,
                            min_dist=0.05) -> float:
        """
        Computes an inverse-square pressure score from nearby objects and maps
        it to a probability. High spatial pressure (many close objects) drives
        the value toward 0; no nearby objects returns 1.0.

        :param target_body: The body to measure proximity to.
        :param nearby_objects: Objects considered potential obstructions.
        :param min_dist: Distance floor to prevent division by zero.
        :return: Probability in (0, 1]; 1.0 means no pressure.
        """
        pressure = 1
        for object in nearby_objects:
            pressure =+ 1.0 / max(float(compute_euclidean_planar_distance(object.root, target_body, Vector3.Z())), min_dist) ** 2
        return 1.0 / (1.0 + pressure)  # high pressure → near 0

    def record(self, **kwargs):
        pass

    def estimate(self, context : Context, task_list: list[Task], found_objects: dict | None = None,
                 probability_threshold: float = 0.2) -> list[Task]:
        """
        Multiplies per-action base probabilities across each task's steps and stores the joint
        probability on every task.

        Uncertainty is driven by *belief* (`found_objects`), not the ground-truth world: a step
        whose object has not been perceived anywhere yet is charged an EXPLORE -- its probability
        is discounted (multiplied) by the explore find-probability, and the step is flagged
        `uncertain` so the score/time side can charge the search cost. Spatial penalties (distance,
        clutter) are only folded in once the object is actually known.

        :param context: world + robot context.
        :param task_list: tasks to evaluate.
        :param found_objects: perceived-object belief (annotation instance -> location). Empty/None = nothing perceived yet.
        :param probability_threshold: below this, the step is marked assisted and reset to 1.
        :return: the same tasks with probability fields filled in.
        """
        world = context.world
        robot = context.robot
        found = found_objects or {}

        for task in task_list:
            temp_robot = deepcopy(robot)
            joint_probability = 1
            for step in task.task_steps:
                object_annotation: SemanticAnnotation = step.object_annotations
                location: str = step.location
                action_type = step.action_type

                if step.action_outcome == Status.SUCCESS:
                    step_probability = 1
                else:
                    step_probability = evaluation(action_type, object_annotation, location).probability * (0.5/step.action_failures)

                    # belief-driven uncertainty: object not perceived anywhere yet -> charge an EXPLORE
                    step.uncertain = (object_annotation is not None
                                      and not any(isinstance(seen, object_annotation) for seen in found))
                    if step.uncertain:
                        # discount by the explore find-probability (multiplicative, not additive)
                        step_probability *= evaluation(ActionType.EXPLORE).probability
                    else:
                        # location known -> place the robot at the approach pose so distance is measured from there
                        if location in NAVIGATION_POSES:
                            x, y, _ = NAVIGATION_POSES[location]
                            goal = Point3(x,y)

                            step_probability *= self.p_robot_distance_nav(target_point=temp_robot.root.global_pose.to_position(),
                                                                          robot_point=goal)
                            temp_robot.root.global_pose.x = x
                            temp_robot.root.global_pose.y = y

                        # fold in spatial penalties (distance, clutter) only when we can actually locate the body
                        if object_annotation is not None:
                            try:
                                object_body: Body | None = world.get_semantic_annotations_by_type(object_annotation)[0].bodies[0]
                            except Exception:
                                object_body = None
                            if object_body is not None:
                                step_probability *= self.p_robot_distance_obj(target_body=object_body, robot_body=temp_robot.bodies[0])
                                # clutter only matters for objects sharing a surface
                                if find_surface_of_object(body=object_body, context=context) is not None:
                                    step_probability *= self.p_clutter_count(target_body=object_body)

                if step_probability < probability_threshold:
                    step.action_assisted = True
                    step_probability = 1

                step.action_probability = step_probability
                joint_probability = round(joint_probability * step_probability, 2)

            task.probability = joint_probability

        return task_list

    # --- old_estimate kept for reference; superseded by the belief-driven estimate above ---
    # def old_estimate(self, context: Context, task_list: list[Task], probability_threshold: float = 0.2) -> list[Task]:
    #     world = context.world
    #     robot = context.robot
    #     for task in task_list:
    #         temp_robot = deepcopy(robot)
    #         joint_probability = 1
    #         for step in task.task_steps:
    #             step_probability: float = 1
    #             object_annotation = step.object_annotations
    #             location = step.location
    #             action_type = step.action_type
    #             if step.action_outcome == Status.SUCCESS:
    #                 step_probability = 1
    #             elif location not in ("", None):
    #                 step_probability = evaluation(action_type, object_annotation, location).probability
    #                 if location in NAVIGATION_POSES:
    #                     x, y, _ = NAVIGATION_POSES[location]
    #                     temp_robot.root.global_pose.x = x
    #                     temp_robot.root.global_pose.y = y
    #             elif object_annotation not in ("", None):
    #                 step_probability = evaluation(action_type, object_annotation, location).probability
    #                 # spatial penalties / explore fallback (object body lookup against ground-truth world)
    #                 try:
    #                     object_body = world.get_semantic_annotations_by_type(object_annotation)[0].bodies[0]
    #                 except Exception:
    #                     object_body = None
    #                     step.uncertain = True
    #                     evaluation_explore = evaluation(ActionType.EXPLORE)
    #                 if object_body is None:
    #                     step.probability = step_probability + evaluation_explore.probability
    #                     joint_probability = round(joint_probability * step_probability, 2)
    #                     continue
    #                 step_probability *= self.p_robot_distance(target_body=object_body, robot_body=temp_robot.bodies[0])
    #                 if find_surface_of_object(body=object_body, context=context) is not None:
    #                     step_probability *= self.p_clutter_count(target_body=object_body)
    #             else:
    #                 step_probability = evaluation(action_type, object_annotation, location).probability
    #             if step_probability < probability_threshold:
    #                 step.action_assisted = True
    #                 step_probability = 1
    #             step.action_probability = step_probability
    #             joint_probability = round(joint_probability * step_probability, 2)
    #         task.probability = joint_probability
    #     return task_list

    def get_probability_task_list(self, task_list : list[TaskStep]) -> float:
        return product(t.action_probability for t in task_list)