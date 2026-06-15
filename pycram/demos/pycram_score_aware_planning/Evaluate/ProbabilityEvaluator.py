import math
from copy import deepcopy
from dataclasses import dataclass, field

from helper_methods import NAVIGATION_POSES
from pycram.datastructures.dataclasses import Context
from pycram.locations.costmaps import OccupancyCostmap
from demos.pycram_score_aware_planning.common.types import Task
from demos.pycram_score_aware_planning.common.values import evaluation
from demos.pycram_score_aware_planning.helper_methods import find_surface_of_object

from semantic_digital_twin.reasoning.predicates import compute_euclidean_planar_distance
from semantic_digital_twin.spatial_types import Vector3, HomogeneousTransformationMatrix
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

    # TODO: find perfect distance_max, bcs what even
    def p_robot_distance(self, robot_body: Body, target_body: Body, distance_optimal=2.0, distance_max=5) -> float:
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


    def p_robot_distance(self, robot_body: Body, target_body: Pose, distance_optimal=2.0, distance_max=5) -> float:
        """
        A method used for calculating, that a action from the robot to a specific ose works. In this case especially for the pickup.
        :param robot_body = robot body
        :param target_pose = object pose for distance
        :param distance_optimal = optimal distance
        :param distance_max = max distance
        """
        # TODO: implement robots arm length to consider in optimality AND its dofs: Infos either from pose_validator, robot_predicates
        distance = float(compute_euclidean_planar_distance(body1=robot_body, body2=target_body, ignore_dimension=Vector3.Z()))
        # Full probability up to distance_optimal, sigmoid decay after distance_max
        if distance <= distance_optimal:
            return 1.0
        return 1.0 / (1.0 + math.exp(5 * (distance - distance_max)))

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

    # noinspection D
    def estimate(self, context : Context, task_list: list[Task], probability_threshold: float = 0.2) -> list[Task]:
        """
        Multiplies per-action base probabilities across each task's steps, then
        returns the resulting list sorted descending by expected probability.

        :param task_list: list of tasks
        :return: ExpectedProbabilityModel list sorted best-first.
        """
        world = context.world
        robot = context.robot

        for task in task_list:
            temp_robot = deepcopy(robot)
            # setting probability for evaluation of single task
            joint_probability = 1
            for step in task.task_steps:
                # Base probabilities for later re-assertion.
                step_probability: float = 1
                default_probability : float= 0.95 # reevaluate on known data
                object_annotation: SemanticAnnotation = step.object_annotations
                location: str = step.location
                action_type = step.action_type

                # print(task.task_steps)
                # Retrieving values
                if location not in ("", None):
                    step_probability = evaluation(action_type, object_annotation, location).probability
                    if location in NAVIGATION_POSES:
                        x, y, _ = NAVIGATION_POSES[location]
                        temp_robot.root.global_pose.x = x
                        temp_robot.root.global_pose.y = y

                elif object_annotation not in ("", None):
                    # evaluating the concatination of these actions into a wholeistic task
                    step_probability = evaluation(action_type, object_annotation, location).probability

                    try:
                        object_body : Body= world.get_semantic_annotations_by_type(object_annotation)[0].bodies[0]
                    except:
                        step.probability = step_probability
                        joint_probability = round(joint_probability * step_probability, 2)
                        continue

                    step_probability = step_probability * self.p_robot_distance(target_body=object_body, robot_body=temp_robot.bodies[0])

                    # TODO: add finding close-by objects of all kinds - not just by on the table ornot
                    # Only can find cluttered objects that are near by, if they are on same surface
                    if find_surface_of_object(body=object_body, context=context) is not None:
                        step_probability = step_probability * self.p_clutter_count(target_body=object_body)
                       # self.p_clutter_proximity() TODO: retrieve list of closest objects
                else:
                    step_probability = evaluation(action_type, object_annotation, location).probability

                if step_probability < probability_threshold:
                    step.action_assisted = True
                    step_probability = 1

                # Asserting step values
                step.action_probability = step_probability
                joint_probability = round(joint_probability * step_probability, 2)

            # asserting values
            task.probability = joint_probability

        return task_list
