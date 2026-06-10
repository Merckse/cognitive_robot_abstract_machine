import math
import time
from dataclasses import dataclass, field
from typing import Optional

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import ApproachDirection
from pycram.locations.costmaps import OccupancyCostmap
from demos.pycram_score_aware_planning.common.types import Task, TaskMode, ExpectedProbabilityModel
from demos.pycram_score_aware_planning.common.values import BASE_PROBABILITY, TASKS
from demos.pycram_score_aware_planning.helper_methods import compute_surface_spaces, objects_on_surface, \
    find_sufrace_of_object

from semantic_digital_twin.reasoning.predicates import compute_euclidean_planar_distance
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.mixins import HasSupportingSurface
from semantic_digital_twin.spatial_types import Vector3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation, Body


# TODO: implement RobotProbability
@dataclass(kw_only=True)
class RobotProbability:
    """
    Estimates execution success probabilities for robot tasks, factoring in
    per-action base rates, distance to target, and surrounding clutter.
    """

    """context containing the world and robot"""

    def p_pose(self):
        pass

    def p_robot_distance(self, robot_body: Body, target_body: Body, distance_optimal=0.8, distance_max=2.5) -> float:
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
        surface = find_sufrace_of_object(body=target_body, context=context)
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

    def evaluate(self, **kwargs):
        pass

    def summary(self, **kwargs):
        pass

    def summary_estimate(self, estimates: list[ExpectedProbabilityModel]) -> None:
        """
        Prints a ranked table of task probability estimates to stdout.

        :param estimates: List of probability estimates, expected best-first.
        """
        sum_score = 0
        sum_time = 0
        sum_score_per_seconds = 0
        sum_tasksteps = 0

        if not estimates:
            print(f"There were no estimates.")
        best = estimates[0]
        lines = [
            f"Estimate",
            "=" * 56,
            f"{'Rank':<6} {'Task ID':<10} {'p Percentage':>8}",
            "-" * 56,
        ]
        for rank, e in enumerate(estimates, 1):
            lines.append(f"{rank:<6} {e.task_id:<10} {e.expected_probability:>8} ")
        print("\n".join(lines))

    def estimate(self, context : Context, task_list: list[Task]) -> list[Task]:
        """
        Multiplies per-action base probabilities across each task's steps, then
        returns the resulting list sorted descending by expected probability.

        :param task_list: list of tasks
        :return: ExpectedProbabilityModel list sorted best-first.
        """
        for task in task_list:
            # setting probability for evaluation of single task
            joint_probability = 1
            for step in task.task_steps:
                # Base probabilities for later re-assertion.
                step_probability: float = 0
                joint_probability: float = 0

                # print(task.task_steps)
                # Retrieving values
                object_name = step.object_name
                action_type = step.action_type
                location = step.location
                world = context.world
                robot = context.robot

                # evaluating the concatination of these actions into a wholeistic task
                step_probability = BASE_PROBABILITY.get((action_type, object_name),
                                                                BASE_PROBABILITY.get((action_type, ""), 0.5))

                # TODO: add p_clutter_proximity
                if object_name not in ("", None):
                    try:
                        object_body = world.get_body_by_name(object_name)
                    except:
                        step.probability = step_probability
                        joint_probability = round(joint_probability * step_probability, 2)
                        continue

                    step_probability = step_probability * self.p_robot_distance(target_body=object_body, robot_body=robot.bodies[0])

                    # TODO: add finding close-by objects of all kinds
                    # Only can find cluttered objects that are near by, if they are on same surface
                    if find_sufrace_of_object(body=object_body, context=context) is not None:
                        step_probability = step_probability *self.p_clutter_count(target_body=object_body)
                       # self.p_clutter_proximity() TODO: retrieve list of closest objects

                # Asserting step values
                step.probability = step_probability
                joint_probability = round(joint_probability * step_probability, 2)

            # asserting values
            task.probability = joint_probability

        # OccupancyCostmap() TODO: implement to see if reachable, MAYBEEEE but runtime is loooong af
        return task_list
