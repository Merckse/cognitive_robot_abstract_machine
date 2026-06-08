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

    context : Context
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

    def estimate(self, task_list: list[Task]) -> list[Task]:
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
                # print(task.task_steps)
                object_name = step.object_name
                action_type = step.action_type
                location = step.location

                # evaluating the concatination of these actions into a wholeistic task
                step_probability: float = BASE_PROBABILITY.get((action_type, object_name),
                                                                BASE_PROBABILITY.get((action_type, ""), 0.5))
                world = self.context.world
                robot = self.context.robot

                # TODO: add calculations here, like p_robot_distance, p_clutter_count, p_clutter_proximity
                if object_name is not "" or None:
                    object_body = world.get_body_by_name(object_name)
                    step_probability = step_probability * self.p_robot_distance(target_body=object_body, robot_body=robot.bodies[0])
                    step_probability = step_probability *self.p_clutter_count(target_body=object_body)
                    # self.p_clutter_proximity() TODO: retrieve list of closest objects
                step.probability = step_probability
                joint_probability: float = round(joint_probability * step_probability, 2)

            task.probability = joint_probability
            # creating a list of all tasks and having one probability of success for the whole task.

        # OccupancyCostmap() TODO: implement to see if reachable, MAYBEEEE
        return task_list


if __name__ == "__main__":
    from unittest.mock import MagicMock

    mock_context = MagicMock(spec=Context)
    mock_context.world = MagicMock(spec=World)
    mock_context.robot = MagicMock(spec=HSRB)

    mock_context.world.get_body_by_name.return_value = MagicMock()
    mock_context.robot.bodies = [MagicMock()]

    estimator = RobotProbability(context=mock_context)

    for mode in TaskMode:
        print(f"\n--- TaskMode: {mode.value.upper()} ---")
        result = estimator.estimate(mode)
        estimator.summary_estimate(result)

#     # TODO implement basically proximity estimation
#     def estimate_pickup(self, object_semantic_annotations : SemanticAnnotation, context : Context) -> float:
#         """
#         Estimates the probability of a successful pickup based on robot-to-object
#         approach vector and surrounding surface occupancy.
#
#         :param object_semantic_annotations: The target object with pose and body data.
#         :param context: Current world context providing robot pose.
#         :return: Estimated pickup probability in [0, 1].
#         """
#         from demos.pycram_score_aware_planning.helper_methods import objects_on_surface
#         probability = 1
#         world = context.world
#         object_name = object_semantic_annotations.name
#         object_pose = object_semantic_annotations.bodies[0].global_pose
#         robot_pose = context.robot.root.global_pose
#         world.get_semantic_annotations_by_type(HasSupportingSurface)
#
#         # TODO: way between object and robot, to have path
#         # distance of robot to object
#         approach_vec = (object_pose.x - robot_pose.x, object_pose.y - robot_pose.y)
#         approach_distance = (abs(approach_vec[0]), abs(approach_vec[1]))
#
#         # unit_vector, ig.
#         approach_dir = approach_vec / approach_distance
#
#         # TODO: check if other objects are within this vector
#         return probability
#
#     # TODO implement basically proximity estimation
#     def estimate_place(self, action, context : Context):
#         pass