import math
from dataclasses import dataclass

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import ApproachDirection
from pycram.locations.costmaps import OccupancyCostmap
from pycram_score_aware_planning.common.types import Task, TaskMode, ExpectedProbabilityModel
from pycram_score_aware_planning.common.values import BASE_PROBABILITY, TASKS
from pycram_score_aware_planning.helper_methods import compute_surface_spaces, objects_on_surface, \
    find_sufrace_of_object
from semantic_digital_twin.reasoning.predicates import compute_euclidean_planar_distance
from semantic_digital_twin.semantic_annotations.mixins import HasSupportingSurface
from semantic_digital_twin.spatial_types import Vector3
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation, Body


# TODO: implement RobotProbability
@dataclass(kw_only=True)
class RobotProbability:
    def estimate(self, task_mode: TaskMode) -> list[ExpectedProbabilityModel]:
        probability = 1
        expected_probability_list = []

        tasks: list[Task] = TASKS.get(task_mode)
        for task in tasks:
            for action in task.task_steps:
                probability = round(
                    probability * BASE_PROBABILITY.get((action.action_type, action.object_name)),2,)
            expected_probability_list.append(
                ExpectedProbabilityModel(
                    task_id=task.task_id, expected_probability=probability
                )
            )
            probability = 1
        # OccupancyCostmap() TODO: implement to see if reachable
        # Number of close objects
        sorted_list = sorted(
            expected_probability_list,
            key=lambda x: x.expected_probability,
            reverse=True,
        )
        return sorted_list

    def p_pose(self):

        pass

    """
    Calculates euclidean distance between robot and object
    :param context = context containing robot
    :param target_body = object for distance
    :param distance_optimal = optimal distance
    :param distance_max = max distance
    """
    def p_robot_distance(self, target_body: Body, distance_optimal=0.8, distance_max=2.5) -> float:
        context = Context.from_world(target_body._world)
        robot_body = context.robot.bodies[0]
        # TODO: implement robots arm length to consider in optimality AND its dofs
        distance = float(compute_euclidean_planar_distance(body1=robot_body, body2=target_body, ignore_dimension=Vector3.Z()))
        # Full probability up to distance_optimal, sigmoid decay after distance_max
        if distance <= distance_optimal:
            return 1.0
        return 1.0 / (1.0 + math.exp(5 * (distance - distance_max)))

    """
    Calculates how many objects are in radius and therefore potential risks for execution
    :param target_body = target body to which the objects should be counted to
    :param surface = the surface on which the object is
    :param radius = the radius in which the objects are expected to be a disruption
    """
    def p_clutter_count(self, target_body: Body, radius=0.4) -> float:
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
        pressure = 1
        for object in nearby_objects:
            pressure =+ 1.0 / max(float(compute_euclidean_planar_distance(object.root, target_body, Vector3.Z())), min_dist) ** 2
        return 1.0 / (1.0 + pressure)  # high pressure → near 0

    # TODO implement basically proximity estimation
    def estimate_pickup(self, object_semantic_annotations : SemanticAnnotation, context : Context) -> float:
        from pycram_score_aware_planning.helper_methods import objects_on_surface
        probability = 1
        world = context.world
        object_name = object_semantic_annotations.name
        object_pose = object_semantic_annotations.bodies[0].global_pose
        robot_pose = context.robot.root.global_pose
        world.get_semantic_annotations_by_type(HasSupportingSurface)

        # TODO: way between object and robot, to have path
        # distance of robot to object
        approach_vec = (object_pose.x - robot_pose.x, object_pose.y - robot_pose.y)
        approach_distance = (abs(approach_vec[0]), abs(approach_vec[1]))

        # unit_vector, ig.
        approach_dir = approach_vec / approach_distance

        # TODO: check if other objects are within this vector
        return probability

    # TODO implement basically proximity estimation
    def estimate_place(self, action, context : Context):
        pass

    def record(self, **kwargs):

        pass

    def evaluate(self, **kwargs):
        pass

    def summary(self, **kwargs):
        pass

    def summary_estimate(self, estimates: list[ExpectedProbabilityModel]) -> None:
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

