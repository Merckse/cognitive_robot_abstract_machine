import math
from dataclasses import dataclass

from numpy.ma.core import product

from common.cram_types import TaskStep, ActionType
from helper_methods import get_navigation_poses
from pycram.datastructures.dataclasses import Context
from demos.pycram_score_aware_planning.common.cram_types import Task, Status
from demos.pycram_score_aware_planning.common.values import evaluation
from demos.pycram_score_aware_planning.helper_methods import find_surface_of_object, perceived_instance

from semantic_digital_twin.reasoning.predicates import compute_euclidean_planar_distance
from semantic_digital_twin.spatial_types import Vector3, Point3
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation, Body


@dataclass(kw_only=True)
class RobotProbability:
    """
    Estimates execution success probabilities for robot tasks, factoring in
    per-action base rates, distance to target, and surrounding clutter.
    """

    def p_robot_distance_obj(self, predicted_point: Point3, target_body: Body, distance_optimal=2.0, distance_max=5) -> float:

        # TODO: implement robots arm length to consider in optimality AND its dofs
        target_position = target_body.global_pose.to_position()
        distance = float(predicted_point.euclidean_distance(Point3(target_position.x, target_position.y)))
        # full probability up to distance_optimal, sigmoid decay after distance_max
        if distance <= distance_optimal:
            return 1.0
        return 1.0 / (1.0 + math.exp(5 * (distance - distance_max)))

    def p_robot_distance_nav(self, robot_point: Point3, target_point: Point3, distance_optimal=3.5, distance_max=8.6) -> float:

        distance = float(robot_point.euclidean_distance(target_point))
        # full probability up to distance_optimal, then sigmoid decay
        if distance <= distance_optimal:
            return 1.0
        return 1.0 / (1.0 + math.exp(0.7 * (distance - distance_max)))

    def p_clutter_count(self, target_body: Body, radius=0.4) -> float:

        world = target_body._world
        context = Context.from_world(world)
        surface = find_surface_of_object(body=target_body, context=context)
        nearby = []
        for obj in surface.objects:
            if obj.root != target_body and float(compute_euclidean_planar_distance(body1=obj.root, body2=target_body, ignore_dimension=Vector3.Z())) < radius:
                nearby.append(obj)
        return 1.0 / (1.0 + len(nearby))

    def estimate(self, context : Context, task_list: list[Task], found_objects: dict | None = None,
                 probability_threshold: float = 0.2) -> list[Task]:

        world = context.world
        robot = context.robot
        found = found_objects or {}

        challenge_mode = context.challenge_mode
        for task in task_list:
            robot_position = robot.root.global_pose.to_position()
            predicted_position = (float(robot_position.x), float(robot_position.y))
            joint_probability = 1

            for step in task.task_steps:
                step_probability, predicted_position = self._step_probability(
                    step, predicted_position, context, found, challenge_mode)

                if step_probability < probability_threshold:
                    step.action_assisted = True
                    step_probability = 1

                step.action_probability = step_probability
                joint_probability = round(joint_probability * step_probability, 2)

            task.probability = joint_probability

        return task_list

    def _step_probability(self, step: TaskStep, predicted_position: tuple[float, float],
                          context: Context, found: dict, challenge_mode) -> tuple[float, tuple[float, float]]:

        if step.action_outcome == Status.SUCCESS:
            return 1, predicted_position

        step_probability = (evaluation(step.action_type, step.object_annotations, step.location).probability
                            * (0.5 / step.action_failures))

        step.uncertain = (step.object_annotations is not None
                          and perceived_instance(found, step.object_annotations) is None)
        if step.uncertain:
            return step_probability * evaluation(ActionType.EXPLORE).probability, predicted_position

        step_probability *= self._navigation_term(step, predicted_position, challenge_mode)
        predicted_position = self._position_after(step, predicted_position, challenge_mode)
        step_probability *= self._spatial_term(step, predicted_position, context)
        return step_probability, predicted_position

    def _position_after(self, step: TaskStep, predicted_position: tuple[float, float],
                        challenge_mode) -> tuple[float, float]:
        """Where the robot ends up once this step's navigation has run."""
        navigation_pose = get_navigation_poses(challenge_mode, step.location) if step.location else None
        if navigation_pose is None:
            return predicted_position
        x, y, _ = navigation_pose
        return x, y

    def _navigation_term(self, step: TaskStep, predicted_position: tuple[float, float],
                         challenge_mode) -> float:
        navigation_pose = get_navigation_poses(challenge_mode, step.location) if step.location else None
        if navigation_pose is None:
            return 1.0
        x, y, _ = navigation_pose
        return self.p_robot_distance_nav(target_point=Point3(*predicted_position),
                                         robot_point=Point3(x, y))

    def _spatial_term(self, step: TaskStep, predicted_position: tuple[float, float],
                      context: Context) -> float:
        if step.object_annotations is None:
            return 1.0
        object_body = self._find_body(step.object_annotations, context.world)
        if object_body is None:
            return 1.0

        probability = self.p_robot_distance_obj(target_body=object_body,
                                                predicted_point=Point3(*predicted_position))
        if find_surface_of_object(body=object_body, context=context) is not None:
            probability *= self.p_clutter_count(target_body=object_body)
        return probability

    @staticmethod
    def _find_body(object_annotation: SemanticAnnotation, world) -> Body | None:
        try:
            return world.get_semantic_annotations_by_type(object_annotation)[0].bodies[0]
        except Exception:
            return None

    def get_probability_task_list(self, task_list : list[TaskStep]) -> float:
        return product(t.action_probability for t in task_list)
