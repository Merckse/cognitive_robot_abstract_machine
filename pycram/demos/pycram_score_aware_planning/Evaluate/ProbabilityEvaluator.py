from dataclasses import dataclass

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import ApproachDirection
from pycram_score_aware_planning.common.types import Task, TaskMode, ExpectedProbabilityModel
from pycram_score_aware_planning.common.values import BASE_PROBABILITY, TASKS
from pycram_score_aware_planning.helper_methods import compute_surface_spaces, objects_on_surface
from semantic_digital_twin.semantic_annotations.mixins import HasSupportingSurface
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation

# TODO: implement RobotProbability
@dataclass(kw_only=True)
class RobotProbability():

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
        sorted_list = sorted(
            expected_probability_list,
            key=lambda x: x.expected_probability,
            reverse=True,
        )
        return sorted_list

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

