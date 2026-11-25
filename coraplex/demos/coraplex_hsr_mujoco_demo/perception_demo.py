"""
The real HSR looks at what is in front of it, asks RoboKudo which objects it sees and
adds each of them to the world with its semantic annotation.

Run it from this folder, with the robot's controller and a RoboKudo pipeline that
answers object queries (such as ``live_yolo``) running::

    python perception_demo.py
"""

from __future__ import annotations

from dataclasses import dataclass, field

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import RealRobotPlanExecutor
from coraplex.robot_plans.actions.core.robot_body import (
    MoveTorsoAction,
    ParkArmsAction,
)
from coraplex.ros_scene_perception import RoboKudoScenePerception
from coraplex.scene_perception import SceneObservation, SceneQuery
from cramph.composites import Sequence
from cramph.statechart import Statechart
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.mixins import IsPerceivable
from semantic_digital_twin.semantic_annotations.semantic_annotations import CounterTop
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from coraplex.plans.executors import PlanExecutor
    from cramph.context import ContextExtension, StatechartContext
    from semantic_digital_twin.robots.robot_parts import AbstractRobot
    from semantic_digital_twin.world import World

# %% the demonstration


@dataclass
class PerceptionDemonstration(RobotDemonstration):
    """
    The real HSR gets ready to look, then adds every object RoboKudo sees to the world
    as a body with its semantic annotation, at the pose and with the size it was seen.

    A demonstration that acts on the seen objects extends :meth:`build_statechart` to
    get ready, or :meth:`perform_plan` to act after the objects were added.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The robot that looks.
    """

    ros_node_name: str = "hsr_perception_demo"
    """
    Name of the node this demonstration registers.
    """

    executor_type: type[PlanExecutor] = RealRobotPlanExecutor
    """
    The executor running the plan, which drives the real robot.
    """

    map_T_scene: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.387, y=0.040, yaw=-0.0594
        )
    )
    """
    Where the kitchen lies in the map frame of the laser map ``apartment_master1``, as
    :class:`MilkPickUpDemonstration` of ``temp.py`` places it.
    """

    observation: SceneObservation | None = field(init=False, default=None)
    """
    What RoboKudo saw in the last run, None before the first run.
    """

    spawned_objects: list[IsPerceivable] = field(init=False, default_factory=list)
    """
    The annotations of the objects the last run added to the world.
    """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(world_source=KitchenEnvironment())

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(semantic_annotation_type=self.used_robot)

    def is_scene_populated(self, world: World) -> bool:
        return bool(world.get_semantic_annotations_by_type(CounterTop))

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        :return: The statechart that parks the arms out of the camera's view and raises
            the torso, so the camera looks down on the furniture in front of the robot.
        """
        robot = context.require_extension(RobotAccess).robot
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=robot.all_arms),
                    MoveTorsoAction(torso_state=TorsoState.LOW),
                ]
            )
        )
        return statechart

    def perform_plan(self, world: World) -> None:
        """
        As :meth:`RobotDemonstration.perform_plan`, then ask RoboKudo for the objects in
        view and add them to ``world``, see :meth:`SceneObservation.spawn_objects`.
        """
        super().perform_plan(world)
        self.observation = self.observe_scene(world)
        self.spawned_objects = self.observation.spawn_objects(world)

    def observe_scene(self, world: World) -> SceneObservation:
        """
        :param world: The world the robot looks at.
        :return: The objects RoboKudo sees in view, as they are added to ``world``.
        """
        return RoboKudoScenePerception(ros_node=self.ros_node).observe(
            SceneQuery(world=world)
        )


# %% direct start


def main() -> World:
    """
    Run the demonstration and print what was seen and what was added to the world.

    :return: The world the seen objects were added to.
    """
    demonstration = PerceptionDemonstration()
    world = demonstration.run()
    for observed in demonstration.observation.objects:
        print(
            f"seen: {observed.category or 'unclassified'}, spawnable: {observed.is_spawnable}"
        )
    for annotation in demonstration.spawned_objects:
        print(f"added: {annotation.name} at {annotation.root.global_pose}")
    return world


if __name__ == "__main__":
    main()
