"""
The real HSR picks up a milk carton in the lab kitchen from where it stands.

Run it from this folder, with the robot's controller running::

    python temp.py
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

import numpy as np
from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import RealRobotPlanExecutor
from coraplex.querying.grasp_closeness import GraspsByCloseness
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction
from cramph.composites import Sequence
from cramph.statechart import Statechart
from krrood.patterns.path_enumeration import PathEnumeration
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Point3,
    Pose,
)
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.specifications.connections import (
    Connection6DoFSpecification,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from coraplex.plans.executors import PlanExecutor
    from cramph.context import ContextExtension, StatechartContext
    from semantic_digital_twin.robots.robot_parts import AbstractRobot
    from semantic_digital_twin.world import World

# %% the scene


class SceneFile(PathEnumeration):
    """
    The files the scene is built from, under the coraplex resources.
    """

    MILK = Path(__file__).resolve().parents[2] / "resources" / "objects" / "milk.stl"
    """
    The mesh of the milk carton.
    """


class SceneBody(StrEnum):
    """
    The names of the scene's bodies the demonstration acts on.
    """

    COUNTER = "counter_top"
    """
    The worktop along the wall, which marks whether the kitchen was already merged in.
    """

    MILK = "milk"
    """
    The milk carton the plan picks up.
    """


# %% the demonstration


@dataclass
class MilkPickUpDemonstration(RobotDemonstration):
    """
    The real HSR parks its arm and picks up the milk from where it stands.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The robot that picks up the milk.
    """

    ros_node_name: str = "hsr_milk_pick_up_demo"
    """
    Name of the node this demonstration registers.
    """

    executor_type: type[PlanExecutor] = RealRobotPlanExecutor
    """
    The executor running the plan, which drives the real robot.
    """

    collision_avoidance = True

    map_T_scene: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.387, y=0.040, yaw=-0.0594
        )
    )
    """
    Where the kitchen lies in the map frame of the laser map ``apartment_master1``,
    fitted to the walls and furniture of that map at laser height, 25 cm above the
    floor.

    .. note:: The room is narrower and longer in the map than in the model, so the
        east wall still lies about 5 to 10 cm outside the wall of the map.
    """

    world_T_milk: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=1.629, y=6.023, z=0.73, yaw=0.351
        )
    )
    """
    Where the milk stands before the plan begins, in the map frame: where RoboKudo
    detected it on the cooking table, lowered onto the table top of the model.
    """

    middle_point: Point3 = field(
        default_factory=lambda: Point3(x=2.83, y=2.49)
    )  # x=2.78, y=2.93))#x=2.83, y=2.9))

    """
    The point the robot drives through on its way to :attr:`destination`, in the map
    frame.
    """

    destination: Point3 = field(default_factory=lambda: Point3(x=1.98, y=4.69))
    """
    Where the robot drives to, in the map frame.
    """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(world_source=KitchenEnvironment())

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(semantic_annotation_type=self.used_robot)

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(SceneBody.COUNTER)

    #
    # def populate_scene(self, world: World) -> None:
    #     """
    #     Put the milk at :attr:`world_T_milk`, free to be picked up, see
    #     :meth:`RobotDemonstration.populate_scene`.
    #     """
    #     Milk.get_annotation_specification(
    #         name=SceneBody.MILK,
    #         root_specification=BodySpecification.mesh(
    #             SceneBody.MILK,
    #             str(SceneFile.MILK),
    #             parent_T_self=self.world_T_milk,
    #             connection_specification=Connection6DoFSpecification(),
    #         ),
    #     ).spawn(world)
    #
    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        robot = context.require_extension(RobotAccess).robot
        middle_pose = self._standing_pose_facing(
            target=self.middle_point, approach_from=robot.root.global_pose
        )
        destination_pose = self._standing_pose_facing(
            target=self.destination, approach_from=middle_pose
        )

        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=robot.all_arms),
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                    # PathPlanningNavigation()
                    NavigateAction(target_location=middle_pose),
                    MoveTorsoAction(torso_state=TorsoState.MID),
                    NavigateAction(target_location=destination_pose),
                    # PickUpAction(grasp=closest_grasp, arm=arm),
                ]
            )
        )
        return statechart

    def _standing_pose_facing(
        self, *, target: Point3, approach_from: Pose, standing_distance: float = 0.0
    ) -> Pose:
        """
        :param target: What the robot goes to reach, in the frame of `approach_from`.
        :param approach_from: Where the robot stands before it goes to `target`.
        :param standing_distance: How far short of `target` the base stops, in meters.
        :return: Where the robot's base stands on the floor, `standing_distance` off
            `target` towards `approach_from` and facing the way from
            `approach_from` to `target`.
        """
        target_position = target.to_np()[:2]
        offset = target_position - approach_from.position.to_np()[:2]
        standing_position = (
            target_position - standing_distance * offset / np.linalg.norm(offset)
        )
        return Pose.from_xyz_rpy(
            x=float(standing_position[0]),
            y=float(standing_position[1]),
            yaw=float(np.arctan2(offset[1], offset[0])),
            reference_frame=approach_from.reference_frame,
        )


def main() -> World:
    """
    Run the demonstration.

    :return: The world the milk was picked up in.
    """
    return MilkPickUpDemonstration().run()


if __name__ == "__main__":
    main()
