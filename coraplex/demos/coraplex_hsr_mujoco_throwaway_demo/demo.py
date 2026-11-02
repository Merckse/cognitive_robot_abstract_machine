"""
The HSR carries a milk carton from a counter to a table in MuJoCo.

Run it from this folder::

    python demo.py
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import StrEnum
from pathlib import Path

import mujoco
import numpy as np

from krrood.patterns.path_enumeration import PathEnumeration

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import PhysicallySimulatedPlanExecutor, PlanExecutor
from coraplex.querying.grasp_closeness import GraspsByCloseness
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.statechart import Statechart
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.specifications.connections import (
    Connection6DoFSpecification,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.semantic_annotations import (
    SemanticAnnotationWithRootSpecification,
)
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.contact import ContactParameters
from semantic_digital_twin.world_description.geometry import Box, Mesh, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% the scene


class SceneFile(PathEnumeration):
    """
    The files the scene is built from, under the coraplex resources.
    """

    MILK = Path(__file__).resolve().parents[2] / "resources" / "objects" / "milk.stl"
    """
    The mesh of the milk carton.
    """

    TRASHCAN = (
        Path(__file__).resolve().parents[2] / "resources" / "objects" / "trashcan.stl"
    )
    """
    The mesh of the trash can.
    """


class SceneBody(StrEnum):
    """
    The names of the scene's bodies.
    """

    FLOOR = "floor"
    COUNTER = "counter"
    TABLE = "table"
    MILK = "milk"
    TRASHCAN = "trashcan"


# %% the demonstration


@dataclass
class MilkTransportDemonstration(RobotDemonstration):
    """
    The HSR carries the milk from the counter in front of it to the table behind it.
    """

    used_robot: type[AbstractRobot] = HSRB

    ros_node_name: str = "hsr_milk_transport_demo"

    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor

    floor: BodySpecification = field(
        default_factory=lambda: BodySpecification.box(
            SceneBody.FLOOR,
            Scale(4.0, 4.0, 0.02),
            parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(z=-0.01),
        )
    )
    """
    The floor the robot drives on.
    """

    counter: BodySpecification = field(
        default_factory=lambda: BodySpecification.box(
            SceneBody.COUNTER,
            Scale(0.6, 1.0, 0.75),
            parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=1.0, z=0.375),
        )
    )
    """
    The counter the milk starts on, in front of the robot.
    """
    #
    # table: BodySpecification = field(
    #     default_factory=lambda: BodySpecification.box(
    #         SceneBody.TABLE,
    #         Scale(0.6, 1.0, 0.7),
    #         parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=-1.0, z=0.35),
    #     )
    # )
    # """
    # The table the milk is carried to, behind the robot.
    # """

    trashcan: BodySpecification = field(
        default_factory=lambda: MilkTransportDemonstration._hollow_trashcan()
    )
    """
    The trash can standing on the floor behind the robot.
    """

    milk_depth: float = 0.15
    """
    How far behind the near edge of the counter the milk starts, and of the table it is
    set down, in meters.
    """

    standing_distance: float = 0.5
    """
    How far from the milk the robot's base stands to take it and to set it down, in
    meters.
    """

    grasp_height: float = 0.05
    """
    How far above the milk's origin the hand takes hold of it, in meters.
    """

    friction_cone: mujoco.mjtCone = mujoco.mjtCone.mjCONE_ELLIPTIC
    """
    The friction cone the contacts use: the elliptic cone is the exact one, where the
    default pyramid only approximates it and lets a held carton creep between the
    fingers.
    """

    impedance_ratio: float = 10.0
    """
    How much harder sliding is than pressing in: MuJoCo's own guidance for grasping with
    an elliptic cone.
    """

    no_slip_iterations: int = 10
    """
    Passes spent removing the slip soft contacts leave, so a held carton stays where the
    fingers hold it.
    """

    @classmethod
    def _hollow_trashcan(
        cls,
        width: float = 0.31,
        depth: float = 0.32,
        height: float = 0.39,
        wall_thickness: float = 0.01,
    ) -> BodySpecification:
        """
        :param width: The outer extent of the trash can along x, in meters.
        :param depth: The outer extent of the trash can along y, in meters.
        :param height: The height of the trash can, in meters.
        :param wall_thickness: The thickness of its walls and bottom, in meters.
        :return: The trash can, shown as its mesh but colliding as an open-top box of
            thin walls, since MuJoCo would collide with the mesh's closed convex hull.
        """
        half_height = height / 2
        walls = [
            Box(
                scale=Scale(width, depth, wall_thickness),
                origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                    z=wall_thickness / 2
                ),
            ),
            *(
                Box(
                    scale=Scale(wall_thickness, depth, height),
                    origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=side * (width - wall_thickness) / 2, z=half_height
                    ),
                )
                for side in (-1, 1)
            ),
            *(
                Box(
                    scale=Scale(width, wall_thickness, height),
                    origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                        y=side * (depth - wall_thickness) / 2, z=half_height
                    ),
                )
                for side in (-1, 1)
            ),
        ]
        return BodySpecification(
            SceneBody.TRASHCAN,
            ShapeCollection(walls),
            visual_shapes=Mesh(
                filename=str(SceneFile.TRASHCAN),
                # The mesh is in millimetres, with its base off-centre around
                # (125, 105), enlarged 2.25-fold.
                scale=Scale(0.00225, 0.00225, 0.00225),
                origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=-0.28125, y=-0.23625
                ),
            ).as_shape_collection(),
            parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=-1.0),
        )

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(objects=[self.floor, self.counter, self.trashcan])

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(self.used_robot)

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(SceneBody.MILK)

    def populate_scene(self, world: World) -> None:
        """
        Stand the milk on the counter, free to be picked up.
        """
        milk = SemanticAnnotationWithRootSpecification(
            name=SceneBody.MILK,
            semantic_annotation_type=Milk,
            root_specification=BodySpecification.mesh(
                SceneBody.MILK,
                str(SceneFile.MILK),
                connection_specification=Connection6DoFSpecification(),
            ),
        ).spawn(world)
        ContactParameters.create_for_grasped_object().apply_to([milk.root])
        milk.root.parent_connection.origin = self._milk_pose_on(
            world.get_body_by_name(SceneBody.COUNTER), milk
        ).homogeneous_matrix

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Raise the torso, park the arm, take the milk from the counter, carry it to the
        table and set it down there.
        """
        world = context.world
        hsrb = context.require_extension(RobotAccess).robot
        [milk] = world.get_semantic_annotations_by_type(Milk)
        [arm] = hsrb.all_arms
        taking_pose = self._standing_pose_facing(
            self._milk_pose_on(world.get_body_by_name(SceneBody.COUNTER), milk)
        )
        target = self._milk_pose_on(world.get_body_by_name(SceneBody.TRASHCAN), milk)
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=hsrb.all_arms),
                    NavigateAction(target_location=taking_pose),
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                    PickUpAction(grasp=self._grasp_from(taking_pose, milk), arm=arm),
                    ParkArmsAction(arms=hsrb.all_arms),
                    MoveTorsoAction(torso_state=TorsoState.LOW),
                    NavigateAction(target_location=self._standing_pose_facing(target)),
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                    PlaceAction(object_designator=milk, target_location=target),
                    ParkArmsAction(arms=hsrb.all_arms),
                    MoveTorsoAction(torso_state=TorsoState.LOW),
                ]
            )
        )
        return statechart

    def build_simulation(self, world: World) -> MujocoSim:
        return MujocoSim(
            world=world,
            headless=self.headless,
            cone=self.friction_cone,
            impratio=self.impedance_ratio,
            noslip_iterations=self.no_slip_iterations,
        )

    def _milk_pose_on(self, surface: Body, milk: Milk) -> Pose:
        """
        :param surface: The counter or the table.
        :param milk: The milk.
        :return: Where the milk stands on ``surface``, :attr:`milk_depth` behind the
            edge nearest the robot's start and turned the way the robot faces it.
        """
        world = surface._world
        surface_box = surface.collision.as_bounding_box_collection_in_frame(
            world.root
        ).bounding_box()
        milk_box = milk.root.collision.as_bounding_box_collection_in_frame(
            milk.root
        ).bounding_box()
        if surface_box.min_x > 0:
            x, yaw = surface_box.min_x + self.milk_depth, 0.0
        else:
            x, yaw = surface_box.max_x - self.milk_depth, np.pi
        return Pose.from_xyz_rpy(
            x=x,
            z=surface_box.max_z - milk_box.min_z,
            yaw=yaw,
            reference_frame=world.root,
        )

    def _standing_pose_facing(self, milk_pose: Pose) -> Pose:
        """
        :param milk_pose: Where the milk stands or is set down.
        :return: Where the robot's base stands on the floor to reach the milk there,
            :attr:`standing_distance` before it and facing it.
        """
        return (
            milk_pose.homogeneous_matrix
            @ HomogeneousTransformationMatrix.from_xyz_rpy(
                x=-self.standing_distance, z=-float(milk_pose.z)
            )
        ).pose

    def _grasp_from(self, standing_pose: Pose, milk: Milk) -> GraspCandidate:
        """
        :param standing_pose: Where the robot stands to take the milk.
        :param milk: The milk.
        :return: The milk's grasp closest to the robot standing there, raised
            :attr:`grasp_height` above the milk's origin.
        """
        [closest] = GraspsByCloseness(milk, standing_pose, number_of_grasps=1)()
        milk_T_raised_grasp = (
            HomogeneousTransformationMatrix.from_xyz_rpy(
                z=self.grasp_height, reference_frame=milk.root
            )
            @ milk.root._world.transform(
                closest.grasp_pose, milk.root
            ).homogeneous_matrix
        )
        return replace(closest, grasp_pose=milk_T_raised_grasp.pose)


def main(headless: bool = False) -> World:
    """
    Run the demonstration.

    :param headless: Whether the physics runs without a viewer.
    :return: The world the milk was carried in.
    """
    return MilkTransportDemonstration(headless=headless).run()


if __name__ == "__main__":
    main()
