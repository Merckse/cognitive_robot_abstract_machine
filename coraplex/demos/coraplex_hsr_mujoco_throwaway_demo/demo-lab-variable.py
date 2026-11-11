"""
The HSR throws a milk carton from a table of the SuTuRo lab into the trash can, with the
carton and the trash can drawn anew on every run, to test the standing poses the robot
works out to take and drop it.

Run it::

    python demo-lab-variable.py
"""

from __future__ import annotations

from contextlib import AbstractContextManager
from dataclasses import dataclass, field, replace
from enum import Enum, StrEnum
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray
from typing_extensions import TYPE_CHECKING, Callable, Self

from coraplex import logger
from coraplex.demonstrations import RobotDemonstration
from coraplex.locations.locations import ReachabilityLocation
from coraplex.plans.context_extensions import RobotAccess, StatementGrounding
from coraplex.plans.executors import PhysicallySimulatedPlanExecutor, PlanExecutor
from coraplex.querying.grasp_closeness import GraspsByCloseness
from coraplex.robot_plans.actions.core.navigation import (
    NavigateAction,
)
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.statechart import Statechart
from krrood.exceptions import DataclassException
from krrood.patterns.path_enumeration import PathEnumeration
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Point3,
    Pose,
)
from segmind.detectors.agent_event_detector_nodes import GraspDetector
from segmind.detectors.coarse_event_detector_nodes import (
    PickUpDetector,
    PlacingDetector,
)
from segmind.detectors.spatial_relation_detector_nodes import ContainmentDetector
from segmind.event_segmentation import Segmind
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootKinematicStructureEntity,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Floor,
    Milk,
    TrashCan,
)
from semantic_digital_twin.specifications.connections import Connection6DoFSpecification
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.semantic_annotations import (
    SemanticAnnotationWithRootSpecification,
)
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.contact import ContactParameters
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from semantic_digital_twin.robots.robot_parts import Arm

# %% what the scene is built from


class SceneFile(PathEnumeration):
    """
    The meshes of the garbage, under the coraplex resources.
    """

    MILK = Path(__file__).resolve().parents[2] / "resources" / "objects" / "milk.stl"


class LabBody(StrEnum):
    """
    The lab's own bodies the demonstration refers to.
    """

    COUNTER_TOP = "counter_top"
    """
    The kitchen counter top, which marks whether the scene was already populated.
    """

    ROOM = "root"
    """
    The body the lab is laid out in, whose frame is the room frame.
    """


class HSRBBody(StrEnum):
    """
    The HSR's own bodies the demonstration refers to.
    """

    WRIST = "wrist_roll_link"
    """
    The wrist the hand is mounted on, which is not part of the end effector.
    """


class LabFurniture(StrEnum):
    """
    The pieces of furniture of the lab the demonstration refers to.
    """

    DESK = "desk"
    """
    The top of the desk against the wall of the living room.
    """

    COFFEE_TABLE = "coffee_table"
    """
    The top of the coffee table beside the sofa.
    """

    DINING_TABLE = "dining_table"
    """
    The top of the dining table in the office.
    """

    COOKING_TABLE = "cooking_table"
    """
    The top of the cooking table in the bedroom.
    """


class LivingRoomSurfaceHeight(float, Enum):
    """
    How high above the floor the living room's surfaces the garbage stands on lie.
    """

    DESK = 0.765
    """
    The desk.
    """


# %% the garbage


@dataclass
class Garbage:
    """
    An object the robot throws away: what it is, where it starts in the room frame, and
    how it is let go of above the trash can.
    """

    name: str
    """
    The name of the object's body and of its annotation, by which the plan finds it.
    """

    semantic_annotation_type: type[HasRootKinematicStructureEntity]
    """
    What the object is.
    """

    mesh: SceneFile
    """
    The mesh the object is shaped by.
    """

    start_surface: float
    """
    How high above the floor the top of the furniture the object starts on lies.
    """

    start_x: float
    """
    Where along the room's x axis the object starts.
    """

    start_y: float
    """
    Where along the room's y axis the object starts.
    """

    origin_height: float
    """
    How high the object's origin stands above the bottom of the object.
    """

    start_yaw: float = -np.pi / 2
    """
    Which way the object faces in the room frame where it starts.
    """

    yaw_against_robot: float = 0.0
    """
    How the object is turned against the way the robot faces it while the robot picks
    it up, and so while the robot holds it.
    """

    drop_clearance: float = 0.05
    """
    How far above the trash can's rim the bottom of the object is when it is let go.
    """

    grasp_height: float = 0.05
    """
    How far above the grasp closest to the robot the object is taken hold of.
    """

    def spawn(self, world: World, map_T_room: HomogeneousTransformationMatrix) -> None:
        """
        Put the object where it starts and name it among the world's annotations.

        :param world: The world to spawn into.
        :param map_T_room: Where the frame the lab is laid out in lies in the map.
        """
        garbage = SemanticAnnotationWithRootSpecification(
            name=self.name,
            semantic_annotation_type=self.semantic_annotation_type,
            root_specification=BodySpecification.mesh(
                self.name,
                str(self.mesh),
                connection_specification=Connection6DoFSpecification(),
            ),
        ).spawn(
            world,
            parent_T_self=map_T_room
            @ HomogeneousTransformationMatrix.from_xyz_rpy(
                self.start_x,
                self.start_y,
                self.start_surface + self.origin_height,
                yaw=self.start_yaw,
            ),
        )
        ContactParameters.create_for_grasped_object().apply_to([garbage.root])

    def annotation_in(self, world: World) -> HasRootKinematicStructureEntity:
        """
        :param world: The world the object was spawned into.
        :return: The annotation naming the object, as the plan refers to it.
        """
        return world.get_semantic_annotation_by_name(self.name)

    def drop_point(self, world: World, trash_can: TrashCan) -> Point3:
        """
        :param world: The world the trash can stands in.
        :param trash_can: The trash can the object is thrown into.
        :return: Where the object's origin is when it is let go, in the world frame:
            centred above the trash can's opening, :attr:`drop_clearance` above its rim.
        """
        rim_height = (
            trash_can.as_bounding_box_collection_in_frame(trash_can.root)
            .bounding_box()
            .max_z
        )
        return world.transform(
            Point3(
                z=rim_height + self.drop_clearance + self.origin_height,
                reference_frame=trash_can.root,
            ),
            world.root,
        )

    def drop_location(self, drop_point: Point3, robot_yaw: float) -> Pose:
        """
        :param drop_point: Where the object's origin is when it is let go.
        :param robot_yaw: Which way the robot faces in the frame of `drop_point` while
            it lets go of the object.
        :return: The object at `drop_point`, turned against the robot as it was when
            the robot picked it up.
        """
        return Pose.from_xyz_rpy(
            x=float(drop_point.x),
            y=float(drop_point.y),
            z=float(drop_point.z),
            yaw=robot_yaw + self.yaw_against_robot,
            reference_frame=drop_point.reference_frame,
        )


# %% placing the garbage


@dataclass
class NoFreeGarbageSpot(DataclassException):
    """
    No spot drawn for the garbage on a piece of furniture had open floor for the robot to
    stand on in front of it.
    """

    furniture: str
    """
    The name of the piece of furniture.
    """

    attempts: int
    """
    How many spots were drawn.
    """

    def error_message(self) -> str:
        return (
            f"None of {self.attempts} spots drawn for the garbage on {self.furniture} "
            f"was free."
        )

    def suggest_correction(self) -> str:
        return (
            f"Leave {self.furniture} out of the furniture the garbage stands on, or "
            f"lower the edge margin."
        )


@dataclass
class GarbagePlacement:
    """
    Draws where the garbage starts: somewhere on the top of one of several pieces of
    furniture, with open floor for the robot to stand on in front of it.
    """

    seed: int | None = None
    """
    The seed the spot is drawn with; ``None`` draws with a fresh seed, which is logged so
    the run can be reproduced.
    """

    edge_margin: float = 0.1
    """
    How far from the edges of the furniture's top the garbage stands, in meters.
    """

    clearance: float = 0.1
    """
    How far the robot standing in front of the garbage keeps from the surroundings, in
    meters.
    """

    robot_radius: float = 0.3
    """
    Half the width of the square the robot's base covers on the floor, in meters.
    """

    attempts: int = 10000
    """
    How many spots are drawn before the placement gives up.
    """

    def sample_garbage(
        self,
        garbage: Garbage,
        lab: World,
        furniture: list[Body],
        robot_start: Pose,
        standing_pose: Callable[[Pose, Pose], Pose],
    ) -> Garbage:
        """
        :param garbage: The garbage to place.
        :param lab: The lab, holding its trash can, with nothing else spawned into it.
        :param furniture: The pieces of furniture the garbage is drawn to stand on the
            top of one of.
        :param robot_start: Where the robot stands before the plan begins, in the frame
            of the lab's root.
        :param standing_pose: Where the robot stands to take something at its first
            argument, coming from its second.
        :return: A copy of `garbage` on a drawn one of `furniture`, named after it,
            starting on a drawn spot and facing the way the robot faces it while taking
            it.
        :raises NoFreeGarbageSpot: If no spot drawn on that piece of furniture within
            :attr:`attempts` is free.
        """
        [trash_can] = lab.get_semantic_annotations_by_type(TrashCan)
        floor_plan = FloorPlan.from_world(lab, trash_can.bodies)
        seed_sequence = np.random.SeedSequence(self.seed)
        generator = np.random.default_rng(seed_sequence)
        logger.info(
            f"Drawing the spot of the garbage with seed {seed_sequence.entropy}."
        )
        return self._sample_spot(
            garbage,
            furniture[int(generator.integers(len(furniture)))],
            floor_plan,
            generator,
            lambda pose: standing_pose(pose, robot_start),
        )

    def _sample_spot(
        self,
        garbage: Garbage,
        furniture: Body,
        floor_plan: FloorPlan,
        generator: np.random.Generator,
        taking_pose_at: Callable[[Pose], Pose],
    ) -> Garbage:
        """
        :param garbage: The garbage to place a copy of.
        :param furniture: The piece of furniture the copy stands on the top of.
        :param floor_plan: The floors of the lab and what stands on them.
        :param generator: Draws the spot.
        :param taking_pose_at: Where the robot stands to take something at the given
            pose.
        :return: A copy of `garbage` named after `furniture`, starting on a drawn spot
            of its top and facing the way the robot faces it while taking it.
        :raises NoFreeGarbageSpot: If no spot drawn within :attr:`attempts` is free.
        """
        room = floor_plan.frame
        top = furniture.collision.as_bounding_box_collection_in_frame(
            room
        ).bounding_box()
        bounds = top.to_array_bounds()
        for _ in range(self.attempts):
            middle = generator.uniform(
                bounds.lower[:2] + self.edge_margin,
                bounds.upper[:2] - self.edge_margin,
            )
            taking_pose = taking_pose_at(
                Pose.from_xyz_rpy(
                    x=float(middle[0]), y=float(middle[1]), reference_frame=room
                )
            )
            robot_footprint = Footprints.from_squares(
                taking_pose.position.to_np()[:2], self.robot_radius + self.clearance
            )
            if not floor_plan.fits(robot_footprint)[0]:
                continue
            logger.info(
                f"Garbage placed on {furniture.name.name} at "
                f"x={middle[0]:.3f}, y={middle[1]:.3f} in the room."
            )
            return replace(
                garbage,
                name=f"{garbage.name}_on_{furniture.name.name}",
                start_surface=float(top.max_z),
                start_x=float(middle[0]),
                start_y=float(middle[1]),
                start_yaw=float(taking_pose.yaw),
            )
        raise NoFreeGarbageSpot(furniture=furniture.name.name, attempts=self.attempts)


# %% placing the trash can


@dataclass
class Footprints:
    """
    Axis-aligned rectangles on the floor, in the frame of the world's root.
    """

    lower: NDArray[np.float64]
    """
    The corner of each rectangle with the smallest x and y, one row per rectangle.
    """

    upper: NDArray[np.float64]
    """
    The corner of each rectangle with the largest x and y, one row per rectangle.
    """

    @classmethod
    def from_bodies(cls, bodies: list[Body], frame: Body) -> Self:
        """
        :param bodies: The bodies whose collision shapes are projected onto the floor.
        :param frame: The frame the rectangles are given in.
        :return: The rectangle each body covers, seen from above.
        """
        bounds = [
            body.collision.as_bounding_box_collection_in_frame(frame)
            .bounding_box()
            .to_array_bounds()
            for body in bodies
        ]
        return cls(
            np.array([bound.lower[:2] for bound in bounds]),
            np.array([bound.upper[:2] for bound in bounds]),
        )

    @classmethod
    def from_squares(cls, centers: NDArray[np.float64], half_width: float) -> Self:
        """
        :param centers: The middle of each square, one row per square.
        :param half_width: Half the edge length of every square.
        :return: The squares around `centers`.
        """
        centers = np.atleast_2d(centers)
        return cls(centers - half_width, centers + half_width)

    def overlaps(self, other: Footprints) -> NDArray[np.bool_]:
        """
        :param other: The rectangles to test.
        :return: For each rectangle of `other`, whether it overlaps any of these.
        """
        apart = np.any(
            (other.lower[:, None] >= self.upper[None])
            | (other.upper[:, None] <= self.lower[None]),
            axis=2,
        )
        return ~np.all(apart, axis=1)

    def covers(self, other: Footprints) -> NDArray[np.bool_]:
        """
        :param other: The rectangles to test.
        :return: For each rectangle of `other`, whether each of its corners lies in one
            of these.
        """
        corners = np.stack(
            [
                other.lower,
                other.upper,
                np.column_stack([other.lower[:, 0], other.upper[:, 1]]),
                np.column_stack([other.upper[:, 0], other.lower[:, 1]]),
            ],
            axis=1,
        )
        inside = np.all(
            (corners[:, :, None] >= self.lower[None, None])
            & (corners[:, :, None] <= self.upper[None, None]),
            axis=3,
        )
        return np.all(np.any(inside, axis=2), axis=1)


@dataclass
class FloorPlan:
    """
    The floors of a world and what stands on them, seen from above.
    """

    frame: Body
    """
    The frame the footprints are given in.
    """

    floors: Footprints
    """
    The floors the robot and the trash can stand on.
    """

    obstacles: Footprints
    """
    Everything standing on the floors, which nothing placed on them may overlap.
    """

    @classmethod
    def from_world(cls, world: World, movable_bodies: list[Body]) -> Self:
        """
        :param world: The world whose floors are mapped.
        :param movable_bodies: Bodies that do not count as obstacles, because they are
            about to be moved.
        :return: The floor plan of `world`.
        """
        floor_bodies = [
            floor.root for floor in world.get_semantic_annotations_by_type(Floor)
        ]
        obstacle_bodies = [
            body
            for body in world.bodies_with_collision
            if body not in floor_bodies and body not in movable_bodies
        ]
        return cls(
            world.root,
            Footprints.from_bodies(floor_bodies, world.root),
            Footprints.from_bodies(obstacle_bodies, world.root),
        )

    def fits(self, footprints: Footprints) -> NDArray[np.bool_]:
        """
        :param footprints: The rectangles to test.
        :return: For each rectangle, whether it lies on the floors and clear of
            everything standing on them.
        """
        return self.floors.covers(footprints) & ~self.obstacles.overlaps(footprints)


@dataclass
class NoFreeTrashCanPose(DataclassException):
    """
    No pose drawn for the trash can was clear of the surroundings and of where the robot
    stands, with room for the robot to stand beside it.
    """

    attempts: int
    """
    How many poses were drawn.
    """

    def error_message(self) -> str:
        return f"None of {self.attempts} poses drawn for the trash can was free."

    def suggest_correction(self) -> str:
        return "Widen the area the trash can may stand in, or lower the clearance it keeps."


@dataclass
class TrashCanPlacement:
    """
    Draws where the trash can stands: on open floor around one of several pieces of
    furniture, clear of the furniture, the walls and where the robot stands, with a clear
    straight way for the robot to drive on to it from where it starts.
    """

    seed: int | None = None
    """
    The seed the pose is drawn with; ``None`` draws with a fresh seed, which is logged so
    the run can be reproduced.
    """

    area_margin: float = 1.2
    """
    How far beyond the footprint of the furniture the trash can may stand, in meters.
    """

    clearance: float = 0.1
    """
    How far the trash can keeps from the surroundings and the robot, in meters.
    """

    robot_radius: float = 0.3
    """
    Half the width of the square the robot's base covers on the floor, in meters.
    """

    standing_distance: float = 0.55
    """
    How far from the middle of the trash can the robot's base stands to reach over it, in
    meters.
    """

    attempts: int = 10000
    """
    How many poses are drawn before the placement gives up.
    """

    def sample_pose(
        self,
        lab: World,
        furniture: list[Body],
        robot_stops: list[NDArray[np.float64]],
    ) -> HomogeneousTransformationMatrix:
        """
        :param lab: The lab, holding its trash can, with nothing else spawned into it.
        :param furniture: The pieces of furniture the trash can stands around one of.
        :param robot_stops: Where the robot stands before the plan begins, first, and
            where it stands to take each piece of garbage, in the frame of the lab's
            root.
        :return: Where the middle of the trash can's base stands relative to the lab's
            root.
        :raises NoFreeTrashCanPose: If no pose drawn within :attr:`attempts` is free.
        """
        [trash_can] = lab.get_semantic_annotations_by_type(TrashCan)
        floor_plan = FloorPlan.from_world(lab, trash_can.bodies)
        trash_can_box = trash_can.as_bounding_box_collection_in_frame(
            trash_can.root
        ).bounding_box()
        trash_can_radius = float(
            np.hypot(
                max(abs(trash_can_box.min_x), abs(trash_can_box.max_x)),
                max(abs(trash_can_box.min_y), abs(trash_can_box.max_y)),
            )
        )
        robot_footprints = Footprints.from_squares(
            np.array(robot_stops), self.robot_radius + self.clearance
        )
        furniture_footprints = Footprints.from_bodies(furniture, lab.root)
        area = Footprints(
            furniture_footprints.lower - self.area_margin,
            furniture_footprints.upper + self.area_margin,
        )
        seed_sequence = np.random.SeedSequence(self.seed)
        generator = np.random.default_rng(seed_sequence)
        for _ in range(self.attempts):
            middle = generator.uniform(area.lower.min(axis=0), area.upper.max(axis=0))
            if not area.covers(Footprints.from_squares(middle, 0.0))[0]:
                continue
            trash_can_footprint = Footprints.from_squares(
                middle, trash_can_radius + self.clearance
            )
            if not floor_plan.fits(trash_can_footprint)[0]:
                continue
            if robot_footprints.overlaps(trash_can_footprint)[0]:
                continue
            if not self._has_clear_way(floor_plan, robot_stops[0], middle):
                continue
            yaw = generator.uniform(-np.pi, np.pi)
            logger.info(
                f"Trash can placed at x={middle[0]:.3f}, y={middle[1]:.3f}, "
                f"yaw={yaw:.3f} in the room, drawn with seed {seed_sequence.entropy}."
            )
            return HomogeneousTransformationMatrix.from_xyz_rpy(
                x=float(middle[0]), y=float(middle[1]), yaw=float(yaw)
            )
        raise NoFreeTrashCanPose(attempts=self.attempts)

    def _robot_way_footprints(self, robot_way: list[NDArray[np.float64]]) -> Footprints:
        """
        :param robot_way: The positions the robot drives between in a straight line.
        :return: What the robot's base covers on its way, widened by
            :attr:`clearance`.
        """
        stops = [robot_way[0]]
        for start, end in zip(robot_way, robot_way[1:]):
            steps = int(np.ceil(np.linalg.norm(end - start) / self.robot_radius))
            stops.extend(np.linspace(start, end, steps + 1)[1:])
        return Footprints.from_squares(
            np.array(stops), self.robot_radius + self.clearance
        )

    def _has_clear_way(
        self,
        floor_plan: FloorPlan,
        way_start: NDArray[np.float64],
        middle: NDArray[np.float64],
    ) -> bool:
        """
        :param floor_plan: The floors and what stands on them, without the trash can.
        :param way_start: Where the robot sets off to the trash can from.
        :param middle: Where the middle of the trash can stands.
        :return: Whether the robot's base stays on the floor and clear of everything
            standing on it while it drives in a straight line from `way_start` until it
            stands :attr:`standing_distance` before `middle`.
        """
        direction = way_start - middle
        stand = middle + self.standing_distance * direction / np.linalg.norm(direction)
        return bool(
            np.all(floor_plan.fits(self._robot_way_footprints([way_start, stand])))
        )


# %% the demonstration


@dataclass
class RandomSceneThrowAwayDemonstration(RobotDemonstration):
    """
    The HSR takes the milk from a table and throws it into the trash can; the milk stands
    somewhere new on one of :attr:`furniture_under_garbage` and the trash can somewhere
    new around one of :attr:`furniture_near_trash_can` on every run.
    """

    used_robot: type[HSRB] = HSRB
    """
    The HSR, whose wrist :meth:`relax_collision_rules` refers to by name.
    """

    ros_node_name: str = "suturo_throw_away_variable_demo_node"
    """
    Name of the node this demonstration registers.
    """

    lab: KitchenEnvironment = field(default_factory=KitchenEnvironment)
    """
    The lab the demonstration runs in.
    """

    scene_joints_movable: bool = False
    """
    Whether the drawers and doors of :attr:`lab` can move.

    Parts that cannot move are welded in place, so the physics simulates no joints for
    them.
    """

    map_T_room: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.33, y=0.28, yaw=0.10707963267
        )
    )
    """
    Where the frame :attr:`lab` is laid out in lies in the map the robot localizes in.
    """

    robot_start_in_room: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            1.8, 1.28, yaw=np.pi
        )
    )
    """
    Where the HSR stands before the plan begins, in the room frame: on the open floor of
    the living room, facing the desk.
    """

    standing_distance: float = 0.7
    """
    How far before the garbage the robot's base stands to take it, in meters.
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

    no_slip_iterations: int = 2
    """
    Passes spent removing the slip soft contacts leave, so a held carton stays where the
    fingers hold it.
    """

    garbage: Garbage = field(
        default_factory=lambda: Garbage(
            "milk",
            Milk,
            SceneFile.MILK,
            LivingRoomSurfaceHeight.DESK,
            start_x=0.26,
            start_y=1.28,
            origin_height=0.09,
            start_yaw=np.pi,
        )
    )
    """
    The milk, which is thrown away; where it starts is drawn anew with
    :attr:`garbage_placement` on every run.
    """

    furniture_under_garbage: list[LabFurniture] = field(
        default_factory=lambda: [
            LabFurniture.DESK,
            LabFurniture.DINING_TABLE,
            LabFurniture.COOKING_TABLE,
            LabFurniture.COFFEE_TABLE,
        ]
    )
    """
    The pieces of furniture the milk is drawn to stand on the top of one of.
    """

    garbage_placement: GarbagePlacement = field(default_factory=GarbagePlacement)
    """
    Draws where the milk starts on every run.
    """

    furniture_near_trash_can: list[LabFurniture] = field(
        default_factory=lambda: [LabFurniture.DESK, LabFurniture.COFFEE_TABLE]
    )
    """
    The pieces of furniture the trash can is drawn to stand around one of.
    """

    trash_can_placement: TrashCanPlacement = field(default_factory=TrashCanPlacement)
    """
    Draws where the trash can stands on every run.
    """

    def scene_specification(self) -> WorldSpecification:
        """
        :return: An empty world: the lab is not described by a model file, so
            :meth:`build_scene` builds it with :class:`KitchenEnvironment` instead.
        """
        return WorldSpecification()

    def build_scene(self) -> World:
        """
        Differs from :meth:`RobotDemonstration.build_scene` in building :attr:`lab` at
        :attr:`map_T_room`, after drawing :attr:`garbage` onto one of
        :attr:`furniture_under_garbage` and the trash can around one of
        :attr:`furniture_near_trash_can`, off the robot's way to the milk.
        """
        lab = self.lab.create_world()
        robot_start = self.robot_start_in_room.pose
        self.garbage = self.garbage_placement.sample_garbage(
            self.garbage,
            lab,
            [lab.get_body_by_name(body) for body in self.furniture_under_garbage],
            robot_start,
            self._standing_pose_facing,
        )
        taking_pose = self._standing_pose_facing(
            Pose.from_xyz_rpy(
                x=self.garbage.start_x,
                y=self.garbage.start_y,
                reference_frame=lab.root,
            ),
            robot_start,
        )
        room_T_trash_can = self.trash_can_placement.sample_pose(
            lab,
            [lab.get_body_by_name(body) for body in self.furniture_near_trash_can],
            robot_stops=[
                robot_start.position.to_np()[:2],
                taking_pose.position.to_np()[:2],
            ],
        )
        self.move_trash_can(lab, room_T_trash_can)
        scene = World.create_with_root_body()
        scene.merge_world(
            lab,
            root_connection=FixedConnection(
                parent=scene.root,
                child=lab.root,
                parent_T_connection_expression=self.map_T_room,
            ),
        )
        self.populate_scene(scene)
        if not self.scene_joints_movable:
            self.replace_joints_with_fixed_connections(scene)
        return scene

    @staticmethod
    def move_trash_can(
        lab: World, room_T_trash_can: HomogeneousTransformationMatrix
    ) -> None:
        """
        Stand the trash can of `lab` on the floor at `room_T_trash_can`.

        :param lab: The lab, laid out in the frame of its root.
        :param room_T_trash_can: Where the middle of the trash can's base stands relative
            to the lab's root.
        """
        [trash_can] = lab.get_semantic_annotations_by_type(TrashCan)
        base_height = (
            trash_can.as_bounding_box_collection_in_frame(trash_can.root)
            .bounding_box()
            .min_z
        )
        with lab.modify_world():
            lab.remove_connection(trash_can.root.parent_connection)
            lab.add_connection(
                FixedConnection(
                    parent=lab.root,
                    child=trash_can.root,
                    parent_T_connection_expression=room_T_trash_can
                    @ HomogeneousTransformationMatrix.from_xyz_rpy(z=-base_height),
                )
            )

    def robot_specification(self) -> RobotSpecification:
        """
        :return: The HSR, starting at :attr:`robot_start_in_room`.
        """
        return RobotSpecification(
            semantic_annotation_type=self.used_robot,
            odometry_T_robot_start=self.map_T_room @ self.robot_start_in_room,
        )

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(LabBody.COUNTER_TOP)

    def populate_scene(self, world: World) -> None:
        """
        Put the garbage where it starts.
        """
        self.garbage.spawn(world, self.map_T_room)

    def acquire_world(self) -> World:
        """
        Obtain the world as :meth:`RobotDemonstration.acquire_world` does, with the
        collision rules of :meth:`relax_collision_rules`.
        """
        world = super().acquire_world()
        [robot] = world.get_semantic_annotations_by_type(self.used_robot)
        self.relax_collision_rules(world, robot)
        return world

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [
            RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0]),
            StatementGrounding(),
        ]

    def relax_collision_rules(self, world: World, robot: HSRB) -> None:
        """
        Let the HSR's hand and wrist touch the garbage throughout the run, as physics
        never counts it as held: the HSR's end effector begins at its palm, so the wrist
        behind it would otherwise stall short of the grasp.

        Let the HSR touch the floors of the lab, which its wheels stand on.

        :param world: The world holding the robot, the lab and the garbage.
        :param robot: The HSR.
        """
        [arm] = robot.all_arms
        with world.modify_world():
            world.collision_manager.add_ignore_collision_rule(
                AllowCollisionBetweenGroups(
                    body_group_a=[
                        world.get_body_by_name(HSRBBody.WRIST),
                        *arm.end_effector.bodies_with_collision,
                    ],
                    body_group_b=[self.garbage.annotation_in(world).root],
                )
            )
            world.collision_manager.add_ignore_collision_rule(
                AllowCollisionBetweenGroups(
                    body_group_a=robot.bodies_with_collision,
                    body_group_b=[
                        floor.root
                        for floor in world.get_semantic_annotations_by_type(Floor)
                    ],
                )
            )

    def segment_events(self, world: World) -> AbstractContextManager:
        """
        Detect the garbage being grasped, picked up, placed and contained.
        """
        return Segmind.create_for_semantic_annotation_types(
            world,
            (self.garbage.semantic_annotation_type,),
            detector_types=(
                PickUpDetector,
                PlacingDetector,
                ContainmentDetector,
                GraspDetector,
            ),
        )

    def standing_position_at(
        self,
        context: StatechartContext,
        drop_point: Point3,
        arm: Arm,
        approach_from: Pose,
    ) -> Pose:
        """
        :param context: The context of the plan, whose robot throws the garbage away.
        :param drop_point: Where the garbage is let go, in the world frame.
        :param arm: The arm that holds the garbage over the trash can.
        :param approach_from: Where the robot stands before it goes to the trash can,
            in the world frame.
        :return: Of the poses clear of the surroundings from which `arm` reaches
            `drop_point`, the one closest to standing at the arm's reach
            distance off `drop_point` on the side of `approach_from`.
        """
        location = ReachabilityLocation(
            target_pose=Pose(
                position=drop_point, reference_frame=drop_point.reference_frame
            ),
            arm=arm,
            context=context,
        )
        drop_position = drop_point.to_np()[:2]
        approach_direction = approach_from.position.to_np()[:2] - drop_position
        ideal_position = drop_position + (
            approach_direction
            / np.linalg.norm(approach_direction)
            * float(arm.approximate_length())
            * location.reach_fraction
        )
        return min(
            location,
            key=lambda candidate: np.linalg.norm(
                candidate.position.to_np()[:2] - ideal_position
            ),
        )

    @staticmethod
    def orient_towards(pose: Pose, target: HasRootKinematicStructureEntity) -> Pose:
        """
        :param pose: The pose to turn, in the world frame.
        :param target: The object to face.
        :return: `pose` at its position, turned level about the vertical so that its
            x-axis points at the origin of `target`.
        """
        offset = (
            target.root.global_pose.position.to_np()[:2] - pose.position.to_np()[:2]
        )
        return Pose.from_xyz_rpy(
            x=float(pose.x),
            y=float(pose.y),
            z=float(pose.z),
            yaw=float(np.arctan2(offset[1], offset[0])),
            reference_frame=pose.reference_frame,
        )

    def _grasp_from(
        self, world: World, standing_pose: Pose, milk: Milk
    ) -> GraspCandidate:
        """
        :param world: The world the milk stands in.
        :param standing_pose: Where the robot stands to take the milk.
        :param milk: The milk.
        :return: The milk's grasp closest to the robot standing there, raised
            :attr:`Garbage.grasp_height` along the milk's vertical.
        """
        [closest] = GraspsByCloseness(milk, standing_pose, number_of_grasps=1)()
        milk_T_raised_grasp = (
            HomogeneousTransformationMatrix.from_xyz_rpy(
                z=self.garbage.grasp_height, reference_frame=milk.root
            )
            @ world.transform(closest.grasp_pose, milk.root).homogeneous_matrix
        )
        return replace(closest, grasp_pose=milk_T_raised_grasp.pose)

    def _standing_pose_facing(self, target: Pose, approach_from: Pose) -> Pose:
        """
        :param target: What the robot goes to reach.
        :param approach_from: Where the robot stands before it goes to `target`, in the
            frame of `target`.
        :return: Where the robot's base stands on the floor, :attr:`standing_distance`
            off `target` towards `approach_from` and facing `target`.
        """
        offset = target.position.to_np()[:2] - approach_from.position.to_np()[:2]
        standing_position = target.position.to_np()[
            :2
        ] - self.standing_distance * offset / np.linalg.norm(offset)
        return Pose.from_xyz_rpy(
            x=float(standing_position[0]),
            y=float(standing_position[1]),
            yaw=float(np.arctan2(offset[1], offset[0])),
            reference_frame=target.reference_frame,
        )

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Park the arm, take the garbage from where it starts, carry it to the trash can
        with the arm parked and the torso lowered, and let go of it above the opening.
        """
        world = context.world
        robot = context.require_extension(RobotAccess).robot
        [arm] = robot.all_arms
        [trash_can] = world.get_semantic_annotations_by_type(TrashCan)
        garbage = self.garbage.annotation_in(world)
        taking_pose = self._standing_pose_facing(
            garbage.root.global_pose, robot.root.global_pose
        )
        drop_point = self.garbage.drop_point(world, trash_can)
        throwing_pose = self.orient_towards(
            self.standing_position_at(context, drop_point, arm, taking_pose), trash_can
        )
        drop_location = self.garbage.drop_location(drop_point, float(throwing_pose.yaw))
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=robot.all_arms),
                    NavigateAction(target_location=taking_pose),
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                    PickUpAction(
                        grasp=self._grasp_from(world, taking_pose, garbage), arm=arm
                    ),
                    ParkArmsAction(arms=robot.all_arms),
                    MoveTorsoAction(torso_state=TorsoState.LOW),
                    NavigateAction(target_location=throwing_pose),
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                    PlaceAction(
                        object_designator=garbage, target_location=drop_location
                    ),
                    ParkArmsAction(arms=robot.all_arms),
                    MoveTorsoAction(torso_state=TorsoState.LOW),
                ]
            )
        )

        return statechart

    def build_simulation(self, world: World) -> MujocoSim:
        """
        Build the physics of :meth:`RobotDemonstration.build_simulation` with the contacts
        of :attr:`friction_cone`, :attr:`impedance_ratio` and :attr:`no_slip_iterations`.
        """
        return MujocoSim(
            world=world,
            headless=self.headless,
            cone=self.friction_cone,
            impratio=self.impedance_ratio,
            noslip_iterations=self.no_slip_iterations,
        )


# %% running the demonstration


def main(
    headless: bool = False,
    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor,
    collision_avoidance: bool = True,
    event_segmentation: bool = False,
    debug: bool = False,
    scene_joints_movable: bool = False,
    garbage_seed: int | None = None,
    trash_can_seed: int | None = None,
) -> World:
    """
    Run the demonstration.

    :param headless: Whether to run the simulation without a viewer.
    :param executor_type: The executor running the plan, which decides whether it drives
        the real robot or simulates it.
    :param collision_avoidance: Whether the statechart avoids collisions.
    :param event_segmentation: Whether SegMind segments the run into events.
    :param debug: Whether to run in debug mode, publishing every copy of the world a
        candidate is tried in.
    :param scene_joints_movable: Whether the drawers and doors of the lab can move.
    :param garbage_seed: The seed the milk's spot is drawn with; ``None`` draws a new
        spot every run.
    :param trash_can_seed: The seed the trash can's pose is drawn with; ``None`` draws
        a new pose every run.
    :return: The world the milk was thrown away in.
    """
    return RandomSceneThrowAwayDemonstration(
        scene_joints_movable=scene_joints_movable,
        executor_type=executor_type,
        collision_avoidance=collision_avoidance,
        event_segmentation=event_segmentation,
        debug=debug,
        headless=headless,
        garbage_placement=GarbagePlacement(seed=garbage_seed),
        trash_can_placement=TrashCanPlacement(seed=trash_can_seed),
    ).run()


if __name__ == "__main__":
    x = 0
    for i in range(1, 10):
        try:
            main()
        except:
            x = x + 1
