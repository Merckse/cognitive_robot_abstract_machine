"""
The HSR takes the milk from the desk in the living room of the lab and throws it away,
with the trash can standing somewhere new around the desk or the coffee table on every
run, to show how well it navigates to and places from the standing pose
:meth:`ThrowAwayDemonstration.standing_position_at` works out.

Run it from this folder::

    python demo-lab-variable.py
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass, field
from enum import Enum, StrEnum

import numpy as np
from numpy.typing import NDArray
from typing_extensions import Self

from coraplex import logger
from coraplex.plans.executors import PhysicallySimulatedPlanExecutor, PlanExecutor
from krrood.exceptions import DataclassException
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Floor,
    Milk,
    TrashCan,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

demo_lab = importlib.import_module("demo-lab")

# %% the living room


class LivingRoomBody(StrEnum):
    """
    The living room's own bodies the demonstration refers to.
    """

    DESK = "desk"
    """
    The top of the desk against the wall, which the milk starts on.
    """

    COFFEE_TABLE = "coffee_table"
    """
    The top of the coffee table beside the sofa.
    """


class LivingRoomSurfaceHeight(float, Enum):
    """
    How high above the floor the living room's surfaces the garbage stands on lie.
    """

    DESK = 0.765
    """
    The desk.
    """


@dataclass
class LivingRoomGarbage(demo_lab.Garbage):
    """
    An object the robot throws away that starts on a surface of the living room.
    """

    start_surface: LivingRoomSurfaceHeight
    """
    The surface the object starts on.
    """


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
    No pose drawn for the trash can was clear of the surroundings and of the robot's way
    with room for the robot to stand beside it.
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
    furniture, clear of the furniture, the walls and the robot's way, with a clear
    straight way for the robot to drive on to it.
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
    How far the trash can keeps from the surroundings and the robot's way, in meters.
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
        robot_way: list[NDArray[np.float64]],
    ) -> HomogeneousTransformationMatrix:
        """
        :param lab: The lab, holding its trash can, with nothing else spawned into it.
        :param furniture: The pieces of furniture the trash can stands around one of.
        :param robot_way: The positions the robot drives between in a straight line
            before it carries the garbage on from the last of them to the trash can, in
            the frame of the lab's root.
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
        robot_way_footprints = self._robot_way_footprints(robot_way)
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
            if robot_way_footprints.overlaps(trash_can_footprint)[0]:
                continue
            if not self._has_clear_way(floor_plan, robot_way[-1], middle):
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
class DeskThrowAwayDemonstration(demo_lab.ThrowAwayDemonstration):
    """
    The HSR takes the milk from the desk and throws it into the trash can, which stands
    somewhere new around the desk or the coffee table on every run.
    """

    ros_node_name: str = "suturo_throw_away_variable_demo_node"
    """
    Name of the node this demonstration registers.
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

    garbage: LivingRoomGarbage = field(
        default_factory=lambda: LivingRoomGarbage(
            Milk,
            demo_lab.SceneFile.MILK,
            LivingRoomSurfaceHeight.DESK,
            start_x=0.26,
            start_y=1.28,
            origin_height=0.09,
            start_yaw=np.pi,
        )
    )
    """
    The milk, which starts in the middle of the desk near its edge facing the open
    floor, and is thrown away.
    """

    furniture_near_trash_can: list[LivingRoomBody] = field(
        default_factory=lambda: [LivingRoomBody.DESK, LivingRoomBody.COFFEE_TABLE]
    )
    """
    The pieces of furniture the trash can is drawn to stand around one of.
    """

    trash_can_placement: TrashCanPlacement = field(default_factory=TrashCanPlacement)
    """
    Draws where the trash can stands on every run.
    """

    def build_scene(self) -> World:
        """
        Differs from :meth:`ThrowAwayDemonstration.build_scene` in first drawing
        :attr:`trash_can_in_room` with :attr:`trash_can_placement`, around one of
        :attr:`furniture_near_trash_can` and off the robot's way to where it takes the
        milk.
        """
        lab = self.lab.create_world()
        taking_pose = self._standing_pose_facing(
            Pose.from_xyz_rpy(
                x=self.garbage.start_x,
                y=self.garbage.start_y,
                yaw=self.garbage.start_yaw,
                reference_frame=lab.root,
            )
        )
        self.trash_can_in_room = self.trash_can_placement.sample_pose(
            lab,
            [lab.get_body_by_name(body) for body in self.furniture_near_trash_can],
            robot_way=[
                self.robot_start_in_room.position.to_np()[:2],
                taking_pose.position.to_np()[:2],
            ],
        )
        return super().build_scene()


# %% running the demonstration


def main(
    headless: bool = False,
    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor,
    collision_avoidance: bool = False,
    event_segmentation: bool = True,
    debug: bool = False,
    lab_parts_movable: bool = False,
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
    :param lab_parts_movable: Whether the drawers and doors of the lab can move.
    :param trash_can_seed: The seed the trash can's pose is drawn with; ``None`` draws
        a new pose every run.
    :return: The world the milk was thrown away in.
    """
    return DeskThrowAwayDemonstration(
        lab_parts_movable=lab_parts_movable,
        executor_type=executor_type,
        collision_avoidance=collision_avoidance,
        event_segmentation=event_segmentation,
        debug=debug,
        headless=headless,
        trash_can_placement=TrashCanPlacement(seed=trash_can_seed),
    ).run()


if __name__ == "__main__":
    main()
