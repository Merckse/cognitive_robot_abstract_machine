"""
Objects are scattered on a table at random spots, and the HSR tries to grasp and lift
the ones it is told to, in MuJoCo. Every attempt runs in a process of its own against
the layout of the same seed. Run it from this folder::

    python demo-table-grasps.py                        # try every object on the table
    python demo-table-grasps.py BOWL CUP MILK          # try only these
    python demo-table-grasps.py CUP --on-table CUP BOWL SPOON --seed 7 --headless
    python demo-table-grasps.py CUP --seed 7 --grasp-rank 1    # one attempt, 2nd-closest grasp

The object names are the members of :class:`ObjectMesh` of ``demo-pick-up-objects``.
"""

from __future__ import annotations

import argparse
import importlib
from dataclasses import dataclass, field
from enum import Enum, StrEnum, auto

import numpy as np
from coraplex.plans.context_extensions import RobotAccess
from coraplex.querying.grasp_closeness import GraspsByCloseness
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.statechart import Statechart
from krrood.exceptions import DataclassException
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.specifications.connections import (
    Connection6DoFSpecification,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.semantic_annotations import (
    SemanticAnnotationWithRootSpecification,
)
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.contact import ContactParameters
from semantic_digital_twin.world_description.geometry import AxisAlignedBox, Scale
from semantic_digital_twin.world_description.world_entity import Body

from demo import SceneBody

# The hyphen in the file name rules out an import statement.
pick_up_objects = importlib.import_module("demo-pick-up-objects")

# %% the edges of the table


class TableEdge(Enum):
    """
    An edge of an axis-aligned table, named for the world axis its outward side faces.
    """

    NEGATIVE_X = auto()
    """
    The edge facing the robot's start, in front of it.
    """

    POSITIVE_X = auto()
    """
    The edge facing away from the robot's start.
    """

    NEGATIVE_Y = auto()
    """
    The edge on the robot's right at its start.
    """

    POSITIVE_Y = auto()
    """
    The edge on the robot's left at its start.
    """

    def outward_normal(self) -> np.ndarray:
        """
        :return: The unit vector in the world's xy plane pointing off the table across
            this edge.
        """
        match self:
            case TableEdge.NEGATIVE_X:
                return np.array([-1.0, 0.0])
            case TableEdge.POSITIVE_X:
                return np.array([1.0, 0.0])
            case TableEdge.NEGATIVE_Y:
                return np.array([0.0, -1.0])
            case TableEdge.POSITIVE_Y:
                return np.array([0.0, 1.0])

    def inward_distance(self, *, table_box: AxisAlignedBox, point: np.ndarray) -> float:
        """
        :param table_box: The bounding box of the table in the world frame.
        :param point: A point on the table in the world's xy plane.
        :return: How far ``point`` lies inside this edge, in meters.
        """
        match self:
            case TableEdge.NEGATIVE_X:
                return float(point[0] - float(table_box.min_x))
            case TableEdge.POSITIVE_X:
                return float(float(table_box.max_x) - point[0])
            case TableEdge.NEGATIVE_Y:
                return float(point[1] - float(table_box.min_y))
            case TableEdge.POSITIVE_Y:
                return float(float(table_box.max_y) - point[1])


# %% random spots on the table


@dataclass(kw_only=True)
class Footprint:
    """
    The circle in the world's xy plane an object covers on the table, whatever way it is
    turned.
    """

    center_in_object: np.ndarray
    """
    The center of the object's bounding box in the xy plane of its own frame.
    """

    radius: float
    """
    Half the diagonal of the object's bounding box in the xy plane, in meters.
    """

    lowest_point_in_object: float
    """
    The height of the object's lowest point in its own frame, in meters.
    """

    @classmethod
    def from_body(cls, *, body: Body) -> Footprint:
        """
        :param body: The body of an object, standing upright.
        :return: The footprint of ``body``'s collision.
        """
        box = body.collision.as_bounding_box_collection_in_frame(body).bounding_box()
        minimum = np.array([float(box.min_x), float(box.min_y)])
        maximum = np.array([float(box.max_x), float(box.max_y)])
        return cls(
            center_in_object=(minimum + maximum) / 2,
            radius=float(np.linalg.norm(maximum - minimum) / 2),
            lowest_point_in_object=float(box.min_z),
        )


@dataclass(kw_only=True)
class TableSpot:
    """
    Where on the table one object stands.
    """

    center: np.ndarray
    """
    The center of the object's footprint in the world's xy plane.
    """

    yaw: float
    """
    Which way the object is turned about the world's z axis.
    """

    footprint: Footprint
    """
    The footprint of the object standing here.
    """

    def overlaps(self, *, other: TableSpot, gap: float) -> bool:
        """
        :param other: Another spot on the table.
        :param gap: The free space two objects must keep between them, in meters.
        :return: Whether the objects on this spot and on ``other`` come closer than
            ``gap``.
        """
        distance = float(np.linalg.norm(self.center - other.center))
        return distance < self.footprint.radius + other.footprint.radius + gap

    def compute_object_pose(self, *, table_top: float, world: World) -> Pose:
        """
        :param table_top: The height of the table's top in the world frame.
        :param world: The world the object stands in.
        :return: The pose of the object's frame in the world root at which its footprint
            is centered on this spot and its lowest point rests on the table's top.
        """
        cosine, sine = np.cos(self.yaw), np.sin(self.yaw)
        rotation = np.array([[cosine, -sine], [sine, cosine]])
        origin = self.center - rotation @ self.footprint.center_in_object
        return Pose.from_xyz_rpy(
            x=float(origin[0]),
            y=float(origin[1]),
            z=table_top - self.footprint.lowest_point_in_object,
            yaw=self.yaw,
            reference_frame=world.root,
        )


@dataclass(kw_only=True)
class TableTooCrowded(DataclassException):
    """
    No layout was found that gives every object a free spot within reach.
    """

    object_names: list[str]
    """
    The names of the objects that were to be placed.
    """

    layout_attempts: int
    """
    How many random layouts were tried.
    """

    def error_message(self) -> str:
        return (
            f"No layout with a free spot within reach for each of {self.object_names} "
            f"was found in {self.layout_attempts} random tries."
        )

    def suggest_correction(self) -> str:
        return (
            "Put fewer objects on the table, try another seed, make the table larger or "
            "allow the robot to approach it from more edges."
        )


@dataclass(kw_only=True)
class RandomTableLayout:
    """
    Scatters objects across an axis-aligned table at random, so that none of them
    overlap and each lies within the robot's reach from one of :attr:`approach_edges`.
    """

    seed: int
    """
    The seed of the random spots; the same seed and objects always give the same layout.
    """

    approach_edges: list[TableEdge] = field(
        default_factory=lambda: [TableEdge.NEGATIVE_X]
    )
    """
    The edges of the table the robot may stand at to grasp an object.
    """

    reach_depth: float = 0.3
    """
    How far inside its nearest approach edge an object's center may lie, in meters.
    """

    edge_margin: float = 0.03
    """
    How far every object keeps from every edge of the table, in meters.
    """

    gap: float = 0.04
    """
    The free space every two objects keep between them, in meters.
    """

    attempts_per_object: int = 500
    """
    How many random spots are tried for one object before its layout is given up.
    """

    layout_attempts: int = 200
    """
    How many layouts are tried before the table counts as crowded.
    """

    def nearest_approach_edge(
        self, *, table_box: AxisAlignedBox, point: np.ndarray
    ) -> TableEdge:
        """
        :param table_box: The bounding box of the table in the world frame.
        :param point: A point on the table in the world's xy plane.
        :return: The approach edge ``point`` lies least deep inside.
        """
        return min(
            self.approach_edges,
            key=lambda edge: edge.inward_distance(table_box=table_box, point=point),
        )

    def sample_spots(
        self, *, table_box: AxisAlignedBox, footprints: dict[str, Footprint]
    ) -> dict[str, TableSpot]:
        """
        :param table_box: The bounding box of the table in the world frame.
        :param footprints: The footprint of each object, by the object's name.
        :return: A free spot within reach for each object, by the object's name.
        :raises TableTooCrowded: If no layout within :attr:`layout_attempts` gives every
            object such a spot.
        """
        generator = np.random.default_rng(self.seed)
        largest_first = sorted(
            footprints.items(), key=lambda item: item[1].radius, reverse=True
        )
        for _ in range(self.layout_attempts):
            spots = self._sample_layout(
                generator=generator, table_box=table_box, footprints=largest_first
            )
            if spots is not None:
                return spots
        raise TableTooCrowded(
            object_names=list(footprints), layout_attempts=self.layout_attempts
        )

    def _sample_layout(
        self,
        *,
        generator: np.random.Generator,
        table_box: AxisAlignedBox,
        footprints: list[tuple[str, Footprint]],
    ) -> dict[str, TableSpot] | None:
        """
        :return: A free spot within reach for each object, placed in the order of
            ``footprints``, or ``None`` if one of them finds no spot.
        """
        spots: dict[str, TableSpot] = {}
        for object_name, footprint in footprints:
            spot = self._sample_free_spot(
                generator=generator,
                table_box=table_box,
                footprint=footprint,
                taken=list(spots.values()),
            )
            if spot is None:
                return None
            spots[object_name] = spot
        return spots

    def _sample_free_spot(
        self,
        *,
        generator: np.random.Generator,
        table_box: AxisAlignedBox,
        footprint: Footprint,
        taken: list[TableSpot],
    ) -> TableSpot | None:
        """
        :return: A random spot for an object of ``footprint`` that overlaps none of
            ``taken`` and lies within :attr:`reach_depth` of an approach edge, or
            ``None`` if none is found within :attr:`attempts_per_object`.
        """
        clearance = footprint.radius + self.edge_margin
        low = np.array([float(table_box.min_x), float(table_box.min_y)]) + clearance
        high = np.array([float(table_box.max_x), float(table_box.max_y)]) - clearance
        for _ in range(self.attempts_per_object):
            spot = TableSpot(
                center=generator.uniform(low, high),
                yaw=float(generator.uniform(-np.pi, np.pi)),
                footprint=footprint,
            )
            edge = self.nearest_approach_edge(table_box=table_box, point=spot.center)
            if (
                edge.inward_distance(table_box=table_box, point=spot.center)
                > self.reach_depth
            ):
                continue
            if any(spot.overlaps(other=other, gap=self.gap) for other in taken):
                continue
            return spot
        return None


# %% grasping one object off the table


@dataclass(kw_only=True)
class TableGraspDemonstration(pick_up_objects.ObjectPickUpDemonstration):
    """
    The HSR walks to the edge of the table nearest :attr:`pick_up_object`, which stands
    among :attr:`objects_on_table` at the spots of :attr:`layout`, grasps it and holds it
    up with its arm parked.
    """

    ros_node_name: str = "hsr_table_grasp_demo"
    """
    Name of the node this demonstration registers.
    """

    table: BodySpecification = field(
        default_factory=lambda: BodySpecification.box(
            SceneBody.TABLE,
            Scale(0.8, 3.0, 0.72),
            parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=1.0, z=0.36),
        )
    )
    """
    The table the objects are scattered on, in front of the robot.
    """

    supporting_surface: str = SceneBody.TABLE
    """
    The name of the body the objects are scattered on.
    """

    objects_on_table: list[pick_up_objects.PickUpObject] = field(
        default_factory=lambda: pick_up_objects.PickUpSurvey().objects
    )
    """
    The objects scattered on the table, :attr:`pick_up_object` among them.
    """

    layout: RandomTableLayout = field(default_factory=lambda: RandomTableLayout(seed=0))
    """
    Decides where on the table each of :attr:`objects_on_table` stands.
    """

    base_distance_from_edge: float = 0.35
    """
    How far outside the table's edge the robot's base stands to grasp an object, in
    meters.
    """

    number_of_grasps: int = 3
    """
    How many of the object's grasps closest to the robot :meth:`rank_grasps` ranks.
    """

    grasp_rank: int = 0
    """
    Which of :meth:`rank_grasps` the robot takes, from 0 for the closest.
    """

    def scene_specification(self) -> WorldSpecification:
        """
        The floor and :attr:`table`, without the counter of
        :meth:`MilkTransportDemonstration.scene_specification`.
        """
        return WorldSpecification(objects=[self.floor, self.table])

    def is_scene_populated(self, world: World) -> bool:
        """
        Whether every one of :attr:`objects_on_table` is in ``world``, see
        :meth:`RobotDemonstration.is_scene_populated`.
        """
        return all(
            world.is_kinematic_structure_entity_in_world_by_name(scattered.mesh._name_)
            for scattered in self.objects_on_table
        )

    def populate_scene(self, world: World) -> None:
        """
        Stand each of :attr:`objects_on_table` upright on its spot of :attr:`layout`,
        free to be picked up.
        """
        bodies = {}
        for scattered in self.objects_on_table:
            name = scattered.mesh._name_
            annotation = SemanticAnnotationWithRootSpecification(
                name=name,
                semantic_annotation_type=scattered.semantic_annotation_type,
                root_specification=BodySpecification.mesh(
                    name,
                    str(scattered.mesh),
                    connection_specification=Connection6DoFSpecification(),
                ),
            ).spawn(world)
            ContactParameters.create_for_grasped_object().apply_to([annotation.root])
            bodies[name] = annotation.root
        table_box = self._table_box(world=world)
        spots = self.layout.sample_spots(
            table_box=table_box,
            footprints={
                name: Footprint.from_body(body=body) for name, body in bodies.items()
            },
        )
        for name, body in bodies.items():
            body.parent_connection.origin = (
                spots[name]
                .compute_object_pose(table_top=float(table_box.max_z), world=world)
                .homogeneous_matrix
            )

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Raise the torso, park the arm, walk to the edge nearest the object, grasp it by
        the grasp of :attr:`grasp_rank` among :meth:`rank_grasps` and park the arm
        again, holding the object up.
        """
        hsrb = context.require_extension(extension_type=RobotAccess).robot
        graspable = self.find_graspable(world=context.world)
        [arm] = hsrb.all_arms
        standing_pose = self.compute_standing_pose(graspable=graspable)
        statechart = Statechart(context=context)
        statechart.add_node(
            node=Sequence(
                nodes=[
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                    ParkArmsAction(arms=hsrb.all_arms),
                    NavigateAction(target_location=standing_pose),
                    PickUpAction(
                        grasp=self.rank_grasps(
                            graspable=graspable, standing_pose=standing_pose
                        )[self.grasp_rank],
                        arm=arm,
                    ),
                    ParkArmsAction(arms=hsrb.all_arms),
                ]
            )
        )
        return statechart

    def rank_grasps(
        self, *, graspable: HasGraspCandidates, standing_pose: Pose
    ) -> list[GraspCandidate]:
        """
        :param graspable: An object on the table.
        :param standing_pose: Where the robot stands while it grasps ``graspable``.
        :return: The :attr:`number_of_grasps` grasps of ``graspable`` closest to
            ``standing_pose``, the closest first, each moved to
            :meth:`move_to_grasp_height`.
        """
        return [
            self.move_to_grasp_height(grasp=grasp)
            for grasp in GraspsByCloseness(
                graspable=graspable,
                standing_position=standing_pose,
                number_of_grasps=self.number_of_grasps,
            )()
        ]

    def compute_standing_pose(self, *, graspable: HasGraspCandidates) -> Pose:
        """
        :param graspable: An object on the table.
        :return: Where the robot's base stands on the floor to grasp ``graspable``:
            :attr:`base_distance_from_edge` outside the approach edge nearest it, in line
            with it and facing it.
        """
        world = graspable.root._world
        table_box = self._table_box(world=world)
        object_box = graspable.root.collision.as_bounding_box_collection_in_frame(
            world.root
        ).bounding_box()
        center = np.array(
            [
                (float(object_box.min_x) + float(object_box.max_x)) / 2,
                (float(object_box.min_y) + float(object_box.max_y)) / 2,
            ]
        )
        edge = self.layout.nearest_approach_edge(table_box=table_box, point=center)
        normal = edge.outward_normal()
        depth = edge.inward_distance(table_box=table_box, point=center)
        standing_position = center + normal * (depth + self.base_distance_from_edge)
        return Pose.from_xyz_rpy(
            x=float(standing_position[0]),
            y=float(standing_position[1]),
            yaw=float(np.arctan2(-normal[1], -normal[0])),
            reference_frame=world.root,
        )

    def _table_box(self, *, world: World) -> AxisAlignedBox:
        """
        :return: The bounding box of :attr:`supporting_surface` in the world frame.
        """
        return (
            world.get_body_by_name(self.supporting_surface)
            .collision.as_bounding_box_collection_in_frame(world.root)
            .bounding_box()
        )


# %% trying every object


class TableGraspFlag(StrEnum):
    """
    The options this script understands on its command line.
    """

    ON_TABLE = "--on-table"
    """
    The objects scattered on the table; every object if left out.
    """

    SEED = "--seed"
    """
    The seed of the layout; a random one, which is printed, if left out.
    """

    HEADLESS = "--headless"
    """
    Run the simulation without a viewer.
    """

    GRASP_RANK = "--grasp-rank"
    """
    Run a single attempt in this process, with the grasp of this rank by closeness;
    without it, the closest grasps are tried in turn, each in a process of its own.
    """


@dataclass(kw_only=True)
class TableGraspSurvey(pick_up_objects.PickUpSurvey):
    """
    Tries to grasp each of :attr:`objects` off the table of :attr:`objects_on_table`
    laid out by :attr:`seed` by its closest grasps in turn, each attempt in a process of
    its own, until one of them lifts the object.
    """

    objects_on_table: list[pick_up_objects.PickUpObject] = field(default_factory=list)
    """
    The objects scattered on the table in every attempt.
    """

    seed: int = 0
    """
    The seed of the layout every attempt shares.
    """

    number_of_grasps: int = TableGraspDemonstration.number_of_grasps
    """
    How many of an object's closest grasps are tried before it counts as not lifted.
    """

    def attempt_command(
        self, *, pick_up_object: pick_up_objects.PickUpObject
    ) -> list[str]:
        """
        The command line of :meth:`PickUpSurvey.attempt_command`, which also passes
        :attr:`seed` and :attr:`objects_on_table` on.
        """
        return [
            *super().attempt_command(pick_up_object=pick_up_object),
            TableGraspFlag.SEED,
            str(self.seed),
            TableGraspFlag.ON_TABLE,
            *(scattered.mesh._name_ for scattered in self.objects_on_table),
        ]

    def try_object(
        self, *, pick_up_object: pick_up_objects.PickUpObject
    ) -> pick_up_objects.PickUpOutcome:
        """
        Try the grasps of :meth:`TableGraspDemonstration.rank_grasps` in turn, see
        :meth:`PickUpSurvey.try_object`.

        :return: How the first attempt that lifted the object ended, or the last
            attempt if none did.
        """
        for grasp_rank in range(self.number_of_grasps):
            outcome = self.run_attempt(
                pick_up_object=pick_up_object,
                command=[
                    *self.attempt_command(pick_up_object=pick_up_object),
                    TableGraspFlag.GRASP_RANK,
                    str(grasp_rank),
                ],
            )
            verdict = "lifted" if outcome.succeeded else f"failed: {outcome.error}"
            print(f"  grasp {grasp_rank}: {verdict}", flush=True)
            if outcome.succeeded:
                return outcome
        return outcome


# %% running the demonstration


def grasp_from_table(
    *,
    pick_up_object: pick_up_objects.PickUpObject,
    objects_on_table: list[pick_up_objects.PickUpObject],
    seed: int,
    grasp_rank: int,
    headless: bool = False,
) -> World:
    """
    Try to grasp a single object off the table.

    :param pick_up_object: The object to grasp, one of ``objects_on_table``.
    :param objects_on_table: The objects scattered on the table.
    :param seed: The seed of the layout.
    :param grasp_rank: Which of the object's grasps by closeness the robot takes.
    :param headless: Whether the physics runs without a viewer.
    :return: The world the object was grasped in.
    :raises ObjectNotLifted: If the robot did not lift the object.
    """
    demonstration = TableGraspDemonstration(
        headless=headless,
        pick_up_object=pick_up_object,
        objects_on_table=objects_on_table,
        layout=RandomTableLayout(seed=seed),
        grasp_rank=grasp_rank,
    )
    world = demonstration.run()
    demonstration.check_lifted(world)
    return world


def parse_command_line() -> argparse.Namespace:
    """
    :return: The objects to grasp and the options of this script's command line.
    """
    names = [member._name_ for member in pick_up_objects.ObjectMesh]
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "to_grasp", nargs="*", choices=names, metavar="OBJECT", default=[]
    )
    parser.add_argument(TableGraspFlag.ON_TABLE, nargs="+", choices=names, default=[])
    parser.add_argument(TableGraspFlag.SEED, type=int)
    parser.add_argument(TableGraspFlag.HEADLESS, action="store_true")
    parser.add_argument(TableGraspFlag.GRASP_RANK, type=int)
    return parser.parse_args()


def main() -> None:
    """
    Scatter the objects on the table and try to grasp each one named on the command
    line, or every one on the table if none is named, or run the single attempt of
    :attr:`TableGraspFlag.GRASP_RANK`.
    """
    arguments = parse_command_line()
    from_name = pick_up_objects.PickUpObject.from_mesh_name
    objects_on_table = [
        from_name(name) for name in arguments.on_table
    ] or pick_up_objects.PickUpSurvey().objects
    to_grasp = [from_name(name) for name in arguments.to_grasp]
    objects_on_table += [
        pick_up_object
        for pick_up_object in to_grasp
        if pick_up_object not in objects_on_table
    ]
    to_grasp = to_grasp or objects_on_table
    seed = (
        arguments.seed
        if arguments.seed is not None
        else int(np.random.default_rng().integers(1_000_000))
    )
    print(f"Layout seed: {seed}", flush=True)
    if arguments.grasp_rank is not None:
        [pick_up_object] = to_grasp
        grasp_from_table(
            pick_up_object=pick_up_object,
            objects_on_table=objects_on_table,
            seed=seed,
            grasp_rank=arguments.grasp_rank,
            headless=arguments.headless,
        )
        return
    survey = TableGraspSurvey(
        objects=to_grasp,
        objects_on_table=objects_on_table,
        seed=seed,
        headless=arguments.headless,
    )
    survey.print_report(survey.run())


if __name__ == "__main__":
    main()
