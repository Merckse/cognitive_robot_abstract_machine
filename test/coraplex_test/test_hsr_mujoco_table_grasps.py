"""
Tests for the HSR grasping objects scattered at random spots on a table in MuJoCo.
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass, field
from types import ModuleType

import numpy as np
import pytest
from coraplex.plans.executors import PlanExecutor
from coraplex.querying.grasp_closeness import GraspsByCloseness
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import AxisAlignedBox

from .test_hsr_mujoco_lab_demo import (
    DEMO_FOLDER,
    requires_hsr_description,
    runs_under_every_executor,
)

# %% the demo, loaded from its folder


@pytest.fixture(scope="module")
def table_grasps() -> ModuleType:
    """
    The ``demo-table-grasps`` script, which imports its neighbours in the demo folder by
    name.
    """
    with pytest.MonkeyPatch.context() as patch:
        patch.syspath_prepend(str(DEMO_FOLDER))
        return importlib.import_module("demo-table-grasps")


@pytest.fixture
def demonstration(table_grasps: ModuleType):
    """
    The demonstration with every object on the table and the milk to grasp.
    """
    pick_up_objects = table_grasps.pick_up_objects
    return table_grasps.TableGraspDemonstration(
        pick_up_object=pick_up_objects.PickUpObject.from_mesh_name(
            pick_up_objects.ObjectMesh.MILK._name_
        ),
        layout=table_grasps.RandomTableLayout(seed=3),
    )


@pytest.fixture
def populated_table(demonstration) -> World:
    """
    The table with every object of :func:`demonstration` scattered on it, without a
    robot.
    """
    return demonstration.build_scene()


@pytest.fixture
def table_box(demonstration, populated_table: World) -> AxisAlignedBox:
    """
    The bounding box of the demonstration's table in the world frame.
    """
    return demonstration._table_box(world=populated_table)


@pytest.fixture
def layout(table_grasps: ModuleType):
    """
    A layout with the default reach and spacing.
    """
    return table_grasps.RandomTableLayout(seed=5)


@pytest.fixture
def footprints(table_grasps: ModuleType) -> dict:
    """
    Footprints of a few objects of different sizes, by name.
    """
    return {
        name: table_grasps.Footprint(
            center_in_object=np.array([0.01, -0.02]),
            radius=radius,
            lowest_point_in_object=-0.05,
        )
        for name, radius in [("small", 0.05), ("medium", 0.1), ("large", 0.18)]
    }


# %% the random layout


def test_the_same_seed_gives_the_same_layout(
    layout, table_box: AxisAlignedBox, footprints: dict
):
    """
    Sampling a layout twice with one seed puts every object on the same spot.
    """

    first = layout.sample_spots(table_box=table_box, footprints=footprints)
    second = layout.sample_spots(table_box=table_box, footprints=footprints)

    assert {name: list(spot.center) for name, spot in first.items()} == {
        name: list(spot.center) for name, spot in second.items()
    }


def test_no_two_objects_come_closer_than_the_gap(
    layout, table_box: AxisAlignedBox, footprints: dict
):
    """
    No two objects of a layout come closer than the layout's gap.
    """

    spots = list(
        layout.sample_spots(table_box=table_box, footprints=footprints).values()
    )

    assert not any(
        spot.overlaps(other=other, gap=layout.gap)
        for index, spot in enumerate(spots)
        for other in spots[index + 1 :]
    )


def test_every_object_lies_within_reach_of_an_approach_edge(
    layout, table_box: AxisAlignedBox, footprints: dict
):
    """
    Every object lies within the reach depth of an edge the robot may approach from.
    """

    spots = layout.sample_spots(table_box=table_box, footprints=footprints)

    assert all(
        min(
            edge.inward_distance(table_box=table_box, point=spot.center)
            for edge in layout.approach_edges
        )
        <= layout.reach_depth
        for spot in spots.values()
    )


def test_every_object_keeps_the_margin_from_every_edge(
    table_grasps: ModuleType, layout, table_box: AxisAlignedBox, footprints: dict
):
    """
    No object's footprint comes closer than the edge margin to any edge of the table.
    """

    spots = layout.sample_spots(table_box=table_box, footprints=footprints)

    assert all(
        edge.inward_distance(table_box=table_box, point=spot.center)
        >= spot.footprint.radius + layout.edge_margin
        for spot in spots.values()
        for edge in table_grasps.TableEdge
    )


def test_a_table_without_room_within_reach_is_too_crowded(
    table_grasps: ModuleType, table_box: AxisAlignedBox, footprints: dict
):
    """
    A layout that leaves no spot within reach raises :class:`TableTooCrowded`.
    """

    layout = table_grasps.RandomTableLayout(seed=0, reach_depth=0.0, layout_attempts=2)

    with pytest.raises(table_grasps.TableTooCrowded):
        layout.sample_spots(table_box=table_box, footprints=footprints)


def test_the_object_pose_centers_its_footprint_on_the_spot(
    table_grasps: ModuleType, populated_table: World, footprints: dict
):
    """
    The pose of an object puts the center of its footprint on its spot, however it is turned.
    """

    spot = table_grasps.TableSpot(
        center=np.array([0.8, 0.3]), yaw=1.0, footprint=footprints["medium"]
    )

    pose = spot.compute_object_pose(table_top=0.7, world=populated_table)
    footprint_center = pose.homogeneous_matrix.to_np() @ np.array(
        [*spot.footprint.center_in_object, 0.0, 1.0]
    )

    assert list(footprint_center[:2]) == pytest.approx(list(spot.center))


# %% the scene before the plan


def test_every_object_starts_with_its_lowest_point_on_the_table(
    demonstration, populated_table: World, table_box: AxisAlignedBox
):
    """
    Every scattered object rests with its lowest point on the table's top.
    """

    lowest_points = [
        float(
            populated_table.get_body_by_name(scattered.mesh._name_)
            .collision.as_bounding_box_collection_in_frame(populated_table.root)
            .bounding_box()
            .min_z
        )
        for scattered in demonstration.objects_on_table
    ]

    assert lowest_points == pytest.approx([float(table_box.max_z)] * len(lowest_points))


def test_the_robot_stands_outside_the_near_edge_facing_the_object(
    demonstration, populated_table: World, table_box: AxisAlignedBox
):
    """
    The robot stands outside the near edge, in line with the object and facing it.
    """

    graspable = demonstration.find_graspable(world=populated_table)
    milk_box = graspable.root.collision.as_bounding_box_collection_in_frame(
        populated_table.root
    ).bounding_box()

    standing_pose = demonstration.compute_standing_pose(graspable=graspable)

    assert [
        float(standing_pose.x),
        float(standing_pose.y),
        float(standing_pose.yaw),
    ] == pytest.approx(
        [
            float(table_box.min_x) - demonstration.base_distance_from_edge,
            (float(milk_box.min_y) + float(milk_box.max_y)) / 2,
            0.0,
        ]
    )


# %% the grasps tried


def test_the_grasps_are_tried_in_the_order_of_their_closeness(
    demonstration, populated_table: World
):
    """
    The ranked grasps are the closest grasps to the robot, in the same order.
    """
    graspable = demonstration.find_graspable(world=populated_table)
    standing_pose = demonstration.compute_standing_pose(graspable=graspable)

    ranked = demonstration.rank_grasps(graspable=graspable, standing_pose=standing_pose)

    closest = GraspsByCloseness(
        graspable=graspable,
        standing_position=standing_pose,
        number_of_grasps=demonstration.number_of_grasps,
    )()
    assert [grasp.grasp_pose.to_np()[:3, :3].tolist() for grasp in ranked] == [
        grasp.grasp_pose.to_np()[:3, :3].tolist() for grasp in closest
    ]


def test_every_ranked_grasp_is_at_the_grasp_height(
    demonstration, populated_table: World
):
    """
    Every ranked grasp holds the object at the fraction of its height the demonstration
    asks for.
    """
    graspable = demonstration.find_graspable(world=populated_table)
    box = graspable.root.collision.as_bounding_box_collection_in_frame(
        graspable.root
    ).bounding_box()

    ranked = demonstration.rank_grasps(
        graspable=graspable,
        standing_pose=demonstration.compute_standing_pose(graspable=graspable),
    )

    grasp_height = float(box.min_z) + demonstration.fraction_of_height * (
        float(box.max_z) - float(box.min_z)
    )
    assert [float(grasp.grasp_pose.z) for grasp in ranked] == pytest.approx(
        [grasp_height] * demonstration.number_of_grasps
    )


# %% the survey


def test_every_attempt_of_the_survey_shares_the_seed_and_the_table(
    table_grasps: ModuleType,
):
    """
    Every attempt of the survey is given the survey's seed and the objects on its table.
    """

    on_table = table_grasps.pick_up_objects.PickUpSurvey().objects[:3]
    survey = table_grasps.TableGraspSurvey(
        objects=on_table[:1], objects_on_table=on_table, seed=42
    )

    command = survey.attempt_command(pick_up_object=on_table[0])

    seed_at = command.index(table_grasps.TableGraspFlag.SEED)
    on_table_at = command.index(table_grasps.TableGraspFlag.ON_TABLE)
    assert command[seed_at + 1] == str(survey.seed)
    assert command[on_table_at + 1 :] == [
        scattered.mesh._name_ for scattered in on_table
    ]


@pytest.fixture
def survey_lifting_at_rank(table_grasps: ModuleType) -> type:
    """
    A survey whose attempts only lift the object with the grasp of
    :attr:`lifting_rank`, and which records the rank of every attempt it starts.
    """
    pick_up_objects = table_grasps.pick_up_objects

    @dataclass(kw_only=True)
    class SurveyLiftingAtRank(table_grasps.TableGraspSurvey):
        """
        Records each attempt instead of running it.
        """

        lifting_rank: int | None
        """
        The rank of the only grasp that lifts the object, or ``None`` if none does.
        """

        tried_ranks: list[int] = field(default_factory=list)
        """
        The rank of every attempt started, in order.
        """

        def run_attempt(self, *, pick_up_object, command: list[str]):
            """
            Record the attempt's rank instead of running it, see
            :meth:`PickUpSurvey.run_attempt`.
            """
            rank_at = command.index(table_grasps.TableGraspFlag.GRASP_RANK) + 1
            rank = int(command[rank_at])
            self.tried_ranks.append(rank)
            return pick_up_objects.PickUpOutcome(
                pick_up_object, succeeded=rank == self.lifting_rank, error=str(rank)
            )

    return SurveyLiftingAtRank


@pytest.fixture
def milk_to_grasp(table_grasps: ModuleType):
    """
    The milk, as the object a survey tries to grasp.
    """
    pick_up_objects = table_grasps.pick_up_objects
    return pick_up_objects.PickUpObject.from_mesh_name(
        pick_up_objects.ObjectMesh.MILK._name_
    )


def test_the_survey_stops_at_the_first_grasp_that_lifts_the_object(
    survey_lifting_at_rank: type, milk_to_grasp
):
    """
    Once a grasp lifts the object, no further grasp of it is tried.
    """
    survey = survey_lifting_at_rank(objects=[milk_to_grasp], lifting_rank=1)

    outcome = survey.try_object(pick_up_object=milk_to_grasp)

    assert (outcome.succeeded, survey.tried_ranks) == (True, [0, 1])


def test_the_survey_tries_every_closest_grasp_before_giving_up(
    survey_lifting_at_rank: type, milk_to_grasp
):
    """
    An object no grasp lifts is tried with each of its closest grasps, and the outcome
    is that of the last of them.
    """
    survey = survey_lifting_at_rank(objects=[milk_to_grasp], lifting_rank=None)

    outcome = survey.try_object(pick_up_object=milk_to_grasp)

    every_rank = list(range(survey.number_of_grasps))
    assert (outcome.succeeded, outcome.error, survey.tried_ranks) == (
        False,
        str(every_rank[-1]),
        every_rank,
    )


# %% grasping an object off the table


@requires_hsr_description
@runs_under_every_executor
def test_the_milk_is_lifted_off_the_crowded_table(
    demonstration, executor_type: type[PlanExecutor]
):
    """
    The robot lifts the milk off the table among the other objects.
    """

    demonstration.executor_type = executor_type
    demonstration.headless = True

    world = demonstration.run()

    demonstration.check_lifted(world)
