"""
Tests for the HSR carrying a milk carton from the counter to the kitchen island of the lab
in MuJoCo.
"""

from __future__ import annotations

import importlib.util
import sys
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.executors import (
    KinematicallySimulatedPlanExecutor,
    PhysicallySimulatedPlanExecutor,
    PlanExecutor,
)
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.statechart import Statechart
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.world import World

from ..pytest_environment import runs_in_continuous_integration

DEMO_FOLDER = (
    Path(__file__).parents[2] / "coraplex" / "demos" / "coraplex_hsr_mujoco_demo"
)
"""
The folder of the lab demo and of the wrapper CI can run it through.
"""

requires_hsr_description = pytest.mark.skipif(
    not HSRB.is_description_installed(),
    reason="The description of the HSR is not installed",
)
"""
Skips a test that runs the plan, which needs the robot in the world.
"""

runs_under_every_executor = pytest.mark.parametrize(
    "executor_type",
    [
        KinematicallySimulatedPlanExecutor,
        pytest.param(
            PhysicallySimulatedPlanExecutor,
            marks=pytest.mark.skipif(
                not runs_in_continuous_integration(),
                reason="MuJoCo tests only run in CI",
            ),
        ),
    ],
    ids=["kinematic", "physics"],
)
"""
Runs a test with the plan simulated kinematically and in physics, where only friction
holds the milk.
"""

# %% the demo, loaded from its folder


def load_script(file_name: str) -> ModuleType:
    """
    Import a script of the demo's folder, whose hyphenated name an import statement cannot
    spell.

    :param file_name: The name of the script in :data:`DEMO_FOLDER`.
    :return: The imported script.
    """
    specification = importlib.util.spec_from_file_location(
        f"hsr_mujoco_{Path(file_name).stem.replace('-', '_')}",
        DEMO_FOLDER / file_name,
    )
    module = importlib.util.module_from_spec(specification)
    with pytest.MonkeyPatch.context() as patch:
        patch.setitem(sys.modules, specification.name, module)
        specification.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def lab_demo() -> ModuleType:
    """
    The ``demo-lab`` script.
    """
    return load_script("demo-lab.py")


@pytest.fixture
def populated_lab(lab_demo: ModuleType) -> World:
    """
    The lab with the milk standing on the counter, without a robot.
    """
    world = KitchenEnvironment().create_world()
    lab_demo.LabMilkTransportDemonstration().populate_scene(world)
    return world


@pytest.fixture
def demonstration_ending_at_the_pick_up(
    lab_demo: ModuleType,
) -> type[RobotDemonstration]:
    """
    The demonstration with a plan that ends once the robot has picked the milk up, for
    the milk to be looked at while it is still held.
    """

    @dataclass
    class DemonstrationEndingAtThePickUp(lab_demo.LabMilkTransportDemonstration):
        def build_statechart(self, context: StatechartContext) -> Statechart:
            statechart = Statechart(context=context)
            statechart.add_node(Sequence(nodes=self.create_taking_steps(context)))
            return statechart

    return DemonstrationEndingAtThePickUp


# %% the scene before the plan


def test_the_milk_starts_with_its_lowest_point_on_the_counter(
    lab_demo: ModuleType, populated_lab: World
):
    demonstration = lab_demo.LabMilkTransportDemonstration()
    [milk] = populated_lab.get_semantic_annotations_by_type(Milk)
    counter = populated_lab.get_body_by_name(demonstration.milk_start.surface)

    milk_box = milk.root.collision.as_bounding_box_collection_in_frame(
        populated_lab.root
    ).bounding_box()
    counter_box = counter.collision.as_bounding_box_collection_in_frame(
        populated_lab.root
    ).bounding_box()

    assert float(milk_box.min_z) == pytest.approx(float(counter_box.max_z))


def test_the_milk_starts_at_its_spot_on_the_counter(
    lab_demo: ModuleType, populated_lab: World
):
    demonstration = lab_demo.LabMilkTransportDemonstration()
    [milk] = populated_lab.get_semantic_annotations_by_type(Milk)

    position = populated_lab.compute_forward_kinematics_np(
        populated_lab.root, milk.root
    )[:3, 3]

    assert list(position[:2]) == pytest.approx(
        [demonstration.milk_start.x, demonstration.milk_start.y]
    )


# %% the grasp


def test_the_grasp_is_where_the_upper_third_of_the_milk_begins(
    lab_demo: ModuleType, populated_lab: World
):
    [milk] = populated_lab.get_semantic_annotations_by_type(Milk)
    box = milk.root.collision.as_bounding_box_collection_in_frame(
        milk.root
    ).bounding_box()

    grasp = lab_demo.LabMilkTransportDemonstration().find_grasp_pose(milk.root)

    upper_third_begins_at = float(box.max_z) - (float(box.max_z) - float(box.min_z)) / 3
    assert [float(grasp.x), float(grasp.y), float(grasp.z)] == pytest.approx(
        [0.0, 0.0, upper_third_begins_at]
    )


# %% carrying the milk


@requires_hsr_description
@runs_under_every_executor
def test_the_milk_ends_where_it_is_set_down_on_the_kitchen_island(
    lab_demo: ModuleType, executor_type: type[PlanExecutor]
):
    demonstration = lab_demo.LabMilkTransportDemonstration(
        executor_type=executor_type, headless=True
    )

    world = demonstration.run()

    [milk] = world.get_semantic_annotations_by_type(Milk)
    set_down_at = demonstration.milk_target.compute_milk_pose(milk.root)
    position = world.compute_forward_kinematics_np(world.root, milk.root)[:3, 3]
    assert list(position) == pytest.approx(
        [float(set_down_at.x), float(set_down_at.y), float(set_down_at.z)], abs=0.02
    )


# %% a successful grasp


@requires_hsr_description
@runs_under_every_executor
def test_the_picked_up_milk_is_lifted_off_the_counter(
    demonstration_ending_at_the_pick_up: type[RobotDemonstration],
    executor_type: type[PlanExecutor],
):
    demonstration = demonstration_ending_at_the_pick_up(
        executor_type=executor_type, headless=True
    )

    world = demonstration.run()

    [milk] = world.get_semantic_annotations_by_type(Milk)
    counter = world.get_body_by_name(demonstration.milk_start.surface)
    milk_box = milk.root.collision.as_bounding_box_collection_in_frame(
        world.root
    ).bounding_box()
    counter_box = counter.collision.as_bounding_box_collection_in_frame(
        world.root
    ).bounding_box()
    assert float(milk_box.min_z) > float(counter_box.max_z)


@requires_hsr_description
@runs_under_every_executor
def test_the_picked_up_milk_stays_at_the_grasp_between_the_fingers(
    demonstration_ending_at_the_pick_up: type[RobotDemonstration],
    executor_type: type[PlanExecutor],
):
    demonstration = demonstration_ending_at_the_pick_up(
        executor_type=executor_type, headless=True
    )

    world = demonstration.run()

    [robot] = world.get_semantic_annotations_by_type(demonstration.used_robot)
    [arm] = robot.all_arms
    [milk] = world.get_semantic_annotations_by_type(Milk)
    tool_frame_goal = world.transform(
        arm.end_effector.tool_frame_goal_for_grasp(
            demonstration.find_grasp_pose(milk.root)
        ),
        world.root,
    )
    tool_frame_position = world.compute_forward_kinematics_np(
        world.root, arm.end_effector.tool_frame
    )[:3, 3]
    assert list(tool_frame_position) == pytest.approx(
        [
            float(tool_frame_goal.x),
            float(tool_frame_goal.y),
            float(tool_frame_goal.z),
        ],
        abs=0.02,
    )


# %% the wrapper CI can run the demo through


def test_the_wrapper_runs_the_lab_demo_without_a_viewer(
    monkeypatch: pytest.MonkeyPatch,
):
    wrapper = load_script("test_demo-lab.py")
    headless_arguments = []
    monkeypatch.setattr(
        wrapper.importlib,
        "import_module",
        lambda name: SimpleNamespace(
            main=lambda headless: headless_arguments.append(headless)
        ),
    )

    wrapper.main()

    assert headless_arguments == [True]


def test_the_wrapper_exits_non_zero_with_a_traceback_when_the_demo_fails(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
):
    wrapper = load_script("test_demo-lab.py")

    def fail_to_import(name: str) -> None:
        raise ImportError(name)

    monkeypatch.setattr(wrapper.importlib, "import_module", fail_to_import)

    with pytest.raises(SystemExit) as exit_information:
        wrapper.main()

    assert exit_information.value.code == 1
    assert "Traceback" in capsys.readouterr().err
