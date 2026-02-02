from copy import deepcopy

import pytest

from pycram.datastructures.dataclasses import Context
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import MoveTorsoAction, MoveTorsoActionDescription
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.stretch import Stretch
from semantic_digital_twin.robots.tiago import Tiago
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix


@pytest.fixture(scope="session", params=["hsrb", "stretch", "tiago"])
def setup_multi_robot_apartment(
    request, hsr_world_setup, stretch_world, tiago_world, apartment_world_setup
):
    apartment_copy = deepcopy(apartment_world_setup)

    if request.param == "hsrb":
        hsr_copy = deepcopy(hsr_world_setup)
        apartment_copy.merge_world_at_pose(
            hsr_copy, HomogeneousTransformationMatrix.from_xyz_rpy(1.5, 2, 0)
        )
        view = HSRB.from_world(apartment_copy)
        return apartment_copy, view
    elif request.param == "stretch":
        stretch_copy = deepcopy(stretch_world)
        apartment_copy.merge_world_at_pose(
            stretch_copy, HomogeneousTransformationMatrix.from_xyz_rpy(1.5, 2, 0)
        )
        view = Stretch.from_world(apartment_copy)
        return apartment_copy, view

    elif request.param == "tiago":
        tiago_copy = deepcopy(tiago_world)
        apartment_copy.merge_world_at_pose(
            tiago_copy, HomogeneousTransformationMatrix.from_xyz_rpy(1.5, 2, 0)
        )
        view = Tiago.from_world(apartment_copy)
        return apartment_copy, view


@pytest.fixture
def immutable_multiple_robot_apartment(setup_multi_robot_apartment):
    world, view = setup_multi_robot_apartment
    state = deepcopy(world.state.data)
    yield world, view, Context(world, view)
    world.state.data = state


def test_move_torso_multi_test(immutable_multiple_robot_apartment):
    world, view, context = immutable_multiple_robot_apartment
    plan = SequentialPlan(context, MoveTorsoActionDescription(TorsoState.HIGH))

    with simulated_robot:
        plan.perform()

    joint_state = view.torso.get_joint_state_by_type(TorsoState.HIGH)

    for connection, target in joint_state.items():
        assert connection.position == pytest.approx(target, abs=0.01)
