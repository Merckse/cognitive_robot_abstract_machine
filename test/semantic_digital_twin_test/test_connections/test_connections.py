import numpy as np
import pytest
from numpy.testing import assert_allclose

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import (
    HomogeneousTransformationMatrix,
    Vector3,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connection_properties import JointDynamics
from semantic_digital_twin.world_description.connections import (
    Connection6DoF,
    DifferentialDrive,
    FixedConnection,
    OmniDrive,
    RevoluteConnection,
)
from semantic_digital_twin.world_description.world_entity import Body


def _two_body_world() -> tuple[World, Body, Body]:
    """Builds a world with a parent and a child body, ready to receive a connection."""
    world = World()
    parent = Body(name=PrefixedName("parent"))
    child = Body(name=PrefixedName("child"))
    return world, parent, child


class TestCreateWithDofsInterface:
    """The ``create_with_dofs`` factory shares one call shape across all connection types."""

    def test_common_keyword_only_prefix(self):
        world, parent, child = _two_body_world()
        with world.modify_world():
            connection = Connection6DoF.create_with_dofs(
                world, parent, child, name=PrefixedName("free")
            )
            world.add_connection(connection)
        assert connection.name == PrefixedName("free")

    def test_one_dof_threads_parent_T_connection_expression(self):
        world, parent, child = _two_body_world()
        parent_T_connection = HomogeneousTransformationMatrix.from_xyz_rpy(x=0.3, y=0.4)
        with world.modify_world():
            connection = RevoluteConnection.create_with_dofs(
                world,
                parent,
                child,
                axis=Vector3.Z(),
                parent_T_connection_expression=parent_T_connection,
            )
            world.add_connection(connection)
        assert_allclose(connection.origin.to_np(), parent_T_connection.to_np(), atol=1e-9)

    def test_axis_is_keyword_only(self):
        world, parent, child = _two_body_world()
        with pytest.raises(TypeError):
            RevoluteConnection.create_with_dofs(world, parent, child, Vector3.Z())

    def test_name_is_keyword_only(self):
        world, parent, child = _two_body_world()
        with pytest.raises(TypeError):
            Connection6DoF.create_with_dofs(world, parent, child, PrefixedName("x"))


class TestHasHardwareInterfaceSymmetry:
    """The getter reflects exactly what the setter writes: any active dof's hardware interface."""

    def _omni_drive(self) -> OmniDrive:
        world, parent, child = _two_body_world()
        with world.modify_world():
            connection = OmniDrive.create_with_dofs(world, parent, child)
            world.add_connection(connection)
        return connection

    def _differential_drive(self) -> DifferentialDrive:
        world, parent, child = _two_body_world()
        with world.modify_world():
            connection = DifferentialDrive.create_with_dofs(world, parent, child)
            world.add_connection(connection)
        return connection

    @pytest.mark.parametrize("connection_factory", ["_omni_drive", "_differential_drive"])
    def test_setter_getter_round_trip(self, connection_factory):
        connection = getattr(self, connection_factory)()
        assert not connection.has_hardware_interface
        assert connection.controlled_dofs == []

        connection.has_hardware_interface = True
        assert connection.has_hardware_interface
        assert set(connection.controlled_dofs) == set(connection.active_dofs)

        connection.has_hardware_interface = False
        assert not connection.has_hardware_interface
        assert connection.controlled_dofs == []

    @pytest.mark.parametrize("connection_factory", ["_omni_drive", "_differential_drive"])
    def test_getter_reflects_any_active_dof(self, connection_factory):
        connection = getattr(self, connection_factory)()
        connection.yaw.has_hardware_interface = True
        assert connection.has_hardware_interface


class TestDofsDeterminism:
    """``dofs`` is the ordered concatenation of active and passive dofs."""

    def test_omni_drive_dofs_order(self):
        world, parent, child = _two_body_world()
        with world.modify_world():
            connection = OmniDrive.create_with_dofs(world, parent, child)
            world.add_connection(connection)
        assert connection.dofs == connection.active_dofs + connection.passive_dofs


class TestJointProperty:
    def test_default_values(self):
        joint_dynamics = JointDynamics()
        assert_allclose(joint_dynamics.armature, 0.0)
        assert_allclose(joint_dynamics.dry_friction, 0.0)
        assert_allclose(joint_dynamics.damping, 0.0)

    def test_custom_values(self):
        armature = 1.5
        dry_friction = 0.2
        damping = 0.05
        joint_dynamics = JointDynamics(
            armature=armature, dry_friction=dry_friction, damping=damping
        )
        assert_allclose(joint_dynamics.armature, armature)
        assert_allclose(joint_dynamics.dry_friction, dry_friction)
        assert_allclose(joint_dynamics.damping, damping)

        joint_prop_dict = joint_dynamics.__dict__
        assert_allclose(joint_prop_dict["armature"], armature)
        assert_allclose(joint_prop_dict["dry_friction"], dry_friction)
        assert_allclose(joint_prop_dict["damping"], damping)
