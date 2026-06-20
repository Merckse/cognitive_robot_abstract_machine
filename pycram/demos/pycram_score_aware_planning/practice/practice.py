from common.hsrb_testing import setup_world
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram.plans.factories import sequential
from pycram.robot_plans.actions.core.navigation import NavigateAction
from pycram.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.semantic_annotations.semantic_annotations import Bowl
from semantic_digital_twin.spatial_types.spatial_types import Pose

world, dispatcher, hsrb = setup_world()


context = Context(world.root, hsrb)
plan = sequential(
    [
        NavigateAction(
            Pose.from_xyz_quaternion(
                1.6, 1.9, 0, 0, 0, 0, 1, reference_frame=world.root
            ),
            True,
        ),
        MoveTorsoAction(TorsoState.HIGH),
        ParkArmsAction(Arms.BOTH),
    ],
    context=context,
)