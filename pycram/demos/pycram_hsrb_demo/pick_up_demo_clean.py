import logging
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

from object_creation import perceive_and_spawn_all_objects
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import (
    Arms,
    TorsoState,
    ApproachDirection,
    VerticalAlignment,
)
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.process_module import real_robot
from pycram.robot_plans import (
    LookAtActionDescription,
    ParkArmsActionDescription,
    MoveTorsoActionDescription,
    PickUpActionDescription,
)
from pycram.ros import VizMarkerPublisher
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    ModelSynchronizer,
    StateSynchronizer,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB
from pycram_ros_setup import setup_ros_node
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Vector3
from pycram.alternative_motion_mappings import hsrb_motion_mapping
from semantic_digital_twin.spatial_types.derivatives import DerivativeMap
from semantic_digital_twin.world_description.connections import PrismaticConnection
from semantic_digital_twin.world_description.degree_of_freedom import DegreeOfFreedom
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)

# Setup ROS node and fetch the world
node, hsrb_world = setup_ros_node()

# Setup context
context = Context(
    hsrb_world, hsrb_world.get_semantic_annotations_by_type(HSRB)[0], ros_node=node
)

# Perceive objects and spawn them
perceived_objects = perceive_and_spawn_all_objects(hsrb_world)
print(perceived_objects)

grasp = GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)

with hsrb_world.modify_world():
    box = Body(
        name=PrefixedName("milk"),
        collision=ShapeCollection([Box(scale=Scale(0.2, 0.1, 0.2))]),
    )
    dof = DegreeOfFreedom(
        lower_limits=DerivativeMap(data=[None, -1.0, None, None]),
        upper_limits=DerivativeMap(data=[None, 1.0, None, None]),
    )
    hsrb_world.add_degree_of_freedom(dof)
    connection = PrismaticConnection(
        dof_id=dof.id,
        parent=hsrb_world.root,
        child=box,
        axis=Vector3.Z(reference_frame=hsrb_world.root),
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=2, y=0.5, z=0.5, yaw=0
        ),
    )
    hsrb_world.add_connection(connection)

# plan1 = SequentialPlan(
#     context,
#     # LookAtActionDescription(
#     #     target=PoseStamped.from_spatial_type(
#     #         HomogeneousTransformationMatrix.from_xyz_rpy(x=1.0, y=6.22, z=0.8)
#     #     )
#     # ),
#     PickUpActionDescription(
#         arm=Arms.LEFT,
#         object_designator=hsrb_world.get_body_by_name("milk"),
#         grasp_description=grasp,
#     )
# )
#
# plan2 = SequentialPlan(
#     context,
#     ParkArmsActionDescription(arm=Arms.LEFT),
#     MoveTorsoActionDescription(TorsoState.HIGH),
# )
VizMarkerPublisher()
# Execute the plans
with real_robot:
    SequentialPlan(
        context,
        # ParkArmsActionDescription(arm=Arms.LEFT)).perform()
        PickUpActionDescription(
            arm=Arms.LEFT,
            object_designator=hsrb_world.get_body_by_name("milk"),
            grasp_description=grasp,
        ),
    ).perform()

    # plan1.perform()
    # Uncomment to execute plan2
    # plan2.perform()

# exit(0)
