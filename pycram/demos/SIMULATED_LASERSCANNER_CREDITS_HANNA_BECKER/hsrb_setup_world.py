import logging
import os
from typing import Tuple

import rclpy

from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Milk,
)
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import OmniDrive
from semantic_digital_twin.predetermined_maps.kitchen_environment import KitchenEnvironment

from demos.SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.events.event_handler import EventDispatcher, update_perceived_objects



logger = logging.getLogger(__name__)

try:
    from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
        VizMarkerPublisher,
    )
except ImportError:
    logger.info(
        "Could not import VizMarkerPublisher. This is probably because you are not running ROS."
    )

## apartment lab #######################################################################################################
rclpy.init()
node = rclpy.create_node("kitchen_environment")
publisher = VizMarkerPublisher(_world=KitchenEnvironment().get_world(), node=node)
publisher.with_tf_publisher()


########################################################################################################################





def hsrb_setup_world() -> Tuple[World, EventDispatcher]:
    """
    return
    world : the world
    dispatcher : the dispatcher for event handling
    """

    logger.setLevel(logging.DEBUG)

    hsrb_sem_world = URDFParser.from_file(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "resources",
            "robots",
            "hsrb.urdf",
        )
    ).parse()




    # milk_world = STLParser(
    #     os.path.join(
    #         os.path.dirname(__file__), "../..", "resources", "objects", "milk.stl"
    #     )
    # ).parse()
    # cereal_world = STLParser(
    #     os.path.join(
    #         os.path.dirname(__file__),
    #         "../..",
    #         "resources",
    #         "objects",
    #         "breakfast_cereal.stl",
    #     )
    # ).parse()
    # apartment_world.merge_world(pr2_sem_world)
    # apartment_world.merge_world(milk_world)
    # apartment_world.merge_world(cereal_world)

    node = rclpy.create_node("kitchen_environment")
    publisher = VizMarkerPublisher(_world=KitchenEnvironment().get_world(), node=node)
    (publisher.with_tf_publisher())
    apartment_world = publisher._world

    with apartment_world.modify_world():
        hsrb_root = hsrb_sem_world.get_body_by_name("base_footprint")
        apartment_root = apartment_world.root
        c_root_bf = OmniDrive.create_with_dofs(
            parent=apartment_root, child=hsrb_root, world=apartment_world
        )
        apartment_world.merge_world(hsrb_sem_world, c_root_bf)
        c_root_bf.origin = HomogeneousTransformationMatrix.from_xyz_rpy(1.5, 2.5, 0)

    print(apartment_world.bodies)

    # apartment_world.get_body_by_name("milk.stl").parent_connection.origin = (
    #     HomogeneousTransformationMatrix.from_xyz_rpy(
    #         2.37, 2, 1.05, reference_frame=apartment_world.root
    #     )
    # )
    # apartment_world.get_body_by_name(
    #     "breakfast_cereal.stl"
    # ).parent_connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
    #     2.3, 6.2, 1, reference_frame=apartment_world.root
    # )
    # milk_view = Milk(root=apartment_world.get_body_by_name("milk.stl"))
    # with apartment_world.modify_world():
    #     apartment_world.add_semantic_annotation(milk_view)

    dispatcher = EventDispatcher()
    # dispatcher.add_listener(update_perceived_objects)


    return apartment_world, dispatcher
