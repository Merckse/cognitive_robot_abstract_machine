import logging
import os
import unittest

import numpy as np
import rclpy

from demos.SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.events.event_handler import EventDispatcher
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.predetermined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import OmniDrive

logger = logging.getLogger(__name__)

try:
    from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
        VizMarkerPublisher,
    )
except ImportError:
    logger.info(
        "Could not import VizMarkerPublisher. This is probably because you are not running ROS."
    )


def setup_world():
    logger.setLevel(logging.DEBUG)

    hsrb_sem_world = URDFParser.from_file(
        os.path.join(
            os.path.dirname(__file__),
            "../..",
            "..",
            "resources",
            "robots",
            "hsrb.urdf",
        )
    ).parse()



    node = rclpy.create_node("kitchen_environment")
    publisher = VizMarkerPublisher(_world=KitchenEnvironment().get_world(), node=node, show_labels=True)
    (publisher.with_tf_publisher())
    apartment_world : World= publisher._world

    with apartment_world.modify_world():
        hsrb_root = hsrb_sem_world.get_body_by_name("base_footprint")
        apartment_root = apartment_world.root
        c_root_bf = OmniDrive.create_with_dofs(
            parent=apartment_root, child=hsrb_root, world=apartment_world
        )
        apartment_world.merge_world(hsrb_sem_world, c_root_bf)
        c_root_bf.origin = HomogeneousTransformationMatrix.from_xyz_rpy(0, 0, 0)

    print(apartment_world.bodies)
    dispatcher = EventDispatcher()

    return apartment_world, dispatcher