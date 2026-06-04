import logging
import os
from copy import deepcopy
from dataclasses import dataclass

import threading
from typing import Optional, Any, Tuple, Sequence

import rclpy
from rclpy.executors import SingleThreadedExecutor

from old_SORT_OUT.simulation_setup import SpawnSpec
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.abstract_robot import Manipulator, ParallelGripper
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Milk,
    Cereal,
    Bowl,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.connections import OmniDrive
from semantic_digital_twin.world_description.geometry import Color
from semantic_digital_twin.world_description.world_entity import Body
from suturo_resources.suturo_map import load_environment

from pycram.datastructures.dataclasses import Context
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    ModelSynchronizer,
    StateSynchronizer,
)
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.world import World


logger = logging.getLogger(__name__)


def _here(*parts: str) -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), *parts))


@dataclass(frozen=True)
class SetupResult:
    world: World
    robot_view: HSRB
    context: Context
    manipulator: Manipulator
    node: Any
    viz: Optional[object] = None


@dataclass(frozen=True)
class WorldSetupPaths:
    hsrb_urdf: str
    milk_stl: str
    cereal_stl: str
    bowl_stl: str
    cup_stl: str


@dataclass(frozen=True)
class WorldSetup_xyz_rpy:
    robot: Tuple[float, float, float, float, float, float]
    milk: Tuple[float, float, float, float, float, float]
    cereal: Tuple[float, float, float, float, float, float]
    bowl: Tuple[float, float, float, float, float, float]
    cup: Tuple[float, float, float, float, float, float]


def default_paths() -> WorldSetupPaths:
    return WorldSetupPaths(
        hsrb_urdf=_here("..", "..", "..", "resources", "robots", "hsrb.urdf"),
        milk_stl=_here("..", "..", "..", "resources", "objects", "milk.stl"),
        cereal_stl=_here(
            "..", "..", "..", "resources", "objects", "breakfast_cereal.stl"
        ),
        bowl_stl=_here("..", "..", "..", "resources", "objects", "bowl.stl"),
        cup_stl=_here("..", "..", "..", "resources", "objects", "jeroen_cup.stl"),
    )


def default_xyz_rpy():
    return WorldSetup_xyz_rpy(
        milk=(
            1.0,
            6,
            0.78,
            0.0,
            0.0,
            0.0,
        ),
        cereal=(
            0.6,
            6.3,
            0.805,
            0.0,
            0.0,
            0.0,
        ),
        bowl=(
            1.5,
            6.3,
            0.725,
            0.0,
            0.0,
            0.0,
        ),
        cup=(
            1.5,
            6.7,
            0.72,
            0.0,
            0.0,
            0.0,
        ),
        robot=(
            0,
            0,
            0.0,
            0.0,
            0.0,
            0.0,
        ),
    )


def add_objects_and_semantics(
    world,
    objects: Sequence[SpawnSpec],
):
    for spec in objects:
        obj_world = STLParser(spec.world_path).parse()
        x, y, z, r, p, yaw = spec.xyz_rpy
        world.merge_world_at_pose(
            obj_world,
            HomogeneousTransformationMatrix.from_xyz_rpy(
                x, y, z, r, p, yaw, reference_frame=world.root
            ),
        )

    with world.modify_world():
        world.add_semantic_annotation(Milk(root=world.get_body_by_name("milk.stl")))
        for c in world.get_body_by_name("milk.stl").visual.shapes:
            c.color = Color.RED()
        world.add_semantic_annotation(
            Cereal(root=world.get_body_by_name("breakfast_cereal.stl"))
        )
        for c in world.get_body_by_name("breakfast_cereal.stl").visual.shapes:
            c.color = Color.BLUE()
        world.add_semantic_annotation(Bowl(root=world.get_body_by_name("bowl.stl")))
        for c in world.get_body_by_name("bowl.stl").visual.shapes:
            c.color = Color.GREEN()
        world.add_semantic_annotation(
            Bowl(root=world.get_body_by_name("jeroen_cup.stl"))
        )
        for c in world.get_body_by_name("jeroen_cup.stl").visual.shapes:
            c.color = Color.ORANGE()
    return world


def build_hsrb_world(hsrb_urdf: str):
    world = URDFParser.from_file(file_path=hsrb_urdf).parse()
    with world.modify_world():
        odom = Body(name=PrefixedName("odom_combined"))
        world.add_kinematic_structure_entity(odom)
        omni_drive = OmniDrive.create_with_dofs(
            parent=odom, child=world.root, world=world
        )
        world.add_connection(omni_drive)
        omni_drive.has_hardware_interface = True
    return world


def try_make_viz(world: World, node: Any):
    try:
        from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
            VizMarkerPublisher,
        )

        viz = VizMarkerPublisher(_world=world, node=node)
        viz.with_tf_publisher()
        return viz
    except Exception as e:
        logger.info(f"VizMarkerPublisher unavailable: {e}")
        return None


def merge_robot_into_environment(
    hsrb_world,
    environment_world,
    robot_xyz_rpy: Tuple[float, float, float, float, float, float],
) -> World:
    x, y, z, r, p, yaw = robot_xyz_rpy
    environment_world.merge_world_at_pose(
        deepcopy(hsrb_world),
        HomogeneousTransformationMatrix.from_xyz_rpy(x, y, z, r, p, yaw),
    )
    return environment_world


def real_setup(node_name: str = "pycram_node"):
    node = rclpy.create_node(node_name)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Start executor in a separate thread
    thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-executor")
    thread.start()

    hsrb_world: World = fetch_world_from_service(node)
    model_sync = ModelSynchronizer(_world=hsrb_world, node=node)
    state_sync = StateSynchronizer(_world=hsrb_world, node=node)

    env_world = load_environment()
    with hsrb_world.modify_world():
        hsrb_world.merge_world(env_world)
    robot_view = hsrb_world.get_semantic_annotations_by_type(HSRB)[0]

    manipulator: Manipulator = hsrb_world.get_semantic_annotations_by_type(
        ParallelGripper
    )[0]

    # Context
    context = Context(
        world=hsrb_world,
        robot=robot_view,
        ros_node=node,
    )

    viz = try_make_viz(hsrb_world, node)
    return SetupResult(
        world=hsrb_world,
        robot_view=robot_view,
        context=context,
        manipulator=manipulator,
        node=node,
        viz=viz,
    )


def simulated_setup(node_name: str = "pycram_node"):
    # creating node
    node: Any = rclpy.create_node(node_name)
    # default coordinates for the robot
    # building robot and world
    hsrb_world = build_hsrb_world(default_paths().hsrb_urdf)
    env_world: World = load_environment()

    p = default_paths()

    xyz_rpy = default_xyz_rpy()

    # feel free to comment in, if you are still working with these simulated objects
    env_world = add_objects_and_semantics(
        env_world,
        objects=(
            SpawnSpec(world_path=p.milk_stl, xyz_rpy=xyz_rpy.milk),
            SpawnSpec(world_path=p.cereal_stl, xyz_rpy=xyz_rpy.cereal),
            SpawnSpec(world_path=p.bowl_stl, xyz_rpy=xyz_rpy.bowl),
            SpawnSpec(world_path=p.cup_stl, xyz_rpy=xyz_rpy.cup),
        ),
    )

    # Merging the robot into the world and retrieving robot, manipulator and misc.
    hsrb_world = merge_robot_into_environment(
        hsrb_world, env_world, robot_xyz_rpy=xyz_rpy.robot
    )
    # fetching robot_view, manipulator to create context
    robot_view = HSRB.from_world(hsrb_world)
    manipulator = hsrb_world.get_semantic_annotations_by_type(ParallelGripper)[0]
    context = Context(world=hsrb_world, robot=robot_view, ros_node=node)

    # trying to initialize VizMarker, if it fails it just returns a error message
    viz = try_make_viz(hsrb_world, node)

    return SetupResult(
        world=hsrb_world,
        robot_view=robot_view,
        context=context,
        manipulator=manipulator,
        node=node,
        viz=viz,
    )


def robot_setup() -> SetupResult:
    try:
        setup_result = real_setup()
    except Exception as e:
        logger.info(f"Real setup failed: {e}")

    logger.info(f"Falling back to simulation")
    setup_result = simulated_setup()

    return setup_result
