import os

from pycram.datastructures.dataclasses import Context
from pycram.motion_executor import simulated_robot, MotionExecutor
from pycram.datastructures.pose import PoseStamped
import logging

from pycram.language import SequentialPlan
from pycram.robot_plans import NavigateActionDescription
from pycram_suturo_demos.helper_methods_and_useful_classes.nlp_human_robot_interaction import (
    TalkingNode,
)
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.world import World

from pycram.external_interfaces import nav2_move

logger = logging.getLogger(__name__)


def robot_move(
    target_pose_method: PoseStamped,
    context: Context,
    simulated: bool = True,
    frame_id: str = "map",
):
    """
    Sends a navigation goal to Nav2.
    """
    if not simulated:
        os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"
        goal = target_pose_method.ros_message()
        print(f"Moving to {goal}'")
        nav2_move.start_nav_to_pose(goal)
    else:
        logger.info("Navigation failed")

        logger.info("Falling back on simulated robot")
        with simulated_robot:
            SequentialPlan(
                context,
                NavigateActionDescription(
                    target_location=target_pose_method, keep_joint_states=True
                ),
            ).perform()


def move_demo(target_pose: str, world: World, context: Context, simulated: bool = True):
    logger = logging.getLogger(__name__)
    talking_node = TalkingNode()
    standard_delay = 2

    CABINET = PoseStamped.from_list(
        position=[3.8683114051818848, 5.459158897399902, 0.0],
        orientation=[0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
        frame=world.root,
    )
    POPCORN_TABLE = PoseStamped.from_list(
        position=[1.3, 5.3, 0.0],
        orientation=[0.0, 0.0, 0.72, 0.64],
        frame=world.root,
    )
    # TODO edit on robocup to new table pose
    TABLE = PoseStamped.from_list(
        position=[1.3, 5.3, 0.0],
        orientation=[0.0, 0.0, 0.72, 0.64],
        frame=world.root,
    )
    ROBOT_PRE_START_POSE = PoseStamped.from_list(
        position=[1.3, 0.0, 0.0],
        orientation=[0.0, 0.0, -0.72, 0.64],
        frame=world.root,
    )
    ROBOT_PRE_START_POSE_TO_TABLE = PoseStamped.from_list(
        position=[1.3, 0.0, 0.0],
        orientation=[0.0, 0.0, 0, 1],
        frame=world.root,
    )
    ROBOT_START_POSE = PoseStamped.from_list(
        position=[0.0, 0.0, 0.0],
        orientation=[0, 0, 0, 1],
        frame=world.root,
    )
    PERCEPTION_ANGLE_0 = PoseStamped.from_list(
        position=[0.0, 0.0, 0.0],
        orientation=[0, 0, 0, 1],
        frame=world.root,
    )
    PERCEPTION_ANGLE_1 = PoseStamped.from_list(
        position=[0.0, 0.0, 0.0],
        orientation=[0, 0, 0, 1],
        frame=world.root,
    )
    PERCEPTION_ANGLE_2 = PoseStamped.from_list(
        position=[0.0, 0.0, 0.0],
        orientation=[0, 0, 0, 1],
        frame=world.root,
    )

    match target_pose:
        case "TABLE":
            logger.info("Moving to table")
            talking_node.pub(text="Now moving to table", delay=standard_delay)
            robot_move(target_pose_method=TABLE, context=context, simulated=simulated)
        case "ROBOT_START_POSE":
            logger.info("Moving to robot start pose")

            talking_node.pub(text="Now moving to starting pose", delay=standard_delay)
            robot_move(
                target_pose_method=ROBOT_PRE_START_POSE,
                context=context,
                simulated=simulated,
            )
            robot_move(
                target_pose_method=ROBOT_START_POSE,
                context=context,
                simulated=simulated,
            )
        case "CABINET":
            logger.info("Moving to cabinet")
            talking_node.pub(text="Now moving to cabinet", delay=standard_delay)
            robot_move(target_pose_method=CABINET, context=context, simulated=simulated)
        case "POPCORN_TABLE":
            logger.info("Moving to popcorn table")
            talking_node.pub(text="Now moving to popcorn table", delay=standard_delay)
            robot_move(
                target_pose_method=POPCORN_TABLE, context=context, simulated=simulated
            )
        case "PERCEPTION_ANGLE_0":
            logger.info("Moving to perception angle 0")
            talking_node.pub(
                text="Now moving to perception angle 0", delay=standard_delay
            )
            robot_move(
                target_pose_method=PERCEPTION_ANGLE_0,
                context=context,
                simulated=simulated,
            )
        case "PERCEPTION_ANGLE_1":
            logger.info("Moving to perception angle 1")
            talking_node.pub(
                text="Now moving to perception angle 1", delay=standard_delay
            )
            robot_move(
                target_pose_method=PERCEPTION_ANGLE_1,
                context=context,
                simulated=simulated,
            )
        case "PERCEPTION_ANGLE_2":
            logger.info("Moving to perception angle 2")
            talking_node.pub(
                text="Now moving to perception angle 2", delay=standard_delay
            )
            robot_move(
                target_pose_method=PERCEPTION_ANGLE_2,
                context=context,
                simulated=simulated,
            )
        case _:
            logger.info("Moving to robot start pose")
            talking_node.pub(text="Now moving to starting pose", delay=standard_delay)
            robot_move(
                target_pose_method=ROBOT_PRE_START_POSE,
                context=context,
                simulated=simulated,
            )
