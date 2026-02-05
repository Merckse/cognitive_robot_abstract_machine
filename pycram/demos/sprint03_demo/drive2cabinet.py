import logging
from enum import Enum

from suturo_resources.suturo_map import load_environment

from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.process_module import real_robot
from pycram.robot_plans import NavigateActionDescription

logger = logging.getLogger(__name__)


class DriveToPosition(Enum):
    CABINET = PoseStamped.from_list(
        position=[3.8683114051818848, 5.459158897399902, 0.0],
        orientation=[0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
    )


def _require_ros2() -> None:
    """Fail fast with a helpful message if we're not running in a ROS2 env."""
    try:
        import rclpy  # noqa: F401
    except Exception as e:
        raise RuntimeError(
            "This demo is meant to run on the real robot and requires a ROS2 environment. "
            "Make sure you've sourced your ROS2 workspace (e.g. `source /opt/ros/<distro>/setup.bash` and your overlay) "
            "and that `rclpy` is importable in this Python environment."
        ) from e


def main(with_viz: bool = True) -> None:
    _require_ros2()

    try:
        from simulation_setup import setup_hsrb_in_environment  # type: ignore
    except Exception:
        # Fallback to the canonical wrapper used by the HSRB demos.
        from simulation_setup import (
            setup_hsrb_in_environment,
        )

    result = setup_hsrb_in_environment(
        load_environment=load_environment, with_viz=with_viz
    )
    _world, _robot_view, context, _viz = (
        result.world,
        result.robot_view,
        result.context,
        result.viz,
    )

    text_pub = None
    if with_viz:
        try:
            from pycram.ros_utils.text_to_image import TextToImagePublisher

            text_pub = TextToImagePublisher()
        except Exception:
            logger.info(
                "TextToImagePublisher unavailable. Continuing without overlay text."
            )

    for pos in DriveToPosition:
        if text_pub:
            text_pub.publish_text(f"[REAL] Moving to: {pos.name}")

        plan = SequentialPlan(
            context,
            NavigateActionDescription(target_location=[pos.value]),
        )

        with real_robot:
            plan.perform()


if __name__ == "__main__":
    main(with_viz=True)
