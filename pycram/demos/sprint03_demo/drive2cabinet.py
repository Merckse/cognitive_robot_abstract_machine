import logging
from enum import Enum

from suturo_resources.suturo_map import load_environment

from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import NavigateActionDescription
from pycram.ros_utils.text_to_image import TextToImagePublisher

# Register HSRB motion mapping (side-effects).
from pycram.alternative_motion_mappings import hsrb_motion_mapping  # noqa: F401

logger = logging.getLogger(__name__)


class DriveToPosition(Enum):
    CABINET = PoseStamped.from_list(
        position=[3.8683114051818848, 5.459158897399902, 0.0],
        orientation=[0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
    )


def main(with_viz: bool = True) -> None:
    """Drive the robot to the cabinet pose inside the simulation world (no real robot / Nav2)."""

    try:
        from .simulation_setup import setup_hsrb_in_environment  # type: ignore
    except Exception:
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

    text_pub = TextToImagePublisher() if with_viz else None

    for pos in DriveToPosition:
        if text_pub:
            text_pub.publish_text(f"[SIM] Moving to: {pos.name}")

        plan = SequentialPlan(
            context,
            NavigateActionDescription(target_location=[pos.value]),
        )

        with simulated_robot:
            plan.perform()


if __name__ == "__main__":
    main(with_viz=True)
