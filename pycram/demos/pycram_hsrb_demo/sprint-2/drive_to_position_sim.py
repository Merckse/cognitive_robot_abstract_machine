import logging
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from suturo_resources.suturo_map import load_environment

from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import NavigateActionDescription
from pycram.ros_utils.text_to_image import TextToImagePublisher

from simulation_setup_drive import setup_hsrb_in_environment

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class NavTarget:
    """Navigation target in world.root frame (map)."""

    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


def main() -> None:
    """HSRB simulation: navigate to one or more targets sequentially.

    This version mirrors the other simulation demos in `hsrb_simulation/`:
    - build a SemDT world (HSRB + environment)
    - create a PyCRAM Context
    - execute navigation sequentially via `SequentialPlan` (sequential nodes)

    No Nav2 action server is required.
    """

    logging.basicConfig(level=logging.INFO)

    if not rclpy.ok():
        rclpy.init()

    result = setup_hsrb_in_environment(load_environment=load_environment, with_viz=True)
    world = result.world  # SemDT world (has `root` at runtime)
    context = result.context

    targets: List[NavTarget] = [
        NavTarget(
            position=(3.8683114051818848, 5.459158897399902, 0.0),
            orientation=(0.0, 0.0, 0.04904329912700753, 0.9987966533838301),
        )
    ]

    text_pub = TextToImagePublisher()
    actions = []
    for i, t in enumerate(targets, start=1):
        text_pub.publish_text(f"[sim] Moving to target {i}")
        pose = PoseStamped.from_list(
            position=list(t.position),
            orientation=list(t.orientation),
            frame=getattr(world, "root"),
        )
        actions.append(NavigateActionDescription(target_location=pose))

    plan = SequentialPlan(context, *actions)

    with simulated_robot:
        plan.perform()


if __name__ == "__main__":
    main()
