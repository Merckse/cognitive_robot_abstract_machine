"""
The real HSR moves its wrist to a joint position through Giskard, leaving every other
joint where it is.

Run it from this folder, with the robot's controller running::

    python move_wrist.py
"""

from __future__ import annotations

from dataclasses import dataclass

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import RealRobotPlanExecutor
from cramph.composites import Sequence
from cramph.statechart import Statechart
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
from semantic_digital_twin.datastructures.joint_state import JointState
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.robots.hsrb import HSRB, HSRBJoint
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from coraplex.plans.executors import PlanExecutor
    from cramph.context import ContextExtension, StatechartContext
    from semantic_digital_twin.robots.robot_parts import AbstractRobot
    from semantic_digital_twin.world import World

# %% the demonstration


@dataclass
class MoveWristDemonstration(RobotDemonstration):
    """
    The real HSR sends a joint goal for its wrist alone to Giskard.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The robot whose wrist moves.
    """

    ros_node_name: str = "hsr_move_wrist_demo"
    """
    Name of the node this demonstration registers.
    """

    executor_type: type[PlanExecutor] = RealRobotPlanExecutor
    """
    The executor running the plan, which drives the real robot.
    """

    wrist_joint: HSRBJoint = HSRBJoint.WRIST_ROLL
    """
    The wrist joint that moves.
    """

    wrist_position: float = -1.0
    """
    The position the wrist joint is driven to, in radians.
    """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(world_source=KitchenEnvironment())

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(semantic_annotation_type=self.used_robot)

    def is_scene_populated(self, world: World) -> bool:
        return not world.is_empty()

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        wrist_goal = JointState.from_str_dict(
            {self.wrist_joint: self.wrist_position}, context.world
        )
        wrist_goal2 = JointState.from_str_dict(
            {self.wrist_joint: -self.wrist_position}, context.world
        )
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(nodes=[JointPositionList(goal_state=wrist_goal, threshold=0.001),
                            JointPositionList(goal_state=wrist_goal2, threshold=0.001)])
        )
        return statechart


def main() -> World:
    """
    Run the demonstration.

    :return: The world the wrist was moved in.
    """
    return MoveWristDemonstration().run()


if __name__ == "__main__":
    main()
