"""
The real HSR looks at what is in front of it, adds the objects RoboKudo sees to the
world and picks up the milk among them from where it stands.

Run it from this folder, with the robot's controller and a RoboKudo pipeline that
answers object queries (such as ``live_yolo``) running::

    python perceive_and_pick_up_milk_demo.py
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from coraplex.plans.context_extensions import RobotAccess
from coraplex.querying.grasp_closeness import GraspsByCloseness
from coraplex.robot_plans.actions.composite.transporting import MoveAndPickUpAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction
from cramph.composites import Sequence
from cramph.statechart import Statechart
from exceptions import MilkNotSeen
from perception_demo import PerceptionDemonstration
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from coraplex.scene_perception import SceneObservation
    from cramph.context import StatechartContext
    from semantic_digital_twin.world import World

# %% the demonstration


@dataclass
class PerceiveAndPickUpMilkDemonstration(PerceptionDemonstration):
    """
    The real HSR adds what it sees to the world, as
    :class:`~perception_demo.PerceptionDemonstration` does, and then turns to the seen
    milk and picks it up without driving away from where it stands.
    """

    ros_node_name: str = "hsr_perceive_and_pick_up_milk_demo"
    """
    Name of the node this demonstration registers.
    """

    maximum_milk_width: float = 0.06
    """
    The widest horizontal extent, in meters, the seen milk is added to the world with,
    so the gripper fits around it whichever side it is grasped from.
    """

    def observe_scene(self, world: World) -> SceneObservation:
        """
        As :meth:`PerceptionDemonstration.observe_scene`, with each seen milk narrowed
        to at most :attr:`maximum_milk_width` along both horizontal axes.
        """
        observation = super().observe_scene(world)
        for observed in observation.objects:
            if (
                observed.dimensions is None
                or observed.semantic_annotation_type is not Milk
            ):
                continue
            observed.dimensions = replace(
                observed.dimensions,
                x=min(observed.dimensions.x, self.maximum_milk_width),
                y=min(observed.dimensions.y, self.maximum_milk_width),
            )
        return observation

    def perform_plan(self, world: World) -> None:
        """
        As :meth:`PerceptionDemonstration.perform_plan`, then pick up the seen milk with
        :meth:`build_pick_up_statechart`.

        :raises MilkNotSeen: If RoboKudo reported no milk with a pose and a size.
        """
        super().perform_plan(world)
        if not any(isinstance(seen, Milk) for seen in self.spawned_objects):
            raise MilkNotSeen(
                [observed.category for observed in self.observation.objects]
            )
        self.execute_statechart(world, self.build_pick_up_statechart)

    def build_pick_up_statechart(self, context: StatechartContext) -> Statechart:
        """
        :param context: The context the statechart is built in.
        :return: A statechart that turns the robot to the first milk this run added,
            looks at it and picks it up by its grasp closest to the robot, standing
            where the robot stands now.
        """
        robot = context.require_extension(RobotAccess).robot
        milk = next(seen for seen in self.spawned_objects if isinstance(seen, Milk))
        [arm] = robot.all_arms
        standing_pose = robot.root.global_pose
        [closest_grasp] = GraspsByCloseness(milk, standing_pose, number_of_grasps=1)()
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    MoveAndPickUpAction.from_standing_position(
                        standing_position=standing_pose, grasp=closest_grasp, arm=arm
                    )
                ]
            )
        )
        return statechart


# %% direct start


def main() -> World:
    """
    Run the demonstration.

    :return: The world the milk was picked up in.
    """
    return PerceiveAndPickUpMilkDemonstration().run()


if __name__ == "__main__":
    main()
