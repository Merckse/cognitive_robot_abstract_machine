from dataclasses import dataclass, field

from giskardpy.motion_statechart.context import BuildContext
from giskardpy.motion_statechart.goals.templates import Sequence
from giskardpy.motion_statechart.graph_node import Goal, NodeArtifacts
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from pycram.datastructures.pose import PoseStamped
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)


@dataclass(repr=False, eq=False)
class PickUp(Goal):
    tip_link: KinematicStructureEntity = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)
        Sequence([PrePose, Grasping, CloseHand, PullUp])

    def build(self, context: BuildContext) -> NodeArtifacts:
        return super().build(context)


@dataclass(repr=False, eq=False)
class PrePose(Goal):
    tip_link: KinematicStructureEntity = field(kw_only=True)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)

        pose = PoseStamped()

        pose = CartesianPose(
            root_link=context.world.root, tip_link=self.tip_link, goal_pose=pose
        )

        self.add_node(pose)

    def build(self, context: BuildContext) -> NodeArtifacts:
        return super().build(context)


@dataclass(repr=False, eq=False)
class Grasping(Goal):
    tip_link: KinematicStructureEntity = field(kw_only=True)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)

    def build(self, context: BuildContext) -> NodeArtifacts:
        return super().build(context)


@dataclass(repr=False, eq=False)
class CloseHand(Goal):
    tip_link: KinematicStructureEntity = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)

    def build(self, context: BuildContext) -> NodeArtifacts:
        return super().build(context)


@dataclass(repr=False, eq=False)
class PullUp(Goal):
    tip_link: KinematicStructureEntity = field(kw_only=True)
    ft: bool = field(kw_only=True, default=False)

    def expand(self, context: BuildContext) -> None:
        super().expand(context)

    def build(self, context: BuildContext) -> NodeArtifacts:
        return super().build(context)
