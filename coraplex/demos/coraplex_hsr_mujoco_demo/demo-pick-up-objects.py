"""
The HSR tries to pick up each object of the coraplex resources from the counter of
:mod:`demo`, to show which of them it can lift.

Each object is tried in a process of its own, so one that fails cannot affect the next;
the viewer opens for each object in turn. Run the whole survey from this folder::

    python demo-pick-up-objects.py

or try a single object by naming it::

    python demo-pick-up-objects.py BOWL

Add ``--headless`` to either to run without a viewer.
"""

from __future__ import annotations

import re
import subprocess
import sys
from copy import deepcopy
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

from coraplex.plans.context_extensions import RobotAccess
from coraplex.querying.grasp_closeness import GraspsByCloseness
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.statechart import Statechart
from krrood.exceptions import DataclassException
from krrood.patterns.path_enumeration import PathEnumeration
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Bottle,
    Bowl,
    Bread,
    Cereal,
    Container,
    Cup,
    Knife,
    Milk,
    Spoon,
    Whisk,
)
from semantic_digital_twin.specifications.connections import (
    Connection6DoFSpecification,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.semantic_annotations import (
    SemanticAnnotationWithRootSpecification,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.contact import ContactParameters
from typing_extensions import Self

from demo import MilkTransportDemonstration, SceneBody

# %% the objects


class ObjectMesh(PathEnumeration):
    """
    The meshes of the objects under the coraplex resources that are given in meters.

    ``apartment_bowl.stl`` is left out: it is given in millimeters, which an STL file
    cannot declare, so it would load a thousand times too large.
    """

    _ignore_ = ["folder"]
    folder = Path(__file__).resolve().parents[2] / "resources" / "objects"

    COKE_BOTTLE = folder / "Static_CokeBottle.stl"
    """
    A bottle of coke.
    """

    MILK_PITCHER = folder / "Static_MilkPitcher.stl"
    """
    A small jug for milk.
    """

    BIG_KNIFE = folder / "big-knife.stl"
    """
    A kitchen knife, lying flat.
    """

    BOWL = folder / "bowl.stl"
    """
    A bowl.
    """

    BREAD = folder / "bread.stl"
    """
    A loaf of bread.
    """

    BREAKFAST_CEREAL = folder / "breakfast_cereal.stl"
    """
    A box of breakfast cereal.
    """

    CUP = folder / "jeroen_cup.stl"
    """
    A cup.
    """

    MILK = folder / "milk.stl"
    """
    A milk carton.
    """

    SPOON = folder / "spoon.stl"
    """
    A spoon, lying flat.
    """

    WHISK = folder / "whisk.stl"
    """
    A whisk, lying flat.
    """


@dataclass
class PickUpObject:
    """
    An object the robot tries to pick up: its mesh and what it is.
    """

    mesh: ObjectMesh
    """
    The mesh the object is shaped by.
    """

    semantic_annotation_type: type[HasGraspCandidates]
    """
    What the object is, which decides the grasps it offers.
    """

    @classmethod
    def from_mesh_name(cls, mesh_name: str) -> Self:
        """
        :param mesh_name: The name of a member of :class:`ObjectMesh`.
        :return: The object of :class:`PickUpSurvey`'s default objects shaped by that
            mesh.
        """
        [pick_up_object] = [
            pick_up_object
            for pick_up_object in PickUpSurvey().objects
            if pick_up_object.mesh._name_ == mesh_name
        ]
        return pick_up_object


class CommandFlag(StrEnum):
    """
    The flags this script understands on its command line.
    """

    HEADLESS = "--headless"
    """
    Run the simulation without a viewer.
    """


# %% picking up one object


@dataclass
class ObjectNotLifted(DataclassException):
    """
    The plan finished, but the object does not hang above the counter.
    """

    mesh_name: str
    """
    The name of the object's mesh.
    """

    lowest_point_height: float
    """
    How high the object's lowest point is above the counter's top, in meters.
    """

    def error_message(self) -> str:
        return (
            f"The {self.mesh_name} was not lifted: its lowest point is "
            f"{self.lowest_point_height:.3f} m above the counter."
        )

    def suggest_correction(self) -> str:
        return "Watch the run with a viewer to see whether the grasp slips or misses."


@dataclass
class ObjectPickUpDemonstration(MilkTransportDemonstration):
    """
    The HSR takes an object from the counter in front of it and holds it up with its arm
    parked.
    """

    ros_node_name: str = "hsr_object_pick_up_demo"
    """
    Name of the node this demonstration registers.
    """

    pick_up_object: PickUpObject = field(
        default_factory=lambda: PickUpObject(ObjectMesh.MILK, Milk)
    )
    """
    The object the robot picks up.
    """

    fraction_of_height: float = 0.5
    """
    How far up the object's bounding box the hand takes hold of it, from 0 at its bottom
    to 1 at its top.
    """

    lift_height: float = 0.02
    """
    How far above :attr:`supporting_surface` the object's lowest point has to hang for it
    to count as lifted, in meters.
    """

    supporting_surface: str = SceneBody.COUNTER
    """
    The name of the body the object stands on before it is picked up.
    """

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(
            self.pick_up_object.mesh._name_
        )

    def populate_scene(self, world: World) -> None:
        """
        Stand the object on the counter, free to be picked up.
        """
        name = self.pick_up_object.mesh._name_
        graspable = SemanticAnnotationWithRootSpecification(
            name=name,
            semantic_annotation_type=self.pick_up_object.semantic_annotation_type,
            root_specification=BodySpecification.mesh(
                name,
                str(self.pick_up_object.mesh),
                connection_specification=Connection6DoFSpecification(),
            ),
        ).spawn(world)
        ContactParameters.create_for_grasped_object().apply_to([graspable.root])
        graspable.root.parent_connection.origin = self._milk_pose_on(
            world.get_body_by_name(SceneBody.COUNTER), graspable
        ).homogeneous_matrix

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Raise the torso, park the arm, take the object from the counter and park the arm
        again, holding the object up.
        """
        world = context.world
        hsrb = context.require_extension(extension_type=RobotAccess).robot
        graspable = self.find_graspable(world=world)
        [arm] = hsrb.all_arms
        taking_pose = self._standing_pose_facing(
            self._milk_pose_on(world.get_body_by_name(SceneBody.COUNTER), graspable)
        )
        statechart = Statechart(context=context)
        statechart.add_node(
            node=Sequence(
                nodes=[
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                    ParkArmsAction(arms=hsrb.all_arms),
                    NavigateAction(target_location=taking_pose),
                    PickUpAction(
                        grasp=self.find_grasp(graspable, taking_pose), arm=arm
                    ),
                    ParkArmsAction(arms=hsrb.all_arms),
                ]
            )
        )
        return statechart

    def find_grasp(
        self, graspable: HasGraspCandidates, standing_pose: Pose
    ) -> GraspCandidate:
        """
        :param graspable: The object to take, standing upright.
        :param standing_pose: Where the robot stands while it takes the object.
        :return: The object's grasp closest to `standing_pose`, moved to
            :meth:`move_to_grasp_height`.
        """
        [closest] = GraspsByCloseness(
            graspable=graspable, standing_position=standing_pose, number_of_grasps=1
        )()
        return self.move_to_grasp_height(grasp=closest)

    def move_to_grasp_height(self, *, grasp: GraspCandidate) -> GraspCandidate:
        """
        :param grasp: A grasp of an upright object, given in the object's frame.
        :return: ``grasp`` moved up or down to :attr:`fraction_of_height` of the
            object's bounding box, keeping its position across the object and its
            orientation.
        """
        graspable = grasp.graspable
        box = graspable.root.collision.as_bounding_box_collection_in_frame(
            graspable.root
        ).bounding_box()
        grasp_pose = deepcopy(grasp.grasp_pose)
        grasp_pose.z = box.min_z + self.fraction_of_height * (box.max_z - box.min_z)
        return GraspCandidate(graspable=graspable, grasp_pose=grasp_pose)

    def find_graspable(self, *, world: World) -> HasGraspCandidates:
        """
        :param world: The world the scene was populated in.
        :return: The semantic annotation of :attr:`pick_up_object`, found by the name of
            its mesh.
        """
        [graspable] = [
            annotation
            for annotation in world.get_semantic_annotations_by_type(
                self.pick_up_object.semantic_annotation_type
            )
            if annotation.root.name.name == self.pick_up_object.mesh._name_
        ]
        return graspable

    def check_lifted(self, world: World) -> None:
        """
        :param world: The world the plan ran in.
        :raises ObjectNotLifted: If the object's lowest point hangs less than
            :attr:`lift_height` above :attr:`supporting_surface`.
        """
        graspable = self.find_graspable(world=world)
        counter_top = (
            world.get_body_by_name(self.supporting_surface)
            .collision.as_bounding_box_collection_in_frame(world.root)
            .bounding_box()
            .max_z
        )
        lowest_point = (
            graspable.root.collision.as_bounding_box_collection_in_frame(world.root)
            .bounding_box()
            .min_z
        )
        lowest_point_height = float(lowest_point - counter_top)
        if lowest_point_height < self.lift_height:
            raise ObjectNotLifted(
                mesh_name=self.pick_up_object.mesh._name_,
                lowest_point_height=lowest_point_height,
            )


# %% trying every object


@dataclass
class PickUpOutcome:
    """
    How the attempt to pick up one object ended.
    """

    pick_up_object: PickUpObject
    """
    The object the robot tried to pick up.
    """

    succeeded: bool
    """
    Whether the robot lifted the object off the counter.
    """

    error: str
    """
    The last exception the failed attempt reported, empty when it succeeded.
    """


@dataclass
class PickUpSurvey:
    """
    Tries to pick up each of :attr:`objects` in a process of its own and reports which of
    them the robot lifted.
    """

    objects: list[PickUpObject] = field(
        default_factory=lambda: [
            PickUpObject(ObjectMesh.COKE_BOTTLE, Bottle),
            PickUpObject(ObjectMesh.MILK_PITCHER, Container),
            PickUpObject(ObjectMesh.BIG_KNIFE, Knife),
            PickUpObject(ObjectMesh.BOWL, Bowl),
            PickUpObject(ObjectMesh.BREAD, Bread),
            PickUpObject(ObjectMesh.BREAKFAST_CEREAL, Cereal),
            PickUpObject(ObjectMesh.CUP, Cup),
            PickUpObject(ObjectMesh.MILK, Milk),
            PickUpObject(ObjectMesh.SPOON, Spoon),
            PickUpObject(ObjectMesh.WHISK, Whisk),
        ]
    )
    """
    The objects the robot tries to pick up, in order.
    """

    headless: bool = False
    """
    Whether each attempt runs without a viewer.
    """

    def attempt_command(self, *, pick_up_object: PickUpObject) -> list[str]:
        """
        :param pick_up_object: The object one attempt tries to pick up.
        :return: The command line that runs that attempt in a process of its own.
        """
        return [
            sys.executable,
            __file__,
            pick_up_object.mesh._name_,
            *([CommandFlag.HEADLESS] if self.headless else []),
        ]

    def run(self) -> list[PickUpOutcome]:
        """
        :return: How the attempt to pick up each of :attr:`objects` ended, in order.
        """
        outcomes = []
        for pick_up_object in self.objects:
            print(f"Trying to pick up {pick_up_object.mesh._name_} ...", flush=True)
            outcomes.append(self.try_object(pick_up_object=pick_up_object))
        return outcomes

    def try_object(self, *, pick_up_object: PickUpObject) -> PickUpOutcome:
        """
        :param pick_up_object: The object to pick up.
        :return: How the single attempt of :meth:`attempt_command` ended.
        """
        return self.run_attempt(
            pick_up_object=pick_up_object,
            command=self.attempt_command(pick_up_object=pick_up_object),
        )

    @staticmethod
    def run_attempt(
        *, pick_up_object: PickUpObject, command: list[str]
    ) -> PickUpOutcome:
        """
        :param pick_up_object: The object the attempt tries to pick up.
        :param command: The command line that runs the attempt in a process of its own.
        :return: How the attempt ended, with the last exception it reported if it failed.
        """
        attempt = subprocess.run(command, capture_output=True, text=True, check=False)
        if attempt.returncode == 0:
            return PickUpOutcome(pick_up_object, succeeded=True, error="")
        exception_lines = re.findall(
            r"^(?!Suggestion: )[A-Za-z_][\w.]*: .*$",
            attempt.stderr,
            flags=re.MULTILINE,
        )
        return PickUpOutcome(
            pick_up_object,
            succeeded=False,
            error=(exception_lines or [f"exit code {attempt.returncode}"])[-1],
        )

    @staticmethod
    def print_report(outcomes: list[PickUpOutcome]) -> None:
        """
        Print which objects the robot lifted and why the others failed.

        :param outcomes: How the attempts ended.
        """
        print("\nPick-up survey:")
        for outcome in outcomes:
            verdict = "lifted" if outcome.succeeded else "FAILED"
            print(
                f"  {outcome.pick_up_object.mesh._name_:18s} {verdict:7s} {outcome.error}"
            )


# %% running the demonstration


def pick_up(pick_up_object: PickUpObject, headless: bool = False) -> World:
    """
    Try to pick up a single object.

    :param pick_up_object: The object to pick up.
    :param headless: Whether the physics runs without a viewer.
    :return: The world the object was picked up in.
    :raises ObjectNotLifted: If the robot did not lift the object.
    """
    demonstration = ObjectPickUpDemonstration(
        headless=headless, pick_up_object=pick_up_object
    )
    world = demonstration.run()
    demonstration.check_lifted(world)
    return world


def main() -> None:
    """
    Try every object, or the single object named on the command line.
    """
    arguments = sys.argv[1:]
    headless = CommandFlag.HEADLESS in arguments
    mesh_names = [
        argument for argument in arguments if argument != CommandFlag.HEADLESS
    ]
    if not mesh_names:
        PickUpSurvey.print_report(PickUpSurvey(headless=headless).run())
        return
    pick_up(PickUpObject.from_mesh_name(mesh_names[0]), headless=headless)


if __name__ == "__main__":
    main()
