from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from datetime import timedelta

from semantic_digital_twin.world_description.world_entity import SemanticAnnotation
from typing_extensions import Union, Optional, Type, Any, Iterable

from ..core.misc import DetectActionDescription
from ..core.navigation import LookAtActionDescription, NavigateActionDescription
from ....datastructures.enums import DetectionTechnique
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....designators.location_designator import CostmapLocation
from ....failures import PerceptionObjectNotFound
from ....has_parameters import has_parameters
from ....language import TryInOrderPlan, SequentialPlan
from ....robot_plans.actions.base import ActionDescription

@has_parameters
@dataclass
class CountAction(ActionDescription):
    raise NotImplementedError("This feature is not implemented yet")

@has_parameters
@dataclass
class DescribingAction(ActionDescription):
    raise NotImplementedError("This feature is not implemented yet")

