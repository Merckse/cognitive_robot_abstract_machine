from dataclasses import field, dataclass
from typing import Optional, List, Union, Dict, DefaultDict, TypeVar

import numpy as np

import semantic_digital_twin.spatial_types.spatial_types as cas
from giskardpy.data_types.exceptions import (
    GoalInitalizationException,
    DuplicateNameException,
)
from giskardpy.god_map import god_map
from giskardpy.motion_statechart.data_types import LifeCycleValues
from giskardpy.motion_statechart.graph_node import MotionStatechartNode
from giskardpy.qp.constraint import DerivativeEqualityConstraint
from giskardpy.qp.constraint import (
    EqualityConstraint,
    InequalityConstraint,
    DerivativeInequalityConstraint,
)
from giskardpy.qp.weight_gain import QuadraticWeightGain, LinearWeightGain
from semantic_digital_twin.spatial_types.derivatives import Derivatives
from semantic_digital_twin.world_description.degree_of_freedom import DegreeOfFreedom


# quadratic_gains: List[QuadraticWeightGain] = field(default_factory=list, init=False)
# linear_weight_gains: List[LinearWeightGain] = field(
#     default_factory=list, init=False
# )

# def get_quadratic_gains(self) -> List[QuadraticWeightGain]:
#     return self.quadratic_gains
#
# def get_linear_gains(self) -> List[LinearWeightGain]:
#     return self.linear_weight_gains
#
# def add_quadratic_weight_gain(
#     self,
#     name: str,
#     gains: List[DefaultDict[Derivatives, Dict[DegreeOfFreedom, float]]],
# ):
#     q_gain = QuadraticWeightGain(name=name, gains=gains)
#     self.quadratic_gains.append(q_gain)
#
# def add_linear_weight_gain(
#     self,
#     name: str,
#     gains: List[DefaultDict[Derivatives, Dict[DegreeOfFreedom, float]]],
# ):
#     q_gain = LinearWeightGain(name=name, gains=gains)
#     self.linear_weight_gains.append(q_gain)
