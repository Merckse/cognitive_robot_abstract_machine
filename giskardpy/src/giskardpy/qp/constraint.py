from __future__ import annotations
from dataclasses import dataclass, field

import krrood.symbolic_math.symbolic_math as sm
from giskardpy.motion_statechart.data_types import FloatEnum
from krrood.symbolic_math.symbolic_math import Scalar
from semantic_digital_twin.spatial_types.derivatives import Derivatives


class NormalizationFactors(FloatEnum):
    meter_per_second = 0.2
    radian_per_second = 0.2


@dataclass
class Bound: ...


@dataclass
class EqualityBound(Bound):
    bound: Scalar


@dataclass
class InequalityBound(Bound):
    lower_bound: Scalar
    upper_bound: Scalar


@dataclass
class EnforcementStrategy: ...


@dataclass
class PositionStrategy(EnforcementStrategy): ...


@dataclass
class IntegralStrategy(EnforcementStrategy):
    """
    Equality constraints have the form:
    .. math::
        f(q) = b

    where

    .. math::

        target - f = \Delta t \sum_{k=0}^{N-1} J_{f} * sp_k

    ::

        |   t1   |   t2   |   t1   |   t2   |   t1   |   t2   | prediction horizon
        |v1 v2 v3|v1 v2 v3|j1 j2 j3|j1 j2 j3|s1 s2 s3|s1 s2 s3| free variables / slack
        |-----------------------------------------------------|
        |  J1*sp |  J1*sp |  J3*sp | J3*sp  | sp     | sp     |
        |-----------------------------------------------------|
    """

    def create_matrix(self, constraints: list[GiskardConstraint]) -> Matrix:
        if len(constraints) == 0:
            return sm.Matrix()
        jacobian = (
            sm.Vector([c.expression for c in constraints]).jacobian(
                variables=self.position_variables
            )
            * self.config.mpc_dt
        )
        return sm.hstack(
            [jacobian for _ in range(self.config.velocity_horizon)]
            + [sm.Matrix.zeros(jacobian.shape[0], self.number_of_jerk_columns)]
        )

    def create_slack_matrix(self, constraints: list[GiskardConstraint]) -> Matrix:
        if len(constraints) == 0:
            return sm.Matrix()
        return sm.Matrix.diag([self.config.mpc_dt for _ in constraints])

    def create_slack_variables(
        self, constraints: list[GiskardConstraint]
    ) -> DirectLimits:
        return SlackLimits.from_constraints(
            constraints=constraints,
            config=self.config,
        )

    def _apply_cap(
        self,
        value: Scalar,
        dt: float,
        normalization_number: float,
        control_horizon: int,
    ) -> Scalar:
        # todo normalization with jacobian???
        return sm.limit(
            value,
            -normalization_number * dt * control_horizon,
            normalization_number * dt * control_horizon,
        )

    def capped_bound(
        self,
        equality_bound: Scalar,
        dt: float,
        normalization_number: float,
        control_horizon: int,
    ) -> Scalar:
        return self._apply_cap(
            equality_bound, dt, normalization_number, control_horizon
        )

    def create_bounds(
        self, bounds: list[Scalar], normalization_numbers: list[float]
    ) -> Vector:
        return Vector(
            [
                self.capped_bound(
                    bound,
                    self.config.mpc_dt,
                    normalization_number,
                    self.config.velocity_horizon,
                )
                for bound, normalization_number in zip(bounds, normalization_numbers)
            ]
        )


@dataclass
class VelocityStrategy(EnforcementStrategy):
    """
    Constrains velocity at each step in the horizon.
    """


@dataclass
class GiskardConstraint:
    """
    Defines a (slack-relaxed) constraint on expression for a quadratic program.
    """

    name: str

    expression: Scalar

    bound: Bound

    lower_slack_limit: sm.ScalarData
    upper_slack_limit: sm.ScalarData

    quadratic_weight: sm.ScalarData

    linear_weight: sm.ScalarData

    normalization_factor: NormalizationFactors
    """
    This value is important to make constraints with different units comparable.
    The meaning depends on derivative.
    If the derivative is position, the normalization factor is rough velocity with which the expression can change.
    For example:
        - If you have a joint position constraint, the normalization factor should be the joint velocity limit.
        - If you have a cartesian position constraint, the normalization factor should be the cartesian velocity limit.
    In practice, use joint limits from the URDF for joint space constraints and define two values for cartesian constraints:
        - a m/s limit for translation
        - a rad/s value for rotation
    """

    enforcement_strategy: EnforcementStrategy


@dataclass
class IntegralConstraint(GiskardConstraint): ...


@dataclass
class InequalityConstraint(IntegralConstraint):
    """
    Adds
    capped_lower_error(lower_error) <= expression * control_horizon + slack <= capped_upper_error(upper_error)
    lower_slack_limit <= slack <= upper_slack_limit
    """

    lower_error: sm.ScalarData
    upper_error: sm.ScalarData

    lower_slack_limit: sm.ScalarData
    upper_slack_limit: sm.ScalarData

    def capped_lower_error(self, dt: float, control_horizon: int) -> Scalar:
        return self._apply_cap(self.lower_error, dt, control_horizon)

    def capped_upper_error(self, dt: float, control_horizon: int) -> Scalar:
        return self._apply_cap(self.upper_error, dt, control_horizon)


@dataclass
class EqualityConstraint(IntegralConstraint):
    """ """

    bound: sm.ScalarData


@dataclass
class DerivativeConstraint(GiskardConstraint):
    derivative: Derivatives = field(kw_only=True)
    """
    The constraint will be applied to the derivative of the expression.
    Position constraints are implemented by constraining the integral of the expressions' derivative over a prediction horizon.
    All other constraints are applied directly to that derivative of the expression.
    As a result, position constraints are cheaper, as they only require a single constraint.
    """

    # normalization_factor: sm.ScalarData = field(kw_only=True)
    """
    This value is important to make constraints with different units comparable.
    The meaning depends on derivative.
    If the derivative is position, the normalization factor is rough velocity with which the expression can change.
    For example:
        - If you have a joint position constraint, the normalization factor should be the joint velocity limit.
        - If you have a cartesian position constraint, the normalization factor should be the cartesian velocity limit.
    For other derivatives, the normalization factor is the same unit as the expression.
    For example:
        - Joint velocity constraint -> joint velocity limit
        - Cartesian velocity constraint -> cartesian velocity limit
    .. Warning: This number is different from the bounds of the expression. 
                If you want to enforce a bound below the actual limit, the normalization factor should still be the true limit.
    In practice, use joint limits from the URDF for joint space constraints and define two values for cartesian constraints:
        - a m/s limit for translation
        - a rad/s value for rotation
    """

    def normalized_weight(self) -> float:
        return self.quadratic_weight * (1 / self.normalization_factor) ** 2


@dataclass
class DerivativeInequalityConstraint(DerivativeConstraint):
    lower_limit: sm.ScalarData
    upper_limit: sm.ScalarData

    lower_slack_limit: sm.ScalarData
    upper_slack_limit: sm.ScalarData


@dataclass
class DerivativeEqualityConstraint(DerivativeConstraint):
    bound: sm.ScalarData
    lower_slack_limit: sm.ScalarData
    upper_slack_limit: sm.ScalarData
