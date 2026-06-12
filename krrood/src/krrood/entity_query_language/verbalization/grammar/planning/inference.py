"""
Inference-rule **planner** — pure structural analysis of an
:class:`~krrood.entity_query_language.query.query.Entity` whose selected variable is an
:class:`~krrood.entity_query_language.core.variable.InstantiatedVariable` (an inference
rule) into a :class:`RuleStructure` (the IF/THEN decomposition).

This is the analysis half of the planner/assembler split (see
:class:`~krrood.entity_query_language.verbalization.grammar.planning.base.Planner`):
it produces a plain data record of decisions and never builds fragments, touches the
context, or recurses.  The realisation lives in
:class:`~krrood.entity_query_language.verbalization.grammar.assembly.inference.InferenceAssembler`.

Reference: Reiter & Dale (2000) — content/structure determination (microplanning).
"""

from __future__ import annotations

import operator
import uuid
from dataclasses import dataclass, field
from enum import Enum, auto

from typing_extensions import Any, Dict, FrozenSet, List, Optional, Tuple

from krrood.entity_query_language.core.mapped_variable import Attribute
from krrood.entity_query_language.core.variable import InstantiatedVariable, Variable
from krrood.entity_query_language.operators.comparator import Comparator
from krrood.entity_query_language.operators.core_logical_operators import (
    AND,
    flatten_operands,
)
from krrood.entity_query_language.query.query import Entity
from krrood.entity_query_language.verbalization import morphology
from krrood.entity_query_language.verbalization.chain_utils import chain_root
from krrood.entity_query_language.verbalization.grammar.conditions.recognition import (
    attribute_names,
)
from krrood.entity_query_language.verbalization.grammar.planning.base import Planner
from krrood.entity_query_language.verbalization.subquery import (
    unwrap_result_quantifiers,
)
from krrood.entity_query_language.verbalization.vocabulary.english import FallbackNouns


class AggregationStatus(Enum):
    """
    How a consequent binding or antecedent relates to the GROUP BY clause.

    :cvar GROUP_KEY: This expression is one of the ``grouped_by`` key variables.
    :cvar AGGREGATED: Present in the query but not a group key — rendered in plural form.
    :cvar NONE: No grouping context in this query.
    """

    GROUP_KEY = auto()
    AGGREGATED = auto()
    NONE = auto()


@dataclass
class ConditionPlan:
    """One antecedent WHERE condition, with the foldability decided up front.

    ``whose_attribute_name`` is the (singular) attribute name when the condition is a single-hop
    attribute equality that folds into a *"whose <attr> is …"* modifier, else ``None``
    (render the condition normally).  Deciding this here keeps the assembler free of
    analysis.
    """

    expression: Any
    """The raw EQL condition expression."""

    whose_attribute_name: Optional[str]
    """Singular attribute name when foldable to *whose*, else ``None``."""


@dataclass
class AntecedentInfo:
    """Descriptor for one antecedent variable in the IF clause."""

    root: Any
    """The underlying Variable/Entity (unwrapped from any ResultQuantifier)."""

    type_name: str
    """Human-readable Python type name of *root* (e.g. ``"Robot"``)."""

    aggregation_status: AggregationStatus
    """Whether this antecedent is a group key, aggregated, or neither."""

    conditions: List[ConditionPlan] = field(default_factory=list)
    """All WHERE conditions attributable to this antecedent (foldability pre-decided)."""


@dataclass
class ConsequentBinding:
    """Descriptor for one field binding in the THEN clause."""

    field_name: str
    """Python attribute name on the consequent type (e.g. ``"tasks"``)."""

    value_expression: Any
    """EQL expression providing the value for *field_name*."""

    is_plural_field: bool
    """``True`` when *field_name* is already plural."""

    aggregation_status: AggregationStatus
    """Whether the value is a group key, aggregated, or neither."""


@dataclass
class RuleStructure:
    """Complete decomposition of an inference-rule Entity query (the plan)."""

    primary_antecedents: List[AntecedentInfo]
    """Antecedents with at least one condition — items in the IF block."""

    secondary_antecedents: List[AntecedentInfo]
    """Antecedents with no conditions — only registered for coreference."""

    consequent_type: str
    """Python type name of the inferred variable (e.g. ``"Drawer"``)."""

    consequent_bindings: List[ConsequentBinding]
    """Ordered field bindings for the THEN clause."""

    unmatched_conditions: List[Any]
    """Outer WHERE conditions not attributable to any antecedent."""

    group_key_ids: FrozenSet[uuid.UUID]
    """``_id_`` values of the GROUP BY key variables."""


@dataclass
class InferencePlanner(Planner[Entity, RuleStructure]):
    """
    Decompose an inference-rule :class:`~krrood.entity_query_language.query.query.Entity`
    into a :class:`RuleStructure`.

    Algorithm: collect GROUP BY key ids; walk each consequent field binding to build
    :class:`ConsequentBinding` entries and discover antecedent roots; attribute outer
    WHERE conditions to antecedents by matching the left-hand root id; split antecedents
    into primary (have conditions) and secondary (none).
    """

    @staticmethod
    def can_handle(entity) -> bool:
        """Return ``True`` when *entity*'s selected variable is an InstantiatedVariable."""
        entity.build()
        return isinstance(entity.selected_variable, InstantiatedVariable)

    def plan(self) -> RuleStructure:
        self.node.build()
        group_key_ids = self._group_key_ids()
        antecedents, unmatched = self._plan_antecedents(group_key_ids)
        return RuleStructure(
            primary_antecedents=[
                antecedent for antecedent in antecedents if antecedent.conditions
            ],
            secondary_antecedents=[
                antecedent for antecedent in antecedents if not antecedent.conditions
            ],
            consequent_type=self._consequent_type(),
            consequent_bindings=self._plan_consequent(group_key_ids),
            unmatched_conditions=unmatched,
            group_key_ids=group_key_ids,
        )

    # ── shared analysis helpers ──────────────────────────────────────────────────

    @property
    def _inferred(self) -> InstantiatedVariable:
        """The InstantiatedVariable selected by the entity (after build)."""
        return self.node.selected_variable

    def _consequent_type(self) -> str:
        inferred = self._inferred
        return getattr(inferred._type_, "__name__", str(inferred._type_))

    def _group_key_ids(self) -> FrozenSet[uuid.UUID]:
        grouped = self.node._grouped_by_expression_
        if grouped is not None and grouped.variables_to_group_by:
            return frozenset(
                variable._id_ for variable in grouped.variables_to_group_by
            )
        return frozenset()

    @staticmethod
    def _aggregation_status(
        node_id, group_key_ids: FrozenSet[uuid.UUID]
    ) -> AggregationStatus:
        """GROUP_KEY if a group key, else AGGREGATED when grouping is present, else NONE."""
        if node_id in group_key_ids:
            return AggregationStatus.GROUP_KEY
        return AggregationStatus.AGGREGATED if group_key_ids else AggregationStatus.NONE

    # ── consequent (THEN bindings) ───────────────────────────────────────────────

    def _plan_consequent(
        self, group_key_ids: FrozenSet[uuid.UUID]
    ) -> List[ConsequentBinding]:
        return [
            ConsequentBinding(
                field_name=field_name,
                value_expression=child,
                is_plural_field=morphology.is_plural(field_name),
                aggregation_status=self._aggregation_status(child._id_, group_key_ids),
            )
            for field_name, child in self._inferred._child_vars_.items()
        ]

    # ── antecedents (IF roots + their conditions) ────────────────────────────────

    def _plan_antecedents(
        self, group_key_ids: FrozenSet[uuid.UUID]
    ) -> Tuple[List[AntecedentInfo], List[Any]]:
        """Discover antecedent roots, then attribute outer-WHERE conditions to them.

        Returns the antecedents (their ``conditions`` mutated in place) and the
        conditions that matched no antecedent.
        """
        antecedents = self._discover_antecedents(group_key_ids)
        unmatched = self._attribute_conditions(antecedents, self._outer_conditions())
        return antecedents, unmatched

    def _discover_antecedents(
        self, group_key_ids: FrozenSet[uuid.UUID]
    ) -> List[AntecedentInfo]:
        antecedents_by_root_id: Dict[uuid.UUID, AntecedentInfo] = {}
        for child in self._inferred._child_vars_.values():
            root = self._find_root(child)
            if root is None or root._id_ in antecedents_by_root_id:
                continue
            type_name, own_conditions = self._extract_root_info(root)
            antecedents_by_root_id[root._id_] = AntecedentInfo(
                root=root,
                type_name=type_name,
                aggregation_status=self._aggregation_status(root._id_, group_key_ids),
                conditions=own_conditions,
            )
        return list(antecedents_by_root_id.values())

    def _outer_conditions(self) -> List[Any]:
        where = self.node._where_expression_
        return flatten_operands(where.condition, AND) if where is not None else []

    def _attribute_conditions(
        self, antecedents: List[AntecedentInfo], extra_conditions: List[Any]
    ) -> List[Any]:
        """Distribute outer-WHERE conditions to owning antecedents (in place); return unmatched."""
        id_to_antecedent = {
            self._antecedent_variable_id(antecedent): antecedent
            for antecedent in antecedents
        }
        unmatched: List[Any] = []
        for condition in extra_conditions:
            owner_id = self._condition_left_owner_id(condition)
            if owner_id is not None and owner_id in id_to_antecedent:
                id_to_antecedent[owner_id].conditions.append(self._planned(condition))
            else:
                unmatched.append(condition)
        return unmatched

    def _antecedent_variable_id(
        self, antecedent: AntecedentInfo
    ) -> Optional[uuid.UUID]:
        """Stable ``_id_`` of the underlying variable for an antecedent."""
        root = antecedent.root
        if isinstance(root, Entity):
            root.build()
            return getattr(root.selected_variable, "_id_", None)
        return getattr(root, "_id_", None)

    def _condition_left_owner_id(self, condition) -> Optional[uuid.UUID]:
        """``_id_`` of the root variable on the LHS of an equality condition, else ``None``."""
        if (
            not isinstance(condition, Comparator)
            or condition.operation is not operator.eq
        ):
            return None
        current = unwrap_result_quantifiers(chain_root(condition.left))
        return getattr(current, "_id_", None)

    def _find_root(self, expression) -> Optional[Any]:
        current = unwrap_result_quantifiers(chain_root(expression))
        if isinstance(current, (Variable, Entity)):
            return current
        return None

    def _extract_root_info(self, root) -> Tuple[str, List[ConditionPlan]]:
        """Return ``(type_name, own_conditions)`` for a root Variable or Entity."""
        if isinstance(root, Entity):
            root.build()
            selected = root.selected_variable
            type_name = (
                selected._type_.__name__
                if selected and getattr(selected, "_type_", None)
                else FallbackNouns.ENTITY.text
            )
            conditions: List[ConditionPlan] = []
            if root._where_expression_ is not None:
                conditions = [
                    self._planned(conjunct)
                    for conjunct in flatten_operands(
                        root._where_expression_.condition, AND
                    )
                ]
            return type_name, conditions
        if isinstance(root, Variable):
            type_name = (
                root._type_.__name__
                if getattr(root, "_type_", None)
                else FallbackNouns.VARIABLE.text
            )
            return type_name, []
        return FallbackNouns.ENTITY.text, []

    # ── condition foldability (the *whose* analysis — decided here, not in assembly) ──

    def _planned(self, condition) -> ConditionPlan:
        """Wrap a raw condition with its pre-decided *whose*-foldability."""
        return ConditionPlan(
            expression=condition,
            whose_attribute_name=self._whose_attribute_name(condition),
        )

    def _whose_attribute_name(self, condition) -> Optional[str]:
        """Singular attribute name when *condition* folds to *"whose <attr> is …"*, else ``None``.

        Foldable iff it is an attribute equality (``Attribute == value``); the modifier
        attribute is the last hop of the attribute chain.
        """
        if (
            not isinstance(condition, Comparator)
            or condition.operation is not operator.eq
        ):
            return None
        if not isinstance(condition.left, Attribute):
            return None
        names_along_chain = attribute_names(condition.left)
        return names_along_chain[-1] if names_along_chain else None
