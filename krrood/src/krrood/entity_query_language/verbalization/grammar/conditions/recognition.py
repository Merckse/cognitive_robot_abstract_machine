"""
Condition **recognizers** — the single source of truth for the structural shapes a
condition (a :class:`~krrood.entity_query_language.operators.comparator.Comparator` or a
:class:`~krrood.entity_query_language.core.mapped_variable.MappedVariable` chain) can take.

They are pure structural predicates — no fragments, no context — so they live here
once, and every surface-form decision (restriction rules, inference planner, chain
assembler, grammar guards) consults them instead of re-implementing the shape test.
"""

from __future__ import annotations

import operator
from dataclasses import dataclass

from typing_extensions import List, Optional

from krrood.entity_query_language.core.mapped_variable import Attribute, MappedVariable
from krrood.entity_query_language.core.variable import Variable
from krrood.entity_query_language.operators.aggregators import Aggregator, Extreme
from krrood.entity_query_language.operators.comparator import Comparator
from krrood.entity_query_language.query.query import Entity
from krrood.entity_query_language.verbalization.chain_utils import (
    chain_ends_in_boolean_attribute,
    chain_root,
    walk_chain,
)
from krrood.entity_query_language.verbalization.subquery import (
    aggregation_source_root,
    is_collapsible_aggregation_subquery,
    selected_aggregator,
    unwrap_result_quantifiers,
)


def attribute_names(left) -> List[str]:
    """The attribute names along a MappedVariable chain, outermost last (``[]`` if none)."""
    names: List[str] = []
    current = left
    while isinstance(current, MappedVariable):
        if isinstance(current, Attribute):
            names.append(current._attribute_name_)
        current = current._child_
    return names


def single_hop_attribute(expression, subject_variable) -> Optional[Attribute]:
    """The :class:`Attribute` node when *expression* is exactly ``subject_variable.<attr>``, else ``None``."""
    if subject_variable is None or not isinstance(expression, MappedVariable):
        return None
    chain, root = walk_chain(expression)
    if not (isinstance(root, Variable) and root._id_ == subject_variable._id_):
        return None
    if len(chain) != 1 or not isinstance(chain[0], Attribute):
        return None
    return chain[0]


def references(expression, subject_variable) -> bool:
    """``True`` when *expression* mentions *subject_variable* (so it is not a clean RHS value)."""
    try:
        return any(
            getattr(variable, "_id_", None) == subject_variable._id_
            for variable in expression._unique_variables_
        )
    except AttributeError:
        return chain_root(expression) is subject_variable


def is_boolean_attribute_chain(expression) -> bool:
    """``True`` when *expression* is a MappedVariable chain ending in a bool-typed Attribute."""
    if not isinstance(expression, MappedVariable):
        return False
    chain, _ = walk_chain(expression)
    return chain_ends_in_boolean_attribute(chain)


@dataclass(frozen=True)
class SuperlativeFold:
    """A subject restriction of the form ``subject.<chain> == max/min(<same-type>.<same chain>)``
    folded to a superlative selection modifier — *"with the maximum <leaf>"*.

    English's superlative *is* the meaning of "equal to the extreme value over the whole
    population", so this fold is meaning-preserving — **not** an optimisation — under the guard
    in :func:`superlative_aggregation`: an ``==`` against an *unconstrained* ``Max``/``Min``
    sub-query over the *same type* and the *same attribute chain*."""

    aggregator: Aggregator
    """The ``Max`` / ``Min`` aggregator — supplies the superlative word (via ``AGGREGATION_KIND``)
    and the leaf attribute (``aggregator._leaf_attribute_``) for the *"… <leaf>"* tail."""


def superlative_aggregation(comparator, subject) -> Optional[SuperlativeFold]:
    """Recognise a superlative restriction on *subject* — see :class:`SuperlativeFold`.

    Returns the fold when *comparator* is ``subject.<chain> == <unconstrained Max/Min over a
    different same-type variable's identical chain>``, else ``None`` (the conjunct then renders
    normally).  Pure structural analysis; the guard is deliberately strict so it never fires on a
    self-join, a constrained sub-query, a different chain, or a non-extreme aggregation.
    """
    if subject is None or not isinstance(comparator, Comparator):
        return None
    if comparator.operation is not operator.eq:
        return None
    left_root = chain_root(comparator.left)
    if getattr(left_root, "_id_", None) != getattr(subject, "_id_", object()):
        return None

    inner = unwrap_result_quantifiers(comparator.right)
    if not (isinstance(inner, Entity) and is_collapsible_aggregation_subquery(inner)):
        return None
    aggregator = selected_aggregator(inner)
    if not isinstance(aggregator, Extreme) or aggregator._leaf_attribute_ is None:
        return None

    source = aggregation_source_root(inner)
    if source is None or source is subject:
        return None  # the population must be a distinct variable of the same type
    if getattr(source, "_type_", None) is not getattr(subject, "_type_", None):
        return None
    if attribute_names(comparator.left) != attribute_names(aggregator._child_):
        return None
    return SuperlativeFold(aggregator=aggregator)
