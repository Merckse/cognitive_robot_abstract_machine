"""
Restriction **assembler** — render a
:class:`~krrood.entity_query_language.verbalization.grammar.planning.query.RestrictionPlan`
(a subject's WHERE partition) into its surface pieces: superlative selection modifiers
(*"with the maximum <leaf>"*), the appositive *"whose <grouped>"* modifier, and the residual
*"such that …"* / *"where …"* condition.

Realisation-only, but deliberately **not** an :class:`Assembler` subclass: a restriction yields
*several* pieces (:class:`RestrictionFragments`), not a single fragment — the caller drops each
into its own slot (inline after the selection, as a noun modifier, or inside an aggregation
scope).  Shared by ``QueryAssembler`` and ``AggregationValueAssembler`` so the
restriction rendering lives in one place.

Reference: Reiter & Dale (2000) — content structuring (the WHERE partition is the *plan*).
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field

from typing_extensions import Dict, List, Optional

from krrood.entity_query_language.verbalization.fragments.base import (
    oxford_and,
    PhraseFragment,
    VerbFragment,
)
from krrood.entity_query_language.verbalization.grammar.phrase_rule import Ctx
from krrood.entity_query_language.verbalization.grammar.planning.query import (
    RestrictionPlan,
)
from krrood.entity_query_language.verbalization.grammar.restriction import Placement
from krrood.entity_query_language.verbalization.microplanning.coordination import (
    fragment_for_folded_conjunct,
)
from krrood.entity_query_language.verbalization.vocabulary.english import (
    Conjunctions,
    Keywords,
)


class UnplacedRestrictionError(ValueError):
    """A :class:`~krrood.entity_query_language.verbalization.grammar.restriction.RestrictionRule`
    declared a :class:`~krrood.entity_query_language.verbalization.grammar.restriction.Placement`
    that no :class:`RestrictionFragments` slot surfaces.

    Raised loudly instead of dropping the fragment: add a field on
    :class:`RestrictionFragments` and surface it in the consumers
    (``QueryAssembler`` / ``AggregationValueAssembler``)."""


@dataclass(frozen=True)
class RestrictionFragments:
    """The rendered pieces of a subject restriction, each placed by the caller."""

    superlatives: List[VerbFragment] = field(default_factory=list)
    """Selection PP modifiers, e.g. *"with the maximum amount"* (attach right after the noun)."""

    whose: Optional[VerbFragment] = None
    """The appositive *"whose <grouped>"* modifier, or ``None``."""

    residual: Optional[VerbFragment] = None
    """The residual condition for a *"such that …"* / *"where …"* clause, or ``None``."""


@dataclass
class RestrictionAssembler:
    """Render a :class:`RestrictionPlan` into its :class:`RestrictionFragments`."""

    ctx: Ctx
    """The per-node context (recursion entry + microplanning services)."""

    def render(self, restriction: RestrictionPlan, subject) -> RestrictionFragments:
        """Render each matched conjunct via its rule and place it by the rule's
        :class:`Placement`; then build the residual condition."""
        by_placement: Dict[Placement, List[VerbFragment]] = defaultdict(list)
        for rule, item in restriction.matched:
            by_placement[rule.placement].append(rule.render(item, subject, self.ctx))

        superlatives = by_placement.pop(Placement.SELECTION_MODIFIER, [])
        grouped = by_placement.pop(Placement.WHOSE_GROUP, [])
        # Loud, not silent: a rule declaring a placement no slot surfaces is a bug, not a drop.
        if by_placement:
            raise UnplacedRestrictionError(
                "Restriction placement(s) with no RestrictionFragments slot: "
                f"{[placement.name for placement in by_placement]}."
            )

        whose = (
            PhraseFragment(
                parts=[
                    Keywords.WHOSE.as_fragment(),
                    oxford_and(grouped, Conjunctions.AND.as_fragment()),
                ]
            )
            if grouped
            else None
        )
        residual = (
            self._residual(restriction.residual) if restriction.has_residual else None
        )
        return RestrictionFragments(
            superlatives=superlatives, whose=whose, residual=residual
        )

    def _residual(self, items) -> VerbFragment:
        """The residual conjuncts (raw expressions / folded ranges) joined into one condition."""
        parts: List[VerbFragment] = [
            fragment_for_folded_conjunct(
                item, self.ctx.child, compact=self.ctx.config.compact_predicates
            )
            for item in items
        ]
        if len(parts) == 1:
            return parts[0]
        return oxford_and(parts, Conjunctions.AND.as_fragment())
