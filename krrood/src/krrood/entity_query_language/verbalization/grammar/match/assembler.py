from __future__ import annotations

from typing_extensions import List, Optional

from krrood.entity_query_language.core.mapped_variable import Attribute
from krrood.entity_query_language.query.match import Match
from krrood.entity_query_language.verbalization.fragments.base import (
    BlockFragment,
    Fragment,
    oxford_comma,
    PhraseFragment,
    RoleFragment,
    WordFragment,
)
from krrood.entity_query_language.verbalization.grammar.framework.assembler import (
    Assembler,
)
from krrood.entity_query_language.verbalization.grammar.match.planner import (
    AttributeGroup,
    MatchPlan,
    MatchPlanner,
)
from krrood.entity_query_language.verbalization.vocabulary.english import (
    Conjunctions,
    Copulas,
    Directive,
    Keywords,
    Prepositions,
    Pronouns,
)


class MatchAssembler(Assembler[Match, MatchPlan]):
    """
    Realise a match into *"Find/Generate <selection> [, and predict its … values]"* with a
    *"given that"* block (the construction pattern, attributes aggregated per object) and a
    *"where"* block (the free conditions), each condition its own point.

    The selection and every condition/value are recursed through ``context.child``, so the existing
    chain / comparator / coreference machinery renders them; this assembler only decides the
    match-specific structure.

    Reference: Gatt & Reiter (2009), SimpleNLG — surface realisation.
    """

    planner = MatchPlanner

    def realize(self, node: Match, plan: MatchPlan) -> Fragment:
        """
        :param node: The match being verbalised.
        :param plan: The match plan.
        :return: The match's block, sourced at the resolved query so the coreference pass scopes the
            selection as the discourse subject (*"its …"*).
        """
        header_parts: List[Fragment] = [
            Directive.for_underspecified(plan.underspecified).as_fragment(),
            self.context.child(plan.selection),
        ]
        predict = self._predict_clause(plan)
        if predict is not None:
            header_parts.append(predict)

        items: List[Fragment] = []
        given = self._given_that_block(plan)
        if given is not None:
            items.append(given)
        where = self._where_block(plan)
        if where is not None:
            items.append(where)

        return BlockFragment(
            header=PhraseFragment(parts=header_parts),
            items=items,
            source=node.expression,
        )

    # ── header: predict ──────────────────────────────────────────────────────

    def _predict_clause(self, plan: MatchPlan) -> Optional[Fragment]:
        """:return: *", and predict its <attrs> value(s)"* for the selection's Ellipsis attributes,
        or ``None`` when none are predicted on the selection."""
        attributes = [
            assignment.attribute
            for group in plan.groups
            if group.object._id_ == plan.selection._id_
            for assignment in group.predicted
        ]
        if not attributes:
            return None
        noun = "values" if len(attributes) > 1 else "value"
        return PhraseFragment(
            parts=[
                Conjunctions.AND.as_fragment(),
                Keywords.PREDICT.as_fragment(),
                Pronouns.ITS.as_fragment(),
                self._attribute_list(attributes),
                WordFragment(text=noun),
            ]
        )

    # ── given that ───────────────────────────────────────────────────────────

    def _given_that_block(self, plan: MatchPlan) -> Optional[Fragment]:
        """:return: The *"given that"* block — one point per attribute group (concrete assignments)
        and per non-grouping condition — or ``None`` when there is nothing to give."""
        points: List[Fragment] = [
            self._group_point(group) for group in plan.groups if group.concrete
        ]
        points += [self.context.child(condition) for condition in plan.other_conditions]
        if not points:
            return None
        return BlockFragment(header=Keywords.GIVEN_THAT.as_fragment(), items=points)

    def _group_point(self, group: AttributeGroup) -> Fragment:
        """:return: *"x, y, and z of the <object> are 1, 2, and 3 respectively"* for several
        attributes, or *"x of the <object> is 1"* for one."""
        concrete = group.concrete
        attribute_list = self._attribute_list([a.attribute for a in concrete])
        object_phrase = self.context.child(group.object)
        parts: List[Fragment] = [
            attribute_list,
            Prepositions.OF.as_fragment(),
            object_phrase,
        ]
        if len(concrete) == 1:
            parts += [Copulas.IS.as_fragment(), self.context.child(concrete[0].value)]
        else:
            value_list = oxford_comma(
                [self.context.child(a.value) for a in concrete],
                Conjunctions.AND.as_fragment(),
            )
            parts += [
                Copulas.ARE.as_fragment(),
                value_list,
                Keywords.RESPECTIVELY.as_fragment(),
            ]
        return PhraseFragment(parts=parts)

    # ── where ────────────────────────────────────────────────────────────────

    def _where_block(self, plan: MatchPlan) -> Optional[Fragment]:
        """:return: The *"where"* block — one point per free condition — or ``None`` when absent."""
        if not plan.where_conditions:
            return None
        points = [self.context.child(condition) for condition in plan.where_conditions]
        return BlockFragment(header=Keywords.WHERE.as_fragment(), items=points)

    # ── shared ───────────────────────────────────────────────────────────────

    def _attribute_list(self, attributes: List[Attribute]) -> Fragment:
        """:return: The attribute names as a single fragment — *"x, y, and z"* or *"x"*."""
        fragments = [
            RoleFragment.for_attribute(
                attribute._owner_class_, attribute._attribute_name_
            )
            for attribute in attributes
        ]
        if len(fragments) == 1:
            return fragments[0]
        return oxford_comma(fragments, Conjunctions.AND.as_fragment())
