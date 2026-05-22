"""
EntityVerbalizer — Entity, SetOf, and query-body clause rendering.

Handles the full query form ("Find X such that …"), the inline-noun form
(constraint-deferring), and the standalone-noun form used when an Entity is
the selected variable of an outer query.
"""
from __future__ import annotations

from typing import Optional, TYPE_CHECKING

from krrood.entity_query_language.core.variable import InstantiatedVariable, Variable
from krrood.entity_query_language.query.quantifiers import An, ResultQuantifier, The
from krrood.entity_query_language.query.query import Entity, Query, SetOf
from krrood.entity_query_language.verbalization.chain_utils import chain_root, verbalize_plural
from krrood.entity_query_language.verbalization.fragments.base import (
    BlockFragment,
    PhraseFragment,
    RoleFragment,
    VerbFragment,
    WordFragment,
)
from krrood.entity_query_language.verbalization.fragments.roles import SemanticRole
from krrood.entity_query_language.verbalization.utils import _apply_binding_aliases, _str
from krrood.entity_query_language.verbalization.vocabulary.english import (
    Articles,
    Conjunctions,
    Copulas,
    FallbackNouns,
    Keywords,
    SortDirections,
)

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext


def _word(text: str) -> WordFragment:
    return WordFragment(text=text)


def _role(text, role, ref=None):
    return RoleFragment(text=text, role=role, source_ref=ref)


def _phrase(*parts, sep=" "):
    return PhraseFragment(parts=list(parts), separator=sep)


class EntityVerbalizer:
    """Verbalizes Entity and SetOf query expressions."""

    def __init__(self, delegate) -> None:
        self._d = delegate

    # ── Public entry points ────────────────────────────────────────────────────

    def verbalize_query(self, expr: Entity, ctx: VerbalizationContext) -> VerbFragment:
        """Full query form: 'Find X such that …'"""
        if expr._id_ in ctx.seen:
            return _phrase(Articles.THE.as_fragment(), _role(ctx.seen[expr._id_], SemanticRole.VARIABLE))

        expr.build()

        if self._d._rule.can_handle(expr):
            return self._d._rule.verbalize(expr, ctx)

        is_the = (
            expr._quantifier_builder_ is not None
            and expr._quantifier_builder_.type is The
        )
        var = expr.selected_variable

        if isinstance(var, Entity):
            selected = self.as_noun(var, ctx)
        elif var is None:
            selected_type = FallbackNouns.ENTITY.text
            ctx.seen[expr._id_] = selected_type
            selected = FallbackNouns.ENTITY.plural_fragment()
        elif is_the:
            selected_type = var._type_.__name__ if getattr(var, "_type_", None) else FallbackNouns.ENTITY.text
            ctx.seen[var._id_] = selected_type
            ctx.seen[expr._id_] = selected_type
            selected = _phrase(Articles.THE_UNIQUE.as_fragment(), _role(selected_type, SemanticRole.VARIABLE))
        else:
            selected = self._d.build(var, ctx)
            selected_type = ctx.seen.get(getattr(var, "_id_", None), FallbackNouns.ENTITY.text)
            ctx.seen[expr._id_] = selected_type

        return self._verbalize_query_body_(expr, ctx, selected)

    def as_noun(self, expr: Entity, ctx: VerbalizationContext) -> VerbFragment:
        """Standalone-noun form: 'a Robot where …' (for nested Entity selectors)."""
        if expr._id_ in ctx.seen:
            return _phrase(Articles.THE.as_fragment(), _role(ctx.seen[expr._id_], SemanticRole.VARIABLE))

        expr.build()
        is_the = (
            expr._quantifier_builder_ is not None
            and expr._quantifier_builder_.type is The
        )
        var = expr.selected_variable
        selected_type = var._type_.__name__ if var and getattr(var, "_type_", None) else FallbackNouns.ENTITY.text
        ctx.seen[expr._id_] = selected_type
        if var is not None:
            ctx.seen[var._id_] = selected_type

        if is_the:
            article_noun: VerbFragment = _phrase(
                Articles.THE_UNIQUE.as_fragment(), RoleFragment.for_variable(selected_type, var)
            )
        else:
            article_noun = _phrase(
                Articles.indefinite(selected_type),
                RoleFragment.for_variable(selected_type, var),
            )

        where_expr = expr._where_expression_
        if where_expr is not None:
            return _phrase(article_noun, Keywords.WHERE.as_fragment(), self._d.build(where_expr.condition, ctx))
        return article_noun

    def as_inline_noun(self, entity: Entity, ctx: VerbalizationContext) -> VerbFragment:
        """Inline-noun form used as chain root: defers where-condition to ctx constraints."""
        if entity._id_ in ctx.seen:
            return _phrase(Articles.THE.as_fragment(), _role(ctx.seen[entity._id_], SemanticRole.VARIABLE))

        entity.build()
        var = entity.selected_variable
        var_type = getattr(var, "_type_", None)
        type_name = var_type.__name__ if var_type else FallbackNouns.ENTITY.text

        ctx.seen[entity._id_] = type_name
        ctx.seen[var._id_] = type_name

        where_expr = entity._where_expression_
        if where_expr is not None:
            cond_text = self._d.verbalize(where_expr.condition, ctx)
            ctx.add_constraint(cond_text)

        return _phrase(Articles.indefinite(type_name), RoleFragment.for_variable(type_name, var))

    def verbalize_set_of(self, expr: SetOf, ctx: VerbalizationContext) -> VerbFragment:
        expr.build()
        vars_str = ", ".join(self._d.verbalize(v, ctx) for v in expr._selected_variables_)
        prefix = _phrase(Keywords.FIND_SETS_OF.as_fragment(), _word(f"({vars_str})"))
        return self._verbalize_query_body_(expr, ctx, prefix)

    # ── Query body assembly ────────────────────────────────────────────────────

    def _verbalize_query_body_(
        self, expr, ctx: VerbalizationContext, selection: VerbFragment
    ) -> VerbFragment:
        find_header = _phrase(Keywords.FIND.as_fragment(), selection)
        where_expr = expr._where_expression_
        grouped_expr = expr._grouped_by_expression_
        having_expr = expr._having_expression_
        aliases = ctx.binding_aliases
        clauses: list[VerbFragment] = []

        if where_expr is not None:
            clauses.append(_phrase(
                Keywords.SUCH_THAT.as_fragment(),
                self._build_clause_frag_(where_expr.condition, ctx, aliases),
            ))

        if grouped_expr is not None and grouped_expr.variables_to_group_by:
            group_key_root_ids = self._root_var_ids_(grouped_expr.variables_to_group_by)
            groups = [
                _apply_binding_aliases(self._d.verbalize(v, ctx), aliases)
                for v in grouped_expr.variables_to_group_by
            ]
            aggregated = self._aggregated_noun_phrases_(expr, group_key_root_ids, ctx)
            groups_str = self._join_groups_(groups)
            _the = Articles.THE.text
            _are = Copulas.ARE.text
            if aggregated:
                aggregated_str = self._join_plain_(aggregated)
                clauses.append(_phrase(
                    Conjunctions.AND.as_fragment(),
                    _word(f"{_the} {aggregated_str} {_are}"),
                    Keywords.GROUPED_BY.as_fragment(),
                    _word(groups_str),
                ))
            else:
                clauses.append(_phrase(Keywords.GROUPED_BY.as_fragment(), _word(groups_str)))

        if having_expr is not None:
            ctx.compact_predicates = True
            having_frag = self._build_clause_frag_(having_expr.condition, ctx, aliases)
            ctx.compact_predicates = False
            clauses.append(_phrase(Keywords.HAVING.as_fragment(), having_frag))

        ob = expr._ordered_by_builder_
        if ob is not None:
            direction = SortDirections.DESCENDING.text if ob.descending else SortDirections.ASCENDING.text
            ordered_frag = self._build_clause_frag_(ob.variable, ctx, aliases)
            clauses.append(_phrase(
                Keywords.ORDERED_BY.as_fragment(),
                ordered_frag,
                _word(f"({direction})"),
            ))

        return BlockFragment(header=find_header, items=clauses)

    def _build_clause_frag_(self, expr, ctx: VerbalizationContext, aliases: dict) -> VerbFragment:
        """Build a clause condition as a VerbFragment, preserving all semantic roles.

        When *aliases* is empty (all plain Entity queries) the fragment is returned
        as-is, keeping full role/colour information.  When aliases are present
        (InstantiatedVariable selection path) we fall back to string-level
        substitution — a known limitation tracked for Phase 2.
        """
        frag = self._d.build(expr, ctx)
        if not aliases:
            return frag
        return _word(_apply_binding_aliases(_str(frag), aliases))

    @staticmethod
    def _root_var_ids_(exprs) -> set:
        ids: set = set()
        for e in exprs:
            root = chain_root(e)
            if isinstance(root, Variable):
                ids.add(root._id_)
        return ids

    def _aggregated_noun_phrases_(
        self, query_expr, group_key_root_ids: set, ctx: VerbalizationContext
    ) -> list[str]:
        texts: list[str] = []
        selected_var = query_expr.selected_variable if isinstance(query_expr, Entity) else None

        if isinstance(selected_var, InstantiatedVariable):
            for child_expr in selected_var._child_vars_.values():
                root = chain_root(child_expr)
                if isinstance(root, Variable) and root._id_ in group_key_root_ids:
                    continue
                texts.append(_str(verbalize_plural(child_expr, ctx, self._d.build)))
        elif isinstance(query_expr, Query):
            for var in query_expr._selected_variables_:
                if var._id_ not in group_key_root_ids:
                    texts.append(_str(verbalize_plural(var, ctx, self._d.build)))

        return texts

    @staticmethod
    def _join_groups_(groups: list[str]) -> str:
        if len(groups) == 1:
            return groups[0]
        conj = f", {Conjunctions.AND.text} "
        return f"({', '.join(groups[:-1])}{conj}{groups[-1]})"

    @staticmethod
    def _join_plain_(parts: list[str]) -> str:
        if len(parts) == 1:
            return parts[0]
        return ", ".join(parts[:-1]) + " " + parts[-1]
