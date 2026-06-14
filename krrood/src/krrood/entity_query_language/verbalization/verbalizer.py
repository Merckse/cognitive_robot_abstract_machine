from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import Optional

from krrood.entity_query_language.core.base_expressions import SymbolicExpression
from krrood.entity_query_language.verbalization.context import MicroplanningServices
from krrood.entity_query_language.verbalization.engine import fold
from krrood.entity_query_language.verbalization.fragments.base import Fragment
from krrood.entity_query_language.verbalization.grammar.framework.registry import RULES
from krrood.entity_query_language.verbalization.rendering.discourse import DiscourseModel
from krrood.entity_query_language.verbalization.rendering.realization import (
    realize_tree,
)


@dataclass
class EQLVerbalizer:
    """
    Builds the natural-language fragment tree that represents an EQL expression.
    """

    def build(
        self,
        expression: SymbolicExpression,
        services: Optional[MicroplanningServices] = None,
    ) -> Fragment:
        """
        Translate *expression* into its natural-language fragment tree.

        A fresh services bundle is created when *services* is ``None``; pass a shared one across
        calls so repeated mentions corefer (a Robot … the Robot).

        :param expression: Any EQL symbolic expression.
        :param services: Shared verbalization state; created automatically when omitted.
        :return: Root of the fragment tree representing *expression* in natural language.
        """
        if services is None:
            services = MicroplanningServices.from_expression(expression)
        # Referents already introduced by prior builds on these (shared) services, so the same
        # expression verbalized twice reads "a Robot" then "the Robot".  Snapshot BEFORE the
        # fold, which records this build's own mentions in the same set.
        already_seen = set(services.referring.seen)
        fragment = fold(expression, services, RULES)
        # The discourse focus per query scope, projected once from the shared plan read model; the
        # coreference pass consults it instead of rule-emitted subject markers.
        discourse = DiscourseModel.from_expression(expression, services.microplan)
        return realize_tree(
            fragment, already_seen=already_seen, discourse=discourse
        )
