from __future__ import annotations

from krrood.entity_query_language.core.base_expressions import SymbolicExpression
from krrood.entity_query_language.core.mapped_variable import (
    Index,
    MappedVariable,
)
from krrood.entity_query_language.core.variable import Variable
from krrood.entity_query_language.query.query import Entity
from krrood.entity_query_language.verbalization import morphology
from krrood.entity_query_language.verbalization.chain_utils import (
    build_path_parts,
)
from krrood.entity_query_language.verbalization.fragments.base import (
    NounPhrase,
    PhraseFragment,
    PossessiveChain,
    RoleFragment,
    Fragment,
    WordFragment,
)
from krrood.entity_query_language.verbalization.fragments.features import (
    Definiteness,
    Number,
)
from krrood.entity_query_language.verbalization.grammar.assembly.base import Assembler
from krrood.entity_query_language.verbalization.grammar.assembly.query import (
    QueryAssembler,
)
from krrood.entity_query_language.verbalization.grammar.planning.chains import (
    ChainPlan,
    ChainPlanner,
)
from krrood.entity_query_language.verbalization.microplanning.possessive import (
    possessive_path,
)
from krrood.entity_query_language.verbalization.subquery import (
    unwrap_result_quantifiers,
)
from krrood.entity_query_language.verbalization.vocabulary.english import (
    Articles,
    Copulas,
    Prepositions,
)


class ChainAssembler(Assembler[MappedVariable, ChainPlan]):
    """Render a ``MappedVariable`` chain into one of its surface forms.

    The :class:`ChainPlanner` decides the chain's content — its root, display path-parts, and
    whether it ends in a boolean attribute. *Which* surface form to use is no longer decided here:
    that choice lives in the grammar as guarded ``MappedVariable`` rules (plural-attribute /
    boolean-predicative / possessive). This assembler is the shared rendering toolkit those rules
    call: each form is a public method, and :meth:`realize` is the unguarded possessive default.

    The forms are the possessive path *"the attribute of the Root"* (optionally pronominalised to
    *"its …"* when the root is the current coreference subject), the predicative *"<navigation> is
    [not] <attribute>"* for a boolean terminal, and the bare plural *"attributes of Roots"*.

    Reference: Gatt & Reiter (2009), SimpleNLG — surface realisation.
    """

    planner = ChainPlanner

    def realize(self, node: MappedVariable, plan: ChainPlan) -> Fragment:
        """
        :param node: The chain to render.
        :param plan: The chain plan computed for *node*.
        :return: The possessive rendering — the unguarded default form.
        """
        return self.possessive(plan)

    # ── surface forms ──────────────────────────────────────────────────────────

    def possessive(self, plan: ChainPlan) -> Fragment:
        """
        :param plan: The analysed chain.
        :return: The possessive path *"the attribute of the Root"*; for a variable root, deferred to
            the coreference pass as a :class:`PossessiveChain` (it knows whether the root is the
            current discourse subject, for *"its …"*).
        """
        root_fragment = self._chain_root(plan.root)
        if isinstance(plan.root, Variable):
            return PossessiveChain(
                parts=plan.parts,
                root_fragment=root_fragment,
                root_referent_id=plan.root._id_,
            )
        return possessive_path(plan.parts, root_fragment)

    def plural_attribute(self, plan: ChainPlan) -> Fragment:
        """
        :param plan: The analysed chain (a single attribute on a variable — see
            :attr:`ChainPlan.is_single_variable_attribute`).
        :return: The bare plural *"attributes of Roots"*.
        """
        root = plan.root
        numbered = self.context.refer.numbered_label(root)
        attribute = plan.chain[0]
        root_noun_phrase = NounPhrase(
            head=RoleFragment.for_variable(numbered.text, root),
            number=Number.SINGULAR if numbered.is_numbered else Number.PLURAL,
            definiteness=(
                Definiteness.BARE if numbered.is_numbered else Definiteness.INDEFINITE
            ),
            referent_id=root._id_,
        )
        return NounPhrase(
            head=RoleFragment.for_attribute(
                attribute._owner_class_, attribute._attribute_name_
            ),
            number=Number.PLURAL,
            definiteness=Definiteness.INDEFINITE,
            modifiers=[Prepositions.OF.as_fragment(), root_noun_phrase],
        )

    def _chain_root(self, root: SymbolicExpression) -> Fragment:
        """
        :param root: The chain root.
        :return: The noun phrase for the chain root (an inline noun for entity roots).
        """
        inner = unwrap_result_quantifiers(root)
        if isinstance(inner, Entity):
            return QueryAssembler(self.context).inline_noun(inner)
        return self.context.child(root)

    def boolean_predicative(self, plan: ChainPlan, negated: bool = False) -> Fragment:
        """
        Chains ending in an integer index get ordinal navigation.

        :param plan: The analysed chain (a boolean terminal — see
            :attr:`ChainPlan.is_boolean_terminal`).
        :param negated: Whether to negate the predicative.
        :return: The predicative *"<navigation> is [not] <attribute>"*.
        """
        chain = plan.chain
        root_fragment = self._chain_root(plan.root)
        navigation_chain = chain[:-1]

        if not navigation_chain:
            navigation_fragment = root_fragment
        elif isinstance(navigation_chain[-1], Index) and isinstance(
            navigation_chain[-1]._key_, int
        ):
            ordinal = morphology.ordinal(navigation_chain[-1]._key_)
            prefix_fragment = possessive_path(
                build_path_parts(navigation_chain[:-1]), root_fragment
            )
            navigation_fragment = PhraseFragment(
                parts=[
                    Articles.THE.as_fragment(),
                    WordFragment(text=ordinal),
                    Prepositions.OF.as_fragment(),
                    prefix_fragment,
                ]
            )
        else:
            navigation_fragment = possessive_path(
                build_path_parts(navigation_chain), root_fragment
            )

        copula = Copulas.IS_NOT.as_fragment() if negated else Copulas.IS.as_fragment()
        terminal = chain[-1]
        return PhraseFragment(
            parts=[
                navigation_fragment,
                copula,
                RoleFragment.for_attribute(
                    terminal._owner_class_, terminal._attribute_name_
                ),
            ]
        )
