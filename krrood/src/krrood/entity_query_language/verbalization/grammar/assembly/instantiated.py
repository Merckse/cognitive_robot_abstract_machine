"""
InstantiatedVariable **assembler** — realise an :class:`InstantiatedPlan` into
*"a TypeName where the field of the TypeName is … such that …"*.

It owns the order-dependent **constraint-deferral dance** (push a frame → build every
binding value → register the field-reference overrides → pop the frame → render the
deferred constraints): the order matters because no binding's value may be rendered
under a sibling binding's override, and the deferred constraints reference the overrides
(verified order-dependent — it cannot be pre-planned).  Number is only *tagged* here
(``Copulas.for_number`` for the copula, ``ctx.child(value, number=…)`` for the value); the
copula agreement and noun pluralisation are applied later by the
:class:`~krrood.entity_query_language.verbalization.rendering.morphology_processor.MorphologyProcessor`
pass.

Reference: Gatt & Reiter (2009), SimpleNLG — surface realisation.
"""

from __future__ import annotations

import uuid

from typing_extensions import Dict, List, Tuple

from krrood.entity_query_language.core.variable import InstantiatedVariable
from krrood.entity_query_language.verbalization.fragments.base import (
    NounPhrase,
    oxford_and,
    PhraseFragment,
    RoleFragment,
    VerbFragment,
)
from krrood.entity_query_language.verbalization.fragments.roles import SemanticRole
from krrood.entity_query_language.verbalization.fragments.source_ref import SourceRef
from krrood.entity_query_language.verbalization.grammar.assembly.base import Assembler
from krrood.entity_query_language.verbalization.grammar.planning.instantiated import (
    BindingPlan,
    InstantiatedPlan,
    InstantiatedPlanner,
)
from krrood.entity_query_language.verbalization.microplanning.possessive import (
    possessive_path,
)
from krrood.entity_query_language.verbalization.vocabulary.english import (
    Articles,
    Conjunctions,
    Copulas,
    Keywords,
    Punctuation,
)
from krrood.entity_query_language.verbalization.vocabulary.words import Number


class InstantiatedAssembler(Assembler[InstantiatedVariable, InstantiatedPlan]):
    """Realise an InstantiatedVariable from its :class:`InstantiatedPlan`."""

    planner = InstantiatedPlanner

    def realize(self, node, plan: InstantiatedPlan) -> VerbFragment:
        """*"a TypeName, where the <field> of the TypeName is <value> …, such that <deferred>"*.

        A referring NP (referent_id below) — the CoreferenceProcessor reduces a repeat
        mention to *"the <type>"* in document order, so no build-time seen check here.
        """
        self.ctx.scope.push_constraint_frame()
        binding_fragments, overrides = self._bindings(plan, node._type_)
        self.ctx.scope.binding_overrides.update(overrides)
        deferred = self.ctx.scope.pop_constraint_frame()
        constraint_fragments = [self.ctx.child(expression) for expression in deferred]

        return self._phrase(
            node, plan.type_name, binding_fragments, constraint_fragments
        )

    # ── bindings ───────────────────────────────────────────────────────────────

    def _bindings(
        self, plan: InstantiatedPlan, instantiated_type
    ) -> Tuple[List[VerbFragment], Dict[uuid.UUID, VerbFragment]]:
        """Build every binding fragment and collect overrides (registered together after)."""
        binding_fragments: List[VerbFragment] = []
        overrides: Dict[uuid.UUID, VerbFragment] = {}
        for binding in plan.bindings:
            field_reference = self._field_reference(
                binding.field_name, plan.type_name, instantiated_type
            )
            binding_fragments.append(
                PhraseFragment(
                    parts=[field_reference, self._copula(binding), self._value(binding)]
                )
            )
            overrides[binding.value._id_] = field_reference
        return binding_fragments, overrides

    def _field_reference(
        self, field_name: str, type_name: str, instantiated_type
    ) -> VerbFragment:
        """*"the <field> of the <Type>"* — a single-hop possessive, built by the shared
        :func:`~krrood.entity_query_language.verbalization.microplanning.possessive.possessive_path`
        so the genitive structure lives in exactly one place."""
        type_root = PhraseFragment(
            parts=[
                Articles.THE.as_fragment(),
                RoleFragment(
                    text=type_name,
                    role=SemanticRole.VARIABLE,
                    source_ref=(
                        SourceRef.for_type(instantiated_type)
                        if isinstance(instantiated_type, type)
                        else None
                    ),
                ),
            ]
        )
        return possessive_path([(field_name, None)], type_root)

    def _copula(self, binding: BindingPlan) -> VerbFragment:
        """*"is"* / *"are"* agreeing with the binding's plurality (inflected by morphology)."""
        return Copulas.for_number(Number.of(binding.is_plural))

    def _value(self, binding: BindingPlan) -> VerbFragment:
        """The binding's value expression, rendered in the binding's number."""
        return self.ctx.child(binding.value, number=Number.of(binding.is_plural))

    # ── phrase assembly ──────────────────────────────────────────────────────────

    def _phrase(
        self,
        node,
        type_name: str,
        binding_fragments: List[VerbFragment],
        constraint_fragments: List[VerbFragment],
    ) -> VerbFragment:
        """*"a <type>, where <bindings>, such that <constraints>"* — the referring NP with
        its appositive clauses as droppable modifiers."""
        modifiers: List[VerbFragment] = []
        if binding_fragments:
            joined = oxford_and(binding_fragments, Conjunctions.AND.as_fragment())
            modifiers.append(
                PhraseFragment(
                    parts=[
                        Punctuation.COMMA.as_fragment(),
                        Keywords.WHERE.as_fragment(),
                        joined,
                    ]
                )
            )
        if constraint_fragments:
            joined_constraints = oxford_and(
                constraint_fragments, Conjunctions.AND.as_fragment()
            )
            modifiers.append(
                PhraseFragment(
                    parts=[
                        Punctuation.COMMA.as_fragment(),
                        Keywords.SUCH_THAT.as_fragment(),
                        joined_constraints,
                    ]
                )
            )
        # A referring NP: "a <type>" first mention (+ appositive clauses), reduced to
        # "the <type>" on a repeat by the CoreferenceProcessor (which drops the modifiers).
        return NounPhrase(
            head=RoleFragment.for_variable(type_name, node),
            modifiers=modifiers,
            modifier_separator="",
            referent_id=node._id_,
        )
