"""
Inference-rule **assembler** — realise a :class:`RuleStructure` (from
:class:`~krrood.entity_query_language.verbalization.grammar.planning.inference.InferencePlanner`)
into an ``IF … THEN …`` :class:`~krrood.entity_query_language.verbalization.fragments.base.BlockFragment`.

Realisation sub-steps are methods sharing ``self.ctx`` (recursion via ``self.ctx.child``,
coreference via ``self.ctx.refer``).  This assembler only decides + *tags* grammatical
:class:`~krrood.entity_query_language.verbalization.fragments.features.Number` (via
``Copulas.for_number`` / the lexicon frames); the actual copula agreement and noun pluralisation are
applied once, later, by the
:class:`~krrood.entity_query_language.verbalization.rendering.morphology_processor.MorphologyProcessor`
pass.  This is the realisation half of the planner/assembler split (see
:class:`~krrood.entity_query_language.verbalization.grammar.assembly.base.Assembler`).

Reference: Gatt & Reiter (2009), SimpleNLG — surface realisation + the MorphologyProcessor.
"""

from __future__ import annotations

from typing_extensions import List

from krrood.entity_query_language.core.variable import Variable
from krrood.entity_query_language.query.query import Entity
from krrood.entity_query_language.verbalization.chain_utils import (
    build_path_parts,
    walk_chain,
)
from krrood.entity_query_language.verbalization.fragments.base import (
    BlockFragment,
    PhraseFragment,
    VerbFragment,
)
from krrood.entity_query_language.verbalization.grammar.assembly.base import Assembler
from krrood.entity_query_language.verbalization.grammar.conditions.verbalizer import (
    ConditionVerbalizer,
)
from krrood.entity_query_language.verbalization.grammar.planning.inference import (
    AggregationStatus,
    AntecedentInfo,
    ConsequentBinding,
    InferencePlanner,
    ConditionPlan,
    RuleStructure,
)
from krrood.entity_query_language.verbalization.vocabulary.english import (
    Articles,
    ExistentialPhrase,
    FallbackNouns,
    GroupKeyPhrases,
    Keywords,
)
from krrood.entity_query_language.verbalization.vocabulary.words import Number


class InferenceAssembler(Assembler[Entity, RuleStructure]):
    """Realise the IF/THEN block from a :class:`RuleStructure`."""

    planner = InferencePlanner

    def realize(self, node, plan: RuleStructure) -> VerbFragment:
        """*"If <antecedents…>, then <consequent…>"* — the two-block IF/THEN form."""
        return BlockFragment(
            header=None,
            items=[
                BlockFragment(
                    header=Keywords.IF.as_fragment(), items=self._if_items(plan)
                ),
                BlockFragment(
                    header=Keywords.THEN.as_fragment(), items=self._then_items(plan)
                ),
            ],
        )

    @staticmethod
    def _number(antecedent: AntecedentInfo) -> Number:
        """The grammatical number of an antecedent — plural iff aggregated."""
        return Number.of(antecedent.aggregation_status == AggregationStatus.AGGREGATED)

    # ── IF clause ───────────────────────────────────────────────────────────────

    def _if_items(self, structure: RuleStructure) -> List[VerbFragment]:
        """One item per antecedent — *"there's a <Type> [whose …]"* — plus any unmatched
        conditions; *"true"* when there are none."""
        for antecedent in structure.secondary_antecedents:
            self._register_antecedent(antecedent)

        items: List[VerbFragment] = []
        for antecedent in structure.primary_antecedents:
            intro = self._antecedent_intro(antecedent)
            self._register_antecedent(antecedent)
            condition_fragments = self._condition_fragments(
                antecedent.conditions, antecedent
            )
            items.append(
                BlockFragment(header=intro, items=condition_fragments)
                if condition_fragments
                else intro
            )

        for condition in structure.unmatched_conditions:
            items.append(self.ctx.child(condition))

        return items or [Keywords.TRUE.as_fragment()]

    def _antecedent_intro(self, antecedent: AntecedentInfo) -> VerbFragment:
        """*"there's a <Type>"* / *"there are <Types>"* — the antecedent's existential intro.

        Passes the antecedent's referent so the coreference pass marks it introduced — a later
        mention (e.g. *"the parent of the FixedConnection"* in the THEN clause) then reads
        *"the"*."""
        return ExistentialPhrase.for_number(self._number(antecedent)).build_phrase(
            antecedent.type_name, referent_id=self._antecedent_referent_id(antecedent)
        )

    @staticmethod
    def _antecedent_referent_id(antecedent: AntecedentInfo):
        """The antecedent's canonical referent id — the selected variable for an Entity root,
        else the root's own id (matching the variable the THEN-clause chains reference).
        """
        root = antecedent.root
        if isinstance(root, Entity):
            root.build()
            return getattr(root.selected_variable, "_id_", None)
        return getattr(root, "_id_", None)

    def _register_antecedent(self, antecedent: AntecedentInfo) -> None:
        """Mark the antecedent (and its selected variable) introduced, so later mentions in
        the THEN clause read *"the <Type>"*."""
        root = antecedent.root
        self.ctx.refer.mark_introduced(root)
        if isinstance(root, Entity):
            root.build()
            selected = root.selected_variable
            if selected is not None and hasattr(selected, "_id_"):
                self.ctx.refer.mark_introduced(selected)

    def _condition_fragments(
        self, conditions: List[ConditionPlan], antecedent: AntecedentInfo
    ) -> List[VerbFragment]:
        """One fragment per antecedent condition (see :meth:`_condition_fragment`)."""
        return [
            self._condition_fragment(condition_plan, antecedent)
            for condition_plan in conditions
        ]

    def _condition_fragment(
        self, condition_plan: ConditionPlan, antecedent: AntecedentInfo
    ) -> VerbFragment:
        """Render one condition: a *"whose <attr> is …"* modifier when foldable, else recurse."""
        if condition_plan.whose_attribute_name is None:
            return self.ctx.child(condition_plan.expression)
        number = self._number(antecedent)
        value = self._value(condition_plan.expression.right, number)
        return ConditionVerbalizer(self.ctx).whose_attribute(
            condition_plan.whose_attribute_name, number, value
        )

    def _value(self, expression, number: Number) -> VerbFragment:
        """Render a value expression agreeing with *number* (plural folds the chain)."""
        return self.ctx.child(expression, number=number)

    # ── THEN clause ───────────────────────────────────────────────────────────

    def _then_items(self, structure: RuleStructure) -> List[VerbFragment]:
        """*"there's a <Consequent> [whose <field> is <value> …]"* — the THEN-clause block."""
        intro: VerbFragment = ExistentialPhrase.for_number(
            Number.SINGULAR
        ).build_phrase(structure.consequent_type)
        binding_fragments = [
            self._binding_fragment(binding) for binding in structure.consequent_bindings
        ]
        if not binding_fragments:
            return [intro]
        return [BlockFragment(header=intro, items=binding_fragments)]

    def _binding_fragment(self, binding: ConsequentBinding) -> VerbFragment:
        """*"whose <field> is/are <value>"* — one consequent field binding."""
        number = Number.of(binding.is_plural_field)
        return ConditionVerbalizer(self.ctx).whose_attribute(
            binding.field_name, number, self._binding_value(binding)
        )

    def _binding_value(self, binding: ConsequentBinding) -> VerbFragment:
        """The binding's value: *"the <plural chain>"* (aggregated), bare plural, the group-key
        *"common …"* phrase, or the plain rendering."""
        if (
            binding.is_plural_field
            and binding.aggregation_status == AggregationStatus.AGGREGATED
        ):
            return PhraseFragment(
                parts=[
                    Articles.THE.as_fragment(),
                    self.ctx.child(binding.value_expression, number=Number.PLURAL),
                ]
            )
        if binding.is_plural_field:
            return self.ctx.child(binding.value_expression, number=Number.PLURAL)
        if binding.aggregation_status == AggregationStatus.GROUP_KEY:
            return self._group_key_value(binding.value_expression)
        return self.ctx.child(binding.value_expression)

    def _group_key_value(self, expression) -> VerbFragment:
        """*"the common <field> of the <Roots>"* — a binding that refers to a GROUP BY key."""
        chain, current = walk_chain(expression)
        if not chain or not isinstance(current, Variable):
            return self.ctx.child(expression)
        root_type = (
            current._type_.__name__
            if getattr(current, "_type_", None)
            else FallbackNouns.ENTITY.text
        )
        self.ctx.refer.mark_introduced(current)
        parts = build_path_parts(chain)
        field = list(reversed(parts))[0][0] if parts else root_type
        return GroupKeyPhrases.COMMON_OF.build_phrase(field, root_type)
