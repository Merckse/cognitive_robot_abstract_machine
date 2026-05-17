from __future__ import annotations

import operator
from typing import Optional

from krrood.entity_query_language.core.base_expressions import SymbolicExpression
from krrood.entity_query_language.core.mapped_variable import (
    Attribute,
    Call,
    FlatVariable,
    Index,
    MappedVariable,
)
from krrood.entity_query_language.core.variable import (
    InstantiatedVariable,
    Literal,
    Variable,
)
from krrood.entity_query_language.operators.aggregators import (
    Average,
    Count,
    CountAll,
    Max,
    Min,
    Mode,
    MultiMode,
    Sum,
)
from krrood.entity_query_language.operators.comparator import Comparator, not_contains
from krrood.entity_query_language.operators.core_logical_operators import AND, OR, Not
from krrood.entity_query_language.operators.logical_quantifiers import Exists, ForAll
from krrood.entity_query_language.query.operations import (
    GroupedBy,
    Having,
    OrderedBy,
    Where,
)
from krrood.entity_query_language.query.quantifiers import An, ResultQuantifier, The
from krrood.entity_query_language.query.query import Entity, SetOf
from krrood.entity_query_language.verbalization.context import VerbalizationContext, _article

_OP_WORDS = {
    operator.eq: "equals",
    operator.ne: "does not equal",
    operator.lt: "is less than",
    operator.le: "is at most",
    operator.gt: "is greater than",
    operator.ge: "is at least",
    operator.contains: "contains",
    not_contains: "does not contain",
}


class EQLVerbalizer:
    """
    Visitor-based verbalizer: maps an EQL expression tree to readable English.

    Usage::

        verbalizer = EQLVerbalizer()
        text = verbalizer.verbalize(query)

    Each ``_v_<ClassName>_`` method handles one node type.  Unknown types fall
    back to :meth:`_v_default_` which returns the node's ``_name_`` property.
    """

    # ── Dispatcher ─────────────────────────────────────────────────────────────

    def verbalize(
        self,
        expr: SymbolicExpression,
        ctx: Optional[VerbalizationContext] = None,
    ) -> str:
        if ctx is None:
            ctx = VerbalizationContext()
        method = getattr(self, f"_v_{type(expr).__name__}_", self._v_default_)
        return method(expr, ctx)

    # ── Leaves ─────────────────────────────────────────────────────────────────

    def _v_Variable_(self, expr: Variable, ctx: VerbalizationContext) -> str:
        return ctx.noun_for(expr)

    def _v_Literal_(self, expr: Literal, ctx: VerbalizationContext) -> str:
        return ctx.type_name_of_value(expr._value_)

    def _v_ExternallySetVariable_(self, expr, ctx: VerbalizationContext) -> str:
        type_name = expr._type_.__name__ if getattr(expr, "_type_", None) else "variable"
        return f"{_article(type_name)} {type_name}"

    # ── MappedVariables ────────────────────────────────────────────────────────

    def _v_Attribute_(self, expr: Attribute, ctx: VerbalizationContext) -> str:
        return self._verbalize_mapped_chain_(expr, ctx)

    def _v_Index_(self, expr: Index, ctx: VerbalizationContext) -> str:
        return self._verbalize_mapped_chain_(expr, ctx)

    def _v_Call_(self, expr: Call, ctx: VerbalizationContext) -> str:
        return self._verbalize_mapped_chain_(expr, ctx)

    def _v_FlatVariable_(self, expr: FlatVariable, ctx: VerbalizationContext) -> str:
        return self.verbalize(expr._child_, ctx)

    def _verbalize_mapped_chain_(self, expr: MappedVariable, ctx: VerbalizationContext) -> str:
        """
        Natural-language path for a MappedVariable chain.

        * Single-hop ``Attribute`` directly on a root: possessive —
          ``"Robot's battery"``.
        * Longer or mixed chains: ``"of"`` form —
          ``"completed of tasks[0] of the Robot"``.
        """
        chain: list[MappedVariable] = []
        current = expr
        while isinstance(current, MappedVariable):
            chain.append(current)
            current = current._child_
        root_text = self.verbalize(current, ctx)
        chain.reverse()  # root-side first

        path_parts = self._build_path_parts_(chain)

        if len(path_parts) == 1 and isinstance(expr, Attribute):
            return f"{root_text}'s {path_parts[0]}"
        # Multi-hop: "leaf of ... of root"
        return " of ".join(reversed(path_parts)) + f" of {root_text}"

    def _build_path_parts_(self, chain: list) -> list[str]:
        """Build readable string fragments for a root-to-leaf MappedVariable chain."""
        parts: list[str] = []
        i = 0
        while i < len(chain):
            node = chain[i]
            if isinstance(node, Attribute):
                name = node._attribute_name_
                # Eagerly absorb immediately following Index nodes into the attr name.
                while i + 1 < len(chain) and isinstance(chain[i + 1], Index):
                    i += 1
                    name += f"[{repr(chain[i]._key_)}]"
                parts.append(name)
            elif isinstance(node, Index):
                parts.append(f"[{repr(node._key_)}]")
            elif isinstance(node, Call):
                parts.append("()")
            elif isinstance(node, FlatVariable):
                pass  # FlatVariable handled by _v_FlatVariable_
            i += 1
        return parts

    # ── Instantiated (predicates / inference variables) ────────────────────────

    def _v_InstantiatedVariable_(
        self, expr: InstantiatedVariable, ctx: VerbalizationContext
    ) -> str:
        template: Optional[str] = getattr(expr._type_, "_verbalization_template_", None)
        if template is not None:
            kwargs = {
                name: (
                    ctx.type_name_of_value(child._value_)
                    if isinstance(child, Literal)
                    else self.verbalize(child, ctx)
                )
                for name, child in expr._child_vars_.items()
            }
            return template.format(**kwargs)

        type_name = getattr(expr._type_, "__name__", str(expr._type_))
        if expr._child_vars_:
            args_str = ", ".join(
                f"{name}={ctx.type_name_of_value(child._value_) if isinstance(child, Literal) else self.verbalize(child, ctx)}"
                for name, child in expr._child_vars_.items()
            )
            return f"{_article(type_name)} {type_name}({args_str})"
        return f"{_article(type_name)} {type_name}"

    # ── Logical operators ──────────────────────────────────────────────────────

    def _v_AND_(self, expr: AND, ctx: VerbalizationContext) -> str:
        parts = [self.verbalize(c, ctx) for c in ctx.flatten_same_type(expr, AND)]
        if len(parts) == 1:
            return parts[0]
        return ", ".join(parts[:-1]) + f", and {parts[-1]}"

    def _v_OR_(self, expr: OR, ctx: VerbalizationContext) -> str:
        parts = [self.verbalize(c, ctx) for c in ctx.flatten_same_type(expr, OR)]
        if len(parts) == 1:
            return parts[0]
        return "either " + ", ".join(parts[:-1]) + f", or {parts[-1]}"

    def _v_Not_(self, expr: Not, ctx: VerbalizationContext) -> str:
        return f"not ({self.verbalize(expr._child_, ctx)})"

    # ── Quantifiers ────────────────────────────────────────────────────────────

    def _v_ForAll_(self, expr: ForAll, ctx: VerbalizationContext) -> str:
        var_text = self.verbalize(expr.variable, ctx)
        cond_text = self.verbalize(expr.condition, ctx)
        return f"for all {var_text}, {cond_text}"

    def _v_Exists_(self, expr: Exists, ctx: VerbalizationContext) -> str:
        var_text = self.verbalize(expr.variable, ctx)
        cond_text = self.verbalize(expr.condition, ctx)
        return f"there exists {var_text} such that {cond_text}"

    # ── Comparators ────────────────────────────────────────────────────────────

    def _v_Comparator_(self, expr: Comparator, ctx: VerbalizationContext) -> str:
        left = self.verbalize(expr.left, ctx)
        right = self.verbalize(expr.right, ctx)
        op_word = _OP_WORDS.get(expr.operation, expr._name_)
        return f"{left} {op_word} {right}"

    # ── Aggregators ────────────────────────────────────────────────────────────

    def _v_Count_(self, expr: Count, ctx: VerbalizationContext) -> str:
        return f"count of {self.verbalize(expr._child_, ctx)}"

    def _v_CountAll_(self, expr: CountAll, ctx: VerbalizationContext) -> str:
        return "count of all"

    def _v_Sum_(self, expr: Sum, ctx: VerbalizationContext) -> str:
        return f"sum of {self.verbalize(expr._child_, ctx)}"

    def _v_Average_(self, expr: Average, ctx: VerbalizationContext) -> str:
        return f"average of {self.verbalize(expr._child_, ctx)}"

    def _v_Max_(self, expr: Max, ctx: VerbalizationContext) -> str:
        return f"maximum {self.verbalize(expr._child_, ctx)}"

    def _v_Min_(self, expr: Min, ctx: VerbalizationContext) -> str:
        return f"minimum {self.verbalize(expr._child_, ctx)}"

    def _v_Mode_(self, expr: Mode, ctx: VerbalizationContext) -> str:
        return f"mode of {self.verbalize(expr._child_, ctx)}"

    def _v_MultiMode_(self, expr: MultiMode, ctx: VerbalizationContext) -> str:
        return f"all modes of {self.verbalize(expr._child_, ctx)}"

    # ── Query: Entity and SetOf ────────────────────────────────────────────────

    def _v_Entity_(self, expr: Entity, ctx: VerbalizationContext) -> str:
        if expr._id_ in ctx.seen:
            return f"the {ctx.seen[expr._id_]}"

        expr.build()
        is_the = (
            expr._quantifier_builder_ is not None
            and expr._quantifier_builder_.type is The
        )
        var = expr.selected_variable

        if isinstance(var, Entity):
            selected = self._verbalize_entity_as_noun_(var, ctx)
        elif var is None:
            selected_type = "entity"
            ctx.seen[expr._id_] = selected_type
            selected = "entities"
        elif is_the:
            selected_type = var._type_.__name__ if getattr(var, "_type_", None) else "entity"
            ctx.seen[var._id_] = selected_type
            ctx.seen[expr._id_] = selected_type
            selected = f"the unique {selected_type}"
        else:
            selected = self.verbalize(var, ctx)
            selected_type = ctx.seen.get(getattr(var, "_id_", None), "entity")
            ctx.seen[expr._id_] = selected_type

        return self._verbalize_query_body_(expr, ctx, f"Find {selected}")

    def _verbalize_entity_as_noun_(self, expr: Entity, ctx: VerbalizationContext) -> str:
        """
        Compact form used when an ``Entity`` acts as the selected variable of an outer query.

        Produces ``"the unique Container where its name equals …"`` rather than
        ``"Find the unique Container, such that …"``.
        """
        if expr._id_ in ctx.seen:
            return f"the {ctx.seen[expr._id_]}"

        expr.build()
        is_the = (
            expr._quantifier_builder_ is not None
            and expr._quantifier_builder_.type is The
        )
        var = expr.selected_variable
        selected_type = var._type_.__name__ if var and getattr(var, "_type_", None) else "entity"

        ctx.seen[expr._id_] = selected_type
        if var is not None:
            ctx.seen[var._id_] = selected_type

        if is_the:
            article_noun = f"the unique {selected_type}"
        else:
            article_noun = f"{_article(selected_type)} {selected_type}"

        where_expr = expr._where_expression_
        if where_expr is not None:
            cond = self.verbalize(where_expr.condition, ctx)
            return f"{article_noun} where {cond}"
        return article_noun

    def _v_SetOf_(self, expr: SetOf, ctx: VerbalizationContext) -> str:
        expr.build()
        vars_str = ", ".join(self.verbalize(v, ctx) for v in expr._selected_variables_)
        return self._verbalize_query_body_(expr, ctx, f"Find sets of ({vars_str})")

    def _verbalize_query_body_(self, expr, ctx: VerbalizationContext, prefix: str) -> str:
        """Append where / grouped-by / having / ordered-by clauses to *prefix*."""
        parts = [prefix]

        where_expr = expr._where_expression_
        grouped_expr = expr._grouped_by_expression_
        having_expr = expr._having_expression_

        if where_expr is not None:
            parts.append(f"such that {self.verbalize(where_expr.condition, ctx)}")

        if grouped_expr is not None and grouped_expr.variables_to_group_by:
            groups = [self.verbalize(v, ctx) for v in grouped_expr.variables_to_group_by]
            parts.append(f"grouped by {', '.join(groups)}")

        if having_expr is not None:
            parts.append(f"having {self.verbalize(having_expr.condition, ctx)}")

        ob = expr._ordered_by_builder_
        if ob is not None:
            direction = "descending" if ob.descending else "ascending"
            parts.append(f"ordered by {self.verbalize(ob.variable, ctx)} ({direction})")

        return ", ".join(parts)

    # ── Result quantifiers (transparent wrappers) ──────────────────────────────

    def _v_An_(self, expr: An, ctx: VerbalizationContext) -> str:
        return self.verbalize(expr._child_, ctx)

    def _v_The_(self, expr: The, ctx: VerbalizationContext) -> str:
        return self.verbalize(expr._child_, ctx)

    def _v_ResultQuantifier_(self, expr: ResultQuantifier, ctx: VerbalizationContext) -> str:
        return self.verbalize(expr._child_, ctx)

    # ── Filter wrappers (delegate to their condition) ──────────────────────────

    def _v_Where_(self, expr: Where, ctx: VerbalizationContext) -> str:
        return self.verbalize(expr.condition, ctx)

    def _v_Having_(self, expr: Having, ctx: VerbalizationContext) -> str:
        return self.verbalize(expr.condition, ctx)

    def _v_GroupedBy_(self, expr: GroupedBy, ctx: VerbalizationContext) -> str:
        if expr.variables_to_group_by:
            groups = [self.verbalize(v, ctx) for v in expr.variables_to_group_by]
            return f"grouped by {', '.join(groups)}"
        return "grouped"

    def _v_OrderedBy_(self, expr: OrderedBy, ctx: VerbalizationContext) -> str:
        direction = "descending" if expr.descending else "ascending"
        return f"ordered by {self.verbalize(expr.variable, ctx)} ({direction})"

    # ── Fallback ───────────────────────────────────────────────────────────────

    def _v_default_(self, expr: SymbolicExpression, ctx: VerbalizationContext) -> str:
        return expr._name_
