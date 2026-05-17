"""
Natural-language verbalization for EQL expression trees.

Public API mirrors ``explain_inference()``::

    from krrood.entity_query_language.verbalization import verbalize_expression

    text = verbalize_expression(expression)
"""
from krrood.entity_query_language.query.query import Query
from krrood.entity_query_language.verbalization.context import VerbalizationContext
from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer

__all__ = [
    "verbalize_expression",
    "EQLVerbalizer",
    "VerbalizationContext",
]

_default_verbalizer = EQLVerbalizer()


def verbalize_expression(expr) -> str:
    """
    Verbalize any EQL expression — including sub-expressions, conditions, predicates,
    and aggregators — into a human-readable English phrase.

    :param expr: Any ``SymbolicExpression`` instance.
    :return: A natural-language description of the expression.
    """
    if isinstance(expr, Query):
        expr.build()
    return _default_verbalizer.verbalize(expr)
