"""
Natural-language verbalization for EQL expression trees.

Public API mirrors ``explain_inference()``::

    from krrood.entity_query_language.verbalization import verbalize_query, verbalize_expression

    text = verbalize_query(query)
    text = verbalize_expression(expr)
"""

from krrood.entity_query_language.verbalization.context import VerbalizationContext
from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer

__all__ = [
    "verbalize_query",
    "verbalize_expression",
    "EQLVerbalizer",
    "VerbalizationContext",
]

_default_verbalizer = EQLVerbalizer()


def verbalize_query(query) -> str:
    """
    Verbalize a built or unbuilt EQL ``Query`` into a human-readable English sentence.

    Calls ``query.build()`` automatically before verbalizing.

    :param query: An EQL ``Query`` object (``Entity`` or ``SetOf``, possibly wrapped by ``an``/``the``).
    :return: A natural-language description of the query structure.
    """
    query.build()
    return _default_verbalizer.verbalize(query)


def verbalize_expression(expr) -> str:
    """
    Verbalize any EQL expression — including sub-expressions, conditions, predicates,
    and aggregators — into a human-readable English phrase.

    :param expr: Any ``SymbolicExpression`` instance.
    :return: A natural-language description of the expression.
    """
    return _default_verbalizer.verbalize(expr)
