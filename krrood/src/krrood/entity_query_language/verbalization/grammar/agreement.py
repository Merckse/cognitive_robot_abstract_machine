"""
Number-**agreement helpers** for directly-built leaves.

The grammar/lexicon mostly express number by *tagging* a fragment ``number=PLURAL`` and
letting the
:class:`~krrood.entity_query_language.verbalization.rendering.morphology_processor.MorphologyProcessor`
pass realise it.  This module is the tiny seam for leaves that are built *directly* (not via
``ctx.child`` recursion) yet still need a number tag — currently just the copula.
"""

from __future__ import annotations

from krrood.entity_query_language.verbalization.fragments.base import VerbFragment
from krrood.entity_query_language.verbalization.fragments.factory import role
from krrood.entity_query_language.verbalization.fragments.features import Number
from krrood.entity_query_language.verbalization.fragments.roles import SemanticRole
from krrood.entity_query_language.verbalization.vocabulary.english import Copulas


def copula(number: Number) -> VerbFragment:
    """The copula tagged with *number*; the morphology pass agrees it (*is* / *are*)."""
    return role(Copulas.IS.text, SemanticRole.OPERATOR, number=number)
