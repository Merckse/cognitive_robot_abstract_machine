"""
The shared **specificity-selection** primitive of the grammar — one function and one
base class, so every "choose the most specific applicable alternative" decision is
written once.

* :func:`most_specific` — pick the single highest-keyed candidate (or ``None``).
* :class:`SpecificityRule` — a guarded, self-registering alternative chosen by
  :attr:`~SpecificityRule.priority`.  Subfamilies (the restriction registries) declare
  their own ``applies(...)`` signature and payload; this base owns only **registration**
  (``alternatives`` via ``__subclasses__``) and **selection** (``most_applicable``).

:class:`~krrood.entity_query_language.verbalization.grammar.phrase_rule.PhraseRule` is kin
— it is the same "guarded production rule chosen by specificity" idea — but is *not* a
:class:`SpecificityRule`: it is registered as a curated list of **instances** (so the
grammar stays EQL-queryable) and ranked by a richer key (construct MRO-depth + guardedness
+ tiebreak), not a flat ``priority``.  It therefore shares only :func:`most_specific`.

Reference: production-rule selection; the SFL "most delicate system wins" principle.
"""

from __future__ import annotations

from abc import ABC

from typing_extensions import (
    Any,
    Callable,
    ClassVar,
    List,
    Optional,
    Sequence,
    Type,
    TypeVar,
)

_T = TypeVar("_T")


def most_specific(candidates: Sequence[_T], key: Callable[[_T], Any]) -> Optional[_T]:
    """
    Return the single most-specific candidate by *key*, or ``None`` when empty.

    The shared selection primitive — used by
    :func:`~krrood.entity_query_language.verbalization.grammar.phrase_rule.select` over the
    grammar, by :class:`SpecificityRule` over the restriction registries, and anywhere a
    "most specific applicable alternative" is chosen — so the rule is written once.

    :param candidates: Items already filtered to those that apply.
    :param key: Specificity key; the maximum wins.
    :return: The most specific candidate, or ``None``.
    """
    return max(candidates, key=key, default=None)


class SpecificityRule(ABC):
    """
    A guarded alternative selected by specificity: the shared base of the small rule
    registries (restriction folding, restriction-subject resolution).

    An alternative is a **subclass** that implements an ``applies(...)`` guard (its
    signature is the subfamily's concern) and a payload method; alternatives
    *self-register* as the family's direct subclasses and are ranked by :attr:`priority`.
    """

    priority: ClassVar[int] = 0
    """Tiebreak when several alternatives apply (higher wins); mirrors ``PhraseRule.tiebreak``."""

    @classmethod
    def alternatives(cls) -> List[Type[SpecificityRule]]:
        """The alternative subclasses of this family (closed: direct subclasses)."""
        return list(cls.__subclasses__())

    @classmethod
    def most_applicable(cls, *args: Any) -> Optional[Type[SpecificityRule]]:
        """
        The most-specific alternative whose ``applies(*args)`` holds, or ``None``.

        *args* are forwarded verbatim to each alternative's ``applies`` classmethod, so
        the subfamily fixes that signature (e.g. ``(item, subject)``).
        """
        applicable = [alt for alt in cls.alternatives() if alt.applies(*args)]
        return most_specific(applicable, key=lambda alt: alt.priority)
