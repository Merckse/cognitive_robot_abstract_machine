"""
English morphology facade — the single point of access to inflection.

Every call into the ``inflect`` library is concentrated behind this small, named
API.  This mirrors the **MorphologyProcessor** of SimpleNLG (Gatt & Reiter 2009),
which isolates morphological realisation — pluralisation, indefinite-article
selection, ordinals — from content selection and surface layout.  No other
verbalization module imports ``inflect`` directly, so the choice of morphology
engine is a single, replaceable dependency.

Reference:

* Gatt, A. & Reiter, E. (2009), "SimpleNLG: A realisation engine for practical
  applications", *Proceedings of ENLG 2009* — dedicated morphology processor.
"""

from __future__ import annotations

import inflect

from typing_extensions import Dict

#: The one shared ``inflect`` engine for the whole verbalization subsystem.
_engine = inflect.engine()

#: Domain-specific exceptions consulted *before* ``inflect`` — the override hook for
#: terms the general engine gets wrong (irregular/invariant plurals, acronym articles).
_plural_overrides: Dict[str, str] = {}
_article_overrides: Dict[str, str] = {}


def register_plural(singular: str, plural_form: str) -> None:
    """Register a domain-specific plural so the morphology pass uses it instead of ``inflect``.

    For an invariant noun, register the word as its own plural
    (``register_plural("sheep", "sheep")``).

    :param singular: The singular surface form.
    :param plural_form: The plural surface form to emit.
    """
    _plural_overrides[singular] = plural_form


def register_indefinite_article(word: str, article: str) -> None:
    """Register a domain-specific indefinite article (``"a"`` / ``"an"``) for *word* — e.g.
    ``register_indefinite_article("FBI", "an")`` for an initialism ``inflect`` mis-reads.

    :param word: The word the article precedes.
    :param article: ``"a"`` or ``"an"``.
    """
    _article_overrides[word] = article


def clear_overrides() -> None:
    """Drop all registered morphology overrides (chiefly for test isolation)."""
    _plural_overrides.clear()
    _article_overrides.clear()


def plural(word: str) -> str:
    """
    Return the plural form of *word* unconditionally.

    :param word: An English noun (assumed singular).
    :return: The plural form (e.g. ``"Robot"`` → ``"Robots"``).
    :rtype: str
    """
    return _plural_overrides.get(word) or _engine.plural(word)


def ensure_plural(word: str) -> str:
    """
    Return *word* in plural form without double-pluralising already-plural words.

    Uses :meth:`inflect.engine.singular_noun` to detect existing plurals.

    :param word: An English noun in either number.
    :return: The plural form of *word*.
    :rtype: str
    """
    if word in _plural_overrides:
        return _plural_overrides[word]
    if word in _plural_overrides.values():  # already a registered plural form
        return word
    return word if _engine.singular_noun(word) else _engine.plural(word)


def is_plural(word: str) -> bool:
    """
    Return ``True`` when *word* is already in plural form.

    :param word: An English noun.
    :rtype: bool
    """
    if word in _plural_overrides.values():
        return True
    if word in _plural_overrides:
        return False
    return bool(_engine.singular_noun(word))


def indefinite_article(following_word: str) -> str:
    """
    Return the indefinite article (``"a"`` / ``"an"``) for *following_word*,
    chosen phonologically (e.g. ``"hour"`` → ``"an"``, ``"robot"`` → ``"a"``).

    :param following_word: The word the article precedes.
    :return: ``"a"`` or ``"an"``.
    :rtype: str
    """
    if following_word in _article_overrides:
        return _article_overrides[following_word]
    return _engine.a(following_word).split()[0]


def ordinal(index: int) -> str:
    """
    Return the English ordinal *word* for a zero-based *index* (``0`` → ``"first"``).

    :param index: Zero-based integer index.
    :return: English ordinal word (e.g. ``"first"``, ``"second"``).
    :rtype: str
    """
    return _engine.ordinal(_engine.number_to_words(index + 1))
