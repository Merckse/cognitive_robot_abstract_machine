"""
Tests for verbalizing EQL *match* expressions (``match`` / ``underspecified`` / ``match_variable``).

A match constructed via ``match`` or ``underspecified`` is a generative request → *"Generate"*;
a domain search (a ``MatchVariable``) → *"Find"*.  A match has two condition parts: the
construction-pattern equalities (the ``kwargs``) → *"given that"*, and the ``.where(...)``
conditions → *"where"*.  Each condition is its own point; equality assignments on the same object
are grouped (*"x, y, and z of the Position are 1, 2, and 3 respectively"*); an ``Ellipsis`` value
is a value to generate → folded into the header (*"… and predict its x, y, and z values"*).
"""

from __future__ import annotations

from dataclasses import dataclass

from krrood.entity_query_language.factories import match_variable, underspecified
from krrood.entity_query_language.verbalization.pipeline import (
    VerbalizationPipeline,
    verbalize_expression,
)
from krrood.entity_query_language.verbalization.rendering.formatter import (
    PlainFormatter,
)
from krrood.entity_query_language.verbalization.rendering.renderer import (
    HierarchicalRenderer,
)


@dataclass
class Position:
    """A 3-D position — three scalar attributes that group into one *"given that"* point."""

    x: float
    y: float
    z: float


def _hierarchical(expression) -> str:
    """:return: *expression* rendered as a plain-text indented bullet list (point per condition)."""
    return VerbalizationPipeline(HierarchicalRenderer(PlainFormatter())).verbalize(
        expression
    )


# ── Generate vs Find ─────────────────────────────────────────────────────────


def test_underspecified_opens_with_generate():
    """An underspecified (generative) match opens with *"Generate"*, not *"Find"*."""
    assert verbalize_expression(underspecified(Position)(x=1)).startswith("Generate")


def test_match_variable_object_opens_with_find():
    """A domain-search match (a ``MatchVariable``) opens with *"Find"*."""
    search = match_variable(Position, domain=None)
    search(x=1, y=2)  # records the kwargs on the match object
    assert verbalize_expression(search).startswith("Find")


# ── given that: grouped equality conditions ──────────────────────────────────


def test_grouped_attributes_say_respectively():
    """Several equalities on one object aggregate into one *"… respectively"* point."""
    text = verbalize_expression(underspecified(Position)(x=1, y=2, z=3))
    assert text == (
        "Generate a Position given that x, y, and z of the Position are 1, 2, and 3 respectively"
    )


def test_single_attribute_uses_is_not_respectively():
    """A single equality uses *"is"* and never *"respectively"*."""
    text = verbalize_expression(underspecified(Position)(x=5))
    assert text == "Generate a Position given that x of the Position is 5"
    assert "respectively" not in text


def test_given_that_is_its_own_block_with_one_point_per_group():
    """The *"given that"* part is a sub-header block; each group is its own point."""
    text = _hierarchical(underspecified(Position)(x=1, y=2, z=3))
    assert text == (
        "Generate a Position\n"
        "  given that\n"
        "    - x, y, and z of the Position are 1, 2, and 3 respectively"
    )


# ── predict: Ellipsis values folded into the header ──────────────────────────


def test_all_ellipsis_predicts_in_header():
    """All-``Ellipsis`` values are generated → *"and predict its … values"* in the header."""
    text = verbalize_expression(underspecified(Position)(x=..., y=..., z=...))
    assert text == "Generate a Position and predict its x, y, and z values"


def test_single_ellipsis_predicts_singular_value():
    """A single predicted attribute uses the singular *"value"*."""
    text = verbalize_expression(underspecified(Position)(x=...))
    assert text == "Generate a Position and predict its x value"


def test_mixed_concrete_and_ellipsis():
    """Concrete kwargs go to *"given that"*; ``Ellipsis`` kwargs are predicted in the header."""
    text = verbalize_expression(underspecified(Position)(x=1, y=...))
    assert text == (
        "Generate a Position and predict its y value given that x of the Position is 1"
    )


# ── where: free conditions as points ─────────────────────────────────────────


def test_where_conditions_are_their_own_block():
    """``.where(...)`` conditions form a *"where"* block, one point each, distinct from
    *"given that"*."""
    match = underspecified(Position)(x=1)
    match.resolve()
    match.where(match.variable.y > 2)
    text = _hierarchical(match)
    assert text == (
        "Generate a Position\n"
        "  given that\n"
        "    - x of the Position is 1\n"
        "  where\n"
        "    - its y is greater than 2"
    )


def test_where_only_match_has_no_given_that_block():
    """A match with only ``where`` conditions renders just the *"where"* block."""
    match = underspecified(Position)()
    match.resolve()
    match.where(match.variable.x > 0)
    text = _hierarchical(match)
    assert text == "Generate a Position\n  where\n    - its x is greater than 0"
