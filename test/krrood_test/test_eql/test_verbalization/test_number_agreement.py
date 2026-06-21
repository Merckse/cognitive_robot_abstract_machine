"""
Unit tests for grammatical-number agreement.

Number is now realised in one place — the
:class:`~krrood.entity_query_language.verbalization.rendering.morphology_processor.MorphologyProcessor`
pass.  The lexicon (e.g. ``Copulas.for_number``) only *tags* fragments with
:class:`Number`; these tests therefore run the pass to observe the realised surface.
"""

from __future__ import annotations

from krrood.entity_query_language.factories import an, and_, entity, variable
from krrood.entity_query_language.verbalization.fragments.base import (
    flatten_fragment_to_plain_text,
)
from krrood.entity_query_language.verbalization.pipeline import verbalize_expression
from krrood.entity_query_language.verbalization.rendering.morphology_processor import (
    MorphologyProcessor,
)
from krrood.entity_query_language.verbalization.vocabulary.english import (
    Copulas,
    ExistentialPhrase,
)
from krrood.entity_query_language.verbalization.vocabulary.words import Number

from ...dataset.department_and_employee import Employee


def _realised(fragment) -> str:
    """Apply the morphology pass (as ``build`` does), then flatten to plain text."""
    return flatten_fragment_to_plain_text(MorphologyProcessor().process(fragment))


def test_number_of_bridges_boolean_plan_features():
    assert Number.of(True) is Number.PLURAL
    assert Number.of(False) is Number.SINGULAR


def test_copula_agreement_is_applied_by_the_pass():
    assert _realised(Copulas.for_number(Number.SINGULAR)) == "is"
    assert _realised(Copulas.for_number(Number.PLURAL)) == "are"


def test_existential_noun_pluralised_by_the_pass():
    assert (
        _realised(ExistentialPhrase.for_number(Number.SINGULAR).build_phrase("Robot"))
        == "there's a Robot"
    )
    assert (
        _realised(ExistentialPhrase.for_number(Number.PLURAL).build_phrase("Robot"))
        == "there are Robots"
    )


# ── a plural subject governs its restriction and possessives (concord) ───────


def test_ordered_report_agrees_attribute_operator_and_possessive():
    e = variable(Employee, [])
    text = verbalize_expression(an(entity(e).where(e.salary > 5).ordered_by(e.salary)))
    assert text == (
        "Report Employees whose salaries are greater than 5, "
        "ordered by their salaries (ascending)"
    )


def test_ranking_subject_agrees_the_whose_restriction():
    e = variable(Employee, [])
    query = (
        entity(e)
        .where(e.salary > 1000)
        .ordered_by(e.starting_salary, descending=True)
        .limit(3)
    )
    text = verbalize_expression(query)
    assert text == (
        "Find the top three Employees by starting_salary "
        "whose salaries are greater than 1000"
    )


def test_plural_subject_agrees_a_range_restriction():
    e = variable(Employee, [])
    query = an(
        entity(e).where(and_(e.salary > 100, e.salary < 200)).ordered_by(e.salary)
    )
    text = verbalize_expression(query)
    assert text == (
        "Report Employees whose salaries are between 100 and 200, "
        "ordered by their salaries (ascending)"
    )


def test_singular_subject_is_unaffected():
    e = variable(Employee, [])
    text = verbalize_expression(an(entity(e).where(e.salary > 5)))
    assert text == "Find an Employee whose salary is greater than 5"


def test_intermediate_owner_in_a_longer_possessive_stays_singular():
    e = variable(Employee, [])
    query = (
        entity(e)
        .where(e.department.name == "Sales")
        .ordered_by(e.salary, descending=True)
        .limit(3)
    )
    text = verbalize_expression(query)
    # each Employee has one department, so the intermediate owner is not pluralised
    assert "their department" in text
    assert "their departments" not in text
