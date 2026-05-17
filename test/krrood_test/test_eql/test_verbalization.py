"""
Tests for EQL verbalization.

Coverage:
- Unit tests for individual node types (Variable, Literal, Attribute, Comparator,
  AND/OR/Not, ForAll/Exists, aggregators, Entity, SetOf).
- Integration tests derived from existing EQL tests:
    test_presentation_example, test_for_all, test_order_by_aggregation,
    test_complex_having_success, test_nested_rule_explanation,
    test_explanation_condition_graph_and_visualize,
    test_equivalent_to_contains_type_using_exists.
- Predicate template tests: HasType, ContainsType, custom predicates,
  and graceful fallback when no template is set.
"""

from __future__ import annotations

from abc import abstractmethod
from dataclasses import dataclass
from typing import List, Any

import pytest

import krrood.entity_query_language.factories as eql
from krrood.entity_query_language.core.variable import Literal
from krrood.entity_query_language.factories import (
    an,
    a,
    entity,
    set_of,
    variable,
    variable_from,
    flat_variable,
    match_variable,
    inference,
    for_all,
    exists,
    not_,
    and_,
    or_,
)
from krrood.entity_query_language.predicate import HasType, Predicate
from krrood.entity_query_language.verbalization import (
    EQLVerbalizer,
    VerbalizationContext,
    verbalize_expression,
    verbalize_query,
)
from ..dataset.department_and_employee import Department, Employee


@dataclass
class _Task:
    name: str
    completed: bool


@dataclass
class _Robot:
    name: str
    battery: int
    tasks: List[_Task]
from ..dataset.semantic_world_like_classes import (
    Apple,
    Body,
    Cabinet,
    Connection,
    Container,
    ContainsType,
    Drawer,
    FixedConnection,
    FruitBox,
    Handle,
    PrismaticConnection,
)


# ── Helpers ────────────────────────────────────────────────────────────────────

def _v(expr) -> str:
    """Shorthand: verbalize an expression with a fresh context."""
    return verbalize_expression(expr)


def _vq(query) -> str:
    """Shorthand: verbalize a query."""
    return verbalize_query(query)


# ── Unit tests: leaves ─────────────────────────────────────────────────────────


def test_verbalize_variable_first_mention():
    x = variable(int, [1, 2])
    assert _v(x) == "an int"


def test_verbalize_variable_article_consonant():
    x = variable(Body, [])
    assert _v(x) == "a Body"


def test_verbalize_variable_coreference():
    ctx = VerbalizationContext()
    v = EQLVerbalizer()
    x = variable(Handle, [])
    first = v.verbalize(x, ctx)
    second = v.verbalize(x, ctx)
    assert first == "a Handle"
    assert second == "the Handle"


def test_verbalize_literal_plain_value():
    lit = Literal(_value_=42)
    assert "42" in _v(lit)


def test_verbalize_literal_type_object():
    lit = Literal(_value_=Apple)
    assert _v(lit) == "Apple"


def test_verbalize_literal_tuple_of_types():
    lit = Literal(_value_=(Apple, Body))
    text = _v(lit)
    assert "Apple" in text and "Body" in text


# ── Unit tests: MappedVariable chain ──────────────────────────────────────────


def test_verbalize_attribute_single_hop_is_possessive():
    emp = variable(Employee, [])
    text = _v(emp.salary)
    assert "Employee" in text
    assert "salary" in text
    assert "'s" in text


def test_verbalize_attribute_multi_hop_uses_of_form():
    # cabinet.container is one hop → possessive
    cab = variable(Cabinet, [])
    one_hop = _v(cab.container)
    assert "'s" in one_hop

    # Simulate a two-hop chain via attribute access
    emp = variable(Employee, [])
    dept = emp.department
    dept_name = dept.name  # two hops: Employee → department → name
    text = _v(dept_name)
    assert "Employee" in text
    assert "department" in text
    assert "name" in text
    # Multi-hop should NOT use possessive at the root
    assert " of " in text


def test_verbalize_index_access_merged_into_attribute():
    @dataclass
    class Robot:
        tasks: list

    r = variable(Robot, [])
    text = _v(r.tasks[0])
    assert "Robot" in text
    assert "tasks" in text
    assert "[0]" in text


def test_verbalize_flat_variable_delegates_to_child():
    cab = variable(Cabinet, [])
    drawer_var = flat_variable(cab.drawers)
    text = _v(drawer_var)
    assert "Cabinet" in text
    assert "drawers" in text


# ── Unit tests: comparators ────────────────────────────────────────────────────


@pytest.mark.parametrize("op,word", [
    ("__gt__", "greater than"),
    ("__lt__", "less than"),
    ("__ge__", "at least"),
    ("__le__", "at most"),
    ("__eq__", "equals"),
    ("__ne__", "does not equal"),
])
def test_verbalize_comparator_operators(op, word):
    x = variable(int, [1])
    comp = getattr(x, op)(5)
    text = _v(comp)
    assert word in text


def test_verbalize_comparator_greater_than():
    x = variable(int, [])
    text = _v(x > 10)
    assert "greater than" in text
    assert "10" in text


def test_verbalize_comparator_at_least():
    x = variable(int, [])
    text = _v(x >= 10)
    assert "at least" in text


# ── Unit tests: logical operators ─────────────────────────────────────────────


def test_verbalize_and_chain_flattening():
    x = variable(int, [])
    cond = and_(x > 1, x < 10, x != 5)
    text = _v(cond)
    assert "greater than" in text
    assert "less than" in text
    assert "does not equal" in text
    assert ", and " in text


def test_verbalize_and_stops_at_or():
    x = variable(int, [])
    cond = and_(x > 1, or_(x < 10, x == 5))
    text = _v(cond)
    assert "greater than" in text
    assert "either" in text  # the inner OR produces "either ..."


def test_verbalize_or_chain():
    x = variable(int, [])
    cond = or_(x > 10, x < 0)
    text = _v(cond)
    assert "either" in text
    assert "greater than" in text
    assert "less than" in text


def test_verbalize_not():
    x = variable(int, [])
    text = _v(not_(x > 5))
    assert "not" in text
    assert "greater than" in text


# ── Unit tests: aggregators ────────────────────────────────────────────────────


def test_verbalize_count():
    x = variable(int, [1, 2])
    text = _v(eql.count(x))
    assert "count" in text and "int" in text


def test_verbalize_average():
    x = variable(int, [1, 2])
    text = _v(eql.average(x))
    assert "average" in text


def test_verbalize_sum():
    x = variable(int, [1, 2])
    text = _v(eql.sum(x))
    assert "sum" in text


def test_verbalize_max_min():
    x = variable(int, [1, 2])
    assert "maximum" in _v(eql.max(x))
    assert "minimum" in _v(eql.min(x))


# ── Integration: target test cases ────────────────────────────────────────────


def test_verbalize_presentation_example():
    robots = [
        _Robot("Robot1", 100, [_Task("Task1", True)]),
        _Robot("Robot3", 75, [_Task("Task5", False)]),
    ]
    r = variable(_Robot, robots)
    q = an(entity(r).where(r.battery > 50, not_(r.tasks[0].completed)))
    text = _vq(q)

    assert "Find" in text
    assert "Robot" in text
    assert "battery" in text
    assert "greater than" in text
    assert "50" in text
    assert "not" in text
    assert "completed" in text


def test_verbalize_for_all(handles_and_containers_world):
    world = handles_and_containers_world
    cabinets = variable(Cabinet, world.views)
    container_var = variable(Container, world.bodies)
    the_cabinet_container = eql.the(
        entity(container_var).where(container_var.name == "Container2")
    )
    query = an(
        entity(the_cabinet_container).where(
            for_all(cabinets.container, the_cabinet_container == cabinets.container)
        )
    )
    text = _vq(query)

    assert "for all" in text
    assert "Cabinet" in text
    assert "Container" in text
    assert "equals" in text


def test_verbalize_order_by_aggregation(handles_and_containers_world):
    world = handles_and_containers_world
    cabinet = variable(Cabinet, domain=world.views)
    drawer = flat_variable(cabinet.drawers)
    query = an(
        entity(cabinet)
        .grouped_by(cabinet)
        .ordered_by(eql.count(drawer), descending=True)
    )
    text = _vq(query)

    assert "Cabinet" in text
    assert "grouped by" in text
    assert "ordered by" in text
    assert "count" in text
    assert "drawers" in text
    assert "descending" in text


def test_verbalize_complex_having(departments_and_employees_fixture):
    departments, employees = departments_and_employees_fixture
    emp = variable(Employee, domain=None)
    department = emp.department
    avg_salary = eql.average(emp.salary)
    query = a(
        set_of(department, avg_salary).grouped_by(department).having(avg_salary > 30000)
    )
    text = _vq(query)

    assert "Employee" in text
    assert "department" in text
    assert "average" in text
    assert "salary" in text
    assert "grouped by" in text
    assert "having" in text
    assert "30000" in text


def test_verbalize_nested_rule(doors_and_drawers_world):
    world = doors_and_drawers_world
    handle = variable(Handle, world.bodies)
    prismatic_connection = variable(PrismaticConnection, world.connections)
    fixed_connection = match_variable(FixedConnection, world.connections)(
        parent=prismatic_connection.child, child=handle
    )
    drawer_var = inference(Drawer)(
        container=fixed_connection.parent, handle=fixed_connection.child
    )
    text = verbalize_expression(drawer_var)

    assert "Drawer" in text
    assert "FixedConnection" in text
    assert "parent" in text or "container" in text
    assert "child" in text or "handle" in text


def test_verbalize_condition_graph_example():
    @dataclass(frozen=True)
    class Item:
        value: int

    val = variable_from([6])
    item_var = inference(Item)(value=val)
    query = entity(item_var).where(or_(and_(val > 5, val < 10), val == 11))
    text = _vq(query)

    assert "Item" in text
    assert "either" in text
    assert "greater than" in text
    assert "less than" in text
    assert "equals" in text


def test_verbalize_has_type_with_exists():
    fb1 = FruitBox("FruitBox1", [Apple("apple"), Body("Body1")])
    fb2 = FruitBox("FruitBox2", [Body("Body3")])
    fb = variable(FruitBox, domain=[fb1, fb2])
    query = an(
        entity(fb).where(
            exists(fb, HasType(flat_variable(fb.fruits), Apple))
        )
    )
    text = _vq(query)

    assert "FruitBox" in text
    assert "exists" in text
    assert "Apple" in text
    assert "is of type" in text


# ── Predicate template tests ───────────────────────────────────────────────────


def test_verbalize_has_type_template():
    fruit = variable(Body, [])
    pred = HasType(fruit, Apple)
    text = _v(pred)
    assert "Body" in text
    assert "is of type" in text
    assert "Apple" in text


def test_verbalize_has_type_tuple_of_types():
    fruit = variable(Body, [])
    pred = HasType(fruit, (Apple, Body))
    text = _v(pred)
    assert "is of type" in text
    assert "Apple" in text
    assert "Body" in text


def test_verbalize_contains_type_template():
    fb = variable(FruitBox, [])
    pred = ContainsType(fb.fruits, Apple)
    text = _v(pred)
    assert "contains an instance of" in text
    assert "Apple" in text
    assert "fruits" in text


def test_verbalize_custom_predicate_robotics_domain(handles_and_containers_world):
    @dataclass(eq=False)
    class IsReachable(Predicate):
        _verbalization_template_ = "{body} is reachable"
        body: Any

        def __call__(self) -> bool:
            return True

    world = handles_and_containers_world
    handle = variable(Handle, world.bodies)
    pred = IsReachable(handle)
    text = _v(pred)
    assert "Handle" in text
    assert "is reachable" in text


def test_verbalize_custom_predicate_employee_domain():
    @dataclass(eq=False)
    class WorksInDepartment(Predicate):
        _verbalization_template_ = "{employee} works in {department}"
        employee: Any
        department: Any

        def __call__(self) -> bool:
            return self.employee.department == self.department

    emp = variable(Employee, [])
    dept = variable(Department, [])
    pred = WorksInDepartment(emp, dept)
    text = _v(pred)
    assert "Employee" in text
    assert "works in" in text
    assert "Department" in text


def test_verbalize_predicate_no_template_fallback():
    @dataclass(eq=False)
    class HasHighSalary(Predicate):
        employee: Any
        threshold: float

        def __call__(self) -> bool:
            return self.employee.salary > self.threshold

    emp = variable(Employee, [])
    pred = HasHighSalary(emp, 50000.0)
    text = _v(pred)
    # No template → generic form mentioning the class and the arg names
    assert "HasHighSalary" in text
    assert "Employee" in text


def test_verbalize_predicate_no_template_no_args_fallback():
    @dataclass(eq=False)
    class IsActive(Predicate):
        entity: Any

        def __call__(self) -> bool:
            return True

    emp = variable(Employee, [])
    pred = IsActive(emp)
    text = _v(pred)
    assert "IsActive" in text


# ── Fixture ────────────────────────────────────────────────────────────────────


@pytest.fixture
def departments_and_employees_fixture():
    d1 = Department("HR")
    d2 = Department("Finance")
    e1 = Employee("John", d1, 10000)
    e2 = Employee("Anna", d2, 40000)
    return [d1, d2], [e1, e2]
