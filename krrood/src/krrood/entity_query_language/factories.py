"""
User interface (grammar & vocabulary) for entity query language.
"""

from __future__ import annotations

import inspect
import operator
from inspect import isclass
from uuid import UUID

from typing_extensions import Iterable, List

from krrood.entity_query_language.core.base_expressions import (
    SymbolicExpression,
    TruthValueOperator,
)
from krrood.entity_query_language.core.mapped_variable import (
    FlatVariable,
    CanBehaveLikeAVariable,
    Attribute,
)
from krrood.entity_query_language.core.variable import (
    DomainType,
    Literal,
    ExternallySetVariable,
)
from krrood.entity_query_language.enums import DomainSource
from krrood.entity_query_language.exceptions import UnsupportedExpressionTypeForDistinct
from krrood.entity_query_language.operators.aggregators import (
    Max,
    Min,
    Sum,
    Average,
    Count,
    CountAll,
    Mode,
    MultiMode,
)
from krrood.entity_query_language.operators.comparator import Comparator
from krrood.entity_query_language.operators.concatenation import Concatenation
from krrood.entity_query_language.operators.conditionals import CaseWhen
from krrood.entity_query_language.operators.core_logical_operators import (
    chained_logic,
    AND,
    OR,
)
from krrood.entity_query_language.operators.logical_quantifiers import ForAll, Exists
from krrood.entity_query_language.predicate import *  # type: ignore
from krrood.entity_query_language.predicate import symbolic_function
from krrood.entity_query_language.query.match import (
    Match,
    MatchVariable,
)
from krrood.entity_query_language.query.quantifiers import (
    ResultQuantificationConstraint,
    An,
    The,
    ResultQuantifier,
)
from krrood.entity_query_language.query.query import Entity, SetOf, Query
from krrood.entity_query_language.rules.conclusion import Add
from krrood.entity_query_language.rules.conclusion_selector import (
    Refinement,
    Alternative,
    Next,
)
from krrood.entity_query_language.utils import is_iterable
from krrood.symbol_graph.symbol_graph import Symbol, SymbolGraph

ConditionType = Union[SymbolicExpression, bool, Predicate, TruthValueOperator]
"""
The possible types for conditions.
"""


# %% Query Construction


def entity(selected_variable: T) -> Entity[T]:
    """
    Create an entity descriptor for a selected variable.

    :param selected_variable: The variable to select in the result.
    :return: Entity descriptor.
    """
    return Entity(_selected_variables_=(selected_variable,))


def set_of(*selected_variables: Union[Selectable[T], Any]) -> SetOf:
    """
    Create a set descriptor for the selected variables.

    :param selected_variables: The variables to select in the result set.
    :return: Set descriptor.
    """
    return SetOf(_selected_variables_=selected_variables)


# %% Match


def match(
        type_: Optional[Union[Type[T], Selectable[T]]] = None,
) -> Union[Type[T], CanBehaveLikeAVariable[T], Match[T]]:
    """
    Create a symbolic variable matching the type and the provided keyword arguments. This is used for easy variable
     definitions when there are structural constraints.

    :param type_: The type of the variable (i.e., The class you want to instantiate).
    :return: The Match instance.
    """
    return Match(factory=type_)


def match_variable(
        type_: Union[Type[T], Selectable[T]], domain: DomainType
) -> Union[T, Entity[T], MatchVariable[T]]:
    """
    Same as :py:func:`krrood.entity_query_language.match.match` but with a domain to use for the variable created
     by the match.

    :param type_: The type of the variable (i.e., The class you want to instantiate).
    :param domain: The domain used for the variable created by the match.
    :return: The Match instance.
    """
    return MatchVariable(factory=type_, domain=domain)


def underspecified(
        expression: Union[Type[T], Callable[..., T]], target_type: Type[T] | None = None
) -> Union[Type[T], Match[T]]:
    """
    Declare an *underspecified* (partially specified) object that a generative backend must complete.

    Unlike :py:func:`match`, which searches an existing domain for objects satisfying the constraints,
    ``underspecified`` hands the constraints to a *generative* backend (e.g.
    :py:class:`~krrood.entity_query_language.backends.ProbabilisticBackend` or
    :py:class:`~krrood.entity_query_language.backends.EntityQueryLanguageBackend`) that *creates* new
    instances. The returned :py:class:`~krrood.entity_query_language.query.match.Match` object can be
    called with keyword arguments to constrain individual fields and chained with ``.where(...)`` to
    add post-generation filter conditions.

    :param expression: The class (or callable factory) whose instances should be generated.
    :param target_type: Explicit return type when it cannot be inferred from ``expression`` alone
        (e.g. when passing a plain callable instead of a class).
    :return: A :py:class:`~krrood.entity_query_language.query.match.Match` instance typed as ``T``.

    **Kwargs vs. where – two very different roles**

    The keyword arguments supplied to the *factory call* and the conditions added via ``.where()``
    look similar on the surface but have fundamentally different semantics inside a probabilistic
    backend:

    * **kwargs (factory arguments)** – these are *structural constraints* on the object being
      built.  They become part of the :py:class:`~krrood.entity_query_language.query.match.Match`
      tree and are processed by
      :py:class:`~krrood.parametrization.parameterizer.UnderspecifiedParameters` *before* sampling:

      - ``Ellipsis (...)`` marks the field as *free* – the backend samples a value from the
        marginal distribution for that dimension.
      - A **concrete value** (literal) becomes a *conditioning assignment*: the probabilistic
        model is conditioned on that field being exactly that value before sampling.  This is
        the Bayesian ``P(rest | field = value)`` operation.
      - A :py:func:`variable` with an explicit domain becomes a *truncation assignment*: the
        model's distribution for that dimension is restricted to the given set of values.

    * ```.where(condition)``` – these are *post-hoc filters* applied **after** the model has
      been conditioned by the kwargs.  They are translated into a random event and used to
      *truncate* the already-conditioned distribution before sampling.  Geometrically, a
      ``.where`` clause carves out a region of the sample space; structurally, it expresses
      relationships *between* dimensions (e.g. one field greater than another) that cannot be
      expressed as a single-field conditioning assignment in the factory call.

    In short: **use kwargs to pin or restrict individual fields; use** ``.where`` **to express
    cross-field constraints or inequality ranges**.

    **Examples**

    *Basic – all fields free (Ellipsis):*

    .. code-block:: python

        from krrood.entity_query_language.factories import underspecified
        from krrood.entity_query_language.backends import ProbabilisticBackend

        query = underspecified(KRROODPosition)(x=..., y=..., z=...)
        backend = ProbabilisticBackend(number_of_samples=10)
        positions = list(backend.evaluate(query))
        # Returns 10 KRROODPosition instances sampled from the prior.

    *Conditioning on a literal value (kwargs):*

    .. code-block:: python

        # Fix x=0.5; the backend conditions the joint model on x==0.5 and samples
        # y and z from the resulting conditional distribution P(y, z | x=0.5).
        query = underspecified(KRROODPosition)(x=0.5, y=..., z=...)
        backend = ProbabilisticBackend(number_of_samples=10)
        positions = list(backend.evaluate(query))
        assert all(p.x == 0.5 for p in positions)

    *Restricting a field to a discrete set (kwargs with variable):*

    .. code-block:: python

        from krrood.entity_query_language.factories import variable

        # z is restricted to {1, 2, 3}; internally this becomes a truncation event
        # that cuts the z-marginal to those three values before sampling.
        query = underspecified(KRROODPosition)(x=..., y=..., z=variable(int, domain=[1, 2, 3]))
        backend = ProbabilisticBackend(number_of_samples=10)
        positions = list(backend.evaluate(query))
        assert all(p.z in (1, 2, 3) for p in positions)

    *Cross-field filter via .where:*

    .. code-block:: python

        # None of the individual fields can express "x > 0.5" as a single-field
        # conditioning.  .where translates the inequality into a truncation event
        # applied to the already-conditioned model.
        query = underspecified(KRROODPose)(
            position=underspecified(KRROODPosition)(x=..., y=..., z=...),
            orientation=KRROODOrientation(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        query.resolve()
        query.where(query.variable.position.x > 0.5)

        backend = ProbabilisticBackend(number_of_samples=10)
        poses = list(backend.evaluate(query))
        assert all(p.position.x > 0.5 for p in poses)
        # Note: orientation is conditioned to (0,0,0,1) via kwargs, not .where.

    *Cross-field inequality using .where (relationship between two fields):*

    .. code-block:: python

        from krrood.entity_query_language.factories import variable_from

        query = underspecified(Atom)(
            element=...,
            type=variable_from([0, 1, 2]),
            charge=variable_from([0.0, 1.0, 2.0]),
            timestamp=datetime.datetime.now(),
        )
        query.resolve()
        query.where(query.variable.type > query.variable.charge)
        backend = EntityQueryLanguageBackend()
        results = list(backend.evaluate(query))
        # Only Atom instances where type > charge are returned.

    *Nested underspecified objects:*

    .. code-block:: python

        query = underspecified(NestedAction)(
            obj=variable(Apple, domain=[apple]),
            pose=underspecified(KRROODPose)(
                position=underspecified(KRROODPosition)(x=0.02, y=..., z=...),
                orientation=underspecified(KRROODOrientation)(x=..., y=..., z=..., w=variable(float, domain=[0.0, 1.0])),
            ),
        )
        # x=0.02 becomes a conditioning assignment (Bayesian evidence).
        # w is truncated to {0.0, 1.0}.
        # y, z, orientation.x/y/z are sampled freely from their marginals.

    .. seealso::

        :py:func:`match` – retrieves existing domain objects instead of generating new ones.

        :py:class:`~krrood.parametrization.parameterizer.UnderspecifiedParameters` – extracts
        variables and conditioning/truncation assignments from a Match tree.

        :py:class:`~krrood.entity_query_language.backends.ProbabilisticBackend` – the generative
        backend that consumes underspecified queries.
    """
    if target_type is not None:
        return Match(factory=expression, type_=target_type)
    return Match(factory=expression)


# %% Variable Declaration


def variable(
        type_: Type[T],
        domain: Optional[DomainType],
) -> Union[T, Selectable[T], Variable[T]]:
    """
    Declare a symbolic variable that can be used inside queries.

    Filters the domain to elements that are instances of T.

    .. warning::

        If no domain is provided, and the type_ is a Symbol type, then the domain will be inferred from the SymbolGraph,
         which may contain unnecessarily many elements.

    :param type_: The type of variable.
    :param domain: Iterable of potential values for the variable or None.
     If None, the domain will be inferred from the SymbolGraph for Symbol types, else should not be evaluated by EQL
      but by another evaluator (e.g., EQL To SQL converter in Ormatic).
    :return: A Variable that can be queried for.
    """
    # Determine the domain source
    if is_iterable(domain):
        domain = filter(lambda x: isinstance(x, type_), domain)
    elif domain is None and issubclass(type_, Symbol):
        domain = SymbolGraph().get_instances_of_type(type_)
    else:
        domain = domain

    result = Variable(
        _type_=type_,
        _domain_=domain,
    )

    return result


def deduced_variable(
        type_: Optional[Type[T]] = None,
) -> Union[Type[T], ExternallySetVariable[T]]:
    """
    Create a variable that is inferred through deductive reasoning.

    :param type_: The type of the variable values.
    :return: An instance of `ExternallySetVariable[T]`.
    """
    return ExternallySetVariable(_type_=type_, _domain_source_=DomainSource.DEDUCTION)


def variable_from(domain: Union[Iterable[T], Selectable[T]]) -> Union[T, Selectable[T]]:
    """
    Create a variable from a given domain.

    :param domain: An iterable or selectable expression to use as the variable's domain.
    :return: A variable that can be queried for.
    """
    return Variable(_domain_=domain)


# %% Operators on Variables


def concatenation(
        *variables: Union[Iterable[T], Selectable[T]],
) -> Union[T, Selectable[T]]:
    """
    Concatenation of two or more variables.
    """
    return Concatenation(_operation_children_=variables)


def contains(
        container: Union[Iterable, CanBehaveLikeAVariable[T]], item: Any
) -> Comparator:
    """
    Check whether a container contains an item.

    :param container: The container expression.
    :param item: The item to look for.
    :return: A comparator expression equivalent to ``item in container``.
    :rtype: SymbolicExpression
    """
    return in_(item, container)


def in_(item: Any, container: Union[Iterable, CanBehaveLikeAVariable[T]]):
    """
    Build a comparator for membership: ``item in container``.

    :param item: The candidate item.
    :param container: The container expression.
    :return: Comparator expression for membership.
    :rtype: Comparator
    """
    return Comparator(container, item, operator.contains)


def flat_variable(
        var: Union[CanBehaveLikeAVariable[T], Iterable[T]],
) -> Union[FlatVariable[T], T]:
    """
    Flatten a nested iterable domain into individual items while preserving the parent bindings.
    This returns a DomainMapping that, when evaluated, yields one solution per inner element
    (similar to SQL UNNEST), keeping existing variable bindings intact.
    """
    return FlatVariable(var)


# %% Logical Operators


def and_(*conditions: ConditionType) -> ConditionType:
    """
    Logical conjunction of conditions.

    :param conditions: One or more conditions to combine.
    :type conditions: SymbolicExpression | bool
    :return: An AND operator joining the conditions.
    :rtype: SymbolicExpression
    """
    return chained_logic(AND, *conditions)


def or_(*conditions: ConditionType) -> ConditionType:
    """
    Logical disjunction of conditions.

    :param conditions: One or more conditions to combine.
    :type conditions: SymbolicExpression | bool
    :return: An OR operator joining the conditions.
    :rtype: SymbolicExpression
    """
    return chained_logic(OR, *conditions)


def not_(operand: ConditionType) -> SymbolicExpression:
    """
    A symbolic NOT operation that can be used to negate symbolic expressions.
    """
    if not isinstance(operand, SymbolicExpression):
        operand = Literal(_value_=operand)
    return operand._invert_()


def for_all(
        universal_variable: Union[CanBehaveLikeAVariable[T], T],
        condition: ConditionType,
) -> ForAll:
    """
    A universal on variable that finds all sets of variable bindings (values) that satisfy the condition for **every**
     value of the universal_variable.

    :param universal_variable: The universal on variable that the condition must satisfy for all its values.
    :param condition: A SymbolicExpression or bool representing a condition that must be satisfied.
    :return: A SymbolicExpression that can be evaluated producing every set that satisfies the condition.
    """
    return ForAll(universal_variable, condition)


def exists(
        universal_variable: Union[CanBehaveLikeAVariable[T], T],
        condition: ConditionType,
) -> Exists:
    """
    A universal on variable that finds all sets of variable bindings (values) that satisfy the condition for **any**
     value of the universal_variable.

    :param universal_variable: The universal on variable that the condition must satisfy for any of its values.
    :param condition: A SymbolicExpression or bool representing a condition that must be satisfied.
    :return: A SymbolicExpression that can be evaluated producing every set that satisfies the condition.
    """
    return Exists(universal_variable, condition)


# %% Result Quantifiers


def an(
        entity_: Union[T, Query],
        quantification: Optional[ResultQuantificationConstraint] = None,
) -> Union[T, Query]:
    """
    Select all values satisfying the given entity description.

    :param entity_: An entity or a set expression to quantify over.
    :param quantification: Optional quantification constraint.
    :return: The entity with the applied quantifier.
    """
    return entity_._quantify_(An, quantification_constraint=quantification)


a = an
"""
This is an alias to accommodate for words not starting with vowels.
"""


def the(
        entity_: Union[T, Query],
) -> Union[T, Query]:
    """
    Select the unique value satisfying the given entity description.

    :param entity_: An entity or a set expression to quantify over.
    :return: The entity with the applied quantifier.
    """
    return entity_._quantify_(The)


# %% Rules


def add(variable: Any, value: Any) -> None:
    """
    Add a value to a variable.

    :param variable: The variable to which the value will be added.
    :param value: The value to be added to the variable.
    :return: None
    """
    Add(variable, value)


def inference(
        type_: Type[T],
) -> Union[Callable[[], Union[T, InstantiatedVariable[T]]]]:
    """
    This returns a factory function that creates a new variable of the given type and takes keyword arguments for the
    type constructor.

    :param type_: The type of the variable (i.e., The class you want to instantiate).
    :return: The factory function for creating a new variable.
    """
    return lambda **kwargs: InstantiatedVariable(
        _type_=type_,
        _kwargs_=kwargs,
    )


def refinement(*conditions: ConditionType) -> SymbolicExpression:
    """
    Add a refinement branch (ExceptIf node with its right the new conditions and its left the base/parent rule/query)
     to the current condition tree.

    Each provided condition is chained with AND, and the resulting branch is
    connected via ExceptIf to the current node, representing a refinement/specialization path.

    :param conditions: The refinement conditions. They are chained with AND.
    :returns: The newly created branch node for further chaining.
    """
    return Refinement.create_and_update_rule_tree(*conditions)


def alternative(*conditions: ConditionType) -> SymbolicExpression:
    """
    Add an alternative branch (logical ElseIf) to the current condition tree.

    Each provided condition is chained with AND, and the resulting branch is
    connected via ElseIf to the current node, representing an alternative path.

    :param conditions: Conditions to chain with AND and attach as an alternative.
    :returns: The newly created branch node for further chaining.
    """
    return Alternative.create_and_update_rule_tree(*conditions)


def next_rule(*conditions: ConditionType) -> SymbolicExpression:
    """
    Add a consequent rule that gets always executed after the current rule.

    Each provided condition is chained with AND, and the resulting branch is
    connected via Next to the current node, representing the next path.

    :param conditions: Conditions to chain with AND and attach as an alternative.
    :returns: The newly created branch node for further chaining.
    """
    return Next.create_and_update_rule_tree(*conditions)


# %% Conditionals

def case_when(
        condition: SymbolicExpression,
        then_value: SymbolicExpression,
        else_value: Optional[SymbolicExpression] = None,
) -> CaseWhen:
    """
    Create a CASE WHEN ... THEN ... ELSE ... END expression.

    .. code-block:: python

        action = variable(MoveAction, domain=[])
        query = an(set_of(
            min(case_when(action.polymorphic_type == 'PickUpActionDAO', action.database_id))
        ))

    :param condition: The condition to evaluate
    :param then_value: The value if condition is true
    :param else_value: The value if condition is false
    :return: A CaseWhen expression
    """
    return CaseWhen(condition, then_value, else_value)


# %% Aggregators


def max(
        variable: Selectable[T],
        key: Optional[Callable] = None,
        default: Optional[T] = None,
        distinct: bool = False,
) -> Union[T, Max[T]]:
    """
    Maps the variable values to their maximum value.

    :param variable: The variable for which the maximum value is to be found.
    :param key: A function that extracts a comparison key from each variable value.
    :param default: The value returned when the iterable is empty.
    :param distinct: Whether to only consider distinct values.
    :return: A Max object that can be evaluated to find the maximum value.
    """
    return Max(
        variable, _key_function_=key, _default_value_=default, _distinct_=distinct
    )


def mode(
        variable: Selectable[T],
        default: Optional[T] = None,
) -> Union[T, Mode[T]]:
    """
    Calculate and return the first mode from the variable values. The mode is the most common value in the iterable. It is found by
    counting the occurrences of each value and returning the one with the highest count. If there are multiple values
    with the same highest count, the first one encountered is returned. This is an aggregation function, thus the query
    will be fully evaluated before the result is returned.

    :param variable: The variable for which the mode value is to be found.
    :param default: The value returned when the iterable is empty.
    :return: A Max object that can be evaluated to find the mode value.
    """
    return Mode(variable, _default_value_=default)


def multimode(
        variable: Selectable[T],
        default: Optional[T] = None,
) -> Union[T, MultiMode[T]]:
    """
    Calculate and return all mode values from the variable values. Similar to :py:func:`krrood.entity_query_language.factories.mode`
    but returns all values that have the same mode value (i.e., all values that have the same highest count).

    :param variable: The variable for which the mode value is to be found.
    :param default: The value returned when the iterable is empty.
    :return: A Max object that can be evaluated to find the mode value.
    """
    return MultiMode(variable, _default_value_=default)


def min(
        variable: Selectable[T],
        key: Optional[Callable] = None,
        default: Optional[T] = None,
        distinct: bool = False,
) -> Union[T, Min[T]]:
    """
    Maps the variable values to their minimum value.

    :param variable: The variable for which the minimum value is to be found.
    :param key: A function that extracts a comparison key from each variable value.
    :param default: The value returned when the iterable is empty.
    :param distinct: Whether to only consider distinct values.
    :return: A Min object that can be evaluated to find the minimum value.
    """
    return Min(
        variable, _key_function_=key, _default_value_=default, _distinct_=distinct
    )


def sum(
        variable: Union[T, Selectable[T]],
        key: Optional[Callable] = None,
        default: Optional[T] = None,
        distinct: bool = False,
) -> Union[T, Sum]:
    """
    Computes the sum of values produced by the given variable.

    :param variable: The variable for which the sum is calculated.
    :param key: A function that extracts a comparison key from each variable value.
    :param default: The value returned when the iterable is empty.
    :param distinct: Whether to only consider distinct values.
    :return: A Sum object that can be evaluated to find the sum of values.
    """
    return Sum(
        variable, _key_function_=key, _default_value_=default, _distinct_=distinct
    )


def average(
        variable: Union[Selectable[T], Any],
        key: Optional[Callable] = None,
        default: Optional[T] = None,
        distinct: bool = False,
) -> Union[T, Average]:
    """
    Computes the sum of values produced by the given variable.

    :param variable: The variable for which the sum is calculated.
    :param key: A function that extracts a comparison key from each variable value.
    :param default: The value returned when the iterable is empty.
    :param distinct: Whether to only consider distinct values.
    :return: A Sum object that can be evaluated to find the sum of values.
    """
    return Average(
        variable, _key_function_=key, _default_value_=default, _distinct_=distinct
    )


def count(variable: Selectable[T], distinct: bool = False) -> Union[T, Count[T]]:
    """
    Count the number of values produced by the given variable.

    :param variable: The variable for which the count is calculated.
    :param distinct: Whether to only consider distinct values.
    :return: A Count object that can be evaluated to count the number of values.
    """
    return Count(variable, _distinct_=distinct)


def count_all(distinct: bool = False) -> Union[T, Count[T]]:
    """
    Count all results (by group).

    :param distinct: Whether to only consider distinct values.
    :return: A Count object that can be evaluated to count the number of values.
    """
    return CountAll(_distinct_=distinct)


def distinct(
        expression: T,
        *on: Any,
) -> T:
    """
    Indicate that the result of the expression should be distinct.
    """
    match expression:
        case Query():
            return expression.distinct(*on)
        case ResultQuantifier():
            return expression._child_.distinct(*on)
        case Selectable():
            return entity(expression).distinct(*on)
        case _:
            raise UnsupportedExpressionTypeForDistinct(type(expression))


def get_conditioned_statements(
        statement, condition: Callable[OperationResult, bool]
) -> List[SymbolicExpression]:
    """
    Iterates over all sub-statements of the statement and returns all statements that satisfy the condition.

    :param statement: The statement to iterate over.
    :param condition: The condition to evaluate each sub-statement against.
    :return: A list of sub-statements that satisfy the condition.
    """
    condition_results = []
    for node in [
        s
        for s in statement._children_
        if not isinstance(s, (Variable, inspect.Attribute))
    ]:
        node_result = node.evaluate()
        if condition(node_result):
            condition_results.append(node)
    if statement in condition_results:
        condition_results.remove(statement)

    return condition_results


def get_false_statements(statement: SymbolicExpression) -> List[SymbolicExpression]:
    """
    The false statements of all statements of this condition.

    :return: The false statements of all statements of this condition.
    """
    return get_conditioned_statements(statement, lambda x: not x == [])


def get_true_statements(statement: SymbolicExpression) -> List[SymbolicExpression]:
    """
    The true statements of all statements of this condition.

    :return: The true statements of this condition.
    """
    return get_conditioned_statements(statement, lambda x: x == [])


def evaluate_condition(condition: ConditionType) -> bool:
    """
    Evaluates the condition to True or False.

    :param condition: The condition to evaluate.
    :return: True if there is a possible solution, False otherwise.
    """
    if type(condition) is bool:
        return condition
    return any(condition.evaluate())


@symbolic_function
def node_id(node: SymbolicExpression) -> UUID:
    return node._id_


@symbolic_function
def node_descendants(node: SymbolicExpression) -> Iterable[SymbolicExpression]:
    return node._descendants_


@symbolic_function
def node_type(node: Selectable) -> Optional[Type]:
    return getattr(node, "_type_", None)


@symbolic_function
def node_children(node: CanBehaveLikeAVariable) -> Iterable[SymbolicExpression]:
    return node._children_


@symbolic_function
def attribute_owner_class(node: Attribute) -> Type:
    return node._owner_class_


@symbolic_function
def node_parents(node: SymbolicExpression) -> Iterable[SymbolicExpression]:
    return node._parents_


@symbolic_function
def issubclass_(cls: Type, cls_or_tuple: Type | Tuple[Type, ...]) -> bool:
    return issubclass(cls, cls_or_tuple)


@symbolic_function
def is_class(obj: Any) -> bool:
    return isclass(obj)


@symbolic_function
def type_(obj: Any) -> Type:
    return obj.__class__
