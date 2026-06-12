"""
Source documentation lookup — extracts the first docstring line for the class or
attribute a :class:`~krrood.entity_query_language.verbalization.fragments.source_ref.SourceRef`
points at.

Used by the renderers to attach a one-line tooltip to source hyperlinks.  Attribute
documentation follows the project convention of a bare string expression immediately
below the field definition (a PEP 257 attribute docstring), which is extracted by
parsing the class source with :mod:`ast`.
"""

from __future__ import annotations

import ast
import inspect
import textwrap
from functools import lru_cache

from typing_extensions import Dict, Optional

from krrood.entity_query_language.verbalization.fragments.source_ref import SourceRef


def first_docstring_line(documented_object: object) -> Optional[str]:
    """:return: The first non-empty line of *documented_object*'s docstring, or ``None``."""
    if documented_object is None:
        return None
    docstring = inspect.getdoc(documented_object)
    if not docstring:
        return None
    for line in docstring.splitlines():
        stripped = line.strip()
        if stripped:
            return stripped
    return None


def _annotated_target_name(node: ast.AST) -> Optional[str]:
    """:return: The target name when *node* is an annotated assignment with a simple name target."""
    if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
        return node.target.id
    return None


def _string_expression_first_line(node: ast.AST) -> Optional[str]:
    """:return: The first stripped line when *node* is a bare string expression (a PEP 257
    attribute docstring), else ``None``."""
    if (
        isinstance(node, ast.Expr)
        and isinstance(node.value, ast.Constant)
        and isinstance(node.value.value, str)
    ):
        for line in node.value.value.splitlines():
            stripped = line.strip()
            if stripped:
                return stripped
    return None


@lru_cache(maxsize=None)
def _attribute_docstrings(cls: type) -> Dict[str, str]:
    """Map field name to its first PEP 257 attribute docstring line for *cls*'s own body.

    Parses the class source with :mod:`ast` and detects the pattern where an
    annotated assignment is immediately followed by a bare string expression.
    Returns an empty mapping when source is unavailable (e.g. C-extension classes).
    Cached per class since AST parsing is comparatively costly.
    """
    try:
        source = textwrap.dedent(inspect.getsource(cls))
    except (OSError, TypeError):
        return {}
    try:
        class_definition = ast.parse(source).body[0]
    except (SyntaxError, IndexError):
        return {}
    body = getattr(class_definition, "body", [])
    docstrings: Dict[str, str] = {}
    for current, following in zip(body, body[1:]):
        name = _annotated_target_name(current)
        if name is not None:
            line = _string_expression_first_line(following)
            if line is not None:
                docstrings[name] = line
    return docstrings


def docstring_for_source_ref(source_ref: SourceRef) -> Optional[str]:
    """The first docstring line for the class or field a :class:`SourceRef` points at.

    For class-level references (``source_ref.attribute is None``), delegates to
    :func:`first_docstring_line` on the class.  For attribute references, walks the MRO
    looking for a PEP 257 attribute docstring extracted via :func:`_attribute_docstrings`.

    :param source_ref: The source reference to document.
    :return: The first docstring line, or ``None`` when no documentation is found.
    """
    if source_ref.attribute is None:
        return first_docstring_line(source_ref.owner_type)
    for klass in source_ref.owner_type.__mro__:
        line = _attribute_docstrings(klass).get(source_ref.attribute)
        if line is not None:
            return line
    return None
