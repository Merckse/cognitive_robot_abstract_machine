"""
Format-specific colour and spacing markup for fragment rendering.

:class:`Formatter` subclasses determine how colours, spaces, newlines,
and hyperlinks are encoded:

* :class:`PlainFormatter` — no colour, ASCII space/newline.
* :class:`ANSIFormatter` — 24-bit ANSI escape sequences with optional OSC 8 links.
* :class:`HTMLFormatter` — ``<span style=\"color:...\">`` tags and ``<a href=\"...\">`` links.

Also defines :class:`BulletStyle` and :class:`IndentSize` enums used by
:class:`~krrood.entity_query_language.verbalization.rendering.renderer.HierarchicalRenderer`.
"""

from __future__ import annotations

import ast
import html
import inspect
import logging
import os
import textwrap
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from functools import lru_cache
from typing_extensions import ClassVar, Optional

from krrood.entity_query_language.verbalization.fragments.roles import (
    ROLE_COLORS,
    SemanticRole,
)

_TOOLTIP_ATTR = "title"


def _first_docstring_line(obj: object) -> Optional[str]:
    """Return the first non-empty line of *obj*'s docstring as plain text, or ``None``."""
    if obj is None:
        return None
    doc = inspect.getdoc(obj)
    if not doc:
        return None
    for line in doc.splitlines():
        stripped = line.strip()
        if stripped:
            return stripped
    return None


def _annotated_target_name(node: ast.AST) -> Optional[str]:
    """Return the target name if *node* is an annotated assignment with a simple name target."""
    if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
        return node.target.id
    return None


def _string_expr_first_line(node: ast.AST) -> Optional[str]:
    """Return the first stripped line if *node* is a bare string expression (PEP 257 attribute doc)."""
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
def _attribute_docstrings(cls: type) -> dict[str, str]:
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
        class_def = ast.parse(source).body[0]
    except (SyntaxError, IndexError):
        return {}
    body = getattr(class_def, "body", [])
    docs: dict[str, str] = {}
    for current, following in zip(body, body[1:]):
        name = _annotated_target_name(current)
        if name is not None:
            line = _string_expr_first_line(following)
            if line is not None:
                docs[name] = line
    return docs


def _docstring_for_source_ref(source_ref: object) -> Optional[str]:
    """Return the first docstring line for the class or field a :class:`SourceRef` points at.

    For class-level refs (``source_ref.attribute is None``), delegates to
    :func:`_first_docstring_line` on the class.  For attribute refs, walks the MRO
    looking for a PEP 257 attribute docstring extracted via :func:`_attribute_docstrings`.
    """
    if source_ref.attribute is None:
        return _first_docstring_line(source_ref.owner_type)
    for klass in source_ref.owner_type.__mro__:
        line = _attribute_docstrings(klass).get(source_ref.attribute)
        if line is not None:
            return line
    return None


_log = logging.getLogger(__name__)


class BulletStyle(Enum):
    """
    Bullet character used by
    :class:`~krrood.entity_query_language.verbalization.rendering.renderer.HierarchicalRenderer`
    for list items.

    :cvar DASH: ``"-"``
    :cvar DOT: ``"•"``
    :cvar ASTERISK: ``"*"``
    """

    DASH = "-"
    DOT = "•"
    ASTERISK = "*"


class IndentSize(Enum):
    """
    Indentation string used by
    :class:`~krrood.entity_query_language.verbalization.rendering.renderer.HierarchicalRenderer`
    per nesting level.

    :cvar TWO_SPACES: Two-space indent (default).
    :cvar FOUR_SPACES: Four-space indent.
    :cvar TAB: Hard tab character.
    """

    TWO_SPACES = "  "
    FOUR_SPACES = "    "
    TAB = "\t"


def _detect_osc8_support() -> bool:
    """Return ``True`` when the current terminal is known to support OSC 8 hyperlinks."""
    if os.environ.get("VTE_VERSION"):  # GNOME Terminal, Tilix, …
        return True
    term_prog = os.environ.get("TERM_PROGRAM", "")
    if term_prog in {"vscode", "WezTerm", "iTerm.app"}:
        return True
    if os.environ.get("TERM") == "xterm-kitty":
        return True
    return False


@dataclass
class Formatter(ABC):
    """
    Single source of truth for all format-specific characters and colour markup.

    Concrete subclasses determine how colours, spaces, newlines, and hyperlinks
    are encoded in the output string.

    Subclasses: :class:`PlainFormatter`, :class:`ANSIFormatter`, :class:`HTMLFormatter`.
    """

    @abstractmethod
    def colorize(self, text: str, role: SemanticRole) -> str:
        """
        Wrap *text* in format-specific colour markup for *role*.

        :param text: Plain display text to colourize.
        :type text: str
        :param role: Semantic role determining the colour.
        :type role: ~krrood.entity_query_language.verbalization.fragments.roles.SemanticRole
        :return: Coloured string (or *text* unchanged when no colour is defined for *role*).
        :rtype: str
        """
        ...

    @property
    @abstractmethod
    def space(self) -> str:
        """
        Inline word separator character(s) (e.g. ``" "`` or ``"&nbsp;"``).

        :rtype: str
        """
        ...

    @property
    @abstractmethod
    def newline(self) -> str:
        """
        Line break character(s) (e.g. ``"\\n"`` or ``"<br>"``).

        :rtype: str
        """
        ...

    def wrap_link(self, text: str, url: str, tooltip: Optional[str] = None) -> str:
        """
        Wrap already-rendered *text* with a hyperlink to *url*.

        The base implementation is a no-op (hyperlinks not supported for this format).
        Subclasses override when the output format supports clickable links.

        :param text: Already-colourized display text.
        :type text: str
        :param url: Destination URL.
        :type url: str
        :param tooltip: Optional single-line docstring summary shown on hover (HTML-escaped).
        :type tooltip: str or None
        :return: *text* unchanged (base); linked string (subclasses).
        :rtype: str
        """
        return text


@dataclass
class PlainFormatter(Formatter):
    """
    No colour markup; standard ASCII space (``" "``) and newline (``"\\n"``).

    The default formatter used by
    :class:`~krrood.entity_query_language.verbalization.rendering.renderer.ParagraphRenderer`
    and :meth:`~krrood.entity_query_language.verbalization.pipeline.VerbalizationPipeline.plain`.
    """

    def colorize(self, text: str, role: SemanticRole) -> str:
        """Return *text* unchanged (no colour markup in plain mode)."""
        return text

    @property
    def space(self) -> str:
        return " "

    @property
    def newline(self) -> str:
        return "\n"


@dataclass
class ANSIFormatter(Formatter):
    """
    True-color ANSI escape sequences (24-bit, ``\\033[38;2;R;G;Bm``).

    Compatible with VS Code terminal, GNOME Terminal, iTerm2, Windows Terminal,
    and any terminal supporting the ISO-8613-3 direct-color extension.

    OSC 8 hyperlinks are enabled automatically when the terminal is detected as
    capable (``VTE_VERSION``, ``TERM_PROGRAM`` in ``{vscode, WezTerm, iTerm.app}``,
    or ``TERM=xterm-kitty``).  On unsupported terminals :meth:`wrap_link` falls
    back to returning plain coloured text with no link markup.
    """

    _RESET: ClassVar[str] = "\033[0m"
    _NAMED: ClassVar[dict[str, tuple[int, int, int]]] = {
        "cornflowerblue": (100, 149, 237),
    }

    _hyperlinks_enabled: bool = field(default_factory=_detect_osc8_support, init=False)

    def colorize(self, text: str, role: SemanticRole) -> str:
        color = ROLE_COLORS.get(role)
        if color is None:
            return text
        r, g, b = self._hex_to_rgb(color)
        return f"\033[38;2;{r};{g};{b}m{text}{self._RESET}"

    def wrap_link(self, text: str, url: str, tooltip: Optional[str] = None) -> str:
        if not self._hyperlinks_enabled:
            return text
        # OSC 8 format: ESC ] 8 ; ; URL ST  text  ESC ] 8 ; ; ST
        return f"\033]8;;{url}\033\\{text}\033]8;;\033\\"

    @property
    def space(self) -> str:
        return " "

    @property
    def newline(self) -> str:
        return "\n"

    def _hex_to_rgb(self, color: str) -> tuple[int, int, int]:
        """Convert a hex colour string (e.g. ``\"#ff7f0e\"``) or named colour to an ``(R, G, B)`` tuple."""
        if color.startswith("#"):
            h = color.lstrip("#")
            return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
        return self._NAMED.get(color.lower(), (255, 255, 255))


@dataclass
class HTMLFormatter(Formatter):
    """
    HTML output with ``<span style="color: …">`` colour tags, ``&nbsp;`` spaces,
    and ``<br>`` newlines.

    Suitable for Jupyter notebooks, GitLab Markdown, and any renderer that
    passes through inline HTML.  Hyperlinks use standard ``<a href="…">`` anchors.

    Used by :meth:`~krrood.entity_query_language.verbalization.pipeline.VerbalizationPipeline.html`.
    """

    def colorize(self, text: str, role: SemanticRole) -> str:
        color = ROLE_COLORS.get(role)
        if color is None:
            return text
        return f'<span style="color:{color}">{text}</span>'

    def wrap_link(self, text: str, url: str, tooltip: Optional[str] = None) -> str:
        tooltip_attr = (
            f' {_TOOLTIP_ATTR}="{html.escape(tooltip, quote=True)}"' if tooltip else ""
        )
        return (
            f'<a target="_blank" rel="noopener" href="{url}"{tooltip_attr}>{text}</a>'
        )

    @property
    def space(self) -> str:
        return "&nbsp;"

    @property
    def newline(self) -> str:
        return "<br>"
