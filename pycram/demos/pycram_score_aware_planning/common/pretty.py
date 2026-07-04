"""Shared terminal-pretty helpers: ANSI palette, boxed tables and inline bars.

Used by both the evaluation summary (CompositeEvaluator) and the structurizer
summary so the two share a single visual theme. Pure stdlib, no project imports,
so it is safe to import from anywhere.
"""

# Flip to False to strip all ANSI colour (e.g. when piping output to a file).
COLOR_ENABLED: bool = True

_C = {
    "reset": "\033[0m", "bold": "\033[1m", "dim": "\033[2m",
    "green": "\033[92m", "red": "\033[91m", "yellow": "\033[93m",
    "cyan": "\033[96m", "grey": "\033[90m", "blue": "\033[94m", "magenta": "\033[95m",
}


def paint(text: str, *keys: str) -> str:
    """Wrap text in the given ANSI colour keys (no-op if colour is disabled)."""
    keys = tuple(k for k in keys if k)
    if not COLOR_ENABLED or not keys:
        return text
    return "".join(_C[k] for k in keys) + text + _C["reset"]


class Cell:
    """A table cell whose colours are applied *after* width padding, so ANSI
    escape codes never break column alignment."""
    __slots__ = ("text", "colors")

    def __init__(self, text: str, *colors: str):
        self.text = str(text)
        self.colors = tuple(c for c in colors if c)


def value_colors(v: float) -> tuple:
    """Green for positive, red for negative, nothing for zero."""
    return ("green",) if v > 0 else (("red",) if v < 0 else ())


def bar_cell(value: float, peak: float, width: int = 12) -> Cell:
    """A fixed-width bar (filled blocks + trailing spaces) coloured by magnitude."""
    peak = peak or 1.0
    n = 0 if value <= 0 else max(0, min(width, round(value / peak * width)))
    text = "█" * n + " " * (width - n)
    if n == 0:
        return Cell(text)
    hue = "green" if value >= 0.66 * peak else ("cyan" if value >= 0.33 * peak else "yellow")
    return Cell(text, hue)


def _pad(text: str, w: int, align: str) -> str:
    return {">": text.rjust, "<": text.ljust, "^": text.center}[align](w)


def render_table(title, headers, rows, widths, aligns, subtitle=None, footer=None):
    """Render a rounded box table and return it as a list of lines.

    :param rows:   list of rows; each cell is either a plain str or a Cell.
    :param footer: optional summary row, drawn below a separator.
    """
    inner = sum(w + 2 for w in widths) + (len(widths) - 1)

    def rule(left, sep, right):
        return left + sep.join("─" * (w + 2) for w in widths) + right

    def render_row(cells):
        out = []
        for c, w, a in zip(cells, widths, aligns):
            if isinstance(c, Cell):
                out.append(paint(_pad(c.text, w, a), *c.colors))
            else:
                out.append(_pad(str(c), w, a))
        return "│ " + " │ ".join(out) + " │"

    lines = ["╭" + "─" * inner + "╮", "│" + paint(title.center(inner), "bold") + "│"]
    if subtitle:
        lines.append("│" + paint(subtitle.center(inner), "dim") + "│")
    lines += [rule("├", "┬", "┤"),
              render_row([Cell(h, "bold") for h in headers]),
              rule("├", "┼", "┤")]
    lines += [render_row(r) for r in rows]
    if footer is not None:
        lines.append(rule("├", "┼", "┤"))
        lines.append(render_row(footer))
    lines.append(rule("╰", "┴", "╯"))
    return lines
