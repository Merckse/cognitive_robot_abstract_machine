from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, TYPE_CHECKING, List, Tuple

from giskardpy.motion_statechart.graph_node import Goal, MotionStatechartNode

import numpy as np

from giskardpy.middleware import get_middleware
from giskardpy.motion_statechart.plotters.styles import (
    LiftCycleStateToColor,
    ObservationStateToColor,
)
from giskardpy.utils.utils import create_path

if TYPE_CHECKING:  # avoid circular import at runtime
    from giskardpy.motion_statechart.motion_statechart import MotionStatechart


@dataclass
class _PlotContext:
    """
    Container for all plot inputs and evolving state.

    This groups immutable inputs (nodes, depths, label names) with the
    small amount of mutable, iteration-time state (current states and
    open segment starts) needed while rendering the chart.
    """

    nodes: List[MotionStatechartNode]
    depths: Dict[MotionStatechartNode, int]
    y_index: Dict[MotionStatechartNode, int]
    names: List[str]
    history: List
    start_cycle: int
    last_cycle: int
    current_life: List[float]
    current_obs: List[float]
    segment_start: List[int]


@dataclass
class HistoryGanttChartPlotter:
    """
    Plot a hierarchy-aware Gantt chart of node states.

    Shows parent-child relationships of Goals by ordering rows in
    preorder and by prefixing labels with tree glyphs (├─, └─, │).
    Optional background bands and goal outlines emphasize grouping.
    """

    motion_statechart: MotionStatechart
    indent_labels: bool = True
    show_hierarchy_bands: bool = True
    outline_goals: bool = True

    def plot_gantt_chart(self, file_name: str) -> None:
        """
        Render the Gantt chart and save it.

        The chart shows life cycle (top half) and observation state (bottom half)
        per node over control cycles and emphasizes hierarchical Goals.
        """
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        import matplotlib.colors as mcolors

        nodes = self.motion_statechart.nodes
        if len(nodes) == 0:
            get_middleware().logwarn(
                "Gantt chart skipped: no nodes in motion statechart."
            )
            return

        history = self.motion_statechart.history.history
        if len(history) == 0:
            get_middleware().logwarn("Gantt chart skipped: empty StateHistory.")
            return

        ordered = self._iter_hierarchy()
        depths, names, y_index = self._build_labels_and_indices(ordered)

        last_cycle = max(item.control_cycle for item in history)
        num_bars = len(names)
        figure_width, figure_height = self._compute_figure_size(num_bars, last_cycle)

        plt.figure(figsize=(figure_width, figure_height))
        ax = plt.gca()
        plt.grid(True, axis="x", zorder=-1)

        ctx = self._init_plot_context(
            ordered=ordered,
            depths=depths,
            names=names,
            y_index=y_index,
            history=history,
            last_cycle=last_cycle,
        )

        if self.show_hierarchy_bands:
            self._draw_hierarchy_bands(ax=ax, ctx=ctx, mcolors=mcolors)

        self._iterate_history_and_draw(plt=plt, ctx=ctx)

        if self.outline_goals:
            self._outline_goal_rows(ax=ax, ctx=ctx, Rectangle=Rectangle)

        self._format_axes(plt=plt, ctx=ctx)
        self._save_figure(plt=plt, file_name=file_name)

    # -------------------- Helpers: data preparation --------------------
    def _iter_hierarchy(self) -> List[Tuple[MotionStatechartNode, int]]:
        """
        Traverse nodes in preorder, yielding each node with its depth.
        """
        def walk(n: MotionStatechartNode, d: int):
            yield n, d
            if isinstance(n, Goal):
                for c in n.nodes:
                    yield from walk(c, d + 1)

        ordered_: List[Tuple[MotionStatechartNode, int]] = []
        for root in self.motion_statechart.top_level_nodes:
            ordered_.extend(list(walk(root, 0)))
        seen = {n for n, _ in ordered_}
        for n in self.motion_statechart.nodes:
            if n not in seen:
                ordered_.append((n, 0))
        return ordered_

    def _build_labels_and_indices(
        self, ordered: List[Tuple[MotionStatechartNode, int]]
    ) -> tuple[
        Dict[MotionStatechartNode, int], List[str], Dict[MotionStatechartNode, int]
    ]:
        """
        Build label strings and y-index mapping.

        Labels reflect hierarchy using box-drawing glyphs instead of spaces.
        """
        depths: Dict[MotionStatechartNode, int] = {n: d for (n, d) in ordered}

        # Build parent->children mapping in traversal order (roots have parent None)
        parent_children: Dict[MotionStatechartNode | None, List[MotionStatechartNode]] = {}
        # Roots: top level nodes in traversal order
        roots: List[MotionStatechartNode] = []
        seen_roots: set[MotionStatechartNode] = set()
        for n, d in ordered:
            if d == 0 and n.parent_node is None and n not in seen_roots:
                roots.append(n)
                seen_roots.add(n)
        parent_children[None] = roots
        for n, _ in ordered:
            if isinstance(n, Goal):
                parent_children[n] = list(n.nodes)

        def is_last_sibling(node: MotionStatechartNode) -> bool:
            parent = node.parent_node
            siblings = parent_children.get(parent, [])
            return siblings and siblings[-1] is node

        def ancestor_chain(node: MotionStatechartNode) -> List[MotionStatechartNode]:
            chain: List[MotionStatechartNode] = []
            cur = node.parent_node
            while cur is not None:
                chain.append(cur)
                cur = cur.parent_node
            return list(reversed(chain))

        def tree_prefix(node: MotionStatechartNode) -> str:
            if not self.indent_labels or depths[node] == 0:
                return ""
            parts: List[str] = []
            ancestors = ancestor_chain(node)
            # For all ancestors except the direct parent, draw continuation if that ancestor is not last
            for anc in ancestors[:-1]:
                parts.append("│  " if not is_last_sibling(anc) else "   ")
            # For the direct parent edge, decide branch glyph
            parts.append("└─ " if is_last_sibling(node) else "├─ ")
            return "".join(parts)

        def label_for(node: MotionStatechartNode) -> str:
            base = node.name[:50]
            return f"{tree_prefix(node)}{base}" if self.indent_labels else base

        nodes = [n for n, _ in ordered]
        names = [label_for(n) for n in nodes]
        y_index: Dict[MotionStatechartNode, int] = {n: i for i, n in enumerate(nodes)}
        return depths, names, y_index

    def _compute_figure_size(
        self, num_bars: int, last_cycle: int
    ) -> tuple[float, float]:
        figure_height = 0.7 + num_bars * 0.25
        figure_width = max(4.0, 0.5 * float(last_cycle + 1))
        return figure_width, figure_height

    def _init_plot_context(
        self,
        ordered: List[Tuple[MotionStatechartNode, int]],
        depths: Dict[MotionStatechartNode, int],
        names: List[str],
        y_index: Dict[MotionStatechartNode, int],
        history: List,
        last_cycle: int,
    ) -> _PlotContext:
        nodes = [n for n, _ in ordered]
        start_cycle = history[0].control_cycle
        current_life = [history[0].life_cycle_state[n] for n in nodes]
        current_obs = [history[0].observation_state[n] for n in nodes]
        segment_start = [start_cycle for _ in nodes]
        return _PlotContext(
            nodes=nodes,
            depths=depths,
            y_index=y_index,
            names=names,
            history=history,
            start_cycle=start_cycle,
            last_cycle=last_cycle,
            current_life=current_life,
            current_obs=current_obs,
            segment_start=segment_start,
        )

    def _draw_hierarchy_bands(self, ax, ctx: _PlotContext, mcolors) -> None:
        def goal_band_color(depth: int):
            palette = ["#f5f5f5", "#eef6ff", "#f7fff0"]
            return mcolors.to_rgba(palette[depth % len(palette)], alpha=0.35)

        for n in ctx.nodes:
            if isinstance(n, Goal) and len(n.nodes) > 0:
                stack = [n]
                rows = []
                while stack:
                    cur = stack.pop()
                    rows.append(ctx.y_index[cur])
                    if isinstance(cur, Goal):
                        stack.extend(cur.nodes)
                y_min, y_max = min(rows), max(rows)
                ax.axhspan(
                    y_min - 0.5,
                    y_max + 0.5,
                    color=goal_band_color(ctx.depths[n]),
                    zorder=0,
                )

    def _flush_segments(self, plt, ctx: _PlotContext, upto_cycle: int) -> None:
        bar_height = 0.8
        for idx, node in enumerate(ctx.nodes):
            y = ctx.y_index[node]
            lc = ctx.current_life[idx]
            oc = ctx.current_obs[idx]
            x0 = ctx.segment_start[idx]
            width = upto_cycle - x0
            if width <= 0:
                continue
            plt.barh(
                y + bar_height / 4,
                width,
                height=bar_height / 2,
                left=x0,
                color=LiftCycleStateToColor[lc],
                zorder=2,
            )
            plt.barh(
                y - bar_height / 4,
                width,
                height=bar_height / 2,
                left=x0,
                color=ObservationStateToColor[oc],
                zorder=2,
            )

    def _iterate_history_and_draw(self, plt, ctx: _PlotContext) -> None:
        for item in ctx.history[1:]:
            next_cycle = item.control_cycle
            changed = False
            for i, node in enumerate(ctx.nodes):
                new_life = item.life_cycle_state[node]
                new_obs = item.observation_state[node]
                if new_life != ctx.current_life[i] or new_obs != ctx.current_obs[i]:
                    changed = True
            if changed:
                self._flush_segments(plt=plt, ctx=ctx, upto_cycle=next_cycle)
                for i, node in enumerate(ctx.nodes):
                    new_life = item.life_cycle_state[node]
                    new_obs = item.observation_state[node]
                    if new_life != ctx.current_life[i] or new_obs != ctx.current_obs[i]:
                        ctx.current_life[i] = new_life
                        ctx.current_obs[i] = new_obs
                        ctx.segment_start[i] = next_cycle
        self._flush_segments(plt=plt, ctx=ctx, upto_cycle=ctx.last_cycle + 1)

    def _outline_goal_rows(self, ax, ctx: _PlotContext, Rectangle) -> None:
        full_width = (ctx.last_cycle + 1) - ctx.start_cycle
        bar_height = 0.8
        for n in ctx.nodes:
            if isinstance(n, Goal):
                y = ctx.y_index[n]
                rect = Rectangle(
                    (ctx.start_cycle, y - bar_height / 2),
                    full_width,
                    bar_height,
                    fill=False,
                    lw=1.0,
                    ec="#444",
                    zorder=3,
                )
                ax.add_patch(rect)

    def _format_axes(self, plt, ctx: _PlotContext) -> None:
        plt.xlabel("Control cycle")
        plt.xlim(ctx.start_cycle, ctx.last_cycle + 1)
        plt.xticks(
            np.arange(
                ctx.start_cycle,
                ctx.last_cycle + 2,
                max(1, (ctx.last_cycle - ctx.start_cycle + 1) // 10),
            )
        )
        plt.ylabel("Nodes")
        num_bars = len(ctx.names)
        plt.ylim(-0.8, num_bars - 1 + 0.8)
        plt.yticks([ctx.y_index[n] for n in ctx.nodes], ctx.names)
        plt.gca().yaxis.tick_right()
        plt.tight_layout()

    def _save_figure(self, plt, file_name: str) -> None:
        create_path(file_name)
        plt.savefig(file_name)
        plt.close()
        get_middleware().loginfo(f"Saved gantt chart to {file_name}.")
