"""Grid path planning (A* with turn penalty).

The search state is ``(cell, heading)`` so that 90-degree turns - which a
differential-drive robot executes by stopping and rotating in place - are
charged an explicit time cost. This produces the long straight runs that
real fleet managers prefer.
"""

from __future__ import annotations

import heapq
import itertools
from typing import Callable, Iterable, Optional

from .layout import Cell

NeighborFn = Callable[[Cell], Iterable[Cell]]
CostFn = Callable[[Cell], float]


def _dir(a: Cell, b: Cell) -> tuple[int, int]:
    return (b[0] - a[0], b[1] - a[1])


def astar(start: Cell, goal: Cell, neighbors: NeighborFn,
          cell_cost: Optional[CostFn] = None, blocked: Optional[set[Cell]] = None,
          turn_cost: float = 1.2, start_heading: Optional[tuple[int, int]] = None,
          max_expansions: int = 50000) -> Optional[list[Cell]]:
    """Return the cell path ``[start, ..., goal]`` or ``None``.

    ``cell_cost(c)`` is an extra cost for entering ``c`` (``inf`` = forbidden);
    ``blocked`` cells are never entered except when they are the goal.
    """
    if start == goal:
        return [start]
    blocked = blocked or set()

    def h(c: Cell) -> float:
        return abs(c[0] - goal[0]) + abs(c[1] - goal[1])

    tie = itertools.count()
    start_state = (start, start_heading)
    open_heap = [(h(start), next(tie), 0.0, start_state)]
    g_best = {start_state: 0.0}
    parent: dict = {start_state: None}
    expansions = 0
    while open_heap:
        _, _, g, state = heapq.heappop(open_heap)
        if g > g_best.get(state, float("inf")):
            continue
        cell, heading = state
        if cell == goal:
            path = []
            s = state
            while s is not None:
                path.append(s[0])
                s = parent[s]
            return path[::-1]
        expansions += 1
        if expansions > max_expansions:
            return None
        for n in neighbors(cell):
            if n in blocked and n != goal:
                continue
            extra = cell_cost(n) if cell_cost else 0.0
            if extra == float("inf"):
                continue
            d = _dir(cell, n)
            step = 1.0 + extra
            if heading is not None and d != heading:
                # 90 deg costs one turn, 180 deg costs two
                step += turn_cost * (2 if d == (-heading[0], -heading[1]) else 1)
            ns = (n, d)
            ng = g + step
            if ng < g_best.get(ns, float("inf")):
                g_best[ns] = ng
                parent[ns] = state
                heapq.heappush(open_heap, (ng + h(n), next(tie), ng, ns))
    return None


def path_length(path: list[Cell]) -> int:
    return max(0, len(path) - 1)
