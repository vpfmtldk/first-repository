"""Traffic management: grid-cell reservation (zone control).

Every robot must *own* a cell before entering it, and keeps it until its
whole footprint (including a carried load) has left. This is the classic
zone-blocking scheme used by AGV/AMR fleet managers and guarantees that two
robots never occupy the same cell. Deadlocks that may arise (head-on
encounters, circular waits) are detected through the wait-for graph and
resolved by the robots themselves (re-routing or evasive moves).
"""

from __future__ import annotations

from typing import Optional

from .layout import Cell


class TrafficManager:
    def __init__(self) -> None:
        self.owner: dict[Cell, str] = {}
        self.waiting_on: dict[str, Optional[str]] = {}

    def holder(self, cell: Cell) -> Optional[str]:
        return self.owner.get(cell)

    def try_lock(self, cell: Cell, robot: str) -> bool:
        cur = self.owner.get(cell)
        if cur is None or cur == robot:
            self.owner[cell] = robot
            return True
        return False

    def release(self, cell: Cell, robot: str) -> None:
        if self.owner.get(cell) == robot:
            del self.owner[cell]

    def cells_of(self, robot: str) -> set[Cell]:
        return {c for c, r in self.owner.items() if r == robot}

    def set_waiting(self, robot: str, on: Optional[str]) -> None:
        self.waiting_on[robot] = on

    def wait_cycle(self, robot: str) -> list[str]:
        """Return the robots forming a wait-for cycle through ``robot`` (or [])."""
        chain = [robot]
        cur = self.waiting_on.get(robot)
        while cur is not None and len(chain) < 64:
            if cur == robot:
                return chain
            if cur in chain:
                return []  # a cycle that does not involve us
            chain.append(cur)
            cur = self.waiting_on.get(cur)
        return []
