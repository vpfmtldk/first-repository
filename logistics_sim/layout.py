"""Warehouse floor plan.

The warehouse is a 1 m grid. Every cell has a type; robots move from cell
center to cell center (4-connected), which is how most commercial fleet
managers (Kiva/Amazon Robotics, Geek+, Hikrobot ...) discretise the floor
with QR-code / magnetic markers.

Legend
------
``#``  wall / not traversable
``X``  fixed equipment (conveyors, machines) - not traversable
``.``  free floor (AMR area)
``S``  storage slot (an AMR lifts a mobile rack here)
``P``  pick station (human picker works on the rack brought by an AMR)
``Q``  queue cell in front of a pick station
``C``  AMR charger / home position
``T``  AGV guide tape (magnetic tape) - one-way loop, AMRs may cross it
``I``  inbound dock (pallet arrives from a truck)
``O``  outbound dock (pallet leaves on a truck)
``A``  AGV parking / charging spot
``o``  operator position (visual only)
"""

from __future__ import annotations

from dataclasses import dataclass, field

# Rows are written top (max y) to bottom (y = 0).
DEFAULT_MAP = [
    "##############################",
    "####o####o####o####o####o#####",
    "#...PQ...PQ...PQ...PQ...PQ...#",
    "#............................#",
    "#C..SS.SS.SS.SS.SS.SS.SS.SS.C#",
    "#C..SS.SS.SS.SS.SS.SS.SS.SS.C#",
    "#C..........................C#",
    "#C..SS.SS.SS.SS.SS.SS.SS.SS.C#",
    "#C..SS.SS.SS.SS.SS.SS.SS.SS.C#",
    "#C..........................C#",
    "#.TTTTTTTTTTTTTTTTTTTTTTTTTT.#",
    "#ITXXXXXXXXXXXXXXXXXXXXXXXXTO#",
    "#.TXXXXXXXXXXXXXXXXXXXXXXXXT.#",
    "#ITXXXXXXXXXXXXXXXXXXXXXXXXTO#",
    "#.TXXXXXXXXXXXXXXXXXXXXXXXXT.#",
    "#ITXXXXXXXXXXXXXXXXXXXXXXXXTO#",
    "#.TXXXXXXXXXXXXXXXXXXXXXXXXT.#",
    "#.TTTTTTTTTTTTTTTTTTTTTTTTTT.#",
    "#....A..A..A..A..A..A........#",
    "##############################",
]

DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # E, N, W, S

Cell = tuple[int, int]


@dataclass
class Layout:
    rows: list[str] = field(default_factory=lambda: list(DEFAULT_MAP))
    cell_size: float = 1.0

    def __post_init__(self) -> None:
        self.height = len(self.rows)
        self.width = len(self.rows[0])
        assert all(len(r) == self.width for r in self.rows), "ragged map"
        self.grid: dict[Cell, str] = {}
        for row_idx, row in enumerate(self.rows):
            y = self.height - 1 - row_idx
            for x, ch in enumerate(row):
                self.grid[(x, y)] = ch
        self._build_tape_graph()

    # ------------------------------------------------------------------ queries
    def kind(self, cell: Cell) -> str:
        return self.grid.get(cell, "#")

    def cells(self, kind: str) -> list[Cell]:
        return sorted((c for c, k in self.grid.items() if k == kind),
                      key=lambda c: (c[0], c[1]))

    def world(self, cell: Cell) -> tuple[float, float]:
        """Cell center in world coordinates (m)."""
        return (cell[0] * self.cell_size, cell[1] * self.cell_size)

    def to_cell(self, x: float, y: float) -> Cell:
        return (int(round(x / self.cell_size)), int(round(y / self.cell_size)))

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    @property
    def center(self) -> tuple[float, float]:
        return ((self.width - 1) * self.cell_size / 2,
                (self.height - 1) * self.cell_size / 2)

    # -------------------------------------------------------------- AMR graph
    AMR_PASSABLE = set(".SPQCTo") - {"o"}

    def amr_neighbors(self, cell: Cell) -> list[Cell]:
        out = []
        for dx, dy in DIRS:
            n = (cell[0] + dx, cell[1] + dy)
            if self.kind(n) in self.AMR_PASSABLE:
                out.append(n)
        return out

    # --------------------------------------------------------------- AGV graph
    def _build_tape_graph(self) -> None:
        """Directed graph of the magnetic-tape loop (clockwise).

        AGVs are *guided* vehicles: they can only follow the tape in its
        direction of travel, plus short bidirectional spurs into docks and
        parking spots.
        """
        tape = {c for c, k in self.grid.items() if k == "T"}
        xs = [c[0] for c in tape]
        ys = [c[1] for c in tape]
        x_min, x_max, y_min, y_max = min(xs), max(xs), min(ys), max(ys)
        edges: dict[Cell, set[Cell]] = {c: set() for c in tape}
        for (x, y) in tape:
            if x == x_min and y < y_max:          # left column  -> north
                nxt = (x, y + 1)
            elif y == y_max and x < x_max:        # top row      -> east
                nxt = (x + 1, y)
            elif x == x_max and y > y_min:        # right column -> south
                nxt = (x, y - 1)
            else:                                 # bottom row   -> west
                nxt = (x - 1, y)
            if nxt in tape:
                edges[(x, y)].add(nxt)
        # Spurs to docks / parking (enter and leave on the same tape cell).
        for c, k in self.grid.items():
            if k in "IOA":
                for dx, dy in DIRS:
                    n = (c[0] + dx, c[1] + dy)
                    if n in tape:
                        edges.setdefault(c, set()).add(n)
                        edges[n].add(c)
        self.tape_edges = edges

    def agv_neighbors(self, cell: Cell) -> list[Cell]:
        return sorted(self.tape_edges.get(cell, ()))
