"""Smoke / unit tests:  python -m pytest -q"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from logistics_sim.fleet import FleetManager, SimConfig  # noqa: E402
from logistics_sim.layout import Layout  # noqa: E402
from logistics_sim.planner import astar  # noqa: E402
from logistics_sim.traffic import TrafficManager  # noqa: E402


def test_layout_is_consistent():
    L = Layout()
    assert len(L.cells("P")) == 5 and len(L.cells("C")) == 12
    # every storage slot is reachable from an aisle
    for s in L.cells("S"):
        assert any(L.kind(n) == "." for n in L.amr_neighbors(s)), s


def test_agv_tape_is_one_way_loop():
    L = Layout()
    path = astar((1, 8), (28, 4), L.agv_neighbors)
    assert path is not None
    # the return trip must go around the loop (no reverse driving on tape)
    back = astar((28, 4), (1, 8), L.agv_neighbors)
    assert back is not None and len(back) > len(path) - 2
    for a, b in zip(back, back[1:]):
        assert b in L.agv_neighbors(a)


def test_astar_prefers_straight_paths():
    L = Layout()
    p = astar((3, 14), (27, 14), L.amr_neighbors)
    assert p is not None and len(p) == 25  # straight down the aisle


def test_traffic_locks_and_cycle_detection():
    tm = TrafficManager()
    assert tm.try_lock((0, 0), "a") and not tm.try_lock((0, 0), "b")
    tm.set_waiting("a", "b")
    tm.set_waiting("b", "a")
    assert set(tm.wait_cycle("a")) == {"a", "b"}
    tm.release((0, 0), "a")
    assert tm.try_lock((0, 0), "b")


def test_fleet_runs_without_collisions():
    fleet = FleetManager(SimConfig(num_amr=6, num_agv=3, order_interval=5.0, seed=3))
    fleet.run(150.0)
    k = fleet.kpis()
    assert k["collisions"] == 0
    assert k["picks_completed"] >= 3
    assert k["pallets_transferred"] >= 1
    # no cell is reserved by two robots and every robot owns its own cell
    for r in fleet.robots.values():
        assert fleet.traffic.holder(r.cell) == r.name
    # carried racks are lifted off the floor
    for ld in fleet.loads.values():
        if ld.carrier is not None:
            assert fleet.data.xpos[ld.body][2] > 0.02
