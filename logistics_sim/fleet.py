"""Fleet management system (FMS) + a tiny warehouse management system (WMS).

Responsibilities
----------------
* builds the MuJoCo world (layout, robots, loads),
* generates customer orders (Poisson arrivals) and inbound pallets,
* turns them into transport tasks and dispatches them to the nearest
  suitable idle robot (battery aware),
* plans routes (AMR: free navigation on the grid, AGV: magnetic-tape graph),
* owns the traffic manager and resolves deadlocks,
* attaches / detaches loads (weld constraint) and keeps KPIs.

Process flows
-------------
AMR  (goods-to-person):  storage slot -> lift rack -> pick station
                          (queue if busy) -> picking -> back to slot -> lower
AGV  (inbound/outbound): inbound dock  -> lift pallet -> outbound dock -> lower
                          -> truck departs, a new pallet arrives at inbound
"""

from __future__ import annotations

import itertools
import math
import random
from dataclasses import dataclass, field
from typing import Optional

import mujoco
import numpy as np

from .layout import DIRS, Cell, Layout
from .planner import astar
from .robot import Job, Robot
from .scene import AGV_COLOR, AMR_COLORS, LoadSpec, RobotSpec, SceneBuilder
from .traffic import TrafficManager

INF = float("inf")


@dataclass
class SimConfig:
    num_amr: int = 8
    num_agv: int = 4
    rack_fill: float = 0.75            # fraction of storage slots holding a rack
    num_pallets: int = 5
    order_interval: float = 10.0       # mean seconds between customer orders
    pick_time: float = 8.0             # seconds a picker needs per rack visit
    ship_time: float = 15.0            # seconds a pallet waits at the outbound dock
    station_capacity: int = 2          # robots per station (1 at station + queue)
    low_battery: float = 25.0
    resume_battery: float = 60.0
    control_dt: float = 0.05
    seed: int = 7


@dataclass
class Load:
    name: str
    kind: str
    body: int
    qpos_adr: int
    qvel_adr: int
    home: Cell
    cell: Optional[Cell]
    carrier: Optional[str] = None
    reserved: bool = False


@dataclass
class Station:
    name: str
    cell: Cell
    queue_cell: Cell
    queue: list[str] = field(default_factory=list)
    picks: int = 0

    def is_turn(self, robot: Robot) -> bool:
        return bool(self.queue) and self.queue[0] == robot.name


@dataclass
class Task:
    id: int
    kind: str                 # "pick" (AMR) or "transfer" (AGV)
    load: Load
    created: float
    target: Optional[Cell] = None
    station: Optional[Station] = None
    robot: Optional[str] = None
    started: Optional[float] = None
    finished: Optional[float] = None


class FleetManager:
    def __init__(self, config: Optional[SimConfig] = None, layout: Optional[Layout] = None):
        self.cfg = config or SimConfig()
        self.layout = layout or Layout()
        self.rng = random.Random(self.cfg.seed)
        self.traffic = TrafficManager()
        self.time = 0.0
        self._task_ids = itertools.count(1)

        L = self.layout
        chargers = L.cells("C")
        parks = L.cells("A")
        if self.cfg.num_amr > len(chargers):
            raise ValueError(f"layout has only {len(chargers)} AMR chargers")
        if self.cfg.num_agv > len(parks):
            raise ValueError(f"layout has only {len(parks)} AGV parking spots")
        # spread AMR homes over both charger columns
        chargers = sorted(chargers, key=lambda c: (-c[1], c[0]))
        specs = [RobotSpec(f"amr{i}", "AMR", chargers[i], AMR_COLORS[i % len(AMR_COLORS)])
                 for i in range(self.cfg.num_amr)]
        specs += [RobotSpec(f"agv{i}", "AGV", parks[i], AGV_COLOR) for i in range(self.cfg.num_agv)]

        slots = L.cells("S")
        n_racks = int(round(len(slots) * self.cfg.rack_fill))
        rack_cells = sorted(self.rng.sample(slots, n_racks))
        load_specs = [LoadSpec(f"rack{i}", "rack", c) for i, c in enumerate(rack_cells)]
        inbound, outbound = L.cells("I"), L.cells("O")
        pallet_cells = (inbound + outbound)[: self.cfg.num_pallets]
        load_specs += [LoadSpec(f"pallet{i}", "pallet", c) for i, c in enumerate(pallet_cells)]

        self.xml = SceneBuilder(L, specs, load_specs, seed=self.cfg.seed).build()
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.robots: dict[str, Robot] = {}
        for i, s in enumerate(specs):
            self.robots[s.name] = Robot(s, self.model, self.data, self, seed=self.cfg.seed * 100 + i)
        self.loads: dict[str, Load] = {}
        for ls in load_specs:
            j = self.model.joint(f"{ls.name}_free")
            self.loads[ls.name] = Load(ls.name, ls.kind, self.model.body(ls.name).id,
                                       j.qposadr[0], j.dofadr[0], ls.cell, ls.cell)
        self.stations = [Station(f"P{i}", c, (c[0] + 1, c[1])) for i, c in enumerate(L.cells("P"))]
        self.inbound, self.outbound = inbound, outbound
        self.ship_timers: dict[str, float] = {}  # pallet name -> departure time
        for name, ld in self.loads.items():
            if ld.kind == "pallet" and ld.cell in outbound:
                self.ship_timers[name] = self.cfg.ship_time * self.rng.uniform(0.3, 1.0)

        self.pending: list[Task] = []
        self.active: dict[int, Task] = {}
        self.done: list[Task] = []
        self.next_order_t = self.rng.expovariate(1.0 / self.cfg.order_interval)
        self.needs_charge: set[str] = set()
        self.collisions = 0
        self._contact_pairs: set[tuple[int, int]] = set()
        self._chassis = {self.model.geom(f"{n}_chassis").id: n for n in self.robots}
        self.events: list[str] = []
        self._substeps = max(1, int(round(self.cfg.control_dt / self.model.opt.timestep)))

    # ================================================================ helpers
    def load_at(self, cell: Cell) -> Optional[Load]:
        for ld in self.loads.values():
            if ld.cell == cell:
                return ld
        return None

    def log(self, msg: str) -> None:
        self.events.append(f"[{self.time:7.1f}s] {msg}")

    def neighbors(self, robot: Robot) -> list[Cell]:
        if robot.kind == "AGV":
            return self.layout.agv_neighbors(robot.cell)
        return self.layout.amr_neighbors(robot.cell)

    def battery_speed_factor(self, robot: Robot) -> float:
        return 1.0 if robot.battery > 10 else 0.6

    # ============================================================== planning
    def cell_cost(self, robot: Robot, cell: Cell, goal: Optional[Cell]) -> float:
        if cell == goal:
            return 0.0
        k = self.layout.kind(cell)
        cost = 0.0
        if robot.kind == "AGV":
            if k in "IOA":
                return INF
        else:
            if k == "C":
                return INF
            if k == "S":
                if robot.load is not None:
                    return INF
                cost += 1.0 if self.load_at(cell) else 0.3
            elif k in "PQ":
                cost += 6.0
            elif k == "T":
                cost += 0.8
        holder = self.traffic.holder(cell)
        if holder is not None and holder != robot.name:
            other = self.robots[holder]
            cost += 6.0 if other.state in ("WORKING", "LIFTING", "QUEUED", "CHARGING") else 2.0
        return cost

    def may_enter(self, robot: Robot, cell: Cell) -> bool:
        """Dynamic admission check performed right before reserving a cell."""
        if robot.load is not None:
            ld = self.load_at(cell)
            if ld is not None and ld is not robot.load:
                return False
        return True

    def plan(self, robot: Robot, start: Cell, goal: Cell,
             extra_blocked: Optional[set[Cell]] = None) -> Optional[list[Cell]]:
        nbr = self.layout.agv_neighbors if robot.kind == "AGV" else self.layout.amr_neighbors
        yaw = robot.yaw()
        heading = DIRS[int(round(yaw / (math.pi / 2))) % 4]
        blocked = set(extra_blocked or ())
        if robot.load is not None:
            blocked |= {ld.cell for ld in self.loads.values() if ld.cell is not None and ld is not robot.load}
        blocked.discard(goal)
        return astar(start, goal, nbr, cell_cost=lambda c: self.cell_cost(robot, c, goal),
                     blocked=blocked, start_heading=heading)

    def resolve_block(self, robot: Robot) -> None:
        """Called when ``robot`` has waited too long for a cell."""
        blocker = self.robots.get(robot.blocked_by) if robot.blocked_by else None
        if blocker is None or robot.path is None:
            return
        cycle = self.traffic.wait_cycle(robot.name)
        remaining = robot.path[robot.idx:]
        # A parked idle robot is in the way: ask it to step aside.
        if blocker.job is None and blocker.path is None and not blocker.detour:
            if blocker.evade(set(remaining)):
                self.log(f"{blocker.name} yields to {robot.name}")
            return
        if robot.kind == "AGV":
            return  # tape-bound; AMRs in the cycle will make room
        goal = robot.path[-1]
        alt = self.plan(robot, robot.cell, goal, extra_blocked=self.traffic.cells_of(blocker.name))
        if alt is not None and len(alt) <= len(remaining) + 14:
            robot.set_path(alt)
            robot.stats.replans += 1
            self.log(f"{robot.name} re-routes around {blocker.name}")
            return
        if cycle:
            avoid = set(blocker.path[blocker.idx:]) if blocker.path else set()
            if robot.evade(avoid | set(remaining[1:2])):
                self.log(f"{robot.name} steps aside for {blocker.name} (deadlock {'->'.join(cycle)})")

    # ======================================================== loads / weld
    def attach(self, robot: Robot, cell: Cell) -> None:
        ld = self.load_at(cell)
        if ld is None:
            self.log(f"WARNING {robot.name}: nothing to lift at {cell}")
            return
        m, d = self.model, self.data
        tt = robot.turntable
        p1, q1 = d.xpos[tt], d.xquat[tt]
        p2, q2 = d.xpos[ld.body], d.xquat[ld.body]
        qinv = np.zeros(4)
        mujoco.mju_negQuat(qinv, q1)
        relp = np.zeros(3)
        mujoco.mju_rotVecQuat(relp, p2 - p1, qinv)
        relq = np.zeros(4)
        mujoco.mju_mulQuat(relq, qinv, q2)
        m.eq_obj2id[robot.eq] = ld.body
        m.eq_data[robot.eq, 0:3] = 0.0
        m.eq_data[robot.eq, 3:6] = relp
        m.eq_data[robot.eq, 6:10] = relq
        d.eq_active[robot.eq] = 1
        ld.cell = None
        ld.carrier = robot.name
        robot.load = ld

    def detach(self, robot: Robot, cell: Cell) -> None:
        ld = robot.load
        self.data.eq_active[robot.eq] = 0
        ld.cell = cell
        ld.carrier = None
        robot.load = None

    def teleport(self, ld: Load, cell: Cell) -> None:
        x, y = self.layout.world(cell)
        q = self.data.qpos
        q[ld.qpos_adr:ld.qpos_adr + 7] = [x, y, 0.001, 1, 0, 0, 0]
        self.data.qvel[ld.qvel_adr:ld.qvel_adr + 6] = 0
        ld.cell = cell

    # =========================================================== WMS / tasks
    def _generate_orders(self) -> None:
        while self.time >= self.next_order_t:
            self.next_order_t += self.rng.expovariate(1.0 / self.cfg.order_interval)
            candidates = [ld for ld in self.loads.values()
                          if ld.kind == "rack" and not ld.reserved and ld.cell == ld.home]
            if not candidates:
                continue
            ld = self.rng.choice(candidates)
            ld.reserved = True
            t = Task(next(self._task_ids), "pick", ld, self.time)
            self.pending.append(t)
            self.log(f"order #{t.id}: rack {ld.name} @ {ld.home}")

    def _generate_transfers(self) -> None:
        free_out = [c for c in self.outbound if self.load_at(c) is None
                    and not any(t.target == c for t in self.pending + list(self.active.values()))]
        for ld in self.loads.values():
            if not free_out:
                break
            if ld.kind == "pallet" and not ld.reserved and ld.cell in self.inbound:
                ld.reserved = True
                tgt = free_out.pop(0)
                t = Task(next(self._task_ids), "transfer", ld, self.time, target=tgt)
                self.pending.append(t)
                self.log(f"inbound pallet {ld.name} {ld.cell} -> outbound {tgt}")

    def _ship_pallets(self) -> None:
        for name, t_dep in list(self.ship_timers.items()):
            if self.time < t_dep:
                continue
            ld = self.loads[name]
            free_in = [c for c in self.inbound if self.load_at(c) is None
                       and self.traffic.holder(c) is None]
            if not free_in:
                continue
            del self.ship_timers[name]
            self.teleport(ld, self.rng.choice(free_in))
            ld.reserved = False
            self.log(f"truck shipped {name}; new inbound pallet at {ld.cell}")

    def _eligible(self, r: Robot) -> bool:
        if r.name in self.needs_charge:
            if r.battery >= self.cfg.resume_battery:
                self.needs_charge.discard(r.name)
            else:
                return False
        if r.battery < self.cfg.low_battery:
            self.needs_charge.add(r.name)
            self.log(f"{r.name} low battery ({r.battery:.0f}%) -> charging")
            return False
        return r.idle and r.load is None and not r.detour

    def _dispatch(self) -> None:
        for task in list(self.pending):
            kind = "AMR" if task.kind == "pick" else "AGV"
            station = None
            if task.kind == "pick":
                station = min(self.stations, key=lambda s: (len(s.queue), s.picks))
                if len(station.queue) >= self.cfg.station_capacity:
                    continue
            robots = [r for r in self.robots.values() if r.kind == kind and self._eligible(r)]
            if not robots:
                continue
            src = task.load.cell
            r = min(robots, key=lambda r: abs(r.cell[0] - src[0]) + abs(r.cell[1] - src[1]))
            self.pending.remove(task)
            task.robot, task.started = r.name, self.time
            self.active[task.id] = task
            if task.kind == "pick":
                task.station = station
                station.queue.append(r.name)
                steps = [("goto", src), ("lift",), ("station", station), ("dwell", self.cfg.pick_time),
                         ("call", self._leave_station(task)), ("goto", task.load.home), ("lower",)]
            else:
                steps = [("goto", src), ("lift",), ("goto", task.target), ("lower",)]
            r.assign(Job(steps, task))
            self.log(f"task #{task.id} ({task.kind}) -> {r.name}")

    def _leave_station(self, task: Task):
        def fn(robot: Robot) -> None:
            st = task.station
            if robot.name in st.queue:
                st.queue.remove(robot.name)
            st.picks += 1
        return fn

    def on_job_done(self, robot: Robot, job: Job) -> None:
        task: Task = job.task
        task.finished = self.time
        self.active.pop(task.id, None)
        self.done.append(task)
        task.load.reserved = False
        if task.kind == "transfer":
            task.load.reserved = True
            self.ship_timers[task.load.name] = self.time + self.cfg.ship_time
        self.log(f"task #{task.id} done by {robot.name} in {task.finished - task.created:.1f}s")

    def _send_idle_home(self) -> None:
        for r in self.robots.values():
            if r.job is None and not r.detour and r.cell != r.home and r.load is None:
                r.assign(Job([("goto", r.home)], None))

    # ============================================================ simulation
    def _check_collisions(self) -> None:
        d = self.data
        pairs = set()
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = c.geom1, c.geom2
            if g1 in self._chassis or g2 in self._chassis:
                if c.dist < -0.002:
                    pairs.add((min(g1, g2), max(g1, g2)))
        new = pairs - self._contact_pairs
        if new:
            self.collisions += len(new)
            for g1, g2 in new:
                self.log(f"COLLISION {self.model.geom(g1).name or g1} <-> {self.model.geom(g2).name or g2}")
        self._contact_pairs = pairs

    def control_step(self) -> None:
        """Run one control period (FMS logic + robot controllers + physics)."""
        dt = self.cfg.control_dt
        self._generate_orders()
        self._generate_transfers()
        self._ship_pallets()
        self._dispatch()
        self._send_idle_home()
        for r in self.robots.values():
            r.update(dt, self.time)
        for _ in range(self._substeps):
            mujoco.mj_step(self.model, self.data)
        self.time = float(self.data.time)
        self._check_collisions()

    def run(self, duration: float, callback=None) -> None:
        end = self.time + duration
        while self.time < end:
            self.control_step()
            if callback is not None:
                callback(self)

    # ================================================================== KPIs
    def kpis(self) -> dict:
        picks = [t for t in self.done if t.kind == "pick"]
        transfers = [t for t in self.done if t.kind == "transfer"]
        hours = max(self.time, 1e-6) / 3600.0

        def avg(xs):
            return float(np.mean(xs)) if xs else 0.0

        robots = {}
        for r in self.robots.values():
            robots[r.name] = {
                "kind": r.kind,
                "tasks": r.stats.tasks_done,
                "distance_m": round(r.stats.distance, 1),
                "utilization": round(r.stats.busy_time / max(self.time, 1e-6), 3),
                "wait_s": round(r.stats.wait_time, 1),
                "replans": r.stats.replans,
                "evasions": r.stats.evasions,
                "battery": round(r.battery, 1),
            }
        return {
            "sim_time_s": round(self.time, 1),
            "orders_created": sum(1 for t in self.done + self.pending + list(self.active.values()) if t.kind == "pick"),
            "picks_completed": len(picks),
            "pallets_transferred": len(transfers),
            "picks_per_hour": round(len(picks) / hours, 1),
            "pallets_per_hour": round(len(transfers) / hours, 1),
            "avg_pick_lead_time_s": round(avg([t.finished - t.created for t in picks]), 1),
            "avg_transfer_lead_time_s": round(avg([t.finished - t.created for t in transfers]), 1),
            "pending_tasks": len(self.pending),
            "active_tasks": len(self.active),
            "collisions": self.collisions,
            "station_picks": {s.name: s.picks for s in self.stations},
            "robots": robots,
        }
