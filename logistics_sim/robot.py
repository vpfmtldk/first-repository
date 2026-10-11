"""On-board robot controller.

Each robot runs a small *job script* handed to it by the fleet manager,
e.g. ``[goto(rack), lift, station(P3), dwell(10), leave_station, goto(slot),
lower]``. Motion is cell-to-cell along a planned grid path:

* cells ahead are reserved through the :class:`TrafficManager` before the
  robot moves into them (look-ahead of a few cells keeps motion smooth),
* the base is steered with pure pursuit on the current straight segment and
  rotates in place at corners (differential drive),
* the speed profile respects acceleration limits and always allows the robot
  to stop at the last reserved cell.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Optional

import mujoco
import numpy as np

from .layout import Cell
from .scene import LIFT_TRAVEL, RobotSpec

if TYPE_CHECKING:  # pragma: no cover
    from .fleet import FleetManager


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class DriveParams:
    v_max: float
    v_max_loaded: float
    accel: float
    w_max: float
    lookahead_cells: int = 3


AMR_PARAMS = DriveParams(v_max=1.5, v_max_loaded=1.1, accel=0.9, w_max=1.8)
AGV_PARAMS = DriveParams(v_max=1.2, v_max_loaded=0.9, accel=0.6, w_max=1.2)

STATE_COLORS = {
    "IDLE": (0.2, 1.0, 0.2), "MOVING": (0.2, 0.5, 1.0), "WAITING": (1.0, 0.9, 0.1),
    "LIFTING": (1.0, 0.5, 0.0), "WORKING": (1.0, 0.5, 0.0), "CHARGING": (0.1, 1.0, 1.0),
    "QUEUED": (1.0, 0.9, 0.1),
}


@dataclass
class RobotStats:
    distance: float = 0.0
    busy_time: float = 0.0
    wait_time: float = 0.0
    tasks_done: int = 0
    replans: int = 0
    evasions: int = 0


@dataclass
class Job:
    steps: list[tuple] = field(default_factory=list)
    task: Any = None


class Robot:
    def __init__(self, spec: RobotSpec, model: mujoco.MjModel, data: mujoco.MjData,
                 fleet: "FleetManager", seed: int = 0):
        self.spec = spec
        self.name = spec.name
        self.kind = spec.kind
        self.home = spec.home
        self.m, self.d = model, data
        self.fleet = fleet
        self.layout = fleet.layout
        self.traffic = fleet.traffic
        self.params = AMR_PARAMS if self.kind == "AMR" else AGV_PARAMS
        self.rng = random.Random(seed)

        n = self.name
        self.body = model.body(n).id
        self.yaw_adr = model.joint(f"{n}_yaw").qposadr[0]
        self.lift_adr = model.joint(f"{n}_lift").qposadr[0]
        self.act_vx = model.actuator(f"{n}_vx").id
        self.act_vy = model.actuator(f"{n}_vy").id
        self.act_wz = model.actuator(f"{n}_wz").id
        self.act_lift = model.actuator(f"{n}_lift").id
        self.act_turn = model.actuator(f"{n}_turn").id
        self.eq = model.equality(f"{n}_grip").id
        self.led = model.geom(f"{n}_led").id
        self.turntable = model.body(f"{n}_turntable").id

        self.cell: Cell = self.home
        self.traffic.try_lock(self.home, self.name)
        self.path: Optional[list[Cell]] = None
        self.idx = 0
        self.v = 0.0
        self.w = 0.0
        self.lift_cmd = 0.0
        self.load = None            # Load currently carried
        self.battery = 100.0
        self.job: Optional[Job] = None
        self.step_timer = 0.0
        self.wait_timer = 0.0
        self.wait_patience = self._new_patience()
        self.blocked_by: Optional[str] = None
        self.replan_pending = False
        self.state = "IDLE"
        self.stats = RobotStats()
        self.last_xy = self.xy().copy()
        self.charging = False
        self.pause = 0.0
        self.detour = False

    # ----------------------------------------------------------------- sensing
    def xy(self) -> np.ndarray:
        return self.d.xpos[self.body][:2]

    def yaw(self) -> float:
        return float(self.d.qpos[self.yaw_adr])

    def lift_pos(self) -> float:
        return float(self.d.qpos[self.lift_adr])

    def center(self, cell: Cell) -> np.ndarray:
        return np.array(self.layout.world(cell))

    @property
    def idle(self) -> bool:
        return self.job is None or self.job.task is None

    def _new_patience(self) -> float:
        return self.rng.uniform(1.5, 3.0) if self.kind == "AMR" else self.rng.uniform(4.0, 6.0)

    # ---------------------------------------------------------------- job API
    def assign(self, job: Job) -> None:
        self.job = job
        self.step_timer = 0.0
        self.charging = False
        self._request_replan()

    def current_step(self) -> Optional[tuple]:
        if self.job and self.job.steps:
            return self.job.steps[0]
        return None

    def _pop_step(self) -> None:
        self.job.steps.pop(0)
        self.step_timer = 0.0
        if not self.job.steps:
            job, self.job = self.job, None
            if job.task is not None:
                self.stats.tasks_done += 1
                self.fleet.on_job_done(self, job)

    # ------------------------------------------------------------- main update
    def update(self, dt: float, t: float) -> None:
        if self.detour:
            if self._follow(dt) == "ARRIVED":
                self.detour = False
                self.pause = self.rng.uniform(1.0, 2.5)
            self.state = "MOVING"
            self._apply_controls()
            self._bookkeeping(dt)
            return
        step = self.current_step()
        self.step_timer += dt
        moving_goal: Optional[Cell] = None
        state = "IDLE"

        if step is None:
            self.v = self.w = 0.0
            if self.cell == self.home and self._at_center(self.cell, 0.1):
                state = "CHARGING" if self.battery < 99.5 else "IDLE"
        else:
            op = step[0]
            if op == "goto":
                moving_goal = step[1]
            elif op == "station":
                st = step[1]
                moving_goal = st.cell if st.is_turn(self) else st.queue_cell
            elif op == "lift":
                self.v = self.w = 0.0
                self.lift_cmd = LIFT_TRAVEL
                state = "LIFTING"
                if self.lift_pos() > LIFT_TRAVEL - 0.008 and self.step_timer > 0.4:
                    self.fleet.attach(self, self.cell)
                    self._pop_step()
            elif op == "lower":
                self.v = self.w = 0.0
                state = "LIFTING"
                if self.load is not None:
                    self.fleet.detach(self, self.cell)
                self.lift_cmd = 0.0
                if self.lift_pos() < 0.004 and self.step_timer > 0.6:
                    self._pop_step()
            elif op == "dwell":
                self.v = self.w = 0.0
                state = "WORKING"
                if self.step_timer >= step[1]:
                    self._pop_step()
            elif op == "call":
                step[1](self)
                self._pop_step()

        if moving_goal is not None:
            state = self._navigate(moving_goal, dt)
            if state == "ARRIVED":
                state = "MOVING"
                op = self.current_step()
                if op[0] == "goto":
                    self._pop_step()
                elif op[0] == "station":
                    if self.cell == op[1].cell:
                        self._pop_step()
                    else:
                        state = "QUEUED"

        self.state = state
        self._apply_controls()
        self._bookkeeping(dt)

    # ------------------------------------------------------------- navigation
    def _request_replan(self) -> None:
        self.replan_pending = True

    def _at_center(self, cell: Cell, tol: float) -> bool:
        return float(np.linalg.norm(self.xy() - self.center(cell))) < tol

    def set_path(self, path: list[Cell]) -> None:
        """Adopt a new path that starts at the robot's current cell."""
        assert path[0] == self.cell, (self.name, path[0], self.cell)
        for c in self.traffic.cells_of(self.name):
            if c != self.cell:
                self.traffic.release(c, self.name)
        self.path = path
        self.idx = 0
        self.replan_pending = False

    def _navigate(self, goal: Cell, dt: float) -> str:
        if self.pause > 0:
            self.pause -= dt
            self.v = self.w = 0.0
            return "WAITING"
        if self.path is not None and self.path[-1] != goal:
            self.replan_pending = True
        need_plan = self.path is None or self.replan_pending
        # (Re)plan only while sitting on a cell center so the reservation
        # invariant (we own the cell we are in) always holds.
        if need_plan and self._at_center(self.cell, 0.12):
            if self.cell == goal:
                self.set_path([goal])
            else:
                path = self.fleet.plan(self, self.cell, goal)
                if path is None:
                    self.pause = 1.0
                    self.v = self.w = 0.0
                    return "WAITING"
                self.set_path(path)
        result = self._follow(dt)
        if result == "ARRIVED" and self.cell != goal:
            return "MOVING"
        return result

    def _follow(self, dt: float) -> str:
        p = self.params
        path = self.path
        if path is None:
            self.v = self.w = 0.0
            return "WAITING"
        pos = self.xy()
        last = len(path) - 1

        # 1) reserve cells ahead
        held_until = self.idx
        blocked_next = False
        for j in range(self.idx + 1, min(last, self.idx + p.lookahead_cells) + 1):
            c = path[j]
            if not self.fleet.may_enter(self, c) or not self.traffic.try_lock(c, self.name):
                if j == self.idx + 1:
                    blocked_next = True
                    self.blocked_by = self.traffic.holder(c)
                break
            held_until = j
        if blocked_next:
            self.traffic.set_waiting(self.name, self.blocked_by)
            self.wait_timer += dt
            if self.wait_timer > self.wait_patience:
                self.wait_timer = 0.0
                self.wait_patience = self._new_patience()
                self.fleet.resolve_block(self)
                if self.path is not path:  # re-routed / evading: restart next tick
                    self.v = self.w = 0.0
                    return "WAITING"
        else:
            self.traffic.set_waiting(self.name, None)
            self.blocked_by = None
            self.wait_timer = 0.0

        # 2) arrived at the end of the path?
        if self.idx == last:
            err = self.center(path[last]) - pos
            if np.linalg.norm(err) < 0.02:
                self.v = self.w = 0.0
                self.path = None
                return "ARRIVED"
            # final creep onto the center along the last segment
            d_prev = self._seg_dir(last - 1) if last > 0 else None
            if d_prev is None:
                self.v = self.w = 0.0
                self.path = None
                return "ARRIVED"
            along = float(err @ d_prev)
            if along <= 0.0:
                self.v = self.w = 0.0
                self.path = None
                return "ARRIVED"
            self.v = min(0.15, along * 2.0)
            self.w = 0.0
            return "MOVING"

        # 3) where must we be able to stop?  (end of reservation or a corner)
        seg = self._seg_dir(self.idx)
        stop = held_until
        for j in range(self.idx + 1, held_until + 1):
            if j == last or self._seg_dir(j) != seg:
                stop = j
                break
        seg_v = np.array(seg, dtype=float)
        origin = self.center(path[self.idx])
        dist_stop = float((self.center(path[stop]) - pos) @ seg_v)

        # 4) heading control: rotate in place if badly misaligned
        heading_ref = math.atan2(seg_v[1], seg_v[0])
        yaw = self.yaw()
        err_dir = wrap(heading_ref - yaw)
        if abs(err_dir) > 0.2:
            self.v = 0.0
            w = 3.0 * err_dir
            w = math.copysign(max(abs(w), 0.25), w)
            self.w = float(np.clip(w, -p.w_max, p.w_max))
            return "MOVING"

        # pure pursuit on the segment line
        s = float((pos - origin) @ seg_v)
        look = origin + seg_v * (s + 0.45)
        desired = math.atan2(look[1] - pos[1], look[0] - pos[0])
        err = wrap(desired - yaw)
        self.w = float(np.clip(4.0 * err, -p.w_max, p.w_max))

        v_max = p.v_max_loaded if self.load is not None else p.v_max
        v_max *= self.fleet.battery_speed_factor(self)
        v_stop = math.sqrt(2 * p.accel * max(dist_stop, 0.0))
        v_target = min(v_max, v_stop) * max(0.0, math.cos(err))
        if dist_stop < 0.01:
            v_target = 0.0
        self.v = min(v_target, self.v + p.accel * dt) if v_target > self.v else v_target

        # 5) progress: passed the next cell center?
        nxt = self.center(path[self.idx + 1])
        to_next = float((nxt - pos) @ seg_v)
        threshold = 0.02 if self.idx + 1 == stop else 0.10
        if to_next < threshold:
            self.traffic.release(path[self.idx], self.name)
            self.idx += 1
            self.cell = path[self.idx]
            if self.replan_pending and self.idx < last:
                # Abort the rest of the current path; replanning happens on
                # the next tick from this cell center.
                self.path = path[: self.idx + 1]
        return "WAITING" if (blocked_next and self.v < 0.05) else "MOVING"

    def _seg_dir(self, i: int) -> tuple[int, int]:
        a, b = self.path[i], self.path[i + 1]
        return (b[0] - a[0], b[1] - a[1])

    def evade(self, avoid: set[Cell]) -> bool:
        """Step aside to a free neighbouring cell (used to break deadlocks)."""
        if self.detour or not self._at_center(self.cell, 0.12):
            return False
        options = []
        for n in self.fleet.neighbors(self):
            if n in avoid or self.traffic.holder(n) is not None:
                continue
            if not self.fleet.may_enter(self, n) or self.fleet.cell_cost(self, n, goal=None) == float("inf"):
                continue
            options.append(n)
        if not options:
            return False
        target = self.rng.choice(options)
        self.set_path([self.cell, target])
        self.detour = True
        self.stats.evasions += 1
        return True

    # ---------------------------------------------------------------- actuation
    def _apply_controls(self) -> None:
        yaw = self.yaw()
        c = self.d.ctrl
        c[self.act_vx] = self.v * math.cos(yaw)
        c[self.act_vy] = self.v * math.sin(yaw)
        c[self.act_wz] = self.w
        c[self.act_lift] = self.lift_cmd
        # turntable counter-rotates so the load keeps its world orientation
        c[self.act_turn] = -yaw
        rgb = STATE_COLORS.get(self.state, (1, 1, 1))
        if self.battery < 20:
            rgb = (1.0, 0.1, 0.1)
        self.m.geom_rgba[self.led, :3] = rgb

    def _bookkeeping(self, dt: float) -> None:
        xy = self.xy().copy()
        step_dist = float(np.linalg.norm(xy - self.last_xy))
        self.last_xy = xy
        self.stats.distance += step_dist
        if not self.idle:
            self.stats.busy_time += dt
        if self.state in ("WAITING",):
            self.stats.wait_time += dt
        # battery model: base load + traction + lift
        drain = 0.01 + 0.10 * step_dist / max(dt, 1e-6) + (0.05 if self.load is not None else 0.0)
        if self.state == "CHARGING":
            self.battery = min(100.0, self.battery + 0.6 * dt)
        else:
            self.battery = max(0.0, self.battery - drain * dt)
