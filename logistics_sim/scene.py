"""MJCF (MuJoCo XML) generator for the warehouse, the robots and the loads.

Robot model
-----------
Both AMRs and AGVs are lift-type mobile robots (the "drive under and lift"
design used by Kiva-style goods-to-person AMRs and lift AGVs):

    robot body   slide x, slide y, hinge yaw  -> planar base, velocity actuators
      lift       slide z                       -> lift actuator (position)
        turntable hinge z                      -> keeps the load's orientation
                                                  fixed while the base rotates

The base is driven with differential-drive kinematics (the controller only
ever commands a forward speed ``v`` and a yaw rate ``w``); the wheels are
visual. Loads (mobile racks / pallet stands) are free bodies resting on four
legs. When a robot lifts a load, a weld equality constraint is switched on
between the turntable and the load, emulating the locking pins of real lift
robots.
"""

from __future__ import annotations

import random
from dataclasses import dataclass

from .layout import Layout

# --------------------------------------------------------------- dimensions
LOAD_HALF = 0.43            # half footprint of a rack / pallet stand (m)
LEG_HALF = 0.025
LEG_HEIGHT = 0.30           # underside of the load plate
PLATE_HALF_T = 0.02
LIFT_TRAVEL = 0.08          # lift stroke (m)
PLATFORM_TOP = 0.27         # lift platform top when lowered
AMR_RADIUS = 0.28
AGV_HALF = (0.35, 0.23)

AMR_COLORS = [
    (0.95, 0.55, 0.10), (0.10, 0.55, 0.95), (0.20, 0.75, 0.35), (0.85, 0.25, 0.30),
    (0.60, 0.35, 0.85), (0.10, 0.75, 0.80), (0.90, 0.80, 0.15), (0.95, 0.45, 0.70),
    (0.55, 0.55, 0.55), (0.45, 0.30, 0.15), (0.30, 0.90, 0.60), (0.20, 0.30, 0.70),
]
AGV_COLOR = (0.95, 0.80, 0.05)


@dataclass
class RobotSpec:
    name: str
    kind: str          # "AMR" or "AGV"
    home: tuple[int, int]
    color: tuple[float, float, float]


@dataclass
class LoadSpec:
    name: str
    kind: str          # "rack" or "pallet"
    cell: tuple[int, int]


def _rgba(c, a=1.0):
    return f"{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} {a:.2f}"


class SceneBuilder:
    def __init__(self, layout: Layout, robots: list[RobotSpec],
                 loads: list[LoadSpec], seed: int = 0):
        self.layout = layout
        self.robots = robots
        self.loads = loads
        self.rng = random.Random(seed)

    # ------------------------------------------------------------------ main
    def build(self) -> str:
        cx, cy = self.layout.center
        W, H = self.layout.width, self.layout.height
        parts = [f"""
<mujoco model="agv_amr_warehouse">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.005" integrator="implicitfast" cone="pyramidal"/>
  <size memory="64M"/>
  <visual>
    <global offwidth="1920" offheight="1080" azimuth="90" elevation="-55"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.45 0.45 0.45"/>
    <map znear="0.01" zfar="200"/>
  </visual>
  <asset>
    <texture name="floor_tex" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.62 0.64 0.66" rgb2="0.58 0.60 0.62" mark="edge" markrgb="0.45 0.47 0.5"/>
    <material name="floor_mat" texture="floor_tex" texrepeat="{W} {H}" reflectance="0.05"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.75 0.8 0.85" rgb2="0.3 0.33 0.36"
             width="256" height="256"/>
    <material name="wall_mat" rgba="0.82 0.84 0.86 1"/>
    <material name="tape_mat" rgba="1.0 0.85 0.0 1" emission="0.3"/>
  </asset>
  <default>
    <geom friction="0.9 0.01 0.001" solref="0.01 1" solimp="0.95 0.99 0.001"/>
    <default class="visual">
      <geom contype="0" conaffinity="0" density="0" group="2"/>
    </default>
    <default class="static">
      <geom contype="1" conaffinity="3" group="1"/>
    </default>
  </default>
  <worldbody>
    <light name="sun" directional="true" pos="{cx} {cy} 20" dir="0.2 0.3 -1" diffuse="0.6 0.6 0.6" castshadow="true"/>
    <light pos="{cx - 8} {cy} 8" dir="0 0 -1" diffuse="0.25 0.25 0.25" castshadow="false"/>
    <light pos="{cx + 8} {cy} 8" dir="0 0 -1" diffuse="0.25 0.25 0.25" castshadow="false"/>
    <camera name="overview" pos="{cx} {cy - 17} 19" xyaxes="1 0 0 0 0.75 0.66"/>
    <camera name="top" pos="{cx} {cy} 30" xyaxes="1 0 0 0 1 0"/>
    <camera name="pick_area" pos="{cx} {cy - 1} 8" xyaxes="1 0 0 0 0.6 0.8"/>
    <geom name="floor" type="plane" pos="{cx} {cy} 0" size="{W / 2 + 1} {H / 2 + 1} 0.1"
          material="floor_mat" contype="1" conaffinity="1"/>
"""]
        parts.append(self._static())
        for i, r in enumerate(self.robots):
            parts.append(self._robot(r))
        for ld in self.loads:
            parts.append(self._load(ld))
        parts.append("  </worldbody>\n")
        parts.append(self._contact_excludes())
        parts.append(self._equalities())
        parts.append(self._actuators())
        parts.append("</mujoco>\n")
        return "".join(parts)

    # --------------------------------------------------------------- static
    def _static(self) -> str:
        L = self.layout
        g = []
        W, H = L.width, L.height
        # Perimeter walls, drawn at the outer edge so loads can rotate freely
        # in boundary cells.
        t, h = 0.1, 1.2
        g.append(f'    <geom class="static" type="box" material="wall_mat" pos="{(W-1)/2} {-0.5-t} {h}" size="{W/2+2*t} {t} {h}"/>')
        g.append(f'    <geom class="static" type="box" material="wall_mat" pos="{(W-1)/2} {H-0.5+t} {h}" size="{W/2+2*t} {t} {h}"/>')
        g.append(f'    <geom class="static" type="box" material="wall_mat" pos="{-0.5-t} {(H-1)/2} {h}" size="{t} {H/2} {h}"/>')
        g.append(f'    <geom class="static" type="box" material="wall_mat" pos="{W-0.5+t} {(H-1)/2} {h}" size="{t} {H/2} {h}"/>')

        for (x, y), k in sorted(L.grid.items()):
            if k == "T":
                g.append(f'    <geom class="visual" type="box" material="tape_mat" pos="{x} {y} 0.001" size="0.5 0.04 0.001"'
                         f' euler="0 0 {self._tape_angle((x, y))}"/>')
                # QR-code marker at the cell center
                g.append(f'    <geom class="visual" type="box" rgba="0.1 0.1 0.1 1" pos="{x} {y} 0.0015" size="0.06 0.06 0.001"/>')
            elif k in ".SPQC":
                g.append(f'    <geom class="visual" type="box" rgba="0.15 0.15 0.15 1" pos="{x} {y} 0.001" size="0.05 0.05 0.001"/>')
            if k == "S":
                for sx, sy, ex, ey in [(0, 0.47, 0.47, 0.01), (0, -0.47, 0.47, 0.01), (0.47, 0, 0.01, 0.47), (-0.47, 0, 0.01, 0.47)]:
                    g.append(f'    <geom class="visual" type="box" rgba="0.2 0.4 0.9 1" pos="{x+sx} {y+sy} 0.0012" size="{ex} {ey} 0.001"/>')
            elif k == "P":
                g.append(f'    <geom class="visual" type="box" rgba="0.1 0.7 0.3 0.6" pos="{x} {y} 0.0012" size="0.48 0.48 0.001"/>')
            elif k == "Q":
                g.append(f'    <geom class="visual" type="box" rgba="0.9 0.8 0.2 0.35" pos="{x} {y} 0.0012" size="0.48 0.48 0.001"/>')
            elif k == "C":
                g.append(f'    <geom class="visual" type="box" rgba="0.2 0.8 0.3 0.5" pos="{x} {y} 0.0012" size="0.4 0.4 0.001"/>')
                side = -1 if x < W / 2 else 1
                g.append(f'    <geom class="static" type="box" rgba="0.25 0.25 0.28 1" pos="{x + side*0.62} {y} 0.3" size="0.06 0.2 0.3"/>')
                g.append(f'    <geom class="visual" type="box" rgba="0.1 1 0.3 1" pos="{x + side*0.55} {y} 0.5" size="0.01 0.08 0.04"/>')
            elif k == "A":
                g.append(f'    <geom class="visual" type="box" rgba="0.95 0.8 0.05 0.5" pos="{x} {y} 0.0012" size="0.45 0.45 0.001"/>')
            elif k in "IO":
                rgba = "0.2 0.5 0.95 0.6" if k == "I" else "0.95 0.45 0.15 0.6"
                side = -1 if k == "I" else 1
                g.append(f'    <geom class="visual" type="box" rgba="{rgba}" pos="{x} {y} 0.0012" size="0.48 0.48 0.001"/>')
                # dock door on the wall
                g.append(f'    <geom class="visual" type="box" rgba="{rgba[:-3]} 1" pos="{x + side*1.42} {y} 1.0" size="0.03 0.6 1.0"/>')
            elif k == "X":
                g.append(self._equipment((x, y)))
            elif k == "o":
                g.append(self._operator((x, y)))
        return "\n".join(g) + "\n"

    def _tape_angle(self, cell) -> float:
        nbrs = self.layout.agv_neighbors(cell)
        tape_n = [n for n in nbrs if self.layout.kind(n) == "T"]
        if tape_n and tape_n[0][0] == cell[0]:
            return 1.5708
        return 0.0

    def _equipment(self, cell) -> str:
        """Conveyor / sortation equipment filling the AGV loop interior."""
        x, y = cell
        out = []
        if y % 2 == 0:
            out.append(f'    <geom class="static" type="box" rgba="0.3 0.32 0.36 1" pos="{x} {y} 0.35" size="0.5 0.4 0.35"/>')
            out.append(f'    <geom class="visual" type="box" rgba="0.15 0.15 0.15 1" pos="{x} {y} 0.71" size="0.5 0.35 0.01"/>')
            if self.rng.random() < 0.5:
                c = self.rng.choice(["0.75 0.55 0.3", "0.65 0.45 0.25", "0.8 0.65 0.4"])
                out.append(f'    <geom class="visual" type="box" rgba="{c} 1" pos="{x + self.rng.uniform(-0.2,0.2)} {y} 0.86" size="0.2 0.18 0.14"/>')
        else:
            out.append(f'    <geom class="visual" type="box" rgba="0.35 0.55 0.35 1" pos="{x} {y} 0.002" size="0.5 0.3 0.002"/>')
        return "\n".join(out)

    def _operator(self, cell) -> str:
        x, y = cell
        return "\n".join([
            # workstation desk with a pick-to-light bar
            f'    <geom class="static" type="box" rgba="0.55 0.57 0.6 1" pos="{x+0.8} {y-0.1} 0.45" size="0.3 0.35 0.45"/>',
            f'    <geom class="visual" type="box" rgba="0.1 0.9 0.2 1" pos="{x} {y-0.42} 1.05" size="0.45 0.03 0.03"/>',
            # operator
            f'    <geom class="visual" type="capsule" rgba="0.2 0.3 0.6 1" fromto="{x} {y} 0.15 {x} {y} 1.25" size="0.18"/>',
            f'    <geom class="visual" type="sphere" rgba="0.95 0.8 0.65 1" pos="{x} {y} 1.55" size="0.13"/>',
            f'    <geom class="visual" type="sphere" rgba="1 0.6 0 1" pos="{x} {y} 1.62" size="0.135"/>',
        ])

    # ---------------------------------------------------------------- robots
    def _robot(self, r: RobotSpec) -> str:
        x, y = self.layout.world(r.home)
        n = r.name
        c = _rgba(r.color)
        if r.kind == "AMR":
            chassis = f"""
      <geom name="{n}_chassis" type="cylinder" size="{AMR_RADIUS} 0.11" pos="0 0 0.14" rgba="{c}" mass="60" contype="2" conaffinity="2"/>
      <geom class="visual" type="cylinder" size="{AMR_RADIUS + 0.01} 0.02" pos="0 0 0.06" rgba="0.1 0.1 0.1 1"/>
      <geom class="visual" type="cylinder" size="0.045 0.025" pos="0.2 0 0.25" rgba="0.05 0.05 0.05 1"/>
      <geom class="visual" type="box" size="0.02 0.12 0.03" pos="{AMR_RADIUS - 0.01} 0 0.16" rgba="0.1 0.1 0.12 1"/>
      <geom class="visual" type="cylinder" size="0.05 0.015" pos="0 0.2 0.05" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
      <geom class="visual" type="cylinder" size="0.05 0.015" pos="0 -0.2 0.05" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>"""
            platform = f'<geom name="{n}_platform" type="cylinder" size="0.24 0.01" pos="0 0 {PLATFORM_TOP - 0.01}" rgba="0.2 0.2 0.22 1" mass="5" contype="2" conaffinity="2"/>'
            led_pos = f"{-0.15} 0 0.255"
        else:
            hx, hy = AGV_HALF
            chassis = f"""
      <geom name="{n}_chassis" type="box" size="{hx} {hy} 0.11" pos="0 0 0.14" rgba="{c}" mass="80" contype="2" conaffinity="2"/>
      <geom class="visual" type="box" size="{hx + 0.01} {hy + 0.01} 0.02" pos="0 0 0.06" rgba="0.1 0.1 0.1 1"/>
      <geom class="visual" type="box" size="0.01 {hy - 0.03} 0.04" pos="{hx} 0 0.15" rgba="0.15 0.15 0.15 1"/>
      <geom class="visual" type="box" size="0.01 {hy - 0.03} 0.04" pos="{-hx} 0 0.15" rgba="0.15 0.15 0.15 1"/>
      <geom class="visual" type="box" size="0.04 0.04 0.002" pos="0.0 0 0.03" rgba="0.2 0.2 0.2 1"/>
      <geom class="visual" type="cylinder" size="0.055 0.015" pos="0 {hy} 0.055" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
      <geom class="visual" type="cylinder" size="0.055 0.015" pos="0 {-hy} 0.055" euler="1.5708 0 0" rgba="0.1 0.1 0.1 1"/>
      <geom class="visual" type="box" size="0.03 0.03 0.015" pos="{hx - 0.06} {hy - 0.06} 0.26" rgba="1 0.3 0 1"/>
      <geom class="visual" type="box" size="0.03 0.03 0.015" pos="{hx - 0.06} {-hy + 0.06} 0.26" rgba="1 0.3 0 1"/>"""
            platform = f'<geom name="{n}_platform" type="box" size="0.26 0.19 0.01" pos="0 0 {PLATFORM_TOP - 0.01}" rgba="0.2 0.2 0.22 1" mass="5" contype="2" conaffinity="2"/>'
            led_pos = f"{-hx + 0.08} 0 0.255"
        return f"""
    <body name="{n}" pos="{x} {y} 0">
      <joint name="{n}_x" type="slide" axis="1 0 0" damping="1"/>
      <joint name="{n}_y" type="slide" axis="0 1 0" damping="1"/>
      <joint name="{n}_yaw" type="hinge" axis="0 0 1" damping="0.5"/>{chassis}
      <geom name="{n}_led" class="visual" type="sphere" size="0.035" pos="{led_pos}" rgba="0.2 1 0.2 1"/>
      <site name="{n}_site" pos="0 0 0.3" size="0.01"/>
      <body name="{n}_lift" pos="0 0 0">
        <joint name="{n}_lift" type="slide" axis="0 0 1" range="0 {LIFT_TRAVEL + 0.01}" damping="3000" armature="5"/>
        <geom class="visual" type="cylinder" size="0.05 0.06" pos="0 0 0.2" rgba="0.5 0.5 0.5 1" mass="2"/>
        <body name="{n}_turntable" pos="0 0 0">
          <joint name="{n}_turn" type="hinge" axis="0 0 1" damping="40" armature="0.5"/>
          {platform}
        </body>
      </body>
    </body>"""

    # ----------------------------------------------------------------- loads
    def _load(self, ld: LoadSpec) -> str:
        x, y = self.layout.world(ld.cell)
        n = ld.name
        a = LOAD_HALF - LEG_HALF
        legs = "\n".join(
            f'      <geom type="box" size="{LEG_HALF} {LEG_HALF} {LEG_HEIGHT / 2}" pos="{sx * a} {sy * a} {LEG_HEIGHT / 2}"'
            f' rgba="0.25 0.25 0.3 1" mass="1" contype="1" conaffinity="3"/>'
            for sx in (-1, 1) for sy in (-1, 1))
        plate_z = LEG_HEIGHT + PLATE_HALF_T
        if ld.kind == "rack":
            vis = self._rack_visual()
            plate_rgba = "0.3 0.35 0.45 1"
        else:
            vis = self._pallet_visual()
            plate_rgba = "0.3 0.3 0.3 1"
        return f"""
    <body name="{n}" pos="{x} {y} 0.001">
      <freejoint name="{n}_free"/>
      <geom name="{n}_plate" type="box" size="{LOAD_HALF} {LOAD_HALF} {PLATE_HALF_T}" pos="0 0 {plate_z}" rgba="{plate_rgba}" mass="12" contype="1" conaffinity="3"/>
{legs}
{vis}
    </body>"""

    def _rack_visual(self) -> str:
        """Mobile shelving unit with totes / cartons (goods-to-person rack)."""
        out = []
        top = 1.75
        base = LEG_HEIGHT + 2 * PLATE_HALF_T
        a = LOAD_HALF - 0.02
        for sx in (-1, 1):
            for sy in (-1, 1):
                out.append(f'      <geom class="visual" type="box" size="0.02 0.02 {(top - base) / 2}" pos="{sx * a} {sy * a} {(top + base) / 2}" rgba="0.15 0.3 0.75 1"/>')
        for z in (0.82, 1.3):
            out.append(f'      <geom class="visual" type="box" size="{LOAD_HALF} {LOAD_HALF} 0.012" pos="0 0 {z}" rgba="0.75 0.4 0.1 1"/>')
        out.append(f'      <geom class="visual" type="box" size="{LOAD_HALF} 0.015 0.015" pos="0 {a} {top}" rgba="0.15 0.3 0.75 1"/>')
        out.append(f'      <geom class="visual" type="box" size="{LOAD_HALF} 0.015 0.015" pos="0 {-a} {top}" rgba="0.15 0.3 0.75 1"/>')
        palette = ["0.85 0.25 0.2", "0.2 0.6 0.85", "0.3 0.7 0.3", "0.95 0.75 0.2", "0.65 0.45 0.25", "0.55 0.35 0.75"]
        for z0, z1 in ((base, 0.82), (0.82, 1.3), (1.3, 1.74)):
            for sx in (-0.2, 0.2):
                for sy in (-0.2, 0.2):
                    if self.rng.random() < 0.8:
                        hz = (z1 - z0) / 2 * self.rng.uniform(0.5, 0.85)
                        out.append(f'      <geom class="visual" type="box" size="0.17 0.17 {hz:.3f}" pos="{sx} {sy} {z0 + 0.012 + hz:.3f}" rgba="{self.rng.choice(palette)} 1"/>')
        return "\n".join(out)

    def _pallet_visual(self) -> str:
        """Pallet stand: wooden pallet with stretch-wrapped cartons."""
        base = LEG_HEIGHT + 2 * PLATE_HALF_T
        out = [f'      <geom class="visual" type="box" size="{LOAD_HALF} {LOAD_HALF} 0.06" pos="0 0 {base + 0.06}" rgba="0.72 0.55 0.33 1"/>']
        z = base + 0.12
        layers = self.rng.randint(2, 4)
        tint = self.rng.choice(["0.78 0.6 0.38", "0.7 0.52 0.32", "0.85 0.7 0.48"])
        for layer in range(layers):
            for sx in (-0.2, 0.2):
                for sy in (-0.2, 0.2):
                    out.append(f'      <geom class="visual" type="box" size="0.19 0.19 0.11" pos="{sx} {sy} {z + 0.11 + layer * 0.22:.3f}" rgba="{tint} 1"/>')
        out.append(f'      <geom class="visual" type="box" size="0.405 0.405 {layers * 0.11:.3f}" pos="0 0 {z + layers * 0.11:.3f}" rgba="0.85 0.9 0.95 0.25"/>')
        return "\n".join(out)

    # ----------------------------------------------------- contacts / eq / act
    def _contact_excludes(self) -> str:
        lines = ["  <contact>"]
        for r in self.robots:
            lines.append(f'    <exclude body1="{r.name}" body2="{r.name}_turntable"/>')
        lines.append("  </contact>\n")
        return "\n".join(lines)

    def _equalities(self) -> str:
        if not self.loads:
            return ""
        lines = ["  <equality>"]
        for r in self.robots:
            # body2 is re-targeted at runtime to whatever load the robot lifts
            lines.append(f'    <weld name="{r.name}_grip" body1="{r.name}_turntable" body2="{self.loads[0].name}"'
                         f' active="false" solref="0.02 1"/>')
        lines.append("  </equality>\n")
        return "\n".join(lines)

    def _actuators(self) -> str:
        lines = ["  <actuator>"]
        for r in self.robots:
            n = r.name
            kv_lin = 4000 if r.kind == "AMR" else 5000
            lines += [
                f'    <velocity name="{n}_vx" joint="{n}_x" kv="{kv_lin}" forcerange="-3000 3000"/>',
                f'    <velocity name="{n}_vy" joint="{n}_y" kv="{kv_lin}" forcerange="-3000 3000"/>',
                f'    <velocity name="{n}_wz" joint="{n}_yaw" kv="400" forcerange="-600 600"/>',
                f'    <position name="{n}_lift" joint="{n}_lift" kp="60000" ctrlrange="0 {LIFT_TRAVEL}" forcerange="-5000 5000"/>',
                f'    <position name="{n}_turn" joint="{n}_turn" kp="800" forcerange="-400 400"/>',
            ]
        lines.append("  </actuator>\n")
        return "\n".join(lines)
