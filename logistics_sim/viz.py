"""Visualisation helpers: planned-path overlay, HUD text and video recording."""

from __future__ import annotations

from typing import TYPE_CHECKING

import mujoco
import numpy as np

if TYPE_CHECKING:  # pragma: no cover
    from .fleet import FleetManager


def add_path_overlay(fleet: "FleetManager", scene: mujoco.MjvScene) -> None:
    """Draw every robot's remaining route and reserved cells into ``scene``."""
    for r in fleet.robots.values():
        rgba = np.array([*r.spec.color, 0.85], dtype=np.float32)
        if r.path is not None and len(r.path) > 1:
            pts = [r.xy()] + [np.array(fleet.layout.world(c)) for c in r.path[r.idx + 1:]]
            for a, b in zip(pts[:-1], pts[1:]):
                if scene.ngeom >= scene.maxgeom:
                    return
                g = scene.geoms[scene.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                                    np.eye(3).flatten(), rgba)
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.025,
                                     np.array([a[0], a[1], 0.02]), np.array([b[0], b[1], 0.02]))
                scene.ngeom += 1
            goal = fleet.layout.world(r.path[-1])
            if scene.ngeom < scene.maxgeom:
                mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_CYLINDER,
                                    np.array([0.12, 0.12, 0.01]), np.array([goal[0], goal[1], 0.02]),
                                    np.eye(3).flatten(), rgba)
                scene.ngeom += 1
        # reserved cells (traffic manager) as translucent squares
        cell_rgba = np.array([*r.spec.color, 0.25], dtype=np.float32)
        for c in fleet.traffic.cells_of(r.name):
            if scene.ngeom >= scene.maxgeom:
                return
            x, y = fleet.layout.world(c)
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_BOX,
                                np.array([0.46, 0.46, 0.003]), np.array([x, y, 0.004]),
                                np.eye(3).flatten(), cell_rgba)
            scene.ngeom += 1


def hud_lines(fleet: "FleetManager") -> list[str]:
    k = fleet.kpis()
    lines = [
        f"t = {k['sim_time_s']:6.1f} s   orders {k['orders_created']}   picks {k['picks_completed']}"
        f"   pallets {k['pallets_transferred']}   collisions {k['collisions']}",
        f"throughput {k['picks_per_hour']:.0f} picks/h, {k['pallets_per_hour']:.0f} pallets/h   "
        f"pending {k['pending_tasks']}  active {k['active_tasks']}",
    ]
    row = []
    for r in fleet.robots.values():
        row.append(f"{r.name}:{r.state[:4]} {r.battery:3.0f}%")
        if len(row) == 4:
            lines.append("   ".join(row))
            row = []
    if row:
        lines.append("   ".join(row))
    return lines


class VideoRecorder:
    """Offscreen renderer that writes an MP4/GIF with a HUD overlay."""

    def __init__(self, fleet: "FleetManager", path: str, width: int = 1280, height: int = 720,
                 fps: int = 20, camera: str = "overview"):
        import imageio.v2 as imageio

        self.fleet = fleet
        self.renderer = mujoco.Renderer(fleet.model, height, width)
        self.camera = camera
        self.path = path
        self.writer = imageio.get_writer(path, fps=fps) if path.endswith(".gif") else \
            imageio.get_writer(path, fps=fps, codec="libx264", quality=6, macro_block_size=8)
        self.frames = 0

    def capture(self) -> np.ndarray:
        r = self.renderer
        r.update_scene(self.fleet.data, camera=self.camera)
        add_path_overlay(self.fleet, r.scene)
        img = r.render()
        img = self._draw_hud(img)
        self.writer.append_data(img)
        self.frames += 1
        return img

    def _draw_hud(self, img: np.ndarray) -> np.ndarray:
        from PIL import Image, ImageDraw

        im = Image.fromarray(img)
        dr = ImageDraw.Draw(im, "RGBA")
        lines = hud_lines(self.fleet)
        h = 16 * len(lines) + 10
        dr.rectangle([0, 0, im.width, h], fill=(0, 0, 0, 150))
        for i, line in enumerate(lines):
            dr.text((8, 5 + 16 * i), line, fill=(255, 255, 255, 255))
        return np.asarray(im)

    def close(self) -> None:
        self.writer.close()
        self.renderer.close()


def save_snapshot(fleet: "FleetManager", path: str, camera: str = "overview",
                  width: int = 1280, height: int = 720) -> None:
    from PIL import Image

    r = mujoco.Renderer(fleet.model, height, width)
    r.update_scene(fleet.data, camera=camera)
    add_path_overlay(fleet, r.scene)
    Image.fromarray(r.render()).save(path)
    r.close()
