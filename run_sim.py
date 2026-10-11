#!/usr/bin/env python3
"""AGV / AMR warehouse logistics simulation in MuJoCo.

Examples
--------
Interactive 3D viewer (needs a display)::

    python run_sim.py --mode viewer

Headless run, print KPIs and save them::

    python run_sim.py --mode headless --duration 600 --out results

Record a video (offscreen rendering, e.g. MUJOCO_GL=egl or osmesa on servers)::

    python run_sim.py --mode video --duration 120 --video results/warehouse.mp4
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

from logistics_sim.fleet import FleetManager, SimConfig
from logistics_sim.viz import add_path_overlay, hud_lines


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", choices=["viewer", "headless", "video"], default="viewer")
    p.add_argument("--duration", type=float, default=600.0, help="simulated seconds")
    p.add_argument("--amr", type=int, default=8, help="number of AMRs (max 12)")
    p.add_argument("--agv", type=int, default=4, help="number of AGVs (max 6)")
    p.add_argument("--order-interval", type=float, default=10.0, help="mean seconds between orders")
    p.add_argument("--pick-time", type=float, default=8.0)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--speed", type=float, default=1.0, help="viewer: real-time factor")
    p.add_argument("--video", default="results/warehouse.mp4", help="video mode: output file (.mp4/.gif)")
    p.add_argument("--fps", type=int, default=20)
    p.add_argument("--size", default="1280x720", help="video mode: WIDTHxHEIGHT")
    p.add_argument("--timelapse", type=float, default=2.0, help="video mode: sim seconds per video second")
    p.add_argument("--camera", default="overview", help="overview | top | pick_area")
    p.add_argument("--out", default="results", help="directory for KPI json / plots")
    p.add_argument("--plot", action="store_true", help="save KPI plots (matplotlib)")
    p.add_argument("--export-xml", metavar="FILE", help="also write the generated MJCF scene")
    p.add_argument("--quiet", action="store_true")
    return p.parse_args()


class Timeline:
    def __init__(self, every: float = 5.0):
        self.every = every
        self.next_t = 0.0
        self.rows: list[dict] = []

    def __call__(self, fleet: FleetManager) -> None:
        if fleet.time >= self.next_t:
            self.next_t += self.every
            k = fleet.kpis()
            self.rows.append({
                "t": k["sim_time_s"], "picks": k["picks_completed"], "pallets": k["pallets_transferred"],
                "pending": k["pending_tasks"], "active": k["active_tasks"],
                "battery": {n: r["battery"] for n, r in k["robots"].items()},
            })


def run_headless(fleet: FleetManager, args, timeline: Timeline) -> None:
    t0 = time.time()
    last_print = 0.0

    def cb(f: FleetManager) -> None:
        nonlocal last_print
        timeline(f)
        if not args.quiet and f.time - last_print >= 60.0:
            last_print = f.time
            print("\n".join(hud_lines(f)), "\n", flush=True)

    fleet.run(args.duration, callback=cb)
    wall = time.time() - t0
    print(f"simulated {fleet.time:.0f}s in {wall:.1f}s wall time ({fleet.time / wall:.1f}x real time)")


def run_video(fleet: FleetManager, args, timeline: Timeline) -> None:
    from logistics_sim.viz import VideoRecorder

    os.makedirs(os.path.dirname(os.path.abspath(args.video)), exist_ok=True)
    w, h = (int(v) for v in args.size.split("x"))
    rec = VideoRecorder(fleet, args.video, width=w, height=h, fps=args.fps, camera=args.camera)
    frame_dt = args.timelapse / args.fps
    next_frame = 0.0
    end = fleet.time + args.duration
    while fleet.time < end:
        fleet.control_step()
        timeline(fleet)
        if fleet.time >= next_frame:
            next_frame += frame_dt
            rec.capture()
    rec.close()
    print(f"wrote {rec.frames} frames to {args.video}")


def run_viewer(fleet: FleetManager, args, timeline: Timeline) -> None:
    import mujoco.viewer

    with mujoco.viewer.launch_passive(fleet.model, fleet.data) as viewer:
        cam = fleet.model.camera(args.camera).id if args.camera else -1
        if cam >= 0:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = cam
        start_wall = time.time()
        start_sim = fleet.time
        last_print = 0.0
        while viewer.is_running() and fleet.time - start_sim < args.duration:
            fleet.control_step()
            timeline(fleet)
            with viewer.lock():
                viewer.user_scn.ngeom = 0
                add_path_overlay(fleet, viewer.user_scn)
            viewer.sync()
            if fleet.time - last_print >= 10.0:
                last_print = fleet.time
                if not args.quiet:
                    print("\n".join(hud_lines(fleet)), "\n", flush=True)
            # real-time pacing
            ahead = (fleet.time - start_sim) / args.speed - (time.time() - start_wall)
            if ahead > 0:
                time.sleep(ahead)


def save_results(fleet: FleetManager, args, timeline: Timeline) -> None:
    os.makedirs(args.out, exist_ok=True)
    k = fleet.kpis()
    with open(os.path.join(args.out, "kpis.json"), "w") as fh:
        json.dump(k, fh, indent=2)
    with open(os.path.join(args.out, "events.log"), "w") as fh:
        fh.write("\n".join(fleet.events))
    with open(os.path.join(args.out, "timeline.json"), "w") as fh:
        json.dump(timeline.rows, fh)
    print("\n=== KPIs ===")
    for key, val in k.items():
        if key != "robots":
            print(f"{key:28s} {val}")
    print("\nrobot   kind tasks distance  util   wait  replans evasions battery")
    for n, r in k["robots"].items():
        print(f"{n:7s} {r['kind']:4s} {r['tasks']:5d} {r['distance_m']:7.1f}m {r['utilization']:5.2f} "
              f"{r['wait_s']:6.1f}s {r['replans']:5d} {r['evasions']:8d} {r['battery']:6.1f}%")
    if args.plot:
        from logistics_sim.report import plot_kpis
        path = plot_kpis(k, timeline.rows, args.out)
        print(f"plots saved to {path}")


def main() -> int:
    args = parse_args()
    cfg = SimConfig(num_amr=args.amr, num_agv=args.agv, order_interval=args.order_interval,
                    pick_time=args.pick_time, seed=args.seed)
    fleet = FleetManager(cfg)
    if args.export_xml:
        with open(args.export_xml, "w") as fh:
            fh.write(fleet.xml)
    print(f"warehouse {fleet.layout.width}x{fleet.layout.height} m, {cfg.num_amr} AMRs, {cfg.num_agv} AGVs, "
          f"{sum(1 for l in fleet.loads.values() if l.kind == 'rack')} racks, "
          f"{sum(1 for l in fleet.loads.values() if l.kind == 'pallet')} pallets, {len(fleet.stations)} pick stations")
    timeline = Timeline()
    try:
        {"viewer": run_viewer, "headless": run_headless, "video": run_video}[args.mode](fleet, args, timeline)
    except KeyboardInterrupt:
        pass
    save_results(fleet, args, timeline)
    return 0


if __name__ == "__main__":
    sys.exit(main())
