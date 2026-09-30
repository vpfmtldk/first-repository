"""HCR-3A pick-and-place demo in MuJoCo.

The arm is driven by joint position actuators. Joint targets come from a
damped-least-squares IK that tracks a Cartesian path for the gripper TCP
(approach -> descend -> grasp -> lift -> move -> lower -> release -> retreat).

Usage:
    python pick_place.py                 # interactive viewer (needs a display)
    python pick_place.py --video out.mp4 # headless render to a video file
"""

import argparse
import os

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "scene.xml")

ARM_JOINTS = [f"joint_{i}" for i in range(1, 7)]
GRIP_OPEN = (-0.02, 0.02)   # finger1, finger2 targets
GRIP_CLOSE = (0.03, -0.03)  # squeeze past contact; force-limited actuators hold the cube

# Gripper pointing straight down, fingers closing along world x.
# Columns: tool-frame x (= -approach dir), finger slide axis, their cross product.
R_PAD_WORLD = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=float)
_u = np.array([0, 1, -1]) / np.sqrt(2)
_w = np.array([0, 1, 1]) / np.sqrt(2)
R_PAD_IN_L6 = np.stack([[1, 0, 0], _u, _w], axis=1)
R_TARGET = R_PAD_WORLD @ R_PAD_IN_L6.T  # desired link_6 / tcp orientation

# IK seed that selects the elbow-up / wrist-down solution branch at the start pose.
START_SEED = np.array([0.93, -1.51, -0.85, -2.35, 1.57, 0.14])


class IK:
    def __init__(self, model):
        self.m = model
        self.d = mujoco.MjData(model)
        self.site = model.site("tcp").id
        self.dofs = np.array([model.joint(j).dofadr[0] for j in ARM_JOINTS])
        self.qadr = np.array([model.joint(j).qposadr[0] for j in ARM_JOINTS])
        self.lo = model.jnt_range[[model.joint(j).id for j in ARM_JOINTS], 0]
        self.hi = model.jnt_range[[model.joint(j).id for j in ARM_JOINTS], 1]

    def solve(self, q0, pos, rot=R_TARGET, iters=100, tol=1e-5):
        m, d = self.m, self.d
        q = q0.copy()
        jacp = np.zeros((3, m.nv))
        jacr = np.zeros((3, m.nv))
        for _ in range(iters):
            d.qpos[self.qadr] = q
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            err_p = pos - d.site_xpos[self.site]
            R = d.site_xmat[self.site].reshape(3, 3)
            q_err = np.zeros(4)
            mujoco.mju_mat2Quat(q_err, (rot @ R.T).flatten())
            err_r = np.zeros(3)
            mujoco.mju_quat2Vel(err_r, q_err, 1.0)
            err = np.concatenate([err_p, err_r])
            if np.linalg.norm(err) < tol:
                break
            mujoco.mj_jacSite(m, d, jacp, jacr, self.site)
            J = np.vstack([jacp, jacr])[:, self.dofs]
            lam = 1e-4
            dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), err)
            q = np.clip(q + np.clip(dq, -0.2, 0.2), self.lo, self.hi)
        return q


def smoothstep(s):
    return s * s * (3 - 2 * s)


def build_plan(cube_pos, place_pos):
    """List of (duration [s], tcp target position, gripper command)."""
    hover = 0.10
    grasp_z = cube_pos[2]
    place_z = place_pos[2] + 0.02 + 0.002  # cube half-height above the table
    above_pick = cube_pos + [0, 0, hover]
    above_place = np.array([place_pos[0], place_pos[1], place_z + hover])
    return [
        (1.0, above_pick, GRIP_OPEN),                          # settle at start
        (1.5, cube_pos * [1, 1, 0] + [0, 0, grasp_z], GRIP_OPEN),  # descend
        (0.8, None, GRIP_CLOSE),                               # close gripper
        (1.5, above_pick, GRIP_CLOSE),                         # lift
        (2.0, above_place, GRIP_CLOSE),                        # transfer
        (1.5, above_place - [0, 0, hover], GRIP_CLOSE),        # lower
        (0.8, None, GRIP_OPEN),                                # release
        (1.5, above_place, GRIP_OPEN),                         # retreat
        (1.0, None, GRIP_OPEN),                                # hold
    ]


class PickPlace:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(SCENE)
        self.d = mujoco.MjData(self.m)
        self.ik = IK(self.m)
        m, d = self.m, self.d

        mujoco.mj_forward(m, d)
        cube_pos = d.xpos[m.body("cube").id].copy()
        cube_pos[2] = 0.1 + 0.02  # resting on the pick table
        place_pos = d.site_xpos[m.site("place_target").id].copy()
        self.plan = build_plan(cube_pos, place_pos)

        # Start already at the first waypoint (elbow-up seed).
        q = self.ik.solve(START_SEED, self.plan[0][1], iters=300)
        d.qpos[self.ik.qadr] = q
        d.ctrl[:6] = q
        d.ctrl[6:8] = GRIP_OPEN
        mujoco.mj_forward(m, d)
        self.q_cmd = q
        self.pos_cmd = self.plan[0][1].copy()

        # Precompute per-step targets for the whole plan.
        self.schedule = []
        t = 0.0
        pos_prev = self.pos_cmd
        for dur, pos, grip in self.plan:
            target = pos_prev if pos is None else np.asarray(pos, float)
            self.schedule.append((t, t + dur, pos_prev, target, grip))
            pos_prev = target
            t += dur
        self.total_time = t

    def targets(self, t):
        for t0, t1, p0, p1, grip in self.schedule:
            if t < t1:
                s = smoothstep((t - t0) / (t1 - t0))
                return p0 + s * (p1 - p0), grip
        _, _, _, p1, grip = self.schedule[-1]
        return p1, grip

    def step(self):
        m, d = self.m, self.d
        pos, grip = self.targets(d.time)
        self.q_cmd = self.ik.solve(self.q_cmd, pos, iters=20)
        d.ctrl[:6] = self.q_cmd
        d.ctrl[6:8] = grip
        mujoco.mj_step(m, d)

    def cube_pos(self):
        return self.d.xpos[self.m.body("cube").id]


def run_video(path, fps=30, width=1280, height=720):
    import imageio

    sim = PickPlace()
    m, d = sim.m, sim.d
    renderer = mujoco.Renderer(m, height, width)
    frames = []
    stills = {}
    steps_per_frame = int(round(1.0 / (fps * m.opt.timestep)))
    n_frames = int(sim.total_time * fps)
    for i in range(n_frames):
        for _ in range(steps_per_frame):
            sim.step()
        renderer.update_scene(d, camera="front")
        frame = renderer.render()
        frames.append(frame)
        for key, ts in (("1_grasp", 3.2), ("2_lift", 4.8), ("3_transfer", 6.3), ("4_place", 8.8)):
            if key not in stills and d.time >= ts:
                stills[key] = frame.copy()
    imageio.mimsave(path, frames, fps=fps, macro_block_size=16)
    base = os.path.splitext(path)[0]
    for key, img in stills.items():
        imageio.imwrite(f"{base}_{key}.png", img)

    target = d.site_xpos[m.site("place_target").id]
    final = sim.cube_pos()
    print(f"cube final position: {np.round(final, 4)}")
    print(f"place target:        {np.round(target, 4)}")
    print(f"xy error: {np.linalg.norm(final[:2] - target[:2]) * 1000:.1f} mm")


def run_viewer():
    import time
    import mujoco.viewer

    sim = PickPlace()
    with mujoco.viewer.launch_passive(sim.m, sim.d) as v:
        while v.is_running():
            start = time.time()
            if sim.d.time < sim.total_time:
                sim.step()
            v.sync()
            dt = sim.m.opt.timestep - (time.time() - start)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", help="render headless to this mp4 path instead of opening the viewer")
    args = ap.parse_args()
    if args.video:
        run_video(args.video)
    else:
        run_viewer()
