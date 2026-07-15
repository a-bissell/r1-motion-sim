#!/usr/bin/env python3
"""r1-motion-sim — config-driven closed-loop MuJoCo replay for R1 policies.

Replays any R1 trajectory-tracking policy (dances, stand-up/sit-down, ...) in
full physics against Unitree's official MuJoCo model. The robot-/policy-specific
numbers live in a YAML config (see configs/), so the same tool covers every
motion and robot variant — no code edits to swap dances.

    python3 r1_motion_sim.py --config configs/dance1_subject2.yaml \
        --model r1_model/scene.xml --policy policy.onnx --traj motion.csv [--gif out.gif]

The obs layout is declared in the config (`policy.obs_terms`); each term maps to
a builder in BUILDERS below. To support a new obs family, add builders and a
config that lists them — the physics/PD loop is unchanged.

Requires: onnxruntime, numpy, mujoco, pyyaml  (+ pillow for --gif)
"""

import argparse
import numpy as np
import onnxruntime as ort
import mujoco
import yaml


# ── math helpers ───────────────────────────────────────────────────────────
def quat_to_mat_xyzw(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_from_wxyz(q):
    w, x, y, z = q
    return quat_to_mat_xyzw(np.array([x, y, z, w]))


def wxyz(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array([w, x, y, z])


def rot6d(R):
    """First two columns, ROW-interleaved [m00,m01,m10,m11,m20,m21] — the
    IsaacLab/BeyondMimic convention. Column-major flattening scrambles the only
    orientation feedback in the tracking obs and the policy falls in ~0.5 s."""
    return R[:, :2].reshape(-1)


# ── observation-term builders (the "motion_tracking" family) ───────────────
# Each takes the shared context `c` (a dict) and returns its slice of the obs.
def _b_motion_command_pos(c): return c["ref_pos"]
def _b_motion_command_vel(c): return c["ref_vel"]
def _b_base_ang_vel(c):       return c["d"].qvel[c["bqv"] + 3:c["bqv"] + 6]
def _b_joint_pos_rel(c):      return c["q_bfs"] - c["default_joint_pos"]
def _b_joint_vel_rel(c):      return c["qd_bfs"]
def _b_last_action(c):        return c["last_action"]
def _b_motion_anchor_ori_b(c):
    return rot6d(mat_from_wxyz(c["d"].xquat[c["torso"]]).T @ c["ref_anchor_R"])

BUILDERS = {
    "motion_command_pos": _b_motion_command_pos,
    "motion_command_vel": _b_motion_command_vel,
    "motion_anchor_ori_b": _b_motion_anchor_ori_b,
    "base_ang_vel": _b_base_ang_vel,
    "joint_pos_rel": _b_joint_pos_rel,
    "joint_vel_rel": _b_joint_vel_rel,
    "last_action": _b_last_action,
}


def sample_traj(traj, t, fps):
    f = t * fps
    i0 = min(int(np.floor(f)), len(traj) - 1)
    i1 = min(i0 + 1, len(traj) - 1)
    a = f - i0
    row = (1 - a) * traj[i0] + a * traj[i1]
    row[3:7] /= np.linalg.norm(row[3:7])
    return row


def run(cfg, model_path, policy_path, traj_path, gif_path=None):
    ctl, mdl, pol, g = cfg["control"], cfg["model"], cfg["policy"], cfg["gains"]
    fps, step_dt, sim_dt = ctl["fps"], ctl["step_dt"], ctl["sim_dt"]
    t0, t1 = ctl["time_start"], ctl["time_end"]
    names = mdl["joint_names"]
    jmap = np.array(mdl["joint_ids_map"]); jinv = np.argsort(jmap)
    default_pos = np.array(g["default_joint_pos"])
    scale, offset = np.array(g["action_scale"]), np.array(g["action_offset"])
    kp, kd = np.array(g["stiffness"]), np.array(g["damping"])
    n_act = pol["num_act"]
    terms = pol["obs_terms"]
    assert sum(x["dim"] for x in terms) == pol["num_obs"], "obs_terms dims != num_obs"
    for x in terms:
        if x["name"] not in BUILDERS:
            raise SystemExit(f"unknown obs term '{x['name']}' — add it to BUILDERS")

    m = mujoco.MjModel.from_xml_path(model_path); m.opt.timestep = sim_dt
    n_sub = int(round(step_dt / sim_dt))
    d = mujoco.MjData(m)
    sess = ort.InferenceSession(policy_path, providers=["CPUExecutionProvider"])
    traj = np.loadtxt(traj_path, delimiter=",")
    assert traj.shape[1] == 31, f"expected 31 CSV columns, got {traj.shape[1]}"

    bqp = m.joint("floating_base_joint").qposadr[0]
    bqv = m.joint("floating_base_joint").dofadr[0]
    qadr = np.array([m.joint(n).qposadr[0] for n in names])
    vadr = np.array([m.joint(n).dofadr[0] for n in names])
    lim = np.array([abs(m.actuator(n.replace("_joint", "")).ctrlrange[1]) for n in names])
    extra = [m.joint(i).name for i in range(m.njnt)
             if m.joint(i).type == mujoco.mjtJoint.mjJNT_HINGE and m.joint(i).name not in names]
    exq = np.array([m.joint(n).qposadr[0] for n in extra])
    exv = np.array([m.joint(n).dofadr[0] for n in extra])
    lf, rf = m.body("left_ankle_roll_link").id, m.body("right_ankle_roll_link").id
    torso = m.body(mdl["anchor_body"]).id
    n_steps = int((t1 - t0) / step_dt)
    tref = mujoco.MjData(m)

    def ref_anchor_R(row):
        tref.qpos[:] = 0
        tref.qpos[bqp:bqp + 3] = row[0:3]; tref.qpos[bqp + 3:bqp + 7] = wxyz(row[3:7])
        tref.qpos[qadr] = row[7:31]
        mujoco.mj_forward(m, tref)
        return mat_from_wxyz(tref.xquat[torso].copy())

    r0 = sample_traj(traj, t0, fps)
    d.qpos[bqp:bqp + 3] = r0[0:3]; d.qpos[bqp + 3:bqp + 7] = wxyz(r0[3:7]); d.qpos[qadr] = r0[7:31]
    mujoco.mj_forward(m, d)
    zmin = min(d.geom_xpos[gi][2] for gi in range(m.ngeom) if m.geom_bodyid[gi] in (lf, rf))
    d.qpos[bqp + 2] -= (zmin - 0.01); mujoco.mj_forward(m, d)

    renderer = mujoco.Renderer(m, 300, 300) if gif_path else None
    cam = mujoco.MjvCamera(); cam.distance, cam.elevation, cam.azimuth = 2.8, -10, 135
    frames = []
    last_action = np.zeros(n_act, dtype=np.float32)
    sim_j = np.zeros((n_steps, n_act)); ref_j = np.zeros((n_steps, n_act)); base_z = np.zeros(n_steps)
    fell_at = None

    for k in range(n_steps):
        t = t0 + k * step_dt
        rn, rx = sample_traj(traj, t, fps), sample_traj(traj, t + 1.0 / fps, fps)
        ref_pos = rn[7:31][jmap]
        c = dict(d=d, bqv=bqv, torso=torso, ref_pos=ref_pos,
                 ref_vel=(rx[7:31][jmap] - ref_pos) * fps,
                 q_bfs=d.qpos[qadr][jmap], qd_bfs=d.qvel[vadr][jmap],
                 default_joint_pos=default_pos, last_action=last_action,
                 ref_anchor_R=ref_anchor_R(rn))
        obs = np.concatenate([np.asarray(BUILDERS[x["name"]](c), dtype=np.float32).ravel()
                              for x in terms]).astype(np.float32)
        action = sess.run(None, {sess.get_inputs()[0].name: obs[None, :]})[0][0]
        target_dfs = (action * scale + offset)[jinv]
        for _ in range(n_sub):
            q, qd = d.qpos[qadr], d.qvel[vadr]
            d.qfrc_applied[:] = 0
            d.qfrc_applied[vadr] = np.clip(kp * (target_dfs - q) - kd * qd, -lim, lim)
            d.qfrc_applied[exv] = 20.0 * (0 - d.qpos[exq]) - 1.0 * d.qvel[exv]
            mujoco.mj_step(m, d)
        last_action = action.astype(np.float32)
        sim_j[k], ref_j[k], base_z[k] = d.qpos[qadr][jmap], ref_pos, d.qpos[bqp + 2]
        if renderer is not None and k % 2 == 0:
            cam.lookat[:] = [d.qpos[bqp], d.qpos[bqp + 1], 0.55]
            renderer.update_scene(d, camera=cam); frames.append(renderer.render().copy())
        if fell_at is None and base_z[k] < 0.40:
            fell_at = t - t0; sim_j, ref_j = sim_j[:k + 1], ref_j[:k + 1]; break

    corr = np.array([np.corrcoef(sim_j[:, j], ref_j[:, j])[0, 1] for j in range(n_act)])
    print(f"[{cfg['name']}] steps {len(sim_j)}/{n_steps}  "
          f"{'balanced whole window' if fell_at is None else f'FELL at {fell_at:.2f}s'}")
    print(f"  joint tracking: corr mean {np.nanmean(corr):.3f} / median {np.nanmedian(corr):.3f} "
          f"| mean err {np.abs(sim_j - ref_j).mean():.4f} rad")
    if gif_path and frames:
        from PIL import Image
        imgs = [Image.fromarray(f).quantize(colors=64, dither=Image.NONE) for f in frames]
        imgs[0].save(gif_path, save_all=True, append_images=imgs[1:],
                     duration=int(1000 * step_dt * 2), loop=0, optimize=True, disposal=2)
        print(f"  wrote {gif_path} ({len(imgs)} frames)")
    return fell_at is None


def main():
    ap = argparse.ArgumentParser(description="config-driven R1 motion-policy sim")
    ap.add_argument("--config", required=True, help="policy config YAML (see configs/)")
    ap.add_argument("--model", required=True, help="unitree_mujoco R1 scene.xml")
    ap.add_argument("--policy", required=True, help="policy .onnx")
    ap.add_argument("--traj", required=True, help="reference motion .csv (31 col)")
    ap.add_argument("--gif", help="optional output .gif")
    a = ap.parse_args()
    with open(a.config) as f:
        cfg = yaml.safe_load(f)
    run(cfg, a.model, a.policy, a.traj, a.gif)


if __name__ == "__main__":
    main()
