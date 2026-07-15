# r1-motion-sim

Config-driven closed-loop MuJoCo replay for **Unitree R1** motion-tracking
policies. Point it at a policy `.onnx`, a reference motion `.csv`, and a small
YAML config, and it rolls the policy out in full physics — balancing and
tracking the motion — against Unitree's official R1 model.

Built by reverse-engineering the on-robot `ai_sport` controller. **No
proprietary files are distributed here** — you supply your own policy weights
and motion clips from your own robot (see [docs/extract_from_robot.md](docs/extract_from_robot.md)).

![dance](docs/dance.gif)

*Reference result — the stock `dance1_subject2` policy balancing and tracking
the full 12.9 s motion in physics (joint-tracking correlation ~0.95 median).*

## Quickstart

```bash
pip install -r requirements.txt
./tools/fetch_mujoco_model.sh ./r1_model            # public model (BSD)

# supply a policy + motion from YOUR robot (see docs/extract_from_robot.md)
python3 r1_motion_sim.py \
    --config configs/dance1_subject2.yaml \
    --model  r1_model/scene.xml \
    --policy policy.onnx \
    --traj   motion.csv \
    --gif    out.gif
# -> "balanced whole window", corr median ~0.95
```

## How it generalizes

Everything policy-/robot-specific lives in the [config](configs/dance1_subject2.yaml)
— joint order, gains, action scale/offset, control rate, time window, and the
observation layout. Those values are exactly what the `ai_sport` controller
writes into its resolved `*_generated.yaml` at startup, so replaying a different
dance (or the stand-up / sit-down motions, or an Air/Pro variant) is just a new
config file, no code changes.

The observation is assembled from named term-builders (`policy.obs_terms` in the
config → `BUILDERS` in `r1_motion_sim.py`). The bundled builders cover the
**motion-tracking family** (dances + stand-up/sit-down: `motion_command`,
`motion_anchor_ori_b`, `base_ang_vel`, `joint_pos_rel`, `joint_vel_rel`,
`last_action`). Adding another obs family (e.g. velocity-command locomotion) is
a few new builder functions plus a config that lists them — the physics/PD loop
is untouched.

## The one gotcha that will bite you

The tracking obs has **no gravity vector**, so `motion_anchor_ori_b` is the
policy's *only* orientation feedback. Its 6D rotation must be flattened
**row-interleaved** (`mat[:, :2].reshape(-1)`, the IsaacLab convention).
Column-major flattening is a silent permutation that makes the policy fall in
~0.5 s no matter what else is right. Full spec: [docs/pipeline_spec.md](docs/pipeline_spec.md).

## Training?

This repo **replays and evaluates** policies; it does not train them.
Whole-body-tracking policies are trained in IsaacLab with
[BeyondMimic](https://github.com/HybridRobotics/whole_body_tracking). What this
repo gives the training loop is the validated R1 obs/action spec, a config
extractor, and a sim-to-sim eval to vet a policy before hardware. See
[docs/training.md](docs/training.md).

## Repo layout

```
r1_motion_sim.py          config-driven replay/eval (the tool)
configs/                  per-policy YAML (dance1_subject2 = default example)
docs/
  pipeline_spec.md        exact obs/action spec + joint maps + gotchas
  extract_from_robot.md   pull policy/motion/config off your own R1
  training.md             how this plugs into BeyondMimic
tools/
  fetch_mujoco_model.sh   grab the public unitree_mujoco R1 model
  dbwalk.py               read an ext4 rootfs image on macOS (no mount)
```

## License

MIT (this code). The R1 MuJoCo model is fetched separately from
[unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) under its
own BSD-3-Clause license. Policy weights and motion clips are yours, from your
own robot — do not commit them here, and note that a rootfs also holds secrets
(SSH/DDS keys, `/etc/shadow`, cloud tokens): extract only the sim files.
