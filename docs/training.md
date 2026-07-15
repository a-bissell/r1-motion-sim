# Training / retraining R1 motion policies

**This repo does not train policies.** It replays and evaluates them. Be clear
about that split before you sink time in the wrong place:

- **Training** whole-body motion-tracking policies for the R1 is a large-scale
  RL job — thousands of GPU-parallel environments in **IsaacLab**, using the
  **[BeyondMimic / whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking)**
  framework. That is where the policies the R1 ships with were made.
- **This repo** is the deployment/eval side: a fast, single-env, CPU MuJoCo
  sim-to-sim check that a policy actually balances and tracks before you risk
  hardware, plus the exact R1 obs/action spec so a trained policy is
  deployment-compatible.

## What this repo contributes to a training effort

1. **A validated R1 registration.** `docs/pipeline_spec.md` + the config files
   give the precise obs layout (129-dim tracking obs), action definition
   (`a*scale+offset` position targets), joint ordering (BFS↔DFS `joint_ids_map`,
   the 24-of-29 controlled set), anchor body, and control rate. Match these in
   your IsaacLab env and your policy will drop straight into `r1_motion_sim.py`.
2. **Reference motions.** The `*.bvh_*.csv` clips on the robot (31-col:
   root pose + 24 joint angles at 30/50/60 Hz) are exactly the tracking targets.
   Retarget your own mocap to the R1 joint set in the same format and they
   become new training references.
3. **A sim-to-sim gate.** After training, export to ONNX and run it here. If it
   balances in MuJoCo with these obs conventions, it has a real shot on
   hardware; if it falls, you have a fast loop to debug obs/action mismatches
   without a robot.

## Recommended loop

```
retarget motion ─► train in BeyondMimic (IsaacLab) ─► export ONNX
      ▲                                                    │
      └──────────── fix obs/reward ◄── r1_motion_sim.py eval (this repo)
```

## Gotchas to carry into training

- The tracking obs has **no `projected_gravity`** — the `motion_anchor_ori_b`
  6D term (row-interleaved) is the orientation channel. Keep the same 6D
  convention in training and deployment or the policy won't transfer.
- Observations are **unnormalized** on the R1 deployment (all scales 1.0). If
  you train with an obs normalizer, either bake it into the exported graph or
  add a matching normalization builder here.
- PD is **motor-side** on hardware (the controller emits joint position targets
  via `unitree_hg::LowCmd`). Train with a PD actuator model using the config
  `stiffness`/`damping` and the real torque limits (from the MuJoCo model's
  actuator `ctrlrange`).
