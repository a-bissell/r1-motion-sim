# R1 dance policy — obs/action spec (for sim & training)

Reverse-engineered from the on-robot `ai_sport` controller (`r1_controller`,
`DanceCore`) + the decrypted `dances.yaml`, and cross-checked against the
open-source framework the policies are trained with:
**BeyondMimic / whole_body_tracking** (github.com/HybridRobotics/whole_body_tracking).
If you want to *train* or fine-tune, that repo is your starting point — this
doc is the R1-specific instantiation of its MDP.

## Observation vector (129 dims, single frame, NO normalization/clipping)

| slice | term | definition |
|---|---|---|
| `[0:24]`   | motion_command (pos) | reference joint positions at motion time t |
| `[24:48]`  | motion_command (vel) | reference joint velocities at t |
| `[48:54]`  | motion_anchor_ori_b  | see below — the **only** orientation feedback |
| `[54:57]`  | base_ang_vel         | robot base angular velocity, **body frame** |
| `[57:81]`  | joint_pos_rel        | robot joint pos − `default_joint_pos` |
| `[81:105]` | joint_vel_rel        | robot joint velocities |
| `[105:129]`| last_action          | previous raw policy output |

`motion_command = torch.cat([joint_pos, joint_vel])` (BeyondMimic
`MotionCommand.command`). All obs scales are 1.0, all clips null — feed raw.

## motion_anchor_ori_b — the gotcha that cost us days

```
ori = R_robot_anchor^T @ R_ref_anchor          # ref anchor in robot's anchor frame
6d  = matrix(ori)[:, :2].reshape(-1)           # ROW-interleaved: [m00,m01,m10,m11,m20,m21]
```

- Anchor body = `torso_link` (the controller logs `ref_torso_quat_world` /
  `current_torso_quat`). Pelvis also works in practice (torso≈pelvis at neutral
  waist).
- The dance obs has **no projected_gravity**, so this 6D is the policy's sole
  orientation signal. Flatten it **row-interleaved** (IsaacLab
  `mat[..., :2].reshape(-1)`). Column-major flattening is a silent permutation
  that makes the policy fall in ~0.5 s regardless of anything else.

## Action

`target_joint_pos = action * action_scale + action_offset` (BFS order), applied
as joint **position targets**. On hardware the controller emits a
`unitree_hg::LowCmd` and the **motors run the PD** (per-joint `stiffness`/
`damping`). In sim, run PD at the physics rate with those gains, clipped to the
model's actuator `ctrlrange` (real torque limits: hip ±88, knee ±139, ankle
±50, arm ±25 N·m).

## Joint ordering (the other easy footgun)

- Trajectory CSV joints and the MuJoCo model joints are in **DFS** (URDF
  document) order. The policy I/O is in **BFS** order (IsaacLab default).
- Convert: `policy = dfs[JOINT_IDS_MAP]`, back: `dfs = policy[JOINT_IDS_INV]`.
- The model has 29 hinge joints; the dance policy controls 24. waist_pitch and
  both wrist pitch/yaw are locked (`ctrlrange [0,0]`) — hold them at 0.
- `client_dof_idx` maps policy joints → physical motor-bus indices (only
  needed on hardware, not in sim).

## Control / timing

- Policy at **50 Hz** (`step_dt = 0.02`); motion clip at **30 Hz** (`fps`) —
  interpolate the clip to the control time.
- Physics/PD at 500 Hz (10 substeps/control) works well.
- Initialize from the reference pose with the base dropped so the feet rest on
  the floor (reference-state init). Segment starts may begin mid-hop — that's
  fine once the obs is correct.

## Reference CSV format (31 columns, headerless, 30 fps)

`[0:3]` root pos · `[3:7]` root quat (x,y,z,w) · `[7:31]` 24 joint angles (DFS,
radians).

## Values that are policy/robot-specific

`default_joint_pos`, action `scale`/`offset`, `stiffness`/`damping`,
`time_start`/`time_end`, `joint_ids_map` — all live in the decrypted
`*_generated.yaml` the controller writes at startup. `r1_dance_sim.py` bakes in
the dance1_subject2 values; swap them for your dance/variant.
