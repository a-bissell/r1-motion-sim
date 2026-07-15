# Extracting the sim files from your own R1

The simulation needs three files. One is public; two live on your robot's
rootfs and are policy-/robot-specific, so pull them from *your* unit.

| File | Where it comes from |
|---|---|
| `scene.xml` + `meshes/` (the MuJoCo model) | public — run `./tools/fetch_mujoco_model.sh` |
| dance policy `.onnx` | your rootfs: `/unitree/module/ai_sport/net/motions/...` |
| reference motion `.csv` | your rootfs: `/unitree/module/ai_sport/config/trajs/...` |
| (optional) decrypted `dances.yaml` | your rootfs: `/unitree/module/ai_sport/config/fsm/motions/` |

> ⚠️ **Your rootfs contains secrets** — SSH host keys, DDS identity keys,
> `/etc/shadow`, cloud tokens in `chat_go`. Extract only the sim files below.
> Do **not** share a full rootfs dump.

## Getting a readable rootfs

**On the robot (easiest):** `scp` the files off directly — they're just files
under `/unitree/module/ai_sport/`.

**From a disk image on Linux:**
```bash
# find the rootfs partition offset, then loop-mount read-only
sudo losetup -Pf r1.img && lsblk        # note the ext4 'linuxroot' partition
sudo mount -o ro /dev/loopNp7 /mnt
find /mnt/unitree/module/ai_sport -name '*.onnx'
```

**From a disk image on macOS** (can't mount ext4 natively):
```bash
brew install e2fsprogs
hdiutil attach -imagekey diskimage-class=CRawDiskImage -nomount r1.img
#   -> prints partitions; rootfs is the ~45 GB one (e.g. /dev/disk4s7)
DBG=/opt/homebrew/opt/e2fsprogs/sbin/debugfs
# NOTE: -c (catastrophic) is required — live-captured images have stale bitmaps
python3 tools/dbwalk.py /dev/disk4s7 /unitree/module/ai_sport | grep -iE '\.onnx|\.csv'
$DBG -c -R "dump <full_path_on_image> ./policy.onnx" /dev/disk4s7
```

## Locating the right policy + trajectory

The FSM config `dances.yaml` pairs each dance with its policy, motion file, and
time window. It's FMX-encrypted; decrypt with the community `fmx_tool.py`
(`fmx_tool.py decrypt <in> <out>`, key version 2 for 1.1.14.1+). At startup the
controller also writes a fully-resolved `*_generated.yaml` next to it — that
plaintext file has everything the config YAML (see `configs/`) needs
(`joint_ids_map`, `stiffness`, `damping`, `default_joint_pos`, action
`scale`/`offset`, `time_start`/`time_end`, `fps`, `step_dt`).

Example pairing shipped on the Edu/Pro build:
- policy: `net/motions/dances/dance1_subject2/dance1_subject2_hard_80k-120k.onnx`
- motion: `config/trajs/dances/dance1_subject2/R1_edu_lh_dance1_subject2.bvh_30hz.csv`
- window: `time_start: 48.56, time_end: 61.50`

## Run it

```bash
pip install -r requirements.txt
./tools/fetch_mujoco_model.sh ./r1_model
python3 r1_motion_sim.py --config configs/dance1_subject2.yaml \
    --model r1_model/scene.xml --policy policy.onnx --traj motion.csv --gif dance.gif
# expect: "balanced whole window", corr median ~0.95
```
