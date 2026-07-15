#!/usr/bin/env bash
# Fetch Unitree's official R1 MuJoCo model (BSD, public) — the sim "body".
# Sparse-clones just unitree_robots/r1 (MJCF + scene + 43 meshes, ~22 MB).
set -euo pipefail

DEST="${1:-./r1_model}"
TMP="$(mktemp -d)"
echo "Cloning unitree_mujoco (r1 only) ..."
git clone --depth 1 --filter=blob:none --sparse \
  https://github.com/unitreerobotics/unitree_mujoco.git "$TMP/unitree_mujoco"
git -C "$TMP/unitree_mujoco" sparse-checkout set unitree_robots/r1
mkdir -p "$DEST"
cp -R "$TMP/unitree_mujoco/unitree_robots/r1/." "$DEST/"
rm -rf "$TMP"
echo "Done -> $DEST  (scene: $DEST/scene.xml, meshes: $(ls "$DEST/meshes" | wc -l | tr -d ' ') files)"
