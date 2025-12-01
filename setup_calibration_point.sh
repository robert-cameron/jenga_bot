#!/usr/bin/env bash
# spawn_block42b.sh
# Publish a single static TF for block42b (row=4, block=2, face=b)
# Uses same geometry/math as the original spawn_jenga_tower.sh script.

set -euo pipefail

# Config (match your original)
TOWER_BASE_FRAME="tower_base_rotated"
BLOCK_L=0.127   # length along row axis (m)
BLOCK_W=0.046   # perp offset between blocks (m)
BLOCK_H=0.03    # block height (m)

# Target block: row 4 (1-indexed), block 2 (1-indexed), face 'b'
layer=$((4-1))   # zero-indexed layer -> 3
block_idx=$((2-1))  # zero-indexed across row -> 1
face="b"

# Quaternions used in original script (precomputed for 90deg etc)
Q_ID="0 0 0 1"
Q_ZP90="0 0 0.70710678 0.70710678"
Q_ZN90="0 0 -0.70710678 0.70710678"
Q_Z180="0 0 1 0"

# helper to publish
publish_tf_quat() {
  local x="$1" y="$2" z="$3" qx="$4" qy="$5" qz="$6" qw="$7" parent="$8" child="$9"
  ros2 run tf2_ros static_transform_publisher "$x" "$y" "$z" "$qx" "$qy" "$qz" "$qw" "$parent" "$child" >/dev/null 2>&1 &
}

# compute z_center = (BLOCK_H/2) + layer*BLOCK_H
z_center=$(python3 - <<PY
h=$BLOCK_H
layer=$layer
print((h/2.0) + layer*h)
PY
)

# Determine row axis (even layer -> x, odd layer -> y)
if (( layer % 2 == 0 )); then
  row_axis="x"
else
  row_axis="y"
fi

# perp_offset = BLOCK_W * (block_idx - 1) in original script idx = b-1,
# but because original set idx=b-1 where b ranged from 0..2, that produced (-W,0,+W).
# In the original script it printed `idx=$((b-1))` then perp_offset = BLOCK_W * idx.
# For block_idx = 1 (block 2), idx = 0 -> perp_offset = 0
idx=$((block_idx - 1))
perp_offset=$(python3 - <<PY
w=$BLOCK_W
idx=$idx
print(w * idx)
PY
)

# Compute x_pos, y_pos and quaternion for face 'b' for a row on the y axis
if [[ "$row_axis" == "x" ]]; then
  # not the case for layer=3, kept for completeness
  if [[ "$face" == "f" ]]; then
    x_pos=$(python3 - <<PY
print($BLOCK_L/2.0)
PY
)
    quat="$Q_Z180"
  else
    x_pos=$(python3 - <<PY
print(-$BLOCK_L/2.0)
PY
)
    quat="$Q_ID"
  fi
  y_pos="$perp_offset"
else
  # row_axis == "y" (this is the case for layer=3)
  if [[ "$face" == "f" ]]; then
    y_pos=$(python3 - <<PY
print($BLOCK_L/2.0)
PY
)
    quat="$Q_ZN90"
  else
    # face == 'b'
    y_pos=$(python3 - <<PY
print(-$BLOCK_L/2.0)
PY
)
    quat="$Q_ZP90"
  fi
  x_pos="$perp_offset"
fi

# Child frame name for block42b
child_frame="calibration_point_block42b"

# parse quaternion into components
read qx qy qz qw <<<"$quat"

echo "Publishing transform for $child_frame relative to $TOWER_BASE_FRAME:"
echo " translation: x=$x_pos y=$y_pos z=$z_center"
echo " rotation (qx qy qz qw): $qx $qy $qz $qw"

publish_tf_quat "$x_pos" "$y_pos" "$z_center" "$qx" "$qy" "$qz" "$qw" "$TOWER_BASE_FRAME" "$child_frame"

echo "Done. To view: run rviz2 → Add → TF (Fixed Frame = world) or list_frames with tf2 tools."
