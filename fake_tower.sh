#!/usr/bin/env bash
# spawn_jenga_tower.sh
# Publishes static TF frames for a Jenga tower in ROS 2.
# - 3 blocks per row, rows alternate orientation (X, then Y, then X, …).
# - A 'tower_base' frame is placed at the center of the table.
# - For each block, two face frames (ends you push/pull) are published:
#   block{row}{block}{face}
#     - row: 1 = bottom, increasing upward
#     - block: 1, 2, 3 within the row
#     - face: 'f' (front / +axis end) or 'b' (back / −axis end)
#   The +x axis of each face frame points inward, toward the block center.
#
# Requires: ros2, tf2_ros (static_transform_publisher)

set -euo pipefail

#########################
# User-configurable params
#########################

WORLD_FRAME="world"
TABLE_FRAME="table"
PUBLISH_TABLE_IDENTITY=1

TOWER_BASE_FRAME="tower_base"
TOWER_X=0.5
TOWER_Y=0.2
TOWER_Z=0.03
TOWER_YAW_DEG=45

BLOCK_L=0.13
BLOCK_W=0.04
BLOCK_H=0.03

NUM_LAYERS=5
ROW_BLOCKS=3

FRAME_PREFIX=""

#########################
# Helpers
#########################

deg2rad() {
  python3 - "$1" <<'PY'
import sys,math
print(float(sys.argv[1])*math.pi/180.0)
PY
}

publish_tf_quat() {
  local x="$1" y="$2" z="$3" qx="$4" qy="$5" qz="$6" qw="$7" parent="$8" child="$9"
  ros2 run tf2_ros static_transform_publisher "$x" "$y" "$z" "$qx" "$qy" "$qz" "$qw" "$parent" "$child" >/dev/null 2>&1 &
}

publish_tf_yaw_deg() {
  local x="$1" y="$2" z="$3" yaw_deg="$4" parent="$5" child="$6"
  python3 - "$x" "$y" "$z" "$yaw_deg" "$parent" "$child" <<'PY'
import sys,math,subprocess
x=float(sys.argv[1]); y=float(sys.argv[2]); z=float(sys.argv[3])
yaw_deg=float(sys.argv[4]); parent=sys.argv[5]; child=sys.argv[6]
yaw=math.radians(yaw_deg)
qx=qy=0.0
qz=math.sin(yaw/2.0)
qw=math.cos(yaw/2.0)
subprocess.Popen(["ros2","run","tf2_ros","static_transform_publisher",
                  str(x),str(y),str(z),str(qx),str(qy),str(qz),str(qw),parent,child])
PY
}

pf() { echo "${FRAME_PREFIX}$1"; }

#########################
# Start publishing
#########################

command -v ros2 >/dev/null 2>&1 || { echo "ERROR: ros2 not found in PATH."; exit 1; }

if [[ "$PUBLISH_TABLE_IDENTITY" -eq 1 ]]; then
  publish_tf_quat 0 0 0 0 0 0 1 "$(pf "$WORLD_FRAME")" "$(pf "$TABLE_FRAME")"
fi

publish_tf_yaw_deg "$TOWER_X" "$TOWER_Y" "$TOWER_Z" "$TOWER_YAW_DEG" "$(pf "$TABLE_FRAME")" "$(pf "$TOWER_BASE_FRAME")"

Q_ID="0 0 0 1"
Q_ZP90="0 0 0.70710678 0.70710678"
Q_ZN90="0 0 -0.70710678 0.70710678"
Q_Z180="0 0 1 0"

# Build tower
for (( layer=0; layer<NUM_LAYERS; layer++ )); do
  row_num=$((layer+1))
  z_center=$(python3 - <<PY
h=$BLOCK_H
print((h/2.0) + $layer*h)
PY
)

  if (( layer % 2 == 0 )); then
    row_axis="x"
  else
    row_axis="y"
  fi

  # Blocks 1,2,3 at offsets -W,0,+W
  for (( b=0; b<ROW_BLOCKS; b++ )); do
    block_num=$((b+1))
    idx=$((b-1))
    perp_offset=$(python3 - <<PY
print($BLOCK_W * $idx)
PY
)

    for face in f b; do
      if [[ "$row_axis" == "x" ]]; then
        if [[ "$face" == "f" ]]; then
          x_pos=$(python3 - <<PY
print($BLOCK_L/2.0)
PY
)
          quat="$Q_Z180"  # inward +x → -X
        else
          x_pos=$(python3 - <<PY
print(-$BLOCK_L/2.0)
PY
)
          quat="$Q_ID"    # inward +x → +X
        fi
        y_pos="$perp_offset"

      else
        if [[ "$face" == "f" ]]; then
          y_pos=$(python3 - <<PY
print($BLOCK_L/2.0)
PY
)
          quat="$Q_ZN90"  # inward +x → -Y
        else
          y_pos=$(python3 - <<PY
print(-$BLOCK_L/2.0)
PY
)
          quat="$Q_ZP90"  # inward +x → +Y
        fi
        x_pos="$perp_offset"
      fi

      # New naming convention
      child_frame="$(pf block${row_num}${block_num}${face})"

      read qx qy qz qw <<<"$quat"
      publish_tf_quat "$x_pos" "$y_pos" "$z_center" "$qx" "$qy" "$qz" "$qw" "$(pf "$TOWER_BASE_FRAME")" "$child_frame"
    done
  done
done

echo "Spawned Jenga TFs:"
echo "- World/Table: $(pf "$WORLD_FRAME") -> $(pf "$TABLE_FRAME") (identity=${PUBLISH_TABLE_IDENTITY})"
echo "- Tower base:  $(pf "$TABLE_FRAME") -> $(pf "$TOWER_BASE_FRAME") at (${TOWER_X}, ${TOWER_Y}, ${TOWER_Z}) yaw=${TOWER_YAW_DEG} deg"
echo "- Blocks:      ${NUM_LAYERS} layers, ${ROW_BLOCKS} per row, faces per block = 2"
echo
echo "Notes:"
echo "• Face frame naming: block{row}{block}{face}"
echo "  - row: 1 = bottom upward"
echo "  - block: 1, 2, 3 across the row"
echo "  - face: f (front/+axis), b (back/−axis)"
echo "• Each face frame's +x axis points inward toward the block center."
echo
echo "To visualize:  rviz2 → Add → TF → set Fixed Frame = world"
echo "To stop:       pkill -f tf2_ros/static_transform_publisher"
