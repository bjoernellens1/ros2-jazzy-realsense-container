#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: build_pseudo_gt_from_realsense_bag.sh [--force] <realsense-ros1.bag>

Build RTAB-Map pseudo-ground-truth odometry from a RealSense SDK ROS1 bag.
Outputs are written next to the input bag:
  <bag-stem>_rtabmap_odom/          rosbag2 MCAP with /rtabmap/odom, /tf, /tf_static
  <bag-stem>_rtabmap_odom_tum.csv   timestamp tx ty tz qx qy qz qw
  <bag-stem>_rtabmap_odom_full.csv  pose, twist, and covariance columns
EOF
}

FORCE=false
if [[ "${1:-}" == "--force" ]]; then
  FORCE=true
  shift
fi

BAG="${1:-}"
if [[ -z "$BAG" ]]; then
  usage >&2
  exit 2
fi

if [[ ! -f "$BAG" ]]; then
  echo "[pseudo-gt] Input bag does not exist: $BAG" >&2
  exit 1
fi

BAG_ABS="$(realpath "$BAG")"
BAG_DIR="$(dirname "$BAG_ABS")"
BAG_FILE="$(basename "$BAG_ABS")"
BAG_STEM="${BAG_FILE%.*}"
OUT_PREFIX="${BAG_STEM}_rtabmap_odom"

EXTRA_ARGS=()
if [[ "$FORCE" == "true" ]]; then
  EXTRA_ARGS+=(--force)
fi

"$(dirname "$0")/compose_cmd.sh" run --rm \
  -v "${BAG_DIR}:/input:rw" \
  pseudo-gt \
  /work/scripts/_inside_build_pseudo_gt_from_realsense_bag.sh \
  "${EXTRA_ARGS[@]}" \
  "/input/${BAG_FILE}" \
  "/input/${OUT_PREFIX}"
