#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

set -u

usage() {
  cat <<'EOF'
Usage: _inside_build_pseudo_gt_from_realsense_bag.sh [--force] <input.bag> <output-prefix>
EOF
}

FORCE=false
if [[ "${1:-}" == "--force" ]]; then
  FORCE=true
  shift
fi

INPUT_BAG="${1:-}"
OUT_PREFIX="${2:-}"
if [[ -z "$INPUT_BAG" || -z "$OUT_PREFIX" ]]; then
  usage >&2
  exit 2
fi

if [[ ! -f "$INPUT_BAG" ]]; then
  echo "[pseudo-gt] Input bag does not exist in container: $INPUT_BAG" >&2
  exit 1
fi

OUT_BAG="${OUT_PREFIX}"
TUM_CSV="${OUT_PREFIX}_tum.csv"
FULL_CSV="${OUT_PREFIX}_full.csv"
LOG_FILE="${OUT_PREFIX}.log"

if [[ "$FORCE" != "true" ]]; then
  for path in "$OUT_BAG" "$TUM_CSV" "$FULL_CSV" "$LOG_FILE"; do
    if [[ -e "$path" ]]; then
      echo "[pseudo-gt] Refusing to overwrite existing output: $path" >&2
      echo "[pseudo-gt] Re-run with --force to replace outputs." >&2
      exit 1
    fi
  done
else
  rm -rf "$OUT_BAG" "$TUM_CSV" "$FULL_CSV" "$LOG_FILE"
fi

RGB_TOPIC="/camera/camera/color/image_raw"
DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC="/camera/camera/color/camera_info"
ODOM_TOPIC="/rtabmap/odom"
WAIT_TIMEOUT="${PSEUDO_GT_WAIT_TIMEOUT:-60}"
POST_PLAYBACK_GRACE="${PSEUDO_GT_POST_PLAYBACK_GRACE:-5}"

PIDS=()
cleanup() {
  set +e
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -INT "$pid" >/dev/null 2>&1 || true
    fi
  done
  sleep 2
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -TERM "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

wait_for_topics() {
  local label="$1"
  shift
  local deadline=$((SECONDS + WAIT_TIMEOUT))
  echo "[pseudo-gt] Waiting up to ${WAIT_TIMEOUT}s for ${label}: $*"
  while (( SECONDS < deadline )); do
    local topics
    topics="$(ros2 topic list 2>/dev/null || true)"
    local missing=false
    for topic in "$@"; do
      if ! grep -Fxq "$topic" <<<"$topics"; then
        missing=true
        break
      fi
    done
    if [[ "$missing" == "false" ]]; then
      return 0
    fi
    sleep 1
  done

  echo "[pseudo-gt] Timed out waiting for ${label}" >&2
  ros2 topic list >&2 || true
  return 1
}

echo "[pseudo-gt] Input: $INPUT_BAG"
echo "[pseudo-gt] Output bag: $OUT_BAG"
echo "[pseudo-gt] TUM CSV: $TUM_CSV"
echo "[pseudo-gt] Full CSV: $FULL_CSV"

{
  echo "[pseudo-gt] Starting RealSense ROS1 bag playback through realsense2_camera."
  ros2 launch realsense2_camera rs_launch.py \
    rosbag_filename:="$INPUT_BAG" \
    camera_name:="${CAMERA_NAME:-camera}" \
    enable_color:=true \
    enable_depth:=true \
    enable_sync:=true \
    align_depth.enable:=true \
    initial_reset:=false \
    enable_accel:=true \
    enable_gyro:=true \
    unite_imu_method:=2 \
    pointcloud.enable:=false
} >"$LOG_FILE" 2>&1 &
CAMERA_PID=$!
PIDS+=("$CAMERA_PID")

wait_for_topics "RealSense RGB-D topics" "$RGB_TOPIC" "$DEPTH_TOPIC" "$CAMERA_INFO_TOPIC"

{
  echo "[pseudo-gt] Starting RTAB-Map RGB-D odometry."
  /work/scripts/_inside_rtabmap_rgbd_odom.sh
} >>"$LOG_FILE" 2>&1 &
RTABMAP_PID=$!
PIDS+=("$RTABMAP_PID")

wait_for_topics "RTAB-Map odometry" "$ODOM_TOPIC"

echo "[pseudo-gt] Exporting odometry CSVs and recording timesynced MCAP bag."
python3 /work/scripts/export_odom_csv.py \
  --topic "$ODOM_TOPIC" \
  --tum-csv "$TUM_CSV" \
  --full-csv "$FULL_CSV" \
  --out-bag "$OUT_BAG" >>"$LOG_FILE" 2>&1 &
CSV_PID=$!
PIDS+=("$CSV_PID")

echo "[pseudo-gt] Waiting for exporter to finish recording."
wait "$CSV_PID" || {
  status=$?
  echo "[pseudo-gt] Exporter exited with status $status. See $LOG_FILE" >&2
  exit "$status"
}

echo "[pseudo-gt] Playback finished; allowing ${POST_PLAYBACK_GRACE}s for odometry flush."
sleep "$POST_PLAYBACK_GRACE"

cleanup
trap - EXIT INT TERM

if [[ ! -d "$OUT_BAG" ]]; then
  echo "[pseudo-gt] Expected output bag was not created: $OUT_BAG" >&2
  exit 1
fi
if [[ ! -s "$TUM_CSV" || ! -s "$FULL_CSV" ]]; then
  echo "[pseudo-gt] Expected CSV outputs were not created or are empty." >&2
  exit 1
fi

echo "[pseudo-gt] Done."
echo "[pseudo-gt] Output bag: $OUT_BAG"
echo "[pseudo-gt] TUM CSV: $TUM_CSV"
echo "[pseudo-gt] Full CSV: $FULL_CSV"
echo "[pseudo-gt] Log: $LOG_FILE"
