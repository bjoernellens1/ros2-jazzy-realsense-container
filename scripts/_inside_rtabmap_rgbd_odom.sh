#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

RTABMAP_FRAME_ID="${RTABMAP_FRAME_ID:-camera_link}"
RTABMAP_ODOM_FRAME_ID="${RTABMAP_ODOM_FRAME_ID:-odom}"
RTABMAP_TOPIC_QUEUE_SIZE="${RTABMAP_TOPIC_QUEUE_SIZE:-30}"
RTABMAP_SYNC_QUEUE_SIZE="${RTABMAP_SYNC_QUEUE_SIZE:-30}"
RTABMAP_APPROX_SYNC_MAX_INTERVAL="${RTABMAP_APPROX_SYNC_MAX_INTERVAL:-0.0}"
RTABMAP_RESET_COUNTDOWN="${RTABMAP_RESET_COUNTDOWN:-0}"
RTABMAP_VIS_MIN_INLIERS="${RTABMAP_VIS_MIN_INLIERS:-8}"
RTABMAP_VIS_MAX_FEATURES="${RTABMAP_VIS_MAX_FEATURES:-1500}"
RTABMAP_ODOM_STRATEGY="${RTABMAP_ODOM_STRATEGY:-0}"
RTABMAP_ODOM_F2M_MAX_SIZE="${RTABMAP_ODOM_F2M_MAX_SIZE:-2000}"
RTABMAP_VIS_MAX_DEPTH="${RTABMAP_VIS_MAX_DEPTH:-8.0}"
RTABMAP_VIS_MIN_DEPTH="${RTABMAP_VIS_MIN_DEPTH:-0.3}"
RTABMAP_WAIT_TIMEOUT="${RTABMAP_WAIT_TIMEOUT:-30}"
RTABMAP_SUBSCRIBE_IMU="${RTABMAP_SUBSCRIBE_IMU:-true}"

RGB_TOPIC="${PSEUDO_GT_RGB_TOPIC:-/camera/camera/color/image_raw}"
DEPTH_TOPIC="${PSEUDO_GT_DEPTH_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
CAMERA_INFO_TOPIC="${PSEUDO_GT_CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
IMU_TOPIC="${PSEUDO_GT_IMU_TOPIC:-/camera/camera/imu}"

echo "[rtabmap] Starting RGB-D + IMU odometry (VIO) against RealSense aligned depth."
echo "[rtabmap] This assumes the camera node is already running."
echo "[rtabmap] Odom/Strategy=${RTABMAP_ODOM_STRATEGY} (0=F2M), OdomF2M/MaxSize=${RTABMAP_ODOM_F2M_MAX_SIZE}, Vis/MaxDepth=${RTABMAP_VIS_MAX_DEPTH}, Vis/MinDepth=${RTABMAP_VIS_MIN_DEPTH}"
echo "[rtabmap] approx_sync_max_interval=${RTABMAP_APPROX_SYNC_MAX_INTERVAL}, topic_queue_size=${RTABMAP_TOPIC_QUEUE_SIZE}, sync_queue_size=${RTABMAP_SYNC_QUEUE_SIZE}"
echo "[rtabmap] Vis/MinInliers=${RTABMAP_VIS_MIN_INLIERS}, Vis/MaxFeatures=${RTABMAP_VIS_MAX_FEATURES}, subscribe_imu=${RTABMAP_SUBSCRIBE_IMU}"
echo "[rtabmap] Topics: rgb=${RGB_TOPIC}, depth=${DEPTH_TOPIC}, camera_info=${CAMERA_INFO_TOPIC}, imu=${IMU_TOPIC}"
echo "[rtabmap] Waiting up to ${RTABMAP_WAIT_TIMEOUT}s for RGB-D camera topics."

deadline=$((SECONDS + RTABMAP_WAIT_TIMEOUT))
while (( SECONDS < deadline )); do
  topics="$(ros2 topic list 2>/dev/null || true)"
  if grep -Fxq "${RGB_TOPIC}" <<<"${topics}" &&
     grep -Fxq "${DEPTH_TOPIC}" <<<"${topics}" &&
     grep -Fxq "${CAMERA_INFO_TOPIC}" <<<"${topics}" &&
     { [[ "${RTABMAP_SUBSCRIBE_IMU}" != "true" ]] || grep -Fxq "${IMU_TOPIC}" <<<"${topics}"; }; then
    break
  fi
  sleep 1
done

topics="$(ros2 topic list 2>/dev/null || true)"
REQUIRED_TOPICS=("${RGB_TOPIC}" "${DEPTH_TOPIC}" "${CAMERA_INFO_TOPIC}")
if [[ "${RTABMAP_SUBSCRIBE_IMU}" == "true" ]]; then
  REQUIRED_TOPICS+=("${IMU_TOPIC}")
fi

for topic in "${REQUIRED_TOPICS[@]}"; do
  if ! grep -Fxq "${topic}" <<<"${topics}"; then
    echo "[rtabmap] Timed out waiting for ${topic}" >&2
    exit 1
  fi
done

ARGS=(
  ros2 run rtabmap_odom rgbd_odometry --ros-args
  -p frame_id:="${RTABMAP_FRAME_ID}"
  -p odom_frame_id:="${RTABMAP_ODOM_FRAME_ID}"
  -p publish_tf:=true
  -p approx_sync:=true
  -p approx_sync_max_interval:="${RTABMAP_APPROX_SYNC_MAX_INTERVAL}"
  -p topic_queue_size:="${RTABMAP_TOPIC_QUEUE_SIZE}"
  -p sync_queue_size:="${RTABMAP_SYNC_QUEUE_SIZE}"
  -p subscribe_rgbd:=false
  -p subscribe_imu:="${RTABMAP_SUBSCRIBE_IMU}"
  -p reset_countdown:="${RTABMAP_RESET_COUNTDOWN}"
  -p "Odom/Strategy:='${RTABMAP_ODOM_STRATEGY}'"
  -p "OdomF2M/MaxSize:='${RTABMAP_ODOM_F2M_MAX_SIZE}'"
  -p "Vis/MinInliers:='${RTABMAP_VIS_MIN_INLIERS}'"
  -p "Vis/MaxFeatures:='${RTABMAP_VIS_MAX_FEATURES}'"
  -p "Vis/MaxDepth:='${RTABMAP_VIS_MAX_DEPTH}'"
  -p "Vis/MinDepth:='${RTABMAP_VIS_MIN_DEPTH}'"
  -r rgb/image:="${RGB_TOPIC}"
  -r depth/image:="${DEPTH_TOPIC}"
  -r rgb/camera_info:="${CAMERA_INFO_TOPIC}"
  -r odom:=/rtabmap/odom
)
if [[ "${RTABMAP_SUBSCRIBE_IMU}" == "true" ]]; then
  ARGS+=(-r imu:="${IMU_TOPIC}")
fi

"${ARGS[@]}"
