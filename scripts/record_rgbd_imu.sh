#!/usr/bin/env bash
set -euo pipefail

NAME="${1:-realsense_$(date +%Y%m%d_%H%M%S)}"
STORAGE="${ROSBAG_STORAGE:-mcap}"

"$(dirname "$0")/compose_cmd.sh" run --rm shell bash -lc "
  set -euo pipefail
  source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash
  mkdir -p /work/bags /work/logs

  echo '[record] Starting camera in background...'
  /work/scripts/_inside_camera_ros.sh > /work/logs/${NAME}_camera.log 2>&1 &
  CAM_PID=\$!

  cleanup() {
    echo '[record] Stopping camera...'
    kill \$CAM_PID >/dev/null 2>&1 || true
    wait \$CAM_PID >/dev/null 2>&1 || true
  }
  trap cleanup EXIT INT TERM

  echo '[record] Waiting for color topic...'
  for i in {1..30}; do
    if ros2 topic list | grep -q '/camera/camera/color/image_raw'; then
      break
    fi
    sleep 1
  done

  echo '[record] Recording to /work/bags/$NAME'
  echo '[record] Stop with Ctrl+C.'
  ros2 bag record \
    --storage '$STORAGE' \
    -o '/work/bags/$NAME' \
    /tf /tf_static \
    /camera/camera/color/image_raw \
    /camera/camera/color/camera_info \
    /camera/camera/depth/image_rect_raw \
    /camera/camera/depth/camera_info \
    /camera/camera/aligned_depth_to_color/image_raw \
    /camera/camera/aligned_depth_to_color/camera_info \
    /camera/camera/imu
"
