#!/usr/bin/env bash
set -euo pipefail

NAME="${1:-realsense_$(date +%Y%m%d_%H%M%S)}"
STORAGE="${ROSBAG_STORAGE:-mcap}"

"$(dirname "$0")/compose_cmd.sh" run --rm shell bash -lc "
  set -e
  source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash
  mkdir -p /work/bags
  echo '[record] Recording existing camera topics to /work/bags/$NAME'
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
