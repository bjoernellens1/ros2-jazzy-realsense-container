#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

echo "[rtabmap] Starting RGB-D odometry against RealSense aligned depth."
echo "[rtabmap] This assumes the camera node is already running."

ros2 run rtabmap_odom rgbd_odometry --ros-args \
  -p frame_id:=camera_link \
  -p odom_frame_id:=odom \
  -p publish_tf:=true \
  -p approx_sync:=true \
  -p queue_size:=30 \
  -p subscribe_rgbd:=false \
  -r rgb/image:=/camera/camera/color/image_raw \
  -r depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
  -r rgb/camera_info:=/camera/camera/color/camera_info \
  -r odom:=/rtabmap/odom
