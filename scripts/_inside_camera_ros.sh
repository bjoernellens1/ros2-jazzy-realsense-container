#!/usr/bin/env bash
set -euo pipefail

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

CAMERA_NAME="${CAMERA_NAME:-camera}"
DEPTH_PROFILE="${DEPTH_PROFILE:-848x480x30}"
COLOR_PROFILE="${COLOR_PROFILE:-1280x720x30}"
ALIGN_DEPTH="${ALIGN_DEPTH:-true}"
ENABLE_IMU="${ENABLE_IMU:-true}"
UNITE_IMU_METHOD="${UNITE_IMU_METHOD:-2}"
ENABLE_POINTCLOUD="${ENABLE_POINTCLOUD:-false}"

echo "[camera] Starting RealSense D435i ROS 2 driver"
echo "[camera] depth=${DEPTH_PROFILE}, color=${COLOR_PROFILE}, align_depth=${ALIGN_DEPTH}, imu=${ENABLE_IMU}, pointcloud=${ENABLE_POINTCLOUD}"

ros2 launch realsense2_camera rs_launch.py \
  camera_name:="${CAMERA_NAME}" \
  enable_color:=true \
  enable_depth:=true \
  enable_sync:=true \
  align_depth.enable:="${ALIGN_DEPTH}" \
  enable_accel:="${ENABLE_IMU}" \
  enable_gyro:="${ENABLE_IMU}" \
  unite_imu_method:="${UNITE_IMU_METHOD}" \
  pointcloud.enable:="${ENABLE_POINTCLOUD}" \
  depth_module.profile:="${DEPTH_PROFILE}" \
  rgb_camera.profile:="${COLOR_PROFILE}"
