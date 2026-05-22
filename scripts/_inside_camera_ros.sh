#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

CAMERA_NAME="${CAMERA_NAME:-camera}"
DEPTH_PROFILE="${DEPTH_PROFILE:-640x480x15}"
COLOR_PROFILE="${COLOR_PROFILE:-640x480x15}"
ALIGN_DEPTH="${ALIGN_DEPTH:-true}"
INITIAL_RESET="${INITIAL_RESET:-true}"
ENABLE_IMU="${ENABLE_IMU:-true}"
UNITE_IMU_METHOD="${UNITE_IMU_METHOD:-2}"
ENABLE_POINTCLOUD="${ENABLE_POINTCLOUD:-false}"

echo "[camera] Starting RealSense D435i ROS 2 driver"
echo "[camera] depth=${DEPTH_PROFILE}, color=${COLOR_PROFILE}, align_depth=${ALIGN_DEPTH}, initial_reset=${INITIAL_RESET}, imu=${ENABLE_IMU}, pointcloud=${ENABLE_POINTCLOUD}"

ros2 launch realsense2_camera rs_launch.py \
  camera_name:="${CAMERA_NAME}" \
  enable_color:=true \
  enable_depth:=true \
  enable_sync:=true \
  align_depth.enable:="${ALIGN_DEPTH}" \
  initial_reset:="${INITIAL_RESET}" \
  enable_accel:="${ENABLE_IMU}" \
  enable_gyro:="${ENABLE_IMU}" \
  unite_imu_method:="${UNITE_IMU_METHOD}" \
  pointcloud.enable:="${ENABLE_POINTCLOUD}" \
  depth_module.depth_profile:="${DEPTH_PROFILE}" \
  rgb_camera.color_profile:="${COLOR_PROFILE}"
