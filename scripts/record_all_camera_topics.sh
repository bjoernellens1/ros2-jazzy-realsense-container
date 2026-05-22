#!/usr/bin/env bash
set -euo pipefail

NAME="${1:-realsense_all_$(date +%Y%m%d_%H%M%S)}"
STORAGE="${ROSBAG_STORAGE:-mcap}"

"$(dirname "$0")/compose_cmd.sh" run --rm shell bash -lc "
  set -e
  source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash
  mkdir -p /work/bags
  echo '[record] Recording all /camera and tf topics to /work/bags/$NAME'
  ros2 bag record --storage '$STORAGE' -o '/work/bags/$NAME' -e '^(/camera/.*|/tf|/tf_static)$'
"
