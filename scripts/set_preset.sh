#!/usr/bin/env bash
set -euo pipefail

PRESET="${1:-Medium Density}"
NODE="${2:-/camera/camera}"

"$(dirname "$0")/compose_cmd.sh" run --rm shell bash -lc "
  set -e
  source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash
  echo '[preset] Available parameters matching preset:'
  ros2 param list '$NODE' | grep -i preset || true
  echo
  echo '[preset] Trying depth_module.visual_preset = $PRESET'
  ros2 param set '$NODE' depth_module.visual_preset '$PRESET' || {
    echo '[preset] Fallback: trying visual_preset = $PRESET'
    ros2 param set '$NODE' visual_preset '$PRESET'
  }
"
