#!/usr/bin/env bash
set -euo pipefail

BAG="${1:-}"
if [[ -z "$BAG" ]]; then
  echo "Usage: $0 <bag-dir-under-bags-or-absolute-path>"
  exit 1
fi

if [[ "$BAG" = /* ]]; then
  BAG_PATH="$BAG"
else
  BAG_PATH="/work/bags/$BAG"
fi

"$(dirname "$0")/compose_cmd.sh" run --rm shell bash -lc "
  set -e
  source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash
  ros2 bag play '$BAG_PATH' --clock
"
