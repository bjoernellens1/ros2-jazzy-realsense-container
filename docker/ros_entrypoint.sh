#!/usr/bin/env bash
set -e

if [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
fi

exec "$@"
