#!/usr/bin/env bash
set -euo pipefail
"$(dirname "$0")/compose_cmd.sh" run --rm shell bash -lc '
  ros2 topic list -t
'
