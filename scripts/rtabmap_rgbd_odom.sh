#!/usr/bin/env bash
set -euo pipefail
"$(dirname "$0")/compose_cmd.sh" run --rm rtabmap-odom
