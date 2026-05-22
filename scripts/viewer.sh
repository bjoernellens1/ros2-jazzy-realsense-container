#!/usr/bin/env bash
set -euo pipefail
"$(dirname "$0")/allow_gui.sh"
"$(dirname "$0")/compose_cmd.sh" run --rm viewer
