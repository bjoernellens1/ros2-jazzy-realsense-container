#!/usr/bin/env bash
set -euo pipefail
CUDA=false
if [[ "${1:-}" == "--cuda" ]]; then
  CUDA=true
fi

SCRIPT_DIR="$(dirname "$0")"
"${SCRIPT_DIR}/compose_cmd.sh" build shell
"${SCRIPT_DIR}/compose_cmd.sh" --profile offline build pseudo-gt
if [[ "$CUDA" == "true" ]]; then
  "${SCRIPT_DIR}/compose_cmd.sh" --profile cuda build pseudo-gt-cuda
fi
