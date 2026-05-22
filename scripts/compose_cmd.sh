#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

ENGINE="${CONTAINER_ENGINE:-auto}"

if [[ "$ENGINE" == "docker" ]]; then
  exec docker compose "$@"
fi

if [[ "$ENGINE" == "podman" ]]; then
  if command -v podman-compose >/dev/null 2>&1; then
    exec podman-compose "$@"
  fi
  exec podman compose "$@"
fi

if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
  exec docker compose "$@"
fi

if command -v podman-compose >/dev/null 2>&1; then
  exec podman-compose "$@"
fi

if command -v podman >/dev/null 2>&1 && podman compose version >/dev/null 2>&1; then
  exec podman compose "$@"
fi

echo "No compose implementation found. Install docker compose or podman-compose." >&2
exit 1
