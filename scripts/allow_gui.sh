#!/usr/bin/env bash
set -euo pipefail

if ! command -v xhost >/dev/null 2>&1; then
  echo "xhost not found. On Fedora install: sudo dnf install xorg-x11-xhost"
  exit 1
fi

# Enough for the root user inside the container. Prefer this over global xhost +.
xhost +SI:localuser:root >/dev/null

# Docker rootless / Podman can sometimes present differently; this fallback is still local only.
xhost +local:root >/dev/null || true

echo "GUI access allowed for local root containers."
