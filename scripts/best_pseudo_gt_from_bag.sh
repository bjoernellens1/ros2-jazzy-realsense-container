#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: best_pseudo_gt_from_bag.sh [--cuda] --profile <profile> [options] <bag-rosbag2-or-tum-dir>

Runs the pseudo-GT pipeline entirely inside Compose containers.
Outputs default to <repo>/output/<input-stem>_pseudo_gt/ unless --output is given.

Common options passed through:
  --methods <csv>
  --input-format auto|bag|tum_rgbd
  --colmap-preset fast|stable|robust
  --rtabmap-preset default|robust|f2f|dense-keyframes
  --workspace-mode ram|disk
  --persist-intermediates
  --allow-unreliable-best
  --max-frames N
  --target-fps FPS
  --output DIR
  --force

Use --cuda to run the CUDA pseudo-GT image and enable COLMAP GPU matching.
EOF
}

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

CUDA=false
ARGS=()
BAG=""
OUTPUT=""
PROFILE_SEEN=false
COLMAP_GPU_SEEN=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --cuda)
      CUDA=true
      shift
      ;;
    --profile)
      PROFILE_SEEN=true
      ARGS+=("$1" "$2")
      shift 2
      ;;
    --output)
      OUTPUT="$2"
      shift 2
      ;;
    --colmap-use-gpu)
      COLMAP_GPU_SEEN=true
      ARGS+=("$1" "$2")
      shift 2
      ;;
    --force|--persist-intermediates|--keep-workspace|--allow-unreliable-best)
      ARGS+=("$1")
      shift
      ;;
    --*)
      if [[ $# -ge 2 && "$2" != --* ]]; then
        ARGS+=("$1" "$2")
        shift 2
      else
        ARGS+=("$1")
        shift
      fi
      ;;
    *)
      if [[ -n "$BAG" ]]; then
        echo "[pseudo-gt] Multiple input paths supplied: $BAG and $1" >&2
        usage >&2
        exit 2
      fi
      BAG="$1"
      shift
      ;;
  esac
done

if [[ -z "$BAG" || "$PROFILE_SEEN" != "true" ]]; then
  usage >&2
  exit 2
fi

if [[ ! -e "$BAG" ]]; then
  echo "[pseudo-gt] Input does not exist: $BAG" >&2
  exit 1
fi

BAG_ABS="$(realpath "$BAG")"
if [[ -d "$BAG_ABS" ]]; then
  BAG_PARENT="$(dirname "$BAG_ABS")"
  BAG_NAME="$(basename "$BAG_ABS")"
  DEFAULT_STEM="$BAG_NAME"
else
  BAG_PARENT="$(dirname "$BAG_ABS")"
  BAG_FILE="$(basename "$BAG_ABS")"
  BAG_NAME="$BAG_FILE"
  DEFAULT_STEM="${BAG_FILE%.*}"
fi

if [[ -z "$OUTPUT" ]]; then
  OUTPUT="${ROOT_DIR}/output/${DEFAULT_STEM}_pseudo_gt"
fi
OUTPUT_ABS="$(realpath -m "$OUTPUT")"
OUTPUT_PARENT="$(dirname "$OUTPUT_ABS")"
OUTPUT_NAME="$(basename "$OUTPUT_ABS")"
mkdir -p "$OUTPUT_PARENT"

SERVICE="pseudo-gt"
if [[ "$CUDA" == "true" ]]; then
  SERVICE="pseudo-gt-cuda"
  if [[ "$COLMAP_GPU_SEEN" != "true" ]]; then
    ARGS+=(--colmap-use-gpu 1)
  fi
fi

PROFILE_ARG=(--profile offline)
if [[ "$CUDA" == "true" ]]; then
  PROFILE_ARG=(--profile cuda)
fi

"$ROOT_DIR/scripts/compose_cmd.sh" "${PROFILE_ARG[@]}" run --rm \
  -v "${BAG_PARENT}:/input:ro" \
  -v "${OUTPUT_PARENT}:/outputs:rw" \
  "$SERVICE" \
  python3 /work/scripts/pseudo_gt_pipeline.py \
  "/input/${BAG_NAME}" \
  --output "/outputs/${OUTPUT_NAME}" \
  "${ARGS[@]}"
