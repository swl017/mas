#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_mrcal_intrinsics.sh \
    --dataset-root <path> \
    --zoom-level <zoom> \
    --object-spacing-m <meters> \
    --object-width-n <corners> \
    --focal-px <pixels> \
    [--lensmodel LENSMODEL_OPENCV8] \
    [--image-glob '*.jpg'] \
    [--observed-pixel-uncertainty 2.0] \
    [--skip-calobject-warp-solve] \
    [--extra-arg <arg>]
EOF
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required executable: $1" >&2
    exit 1
  fi
}

normalize_zoom() {
  local zoom="$1"
  zoom="${zoom%x}"
  printf "%sx" "$zoom"
}

DATASET_ROOT=""
ZOOM_LEVEL=""
OBJECT_SPACING_M=""
OBJECT_WIDTH_N=""
FOCAL_PX=""
LENSMODEL="LENSMODEL_OPENCV8"
IMAGE_GLOB="*.jpg"
OBSERVED_PIXEL_UNCERTAINTY=""
SKIP_CALOBJECT_WARP_SOLVE=0
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dataset-root)
      DATASET_ROOT="$2"
      shift 2
      ;;
    --zoom-level)
      ZOOM_LEVEL="$2"
      shift 2
      ;;
    --object-spacing-m)
      OBJECT_SPACING_M="$2"
      shift 2
      ;;
    --object-width-n)
      OBJECT_WIDTH_N="$2"
      shift 2
      ;;
    --focal-px)
      FOCAL_PX="$2"
      shift 2
      ;;
    --lensmodel)
      LENSMODEL="$2"
      shift 2
      ;;
    --image-glob)
      IMAGE_GLOB="$2"
      shift 2
      ;;
    --observed-pixel-uncertainty)
      OBSERVED_PIXEL_UNCERTAINTY="$2"
      shift 2
      ;;
    --skip-calobject-warp-solve)
      SKIP_CALOBJECT_WARP_SOLVE=1
      shift
      ;;
    --extra-arg)
      EXTRA_ARGS+=("$2")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ -z "$DATASET_ROOT" || -z "$ZOOM_LEVEL" || -z "$OBJECT_SPACING_M" || -z "$OBJECT_WIDTH_N" || -z "$FOCAL_PX" ]]; then
  usage >&2
  exit 1
fi

require_cmd mrcal-calibrate-cameras
require_cmd mrcal-show-residuals
require_cmd mrcal-show-projection-uncertainty
require_cmd mrgingham

ZOOM_DIR="$(normalize_zoom "$ZOOM_LEVEL")"
ROOT="$DATASET_ROOT/$ZOOM_DIR"
IMAGES_DIR="$ROOT/images"
ANALYSIS_DIR="$ROOT/analysis"
CALIB_DIR="$ROOT/calibration"
LOG_DIR="$ROOT/logs"
METRICS_JSON="$ROOT/metrics.json"
CORNERS_CACHE="$ANALYSIS_DIR/corners.vnl"
CALIB_LOG="$LOG_DIR/mrcal-calibrate.log"

mkdir -p "$ANALYSIS_DIR" "$CALIB_DIR" "$LOG_DIR"

if [[ ! -d "$IMAGES_DIR" ]]; then
  echo "Missing images directory: $IMAGES_DIR" >&2
  exit 1
fi

shopt -s nullglob
image_paths=( "$IMAGES_DIR"/$IMAGE_GLOB )
shopt -u nullglob

if [[ ${#image_paths[@]} -eq 0 ]]; then
  echo "No images matched $IMAGES_DIR/$IMAGE_GLOB" >&2
  exit 1
fi

cmd=(
  mrcal-calibrate-cameras
  --corners-cache "$CORNERS_CACHE"
  --lensmodel "$LENSMODEL"
  --focal "$FOCAL_PX"
  --object-spacing "$OBJECT_SPACING_M"
  --object-width-n "$OBJECT_WIDTH_N"
  --outdir "$CALIB_DIR"
)

if [[ -n "$OBSERVED_PIXEL_UNCERTAINTY" ]]; then
  cmd+=(--observed-pixel-uncertainty "$OBSERVED_PIXEL_UNCERTAINTY")
fi

if [[ "$SKIP_CALOBJECT_WARP_SOLVE" -eq 1 ]]; then
  cmd+=(--skip-calobject-warp-solve)
fi

if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  cmd+=("${EXTRA_ARGS[@]}")
fi

# mrcal-calibrate-cameras expects ONE glob per camera, not expanded paths —
# each extra argument is interpreted as another camera. Pass the literal
# pattern and let mrcal expand it internally.
cmd+=("$IMAGES_DIR/$IMAGE_GLOB")

printf 'Running calibration for %s with %d images\n' "$ZOOM_DIR" "${#image_paths[@]}"
printf 'Command:\n' | tee "$CALIB_LOG"
printf '  %q' "${cmd[@]}" | tee -a "$CALIB_LOG"
printf '\n' | tee -a "$CALIB_LOG"

"${cmd[@]}" 2>&1 | tee -a "$CALIB_LOG"

if [[ ! -f "$METRICS_JSON" ]]; then
  cat > "$METRICS_JSON" <<'EOF'
{
  "rms_reprojection_error_px": null,
  "worst_reprojection_error_px": null,
  "max_projection_uncertainty_px": null,
  "parameter_stddev": {
    "fx": null,
    "fy": null,
    "cx": null,
    "cy": null,
    "k1": null,
    "k2": null,
    "p1": null,
    "p2": null,
    "k3": null,
    "k4": null,
    "k5": null,
    "k6": null
  }
}
EOF
fi

echo "Calibration artifacts:"
find "$CALIB_DIR" -maxdepth 1 -type f | sort
echo "Update metrics in $METRICS_JSON before running summarize_mrcal_intrinsics.py"
