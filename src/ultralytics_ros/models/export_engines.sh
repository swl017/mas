#!/usr/bin/env bash
# Engine exports for ticket 032. Run from this directory.
# Each export takes ~3-5 min on the Jetson Orin; full script ~50 min.
set -euo pipefail
cd "$(dirname "$0")"

# Ticket 031 / 032 Phase 1-5 baseline: square 640² engines (imgsz=640).
# Leave the existing *-drone.engine files alone — they're the reference
# for Phase 4 + 5 rows. Only re-export them by uncommenting below.
#
# for s in n s m l x; do
#   yolo export model=yolov11${s}-drone.pt format=engine half=True
# done

# Phase 6: rectangular imgsz matched to rtsp_camera output shapes.
# ultralytics imgsz is (H, W) and each dim must be a multiple of 32.
#   rtsp  640×360  (W×H) → imgsz=( 384,  640)  pads  24 rows vertical
#   rtsp  960×540  (W×H) → imgsz=( 544,  960)  pads   4 rows vertical
#   rtsp 1920×1080 (W×H) → imgsz=(1088, 1920)  pads   8 rows vertical
# Scope trimmed to {s, m, l} yolov11-drone + dronecop9-2 (2-class
# drone+bird detector) to keep the sweep tractable.
MODELS=(
  "yolov11s-drone"
  "yolov11m-drone"
  "yolov11l-drone"
  "dronecop9-2"
)
for m in "${MODELS[@]}"; do
  for HW in "384,640" "544,960" "1088,1920"; do
    tag="${HW/,/x}"  # 384,640 → 384x640
    echo "=== ${m}  imgsz=(${HW}) ==="
    yolo export model=${m}.pt format=engine half=True imgsz=${HW}
    mv ${m}.engine ${m}-${tag}.engine
  done
done

echo
echo "Phase 6 engines written:"
ls -lh yolov11*-drone-{384x640,544x960}.engine
