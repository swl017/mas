#!/usr/bin/env python3

import argparse
import ast
import csv
import json
from pathlib import Path
from typing import Any


INTRINSIC_NAMES = (
    "fx",
    "fy",
    "cx",
    "cy",
    "k1",
    "k2",
    "p1",
    "p2",
    "k3",
    "k4",
    "k5",
    "k6",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize per-zoom mrcal cameramodel outputs into JSON and CSV."
    )
    parser.add_argument(
        "--dataset-root",
        required=True,
        help="Path to one dated calibration session, for example datasets/camera_calibration/2026-04-15",
    )
    return parser.parse_args()


def load_cameramodel(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    cleaned_lines = []
    for line in text.splitlines():
        if line.lstrip().startswith("#"):
            continue
        cleaned_lines.append(line)
    payload = "\n".join(cleaned_lines).strip()
    return ast.literal_eval(payload)


def load_optional_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def first_cameramodel(calibration_dir: Path) -> Path:
    candidates = sorted(calibration_dir.glob("*.cameramodel"))
    if not candidates:
        raise FileNotFoundError(f"no .cameramodel found in {calibration_dir}")
    return candidates[0]


def count_images(images_dir: Path) -> int:
    patterns = ("*.jpg", "*.jpeg", "*.png", "*.bmp", "*.tif", "*.tiff")
    count = 0
    for pattern in patterns:
        count += len(list(images_dir.glob(pattern)))
    return count


def count_corner_observations(corners_path: Path) -> int | None:
    if not corners_path.exists():
        return None

    filenames = set()
    for raw_line in corners_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) < 3:
            continue
        if fields[1] == "-" or fields[2] == "-":
            continue
        filenames.add(fields[0])
    return len(filenames)


def intrinsic_value(values: list[float], index: int) -> float | None:
    if index >= len(values):
        return None
    return values[index]


def intrinsic_std(metrics: dict[str, Any], name: str) -> float | None:
    stddev = metrics.get("parameter_stddev", {})
    return stddev.get(name)


def build_row(zoom_dir: Path) -> dict[str, Any]:
    calibration_dir = zoom_dir / "calibration"
    images_dir = zoom_dir / "images"
    analysis_dir = zoom_dir / "analysis"
    metrics_path = zoom_dir / "metrics.json"

    cameramodel_path = first_cameramodel(calibration_dir)
    model = load_cameramodel(cameramodel_path)
    metrics = load_optional_json(metrics_path)

    intrinsics = model.get("intrinsics", [])
    imagersize = model.get("imagersize", [None, None])

    row: dict[str, Any] = {
        "zoom_level": zoom_dir.name,
        "lensmodel": model.get("lensmodel"),
        "model_path": str(cameramodel_path),
        "image_width": imagersize[0] if len(imagersize) > 0 else None,
        "image_height": imagersize[1] if len(imagersize) > 1 else None,
        "num_images": count_images(images_dir),
        "num_detected_corner_frames": count_corner_observations(analysis_dir / "corners.vnl"),
        "rms_reprojection_error_px": metrics.get("rms_reprojection_error_px"),
        "worst_reprojection_error_px": metrics.get("worst_reprojection_error_px"),
        "max_projection_uncertainty_px": metrics.get("max_projection_uncertainty_px"),
    }

    for index, name in enumerate(INTRINSIC_NAMES):
        row[name] = intrinsic_value(intrinsics, index)
        row[f"{name}_std"] = intrinsic_std(metrics, name)

    return row


def write_summary_json(path: Path, row: dict[str, Any]) -> None:
    payload = {
        "zoom_level": row["zoom_level"],
        "lensmodel": row["lensmodel"],
        "model_path": row["model_path"],
        "image_size": {
            "width": row["image_width"],
            "height": row["image_height"],
        },
        "num_images": row["num_images"],
        "num_detected_corner_frames": row["num_detected_corner_frames"],
        "rms_reprojection_error_px": row["rms_reprojection_error_px"],
        "worst_reprojection_error_px": row["worst_reprojection_error_px"],
        "max_projection_uncertainty_px": row["max_projection_uncertainty_px"],
        "intrinsics": {name: row[name] for name in INTRINSIC_NAMES},
        "intrinsics_stddev": {name: row[f"{name}_std"] for name in INTRINSIC_NAMES},
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def write_aggregate_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "zoom_level",
        "lensmodel",
        "model_path",
        "image_width",
        "image_height",
        "num_images",
        "num_detected_corner_frames",
        "rms_reprojection_error_px",
        "worst_reprojection_error_px",
        "max_projection_uncertainty_px",
    ]
    for name in INTRINSIC_NAMES:
        fieldnames.append(name)
        fieldnames.append(f"{name}_std")

    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    dataset_root = Path(args.dataset_root)

    if not dataset_root.is_dir():
        raise FileNotFoundError(f"dataset root not found: {dataset_root}")

    zoom_dirs = sorted(
        path for path in dataset_root.iterdir() if path.is_dir() and path.name.endswith("x")
    )
    if not zoom_dirs:
        raise FileNotFoundError(f"no zoom directories found in {dataset_root}")

    rows = []
    for zoom_dir in zoom_dirs:
        row = build_row(zoom_dir)
        rows.append(row)
        write_summary_json(zoom_dir / "intrinsics_summary.json", row)

    aggregate_csv = dataset_root / "intrinsics_summary.csv"
    write_aggregate_csv(aggregate_csv, rows)

    print(f"Wrote {aggregate_csv}")
    for zoom_dir in zoom_dirs:
        print(f"Wrote {zoom_dir / 'intrinsics_summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
