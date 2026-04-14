#!/usr/bin/env python3

import argparse
import json
from datetime import date
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a camera calibration session tree for ticket 028."
    )
    parser.add_argument(
        "--dataset-root",
        default="datasets/camera_calibration",
        help="Root directory holding all calibration sessions.",
    )
    parser.add_argument(
        "--date",
        default=date.today().isoformat(),
        help="Session date in YYYY-MM-DD format.",
    )
    parser.add_argument(
        "--zoom-levels",
        nargs="+",
        required=True,
        help="Discrete zoom levels, for example: 1 2 4",
    )
    return parser.parse_args()


def normalize_zoom_level(raw_zoom: str) -> str:
    zoom = raw_zoom.strip().lower()
    if zoom.endswith("x"):
        zoom = zoom[:-1]
    if not zoom:
        raise ValueError("zoom level must not be empty")
    return f"{zoom}x"


def write_text_if_missing(path: Path, content: str) -> None:
    if not path.exists():
        path.write_text(content, encoding="utf-8")


def main() -> int:
    args = parse_args()
    session_root = Path(args.dataset_root) / args.date
    session_root.mkdir(parents=True, exist_ok=True)

    zoom_dirs = []
    for raw_zoom in args.zoom_levels:
        zoom_name = normalize_zoom_level(raw_zoom)
        zoom_root = session_root / zoom_name
        for relpath in ("images", "analysis", "calibration", "logs"):
            (zoom_root / relpath).mkdir(parents=True, exist_ok=True)

        write_text_if_missing(
            zoom_root / "notes.md",
            "\n".join(
                (
                    f"# {zoom_name} capture notes",
                    "",
                    "- Camera:",
                    "- Resolution:",
                    "- Focus setting:",
                    "- Board size:",
                    "- Board square size (m):",
                    "- Accepted frames:",
                    "- Rejected frames:",
                    "- Notes:",
                    "",
                )
            ),
        )

        write_text_if_missing(
            zoom_root / "metrics.json",
            json.dumps(
                {
                    "rms_reprojection_error_px": None,
                    "worst_reprojection_error_px": None,
                    "max_projection_uncertainty_px": None,
                    "parameter_stddev": {
                        "fx": None,
                        "fy": None,
                        "cx": None,
                        "cy": None,
                        "k1": None,
                        "k2": None,
                        "p1": None,
                        "p2": None,
                        "k3": None,
                        "k4": None,
                        "k5": None,
                        "k6": None,
                    },
                },
                indent=2,
            )
            + "\n",
        )
        zoom_dirs.append(zoom_name)

    manifest_path = session_root / "session_manifest.json"
    manifest = {
        "ticket": "028-camera-intrinsic-calibration-mrcal",
        "session_date": args.date,
        "zoom_levels": zoom_dirs,
        "relative_root": str(session_root),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    print(f"Initialized calibration session at {session_root}")
    for zoom_name in zoom_dirs:
        print(f"  - {zoom_name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
