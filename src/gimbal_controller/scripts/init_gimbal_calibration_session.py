#!/usr/bin/env python3

import argparse
import json
from datetime import date
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a gimbal calibration session tree for ticket 026."
    )
    parser.add_argument(
        "--dataset-root",
        default="datasets/gimbal_calibration",
        help="Root directory holding all gimbal calibration sessions.",
    )
    parser.add_argument(
        "--date",
        default=date.today().isoformat(),
        help="Session date in YYYY-MM-DD format.",
    )
    parser.add_argument(
        "--session-name",
        default="bench",
        help="Human-readable suffix for the session.",
    )
    return parser.parse_args()


def normalize_session_name(raw_name: str) -> str:
    cleaned = raw_name.strip().lower().replace(" ", "_")
    return cleaned or "bench"


def write_text_if_missing(path: Path, content: str) -> None:
    if not path.exists():
        path.write_text(content, encoding="utf-8")


def main() -> int:
    args = parse_args()
    session_name = f"{args.date}_{normalize_session_name(args.session_name)}"
    session_root = Path(args.dataset_root) / session_name
    for relpath in ("bag",):
        (session_root / relpath).mkdir(parents=True, exist_ok=True)

    write_text_if_missing(
        session_root / "notes.md",
        "\n".join(
            (
                f"# {session_name}",
                "",
                "- Gimbal hardware:",
                "- Camera stream topic:",
                "- Checkerboard rows x cols:",
                "- Checkerboard square size (m):",
                "- Operator notes:",
                "",
            )
        ),
    )

    manifest = {
        "ticket": "026-gimbal-calibration",
        "session_name": session_name,
        "session_date": args.date,
        "relative_root": str(session_root),
        "artifacts": {
            "bag_dir": str(session_root / "bag"),
            "csv": str(session_root / "samples.csv"),
            "summary_json": str(session_root / "summary.json"),
            "notes": str(session_root / "notes.md"),
        },
    }
    (session_root / "session_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n",
        encoding="utf-8",
    )

    print(f"Initialized gimbal calibration session at {session_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
