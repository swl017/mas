#!/usr/bin/env python3
"""boot_table.py — quick aligned result table from conductor boot CSVs.

Usage:
  python3 boot_table.py /home/usrg/mas/bag/boot_N2BC_results.csv [...more csvs]
  python3 boot_table.py --data-dir results/ticket010   # all boot_*_results.csv
"""
import argparse
import csv
import glob
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csvs", nargs="*", type=Path)
    ap.add_argument("--data-dir", type=Path)
    a = ap.parse_args()
    files = list(a.csvs)
    if a.data_dir:
        files += [Path(p) for p in sorted(glob.glob(str(a.data_dir / "boot_*_results.csv")))]
    if not files:
        ap.error("no CSVs given")
    for f in files:
        with open(f, newline="") as fh:
            rows = list(csv.DictReader(fh))
        print(f"== {f.name} ({len(rows)} rows) ==")
        for r in sorted(rows, key=lambda r: (r["estimator"], r["geometry"],
                                             r["target_condition"])):
            mr = r.get("min_range_m", "")
            mr = f"{float(mr):8.2f}" if mr else "        "
            print(f"  {r['estimator']:18s} {r['geometry']:11s} "
                  f"{r['target_condition']:22s} {r['result']:13s} {mr}")


if __name__ == "__main__":
    main()
