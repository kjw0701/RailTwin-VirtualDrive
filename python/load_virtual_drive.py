"""Load RailTwin paired synthetic sensors from MATLAB output.

Usage:
    python load_virtual_drive.py ../matlab/output/combined_1mm
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import csv


def load_run(run_folder: str | Path) -> tuple[list[dict], list[str], dict]:
    folder = Path(run_folder)
    with (folder / "paired_sensors.csv").open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        columns = reader.fieldnames or []
        data = list(reader)
    with (folder / "metadata.json").open(encoding="utf-8") as f:
        metadata = json.load(f)
    return data, columns, metadata


if __name__ == "__main__":
    folder = sys.argv[1] if len(sys.argv) > 1 else \
        "../matlab/output/combined_1mm"
    rows, columns, meta = load_run(folder)
    print(f"rows={len(rows)}, columns={len(columns)}")
    print("distance columns:", [c for c in columns if "distance" in c])
    print("acceleration columns:", [c for c in columns if "specific" in c])
    print(rows[:2])
