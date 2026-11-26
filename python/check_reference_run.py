"""Compare a completed paper-pipe run to the bundled pre-refactor CSV."""
import math
from pathlib import Path

from load_virtual_drive import load_run


def main():
    root = Path(__file__).resolve().parents[1]
    expected, headers, _ = load_run(root / "example")
    actual, new_headers, _ = load_run(
        root / "matlab/output/paper_pipe_step_2mm_csvfit"
    )
    if new_headers != headers or len(actual) != len(expected):
        raise ValueError("Reference and current CSV layouts differ")
    peaks = dict.fromkeys(headers, 0.0)
    for old, new in zip(expected, actual):
        for name in headers:
            reference, current = float(old[name]), float(new[name])
            if math.isnan(reference) and math.isnan(current):
                continue
            if not all(map(math.isfinite, (reference, current))):
                raise ValueError(f"Finite-value mask differs for {name}")
            peaks[name] = max(peaks[name], abs(reference - current))
    for name, peak in peaks.items():
        print(f"{name}: max absolute difference {peak:.9g}")
        # Numerical regression budgets by quantity, in the units of each column.
        # Unfiltered contact acceleration is more sensitive at tread edges.
        tolerance = 0.0
        if name.endswith("_mps2"):
            tolerance = 0.0001 if "_filtered_" in name else 0.001
        elif name.endswith("_mm"):
            tolerance = 0.0001
        elif name == "chassis_x_m":
            tolerance = 1e-9
        if peak > tolerance:
            raise ValueError(f"Reference tolerance exceeded for {name}")
    print(f"Checked {len(actual)} samples, {len(headers)} columns")


if __name__ == "__main__":
    main()
