from __future__ import annotations

import csv
import os
import re
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".mplconfig"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DATA_DIR = Path(__file__).resolve().parent

# Two circular coils: 10 turns each, 126.5 mm mean radius, 125 mm spacing.
COIL_TURNS = 10
COIL_RADIUS_M = 0.1265
COIL_SPACING_M = 0.125
MU0 = 4.0 * np.pi * 1e-7
COIL_UT_PER_A = (
    MU0
    * COIL_TURNS
    * COIL_RADIUS_M**2
    / (COIL_RADIUS_M**2 + (COIL_SPACING_M / 2.0) ** 2) ** 1.5
    * 1e6
)

# HMC1001/HMC1002 nominal sensitivity: 3.2 mV/V/G.
VBRIDGE_V = 6.0
EXPECTED_SENSOR_MV_PER_UT = 3.2 * VBRIDGE_V / 100.0

# INA851 analog frontend gain, set by a 48.7 ohm gain resistor.
EXPECTED_ANALOG_GAIN: float | None = 124.2
EXPECTED_TOTAL_MV_PER_UT = (
    EXPECTED_SENSOR_MV_PER_UT * EXPECTED_ANALOG_GAIN
    if EXPECTED_ANALOG_GAIN is not None
    else None
)


def current_a_from_filename(path: Path) -> float:
    match = re.fullmatch(r"([+-]?\d+)mA\.csv", path.name)
    if not match:
        raise ValueError(f"Cannot read current from filename: {path.name}")
    return float(match.group(1)) / 1000.0


def read_channel_stats(path: Path) -> dict[str, float]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        channel_names = [
            name for name in (reader.fieldnames or []) if re.fullmatch(r"ch\d+_mv", name)
        ]
        values = {name: [] for name in channel_names}

        for row in reader:
            for name in channel_names:
                values[name].append(float(row[name]))

    stats: dict[str, float] = {}
    for name, samples in values.items():
        arr = np.asarray(samples, dtype=float)
        stats[f"{name}_mean"] = float(np.mean(arr))
        stats[f"{name}_std"] = float(np.std(arr))
    return stats


def main() -> None:
    rows = []
    for path in sorted(DATA_DIR.glob("*mA.csv"), key=current_a_from_filename):
        current_a = current_a_from_filename(path)
        rows.append(
            {
                "file": path.name,
                "current_a": current_a,
                "field_ut": current_a * COIL_UT_PER_A,
                **read_channel_stats(path),
            }
        )

    if not rows:
        raise RuntimeError(f"No calibration CSV files found in {DATA_DIR}")

    all_channel_names = sorted(
        {
            key.removesuffix("_mean")
            for row in rows
            for key in row
            if re.fullmatch(r"ch\d+_mv_mean", key)
        },
        key=lambda name: int(name[2:-3]),
    )

    out_csv = DATA_DIR / "z_axis_calibration_summary.csv"
    fieldnames = ["file", "current_a", "field_ut"]
    for channel in all_channel_names:
        fieldnames.extend([f"{channel}_mean", f"{channel}_std"])

    with out_csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    fields = np.asarray([row["field_ut"] for row in rows], dtype=float)

    channel = "ch0_mv"
    means = np.asarray([row[f"{channel}_mean"] for row in rows], dtype=float)
    zero_row = min(rows, key=lambda row: abs(row["current_a"]))
    zero_mean = float(zero_row[f"{channel}_mean"])
    deltas = means - zero_mean

    slope, intercept = np.polyfit(fields, deltas, 1)
    fit = slope * fields + intercept

    plt.figure(figsize=(8, 5))
    plt.plot(fields, deltas, "o", color="#1f77b4", label="Moyenne mesuree CH0")
    plt.plot(fields, fit, "-", color="#d62728", label=f"Regression lineaire : {slope:.3f} mV/uT")
    if EXPECTED_TOTAL_MV_PER_UT is not None:
        expected = EXPECTED_TOTAL_MV_PER_UT * fields
        plt.plot(
            fields,
            expected,
            "--",
            color="#2ca02c",
            label=f"Valeur attendue : {EXPECTED_TOTAL_MV_PER_UT:.1f} mV/uT",
        )

    plt.axhline(0, color="0.75", linewidth=1)
    plt.axvline(0, color="0.75", linewidth=1)
    plt.title("Calibration de sensibilite de l'axe Z - CH0")
    plt.xlabel("Champ applique par les bobines de Helmholtz selon Z (uT)")
    plt.ylabel("Valeur mesuree - champ ambiant (mV)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    out_png = DATA_DIR / "z_axis_ch0_calibration.png"
    plt.savefig(out_png, dpi=160)

    print(f"Wrote {out_csv}")
    print(f"Wrote {out_png}")
    print(f"CH0 sensitivity: {slope:.6f} mV/uT")
    print(f"CH0 scale: {1.0 / slope:.6f} uT/mV")
    print(f"HMC100x nominal bridge sensitivity at {VBRIDGE_V:g} V: {EXPECTED_SENSOR_MV_PER_UT:.6f} mV/uT")
    if EXPECTED_TOTAL_MV_PER_UT is not None:
        print(f"Expected ADC-side sensitivity: {EXPECTED_TOTAL_MV_PER_UT:.6f} mV/uT")


if __name__ == "__main__":
    main()
