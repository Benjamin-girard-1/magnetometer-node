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


def parse_filename(path: Path) -> tuple[float, str]:
    match = re.fullmatch(r"([+-]?\d+)mA_(SR|R)\.csv", path.name)
    if not match:
        raise ValueError(f"Cannot read current and state from filename: {path.name}")
    current_a = float(match.group(1)) / 1000.0
    state = match.group(2)
    return current_a, state


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


def fit_state(rows: list[dict[str, float | str]], state: str) -> tuple[np.ndarray, np.ndarray, float, float]:
    state_rows = [row for row in rows if row["state"] == state]
    state_rows.sort(key=lambda row: float(row["current_a"]))

    fields = np.asarray([float(row["field_ut"]) for row in state_rows], dtype=float)
    means = np.asarray([float(row["ch1_mv_mean"]) for row in state_rows], dtype=float)
    zero_row = min(state_rows, key=lambda row: abs(float(row["current_a"])))
    zero_mean = float(zero_row["ch1_mv_mean"])
    deltas = means - zero_mean
    slope, intercept = np.polyfit(fields, deltas, 1)
    return fields, deltas, float(slope), float(intercept)


def main() -> None:
    rows: list[dict[str, float | str]] = []
    csv_paths = sorted(DATA_DIR.glob("*mA_*.csv"), key=lambda path: parse_filename(path))

    for path in csv_paths:
        current_a, state = parse_filename(path)
        rows.append(
            {
                "file": path.name,
                "state": state,
                "current_a": current_a,
                "field_ut": current_a * COIL_UT_PER_A,
                **read_channel_stats(path),
            }
        )

    if not rows:
        raise RuntimeError(f"No Y-axis calibration CSV files found in {DATA_DIR}")

    all_channel_names = sorted(
        {
            key.removesuffix("_mean")
            for row in rows
            for key in row
            if re.fullmatch(r"ch\d+_mv_mean", key)
        },
        key=lambda name: int(name[2:-3]),
    )

    out_csv = DATA_DIR / "y_axis_calibration_summary.csv"
    fieldnames = ["file", "state", "current_a", "field_ut"]
    for channel in all_channel_names:
        fieldnames.extend([f"{channel}_mean", f"{channel}_std"])

    with out_csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    sr_fields, sr_deltas, sr_slope, sr_intercept = fit_state(rows, "SR")
    r_fields, r_deltas, r_slope, r_intercept = fit_state(rows, "R")

    fit_fields = np.linspace(
        min(float(np.min(sr_fields)), float(np.min(r_fields))),
        max(float(np.max(sr_fields)), float(np.max(r_fields))),
        100,
    )

    plt.figure(figsize=(8.5, 5.5))
    plt.plot(sr_fields, sr_deltas, "o", color="#1f77b4", label="SR mesure")
    plt.plot(
        fit_fields,
        sr_slope * fit_fields + sr_intercept,
        "-",
        color="#1f77b4",
        label=f"SR regression : {sr_slope:.2f} mV/uT",
    )

    plt.plot(r_fields, r_deltas, "s", color="#d62728", label="R mesure")
    plt.plot(
        fit_fields,
        r_slope * fit_fields + r_intercept,
        "-",
        color="#d62728",
        label=f"R regression : {r_slope:.2f} mV/uT",
    )

    if EXPECTED_TOTAL_MV_PER_UT is not None:
        plt.plot(
            fit_fields,
            EXPECTED_TOTAL_MV_PER_UT * fit_fields,
            "--",
            color="#2ca02c",
            label=f"Attendu SR : {EXPECTED_TOTAL_MV_PER_UT:.1f} mV/uT",
        )
        plt.plot(
            fit_fields,
            -EXPECTED_TOTAL_MV_PER_UT * fit_fields,
            "--",
            color="#9467bd",
            label=f"Attendu R : {-EXPECTED_TOTAL_MV_PER_UT:.1f} mV/uT",
        )

    plt.axhline(0, color="0.75", linewidth=1)
    plt.axvline(0, color="0.75", linewidth=1)
    plt.title("Calibration de sensibilite de l'axe Y - CH1")
    plt.xlabel("Champ applique par les bobines de Helmholtz selon Y (uT)")
    plt.ylabel("Valeur mesuree - champ ambiant (mV)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    out_png = DATA_DIR / "y_axis_ch1_calibration.png"
    plt.savefig(out_png, dpi=160)

    print(f"Wrote {out_csv}")
    print(f"Wrote {out_png}")
    print(f"CH1 SR sensitivity: {sr_slope:.6f} mV/uT")
    print(f"CH1 R sensitivity: {r_slope:.6f} mV/uT")
    print(f"HMC100x nominal bridge sensitivity at {VBRIDGE_V:g} V: {EXPECTED_SENSOR_MV_PER_UT:.6f} mV/uT")
    if EXPECTED_TOTAL_MV_PER_UT is not None:
        print(f"Expected ADC-side sensitivity: +/-{EXPECTED_TOTAL_MV_PER_UT:.6f} mV/uT")


if __name__ == "__main__":
    main()
