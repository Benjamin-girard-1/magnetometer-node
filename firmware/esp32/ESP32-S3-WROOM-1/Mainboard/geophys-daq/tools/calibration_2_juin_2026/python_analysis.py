from __future__ import annotations

import csv
import math
import re
from pathlib import Path

import numpy as np


DATA_DIR = Path(__file__).resolve().parent

# Helmholtz pair: B = (4/5)^(3/2) * mu0 * N * I / R.
COIL_TURNS = 10
COIL_DIAMETER_M = 0.250
COIL_RADIUS_M = COIL_DIAMETER_M / 2.0
MU0 = 4.0 * math.pi * 1e-7
UT_PER_A = (4.0 / 5.0) ** 1.5 * MU0 * COIL_TURNS / COIL_RADIUS_M * 1e6

# HMC100x typical bridge sensitivity is 1 mV/V/G. At 5 V bridge:
# 5 mV/G = 0.05 mV/uT because 1 G = 100 uT.
VBRIDGE_V = 5.0
HMC_MV_PER_UT = VBRIDGE_V * 1.0 / 100.0
EXPECTED_BARE_MV_PER_A = UT_PER_A * HMC_MV_PER_UT


def current_from_name(path: Path) -> float | None:
    match = re.search(r"5Vbridge-(neg)?([0-9]+)(mA|A)_Gain1", path.name)
    if not match:
        return None

    sign = -1.0 if match.group(1) else 1.0
    value = float(match.group(2))
    if match.group(3) == "mA":
        value /= 1000.0
    return sign * value


def load_ch0_mv(path: Path) -> np.ndarray | None:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if "ch0_mv" not in (reader.fieldnames or []):
            return None
        return np.array([float(row["ch0_mv"]) for row in reader], dtype=np.float64)


def write_svg_plot(
    path: Path,
    currents: np.ndarray,
    deltas: np.ndarray,
    fit_slope: float,
    fit_intercept: float,
) -> None:
    width, height = 900, 560
    left, right, top, bottom = 90, 30, 40, 80
    plot_w = width - left - right
    plot_h = height - top - bottom

    x_min, x_max = float(np.min(currents)), float(np.max(currents))
    y_fit_values = fit_slope * currents + fit_intercept
    y_bare_values = EXPECTED_BARE_MV_PER_A * currents
    y_min = float(min(np.min(deltas), np.min(y_fit_values), np.min(y_bare_values)))
    y_max = float(max(np.max(deltas), np.max(y_fit_values), np.max(y_bare_values)))
    y_pad = (y_max - y_min) * 0.08
    y_min -= y_pad
    y_max += y_pad

    def sx(x: float) -> float:
        return left + (x - x_min) / (x_max - x_min) * plot_w

    def sy(y: float) -> float:
        return top + (y_max - y) / (y_max - y_min) * plot_h

    fit_x = np.linspace(x_min, x_max, 80)
    fit_points = " ".join(f"{sx(x):.1f},{sy(fit_slope * x + fit_intercept):.1f}" for x in fit_x)
    bare_points = " ".join(f"{sx(x):.1f},{sy(EXPECTED_BARE_MV_PER_A * x):.1f}" for x in fit_x)

    x_ticks = np.linspace(x_min, x_max, 8)
    y_ticks = np.linspace(y_min, y_max, 8)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width / 2}" y="24" text-anchor="middle" font-family="Arial" font-size="18">CH0 calibration vs 10-turn, 250 mm Helmholtz coil</text>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{height - bottom}" stroke="#333"/>',
        f'<line x1="{left}" y1="{height - bottom}" x2="{width - right}" y2="{height - bottom}" stroke="#333"/>',
    ]
    for tick in x_ticks:
        x = sx(float(tick))
        parts.append(f'<line x1="{x:.1f}" y1="{height - bottom}" x2="{x:.1f}" y2="{height - bottom + 6}" stroke="#333"/>')
        parts.append(f'<text x="{x:.1f}" y="{height - bottom + 24}" text-anchor="middle" font-family="Arial" font-size="12">{tick:.2f}</text>')
    for tick in y_ticks:
        y = sy(float(tick))
        parts.append(f'<line x1="{left - 6}" y1="{y:.1f}" x2="{left}" y2="{y:.1f}" stroke="#333"/>')
        parts.append(f'<line x1="{left}" y1="{y:.1f}" x2="{width - right}" y2="{y:.1f}" stroke="#ddd"/>')
        parts.append(f'<text x="{left - 10}" y="{y + 4:.1f}" text-anchor="end" font-family="Arial" font-size="12">{tick:.0f}</text>')
    parts.append(f'<polyline points="{fit_points}" fill="none" stroke="#1f77b4" stroke-width="3"/>')
    parts.append(f'<polyline points="{bare_points}" fill="none" stroke="#d62728" stroke-width="2" stroke-dasharray="7 5"/>')
    for x, y in zip(currents, deltas):
        parts.append(f'<circle cx="{sx(float(x)):.1f}" cy="{sy(float(y)):.1f}" r="4" fill="#111"/>')
    parts.extend(
        [
            f'<text x="{width / 2}" y="{height - 28}" text-anchor="middle" font-family="Arial" font-size="14">Coil current (A)</text>',
            f'<text x="22" y="{height / 2}" text-anchor="middle" font-family="Arial" font-size="14" transform="rotate(-90 22 {height / 2})">CH0 delta from 0 A (mV)</text>',
            f'<line x1="{width - 330}" y1="58" x2="{width - 290}" y2="58" stroke="#1f77b4" stroke-width="3"/>',
            f'<text x="{width - 280}" y="63" font-family="Arial" font-size="13">CH0 linear fit</text>',
            f'<line x1="{width - 330}" y1="82" x2="{width - 290}" y2="82" stroke="#d62728" stroke-width="2" stroke-dasharray="7 5"/>',
            f'<text x="{width - 280}" y="87" font-family="Arial" font-size="13">Bare HMC bridge expectation</text>',
            "</svg>",
        ]
    )
    path.write_text("\n".join(parts))


def main() -> None:
    rows = []
    zero_mean_mv = None

    for path in sorted(DATA_DIR.glob("5Vbridge-*Gain1.csv")):
        current_a = current_from_name(path)
        if current_a is None:
            continue

        values = load_ch0_mv(path)
        if values is None or len(values) == 0:
            rows.append(
                {
                    "file": path.name,
                    "current_a": current_a,
                    "samples": 0,
                    "mean_mv": np.nan,
                    "std_mv": np.nan,
                    "min_mv": np.nan,
                    "max_mv": np.nan,
                }
            )
            continue

        mean_mv = float(np.mean(values))
        if current_a == 0.0:
            zero_mean_mv = mean_mv

        rows.append(
            {
                "file": path.name,
                "current_a": current_a,
                "samples": len(values),
                "mean_mv": mean_mv,
                "std_mv": float(np.std(values)),
                "min_mv": float(np.min(values)),
                "max_mv": float(np.max(values)),
            }
        )

    if zero_mean_mv is None:
        raise RuntimeError("No 0 A calibration file found.")

    valid_rows = [row for row in rows if row["samples"] > 0]
    currents = np.array([row["current_a"] for row in valid_rows], dtype=np.float64)
    deltas = np.array([row["mean_mv"] - zero_mean_mv for row in valid_rows], dtype=np.float64)

    fit_slope_mv_per_a, fit_intercept_mv = np.polyfit(currents, deltas, 1)
    effective_gain = fit_slope_mv_per_a / EXPECTED_BARE_MV_PER_A
    fitted_ut_per_mv = UT_PER_A / fit_slope_mv_per_a

    out_csv = DATA_DIR / "ch0_helmholtz_comparison.csv"
    with out_csv.open("w", newline="") as handle:
        fieldnames = [
            "file",
            "current_a",
            "samples",
            "mean_mv",
            "std_mv",
            "delta_from_0a_mv",
            "expected_field_ut",
            "expected_bare_hmc_delta_mv",
            "measured_over_bare_expected",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in sorted(rows, key=lambda item: item["current_a"]):
            expected_field_ut = row["current_a"] * UT_PER_A
            expected_delta_mv = expected_field_ut * HMC_MV_PER_UT
            delta_mv = row["mean_mv"] - zero_mean_mv if row["samples"] > 0 else np.nan
            writer.writerow(
                {
                    "file": row["file"],
                    "current_a": row["current_a"],
                    "samples": row["samples"],
                    "mean_mv": row["mean_mv"],
                    "std_mv": row["std_mv"],
                    "delta_from_0a_mv": delta_mv,
                    "expected_field_ut": expected_field_ut,
                    "expected_bare_hmc_delta_mv": expected_delta_mv,
                    "measured_over_bare_expected": (
                        delta_mv / expected_delta_mv if expected_delta_mv else np.nan
                    ),
                }
            )

    print(f"Helmholtz field: {UT_PER_A:.6f} uT/A")
    print(f"Bare HMC bridge output at {VBRIDGE_V:g} V: {EXPECTED_BARE_MV_PER_A:.6f} mV/A")
    print(f"CH0 fitted slope: {fit_slope_mv_per_a:.6f} mV/A")
    print(f"CH0 fitted intercept after 0 A subtraction: {fit_intercept_mv:.6f} mV")
    print(f"Effective analog gain vs bare HMC bridge: {effective_gain:.3f}x")
    print(f"Effective calibrated scale: {fitted_ut_per_mv:.6f} uT/mV")
    print(f"Wrote {out_csv}")

    out_svg = DATA_DIR / "ch0_helmholtz_comparison.svg"
    write_svg_plot(out_svg, currents, deltas, fit_slope_mv_per_a, fit_intercept_mv)
    print(f"Wrote {out_svg}")


if __name__ == "__main__":
    main()
