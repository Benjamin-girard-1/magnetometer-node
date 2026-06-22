#!/usr/bin/env python3
"""Analyze servo settling and waveform preservation from a transient CSV.

The script compares a measured signal column against a reference waveform
column exported from KiCad/ngspice. It ignores absolute gain and offset by
fitting signal ~= offset + scale * reference over the final comparison window.
"""

from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure 95/99% settling and correlation against a reference waveform."
    )
    parser.add_argument("csv", type=Path, help="Transient CSV exported from KiCad/ngspice.")
    parser.add_argument(
        "--time-column",
        default=None,
        help="Time column name. Defaults to the first numeric monotonic column.",
    )
    parser.add_argument(
        "--signal-column",
        default="V(/ADC)",
        help='Signal column to analyze, for example "V(/ADC)".',
    )
    parser.add_argument(
        "--reference-column",
        default="V(/FIELD)",
        help='Reference waveform column, for example "V(/FIELD)".',
    )
    parser.add_argument(
        "--window-start",
        type=float,
        default=None,
        help="Start time in seconds for comparison. Defaults to final --final-seconds.",
    )
    parser.add_argument(
        "--window-end",
        type=float,
        default=None,
        help="End time in seconds for comparison. Defaults to final sample.",
    )
    parser.add_argument(
        "--final-seconds",
        type=float,
        default=10.0,
        help="Number of final seconds used when --window-start is omitted.",
    )
    parser.add_argument(
        "--settle-levels",
        default="0.95,0.99",
        help="Comma-separated settling levels to report, e.g. 0.95,0.99.",
    )
    parser.add_argument(
        "--average-fraction",
        type=float,
        default=0.05,
        help="Fraction of the smoothed trace used to estimate the final offset value.",
    )
    parser.add_argument(
        "--settle-average-seconds",
        type=float,
        default=2.0,
        help="Centered moving-average width used for 95/99% settling.",
    )
    parser.add_argument(
        "--details",
        action="store_true",
        help="Print normalized RMSE, fitted amplitude, and fitted offset.",
    )
    return parser.parse_args()


def sniff_dialect(path: Path) -> csv.Dialect:
    sample = path.read_text(encoding="utf-8-sig", errors="replace")[:4096]
    try:
        return csv.Sniffer().sniff(sample, delimiters=",;\t")
    except csv.Error:
        class Semi(csv.Dialect):
            delimiter = ";"
            quotechar = '"'
            escapechar = None
            doublequote = True
            skipinitialspace = False
            lineterminator = "\n"
            quoting = csv.QUOTE_MINIMAL

        return Semi


def load_csv(path: Path) -> tuple[list[str], list[dict[str, float]]]:
    dialect = sniff_dialect(path)
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle, dialect=dialect)
        if reader.fieldnames is None:
            raise SystemExit("CSV has no header row.")
        headers = [h.strip() for h in reader.fieldnames if h and h.strip()]
        rows: list[dict[str, float]] = []
        for raw in reader:
            row: dict[str, float] = {}
            for key in headers:
                value = (raw.get(key) or "").strip()
                if not value:
                    continue
                try:
                    row[key] = float(value)
                except ValueError:
                    pass
            if row:
                rows.append(row)
    return headers, rows


def choose_time_column(headers: list[str], rows: list[dict[str, float]], explicit: str | None) -> str:
    if explicit:
        return explicit
    for header in headers:
        values = [row[header] for row in rows if header in row]
        if len(values) >= 3 and all(b >= a for a, b in zip(values, values[1:])):
            return header
    raise SystemExit("Could not infer time column; use --time-column.")


def mean(values: list[float]) -> float:
    return sum(values) / len(values)


def pearson(x: list[float], y: list[float]) -> float:
    if len(x) < 3:
        return float("nan")
    mx = mean(x)
    my = mean(y)
    dx = [v - mx for v in x]
    dy = [v - my for v in y]
    sx = sum(v * v for v in dx) ** 0.5
    sy = sum(v * v for v in dy) ** 0.5
    if sx == 0 or sy == 0:
        return float("nan")
    return sum(a * b for a, b in zip(dx, dy)) / (sx * sy)


def linear_fit_metrics(signal: list[float], reference: list[float]) -> dict[str, float]:
    ref_mean = mean(reference)
    sig_mean = mean(signal)
    ref_var = sum((v - ref_mean) ** 2 for v in reference)
    if ref_var == 0:
        raise SystemExit("Reference waveform is constant in the comparison window.")

    scale = sum((s - sig_mean) * (r - ref_mean) for s, r in zip(signal, reference)) / ref_var
    offset = sig_mean - scale * ref_mean
    residual = [s - (offset + scale * r) for s, r in zip(signal, reference)]
    rms = mean([v * v for v in residual]) ** 0.5
    sig_rms = statistics.pstdev(signal)
    return {
        "correlation": abs(pearson(signal, reference)),
        "normalized_rmse": rms / sig_rms if sig_rms else float("nan"),
        "fit_offset": offset,
        "fit_amplitude": abs(scale),
        "fit_peak_to_peak": 2.0 * abs(scale) * ((max(reference) - min(reference)) / 2.0),
        "fit_scale": scale,
    }


def moving_average(
    t: list[float],
    y: list[float],
    width_seconds: float,
) -> tuple[list[float], list[float]]:
    if width_seconds <= 0:
        return t, y

    half_width = width_seconds / 2.0
    prefix = [0.0]
    for value in y:
        prefix.append(prefix[-1] + value)

    out_t: list[float] = []
    out_y: list[float] = []
    left = 0
    right = 0
    n = len(t)
    for i, ti in enumerate(t):
        start = ti - half_width
        end = ti + half_width
        if start < t[0] or end > t[-1]:
            continue
        while left < n and t[left] < start:
            left += 1
        while right < n and t[right] <= end:
            right += 1
        count = right - left
        if count > 0:
            out_t.append(ti)
            out_y.append((prefix[right] - prefix[left]) / count)

    if len(out_y) < 3:
        raise SystemExit("Not enough samples after moving-average settling filter.")
    return out_t, out_y


def settling_times_to_final_value(
    t: list[float],
    y: list[float],
    levels: list[float],
    average_fraction: float,
) -> dict[float, float | None]:
    n = len(y)
    edge_n = max(1, min(n // 2, int(round(n * average_fraction))))
    final = mean(y[-edge_n:])
    errors = [abs(value - final) for value in y]
    span = max(errors)
    if span == 0:
        return {level: t[0] for level in levels}

    out: dict[float, float | None] = {}
    for level in levels:
        tolerance = (1.0 - level) * span
        last_outside = None
        for i, error in enumerate(errors):
            if error > tolerance:
                last_outside = i
        if last_outside is None:
            out[level] = t[0]
        elif last_outside + 1 < n:
            out[level] = t[last_outside + 1]
        else:
            out[level] = None
    return out


def window_samples(
    t: list[float],
    signal: list[float],
    reference: list[float],
    start: float,
    end: float,
) -> tuple[list[float], list[float]]:
    values = [(s, r) for ti, s, r in zip(t, signal, reference) if start <= ti <= end]
    if len(values) < 3:
        raise SystemExit("Not enough samples in the requested comparison window.")
    return [item[0] for item in values], [item[1] for item in values]


def main() -> None:
    args = parse_args()
    headers, rows = load_csv(args.csv)
    time_column = choose_time_column(headers, rows, args.time_column)

    for column_name, label in (
        (args.signal_column, "Signal"),
        (args.reference_column, "Reference"),
    ):
        if column_name not in headers:
            available = "\n  ".join(headers)
            raise SystemExit(f'{label} column "{column_name}" not found. Available columns:\n  {available}')

    samples = [
        (row[time_column], row[args.signal_column], row[args.reference_column])
        for row in rows
        if time_column in row and args.signal_column in row and args.reference_column in row
    ]
    if len(samples) < 3:
        raise SystemExit("Not enough numeric samples.")
    samples.sort()

    t = [sample[0] for sample in samples]
    signal = [sample[1] for sample in samples]
    reference = [sample[2] for sample in samples]
    levels = [float(item) for item in args.settle_levels.split(",") if item.strip()]

    window_end = args.window_end if args.window_end is not None else t[-1]
    window_start = (
        args.window_start
        if args.window_start is not None
        else max(t[0], window_end - args.final_seconds)
    )

    fit_signal, fit_reference = window_samples(t, signal, reference, window_start, window_end)
    metrics = linear_fit_metrics(fit_signal, fit_reference)
    settle_t, settle_signal = moving_average(t, signal, args.settle_average_seconds)
    settle = settling_times_to_final_value(settle_t, settle_signal, levels, args.average_fraction)

    print(f"file: {args.csv}")
    print(f"time column: {time_column}")
    print(f"signal column: {args.signal_column}")
    print(f"reference column: {args.reference_column}")
    print(f"samples: {len(samples)}")
    print(f"settling average: {args.settle_average_seconds:.6g} s")
    for level in levels:
        value = settle[level]
        label = f"{int(level * 100)}%"
        print(f"settling time {label}: {'not settled' if value is None else f'{value:.6g} s'}")
    print(f"comparison window: {window_start:.6g} s to {window_end:.6g} s")
    print(f"waveform correlation: {metrics['correlation']:.6f}")
    if args.details:
        print(f"waveform normalized RMSE: {metrics['normalized_rmse']:.6f}")
        print(f"fitted amplitude: {metrics['fit_amplitude']:.6g}")
        print(f"fitted peak-to-peak: {metrics['fit_peak_to_peak']:.6g}")
        print(f"fitted offset: {metrics['fit_offset']:.6g}")


if __name__ == "__main__":
    main()
