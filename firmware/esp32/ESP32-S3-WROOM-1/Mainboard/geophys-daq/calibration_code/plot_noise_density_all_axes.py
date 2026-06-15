from __future__ import annotations

import csv
import os
import re
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".mplconfig"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DATA_DIR = Path(__file__).resolve().parent


@dataclass(frozen=True)
class AxisConfig:
    name: str
    folder: Path
    channel: str
    sensitivity_by_state_mv_per_ut: dict[str, float]
    output_name: str


AXES = [
    AxisConfig(
        name="Z",
        folder=DATA_DIR / "z axis",
        channel="ch0_mv",
        sensitivity_by_state_mv_per_ut={"": 21.12783552609885},
        output_name="z_axis_ch0_noise_density_all.png",
    ),
    AxisConfig(
        name="Y",
        folder=DATA_DIR / "y axis",
        channel="ch1_mv",
        sensitivity_by_state_mv_per_ut={
            "SR": 21.40904682731101,
            "R": 21.74046577462016,
        },
        output_name="y_axis_ch1_noise_density_all.png",
    ),
    AxisConfig(
        name="X",
        folder=DATA_DIR / "x axis",
        channel="ch2_mv",
        sensitivity_by_state_mv_per_ut={
            "SR": 22.04341243363412,
            "R": 22.44243659247788,
        },
        output_name="x_axis_ch2_noise_density_all.png",
    ),
]


def parse_state(path: Path) -> str:
    match = re.fullmatch(r"[+-]?\d+mA(?:_(SR|R))?\.csv", path.name)
    if not match:
        raise ValueError(f"Cannot parse calibration filename: {path.name}")
    return match.group(1) or ""


def load_channel(path: Path, channel: str) -> tuple[np.ndarray, np.ndarray]:
    time_s = []
    channel_mv = []

    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if channel not in (reader.fieldnames or []):
            raise ValueError(f"{path}: missing {channel}")
        for row in reader:
            time_s.append(float(row["time_s"]))
            channel_mv.append(float(row[channel]))

    return np.asarray(time_s, dtype=float), np.asarray(channel_mv, dtype=float)


def is_clipped(channel_mv: np.ndarray) -> bool:
    if channel_mv.size == 0:
        return True
    if float(np.std(channel_mv)) < 1e-9:
        return True
    return bool(np.any(channel_mv <= -2499.9) or np.any(channel_mv >= 2499.9))


def remove_linear_trend(time_s: np.ndarray, signal_nt: np.ndarray) -> np.ndarray:
    t = time_s - time_s[0]
    slope, intercept = np.polyfit(t, signal_nt, 1)
    return signal_nt - (slope * t + intercept)


def welch_psd(
    signal_nt: np.ndarray,
    sample_rate_hz: float,
    nperseg: int = 8192,
    overlap: float = 0.5,
) -> tuple[np.ndarray, np.ndarray]:
    step = int(nperseg * (1.0 - overlap))
    window = np.hanning(nperseg)
    window_power = np.sum(window**2)

    spectra = []
    for start in range(0, len(signal_nt) - nperseg + 1, step):
        segment = signal_nt[start : start + nperseg]
        segment = segment - np.mean(segment)
        fft = np.fft.rfft(segment * window)
        psd = (np.abs(fft) ** 2) / (sample_rate_hz * window_power)
        psd[1:-1] *= 2.0
        spectra.append(psd)

    if not spectra:
        raise RuntimeError("Not enough samples for Welch PSD estimate.")

    freqs = np.fft.rfftfreq(nperseg, d=1.0 / sample_rate_hz)
    return freqs, np.mean(np.vstack(spectra), axis=0)


def process_axis(axis: AxisConfig) -> dict[str, float | int | str]:
    psds = []
    used_files = []
    skipped_files = []
    sample_rate_hz = None
    freqs = None

    for path in sorted(axis.folder.glob("*.csv")):
        if path.name.endswith("_summary.csv"):
            continue

        state = parse_state(path)
        sensitivity = axis.sensitivity_by_state_mv_per_ut[state]
        time_s, channel_mv = load_channel(path, axis.channel)

        if is_clipped(channel_mv):
            skipped_files.append(path.name)
            continue

        this_sample_rate = 1.0 / float(np.median(np.diff(time_s)))
        if sample_rate_hz is None:
            sample_rate_hz = this_sample_rate

        signal_nt = channel_mv / sensitivity * 1000.0
        signal_nt = remove_linear_trend(time_s, signal_nt)
        freqs, psd = welch_psd(signal_nt, this_sample_rate)
        psds.append(psd)
        used_files.append(path.name)

    if not psds or freqs is None or sample_rate_hz is None:
        raise RuntimeError(f"No valid files for axis {axis.name}")

    psd_mean = np.mean(np.vstack(psds), axis=0)
    asd = np.sqrt(psd_mean)
    mask = (freqs > 0.0) & (freqs <= 50.0)
    plot_freqs = freqs[mask]
    plot_asd = asd[mask]

    floor_mask = (plot_freqs >= 1.0) & (plot_freqs <= 50.0)
    median_floor = float(np.median(plot_asd[floor_mask]))
    mean_floor = float(np.mean(plot_asd[floor_mask]))

    plt.figure(figsize=(8, 5))
    plt.semilogy(plot_freqs, plot_asd, color="#1f77b4", linewidth=1.4)
    plt.axhline(
        median_floor,
        color="#d62728",
        linestyle="--",
        linewidth=1.2,
        label=f"Mediane 1-50 Hz : {median_floor:.3f} nT/sqrt(Hz)",
    )
    plt.title(f"Densite spectrale de bruit - axe {axis.name}, {axis.channel[:-3].upper()}")
    plt.xlabel("Frequence (Hz)")
    plt.ylabel("Densite de bruit (nT/sqrt(Hz))")
    plt.xlim(0, 50)
    plt.grid(True, which="both", alpha=0.3)
    plt.legend()
    plt.tight_layout()

    out_png = axis.folder / axis.output_name
    plt.savefig(out_png, dpi=160)
    plt.close()

    return {
        "axis": axis.name,
        "channel": axis.channel,
        "sample_rate_hz": sample_rate_hz,
        "used_files": len(used_files),
        "skipped_files": len(skipped_files),
        "median_asd_1_50_nt_per_sqrt_hz": median_floor,
        "mean_asd_1_50_nt_per_sqrt_hz": mean_floor,
        "output_png": str(out_png),
        "used_file_names": ";".join(used_files),
        "skipped_file_names": ";".join(skipped_files),
    }


def main() -> None:
    summaries = [process_axis(axis) for axis in AXES]

    out_csv = DATA_DIR / "noise_density_summary_all_axes.csv"
    with out_csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0].keys()))
        writer.writeheader()
        writer.writerows(summaries)

    print(f"Wrote {out_csv}")
    for row in summaries:
        print(
            f"{row['axis']} {row['channel']}: "
            f"{row['median_asd_1_50_nt_per_sqrt_hz']:.3f} nT/sqrt(Hz) median "
            f"from {row['used_files']} files; skipped {row['skipped_files']}"
        )
        print(f"Wrote {row['output_png']}")


if __name__ == "__main__":
    main()
