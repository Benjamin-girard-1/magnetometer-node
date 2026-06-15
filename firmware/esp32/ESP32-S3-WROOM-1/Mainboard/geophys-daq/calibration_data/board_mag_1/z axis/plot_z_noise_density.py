from __future__ import annotations

import csv
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".mplconfig"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DATA_DIR = Path(__file__).resolve().parent
DATA_FILE = DATA_DIR / "0mA.csv"
CHANNEL = "ch0_mv"

# Measured Z-axis sensitivity from the DC calibration.
SENSITIVITY_MV_PER_UT = 21.12783552609885


def load_signal_nt(path: Path) -> tuple[np.ndarray, np.ndarray]:
    time_s = []
    channel_mv = []

    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            time_s.append(float(row["time_s"]))
            channel_mv.append(float(row[CHANNEL]))

    time = np.asarray(time_s, dtype=float)
    mv = np.asarray(channel_mv, dtype=float)

    # Convert mV to nT.
    signal_nt = mv / SENSITIVITY_MV_PER_UT * 1000.0
    return time, signal_nt


def remove_linear_trend(time_s: np.ndarray, signal_nt: np.ndarray) -> np.ndarray:
    t = time_s - time_s[0]
    slope, intercept = np.polyfit(t, signal_nt, 1)
    return signal_nt - (slope * t + intercept)


def welch_asd(
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
    psd_mean = np.mean(np.vstack(spectra), axis=0)
    asd = np.sqrt(psd_mean)
    return freqs, asd


def main() -> None:
    time_s, signal_nt = load_signal_nt(DATA_FILE)
    sample_rate_hz = 1.0 / float(np.median(np.diff(time_s)))
    signal_nt = remove_linear_trend(time_s, signal_nt)

    freqs, asd = welch_asd(signal_nt, sample_rate_hz)
    mask = (freqs > 0.0) & (freqs <= 50.0)
    plot_freqs = freqs[mask]
    plot_asd = asd[mask]

    floor_mask = (plot_freqs >= 1.0) & (plot_freqs <= 50.0)
    median_floor = float(np.median(plot_asd[floor_mask]))

    plt.figure(figsize=(8, 5))
    plt.semilogy(plot_freqs, plot_asd, color="#1f77b4", linewidth=1.4)
    plt.axhline(
        median_floor,
        color="#d62728",
        linestyle="--",
        linewidth=1.2,
        label=f"Mediane 1-50 Hz : {median_floor:.2f} nT/sqrt(Hz)",
    )
    plt.title("Densite spectrale de bruit - axe Z, CH0, 0 mA")
    plt.xlabel("Frequence (Hz)")
    plt.ylabel("Densite de bruit (nT/sqrt(Hz))")
    plt.xlim(0, 50)
    plt.grid(True, which="both", alpha=0.3)
    plt.legend()
    plt.tight_layout()

    out_png = DATA_DIR / "z_axis_ch0_noise_density_0mA.png"
    plt.savefig(out_png, dpi=160)

    print(f"Sample rate: {sample_rate_hz:.3f} Hz")
    print(f"Median ASD, 1-50 Hz: {median_floor:.3f} nT/sqrt(Hz)")
    print(f"Wrote {out_png}")


if __name__ == "__main__":
    main()
