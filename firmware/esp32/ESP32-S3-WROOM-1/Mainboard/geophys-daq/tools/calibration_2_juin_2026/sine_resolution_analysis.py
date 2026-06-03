from __future__ import annotations

import csv
import math
import re
from pathlib import Path

import numpy as np


DATA_DIR = Path(__file__).resolve().parent

COIL_UT_PER_A = 71.93410284585705

# From the DC Helmholtz calibration with ADC gain x1.
GAIN1_SCALE_UT_PER_MV = 0.056919

SINE_FILE_RE = re.compile(
    r"(?P<bridge_v>\d+)Vbridge_Gain(?P<gain>\d+)_(?P<freq>\d+)Hz_"
    r"(?P<vpp>\d+)mVpp_load=(?P<load>\d+)Ohms"
)


def load_channel(path: Path) -> tuple[np.ndarray, np.ndarray, str]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
        channel_columns = [field for field in fieldnames if re.fullmatch(r"ch\d+_mv", field)]
        if "time_s" not in fieldnames or not channel_columns:
            raise ValueError(f"{path.name}: expected time_s and a ch*_mv column")
        channel = channel_columns[0]
        time_s = []
        channel_mv = []
        for row in reader:
            time_s.append(float(row["time_s"]))
            channel_mv.append(float(row[channel]))
    return np.asarray(time_s), np.asarray(channel_mv), channel.removesuffix("_mv")


def fit_sine(time_s: np.ndarray, signal_mv: np.ndarray, freq_hz: float) -> dict[str, float]:
    t = time_s - time_s[0]
    omega = 2.0 * math.pi * freq_hz
    omega_60 = 2.0 * math.pi * 60.0

    # Include offset, slow drift, and a 60 Hz nuisance term. The fitted target
    # amplitude remains phase-independent: sqrt(sin_coef^2 + cos_coef^2).
    design = np.column_stack(
        [
            np.sin(omega * t),
            np.cos(omega * t),
            np.ones_like(t),
            t,
            np.sin(omega_60 * t),
            np.cos(omega_60 * t),
        ]
    )
    coef, *_ = np.linalg.lstsq(design, signal_mv, rcond=None)
    fitted = design @ coef
    residual = signal_mv - fitted

    dof = len(signal_mv) - design.shape[1]
    residual_var = float(residual @ residual / dof)
    covariance = residual_var * np.linalg.inv(design.T @ design)

    sin_coef, cos_coef = float(coef[0]), float(coef[1])
    amplitude_mv_peak = math.hypot(sin_coef, cos_coef)

    # Error propagation for sqrt(a^2 + b^2).
    grad = np.array([sin_coef, cos_coef]) / amplitude_mv_peak if amplitude_mv_peak else np.zeros(2)
    amp_cov = covariance[:2, :2]
    amplitude_se_mv_peak = math.sqrt(float(grad @ amp_cov @ grad)) if amplitude_mv_peak else math.nan

    return {
        "amplitude_mv_peak": amplitude_mv_peak,
        "amplitude_se_mv_peak": amplitude_se_mv_peak,
        "phase_deg": math.degrees(math.atan2(cos_coef, sin_coef)),
        "residual_rms_mv": math.sqrt(residual_var),
        "mains_60hz_mv_peak": math.hypot(float(coef[4]), float(coef[5])),
    }


def local_fft_noise(
    time_s: np.ndarray,
    signal_mv: np.ndarray,
    freq_hz: float,
    field_scale_nt_per_mv: float,
) -> dict[str, float]:
    dt = float(np.median(np.diff(time_s)))
    sample_rate_hz = 1.0 / dt
    t = time_s - time_s[0]

    drift = np.polyfit(t, signal_mv, 1)
    detrended = signal_mv - (drift[0] * t + drift[1])

    window = np.hanning(len(detrended))
    freqs = np.fft.rfftfreq(len(detrended), dt)
    amps_mv_peak = 2.0 * np.abs(np.fft.rfft(detrended * window)) / np.sum(window)
    amps_nt_peak = amps_mv_peak * field_scale_nt_per_mv

    target_idx = int(np.argmin(np.abs(freqs - freq_hz)))
    target_amp_nt_peak = float(amps_nt_peak[target_idx])

    mask = (
        (freqs >= max(0.1, freq_hz - 1.0))
        & (freqs <= freq_hz + 1.0)
        & (np.abs(freqs - freq_hz) > 0.12)
    )
    if freq_hz < 3.0:
        mask &= freqs >= 1.0

    local_bins = amps_nt_peak[mask]
    local_noise_nt_peak_bin = float(np.median(local_bins) * 1.4826)

    return {
        "sample_rate_hz": sample_rate_hz,
        "fft_bin_hz": sample_rate_hz / len(detrended),
        "fft_target_nt_peak": target_amp_nt_peak,
        "local_noise_nt_peak_bin": local_noise_nt_peak_bin,
        "local_fft_snr": target_amp_nt_peak / local_noise_nt_peak_bin
        if local_noise_nt_peak_bin
        else math.nan,
    }


def main() -> None:
    rows = []
    for path in sorted(DATA_DIR.glob("*Vbridge_Gain*_*.csv")):
        match = SINE_FILE_RE.search(path.name)
        if not match:
            continue

        bridge_v = float(match.group("bridge_v"))
        adc_gain = int(match.group("gain"))
        freq_hz = float(match.group("freq"))
        vpp_mv = float(match.group("vpp"))
        load_ohm = float(match.group("load"))
        time_s, channel_mv, channel = load_channel(path)
        fit = fit_sine(time_s, channel_mv, freq_hz)

        field_scale_nt_per_mv = GAIN1_SCALE_UT_PER_MV * 1000.0 / adc_gain * (5.0 / bridge_v)
        fft = local_fft_noise(time_s, channel_mv, freq_hz, field_scale_nt_per_mv)
        measured_nt_peak = fit["amplitude_mv_peak"] * field_scale_nt_per_mv
        measured_se_nt_peak = fit["amplitude_se_mv_peak"] * field_scale_nt_per_mv

        expected_nominal_nt_peak = COIL_UT_PER_A * ((vpp_mv / 1000.0 / 2.0) / load_ohm) * 1000.0
        expected_highz_nt_peak = 2.0 * expected_nominal_nt_peak

        rows.append(
            {
                "file": path.name,
                "channel": channel,
                "bridge_v": bridge_v,
                "adc_gain": adc_gain,
                "freq_hz": freq_hz,
                "vpp_mv_setting": vpp_mv,
                "load_ohm": load_ohm,
                "samples": len(channel_mv),
                "duration_s": float(time_s[-1] - time_s[0]) if len(time_s) > 1 else 0.0,
                "measured_nt_peak": measured_nt_peak,
                "measured_se_nt_peak": measured_se_nt_peak,
                "snr_lockin": measured_nt_peak / measured_se_nt_peak if measured_se_nt_peak else math.nan,
                "one_sigma_resolution_nt_peak": measured_se_nt_peak,
                "three_sigma_resolution_nt_peak": 3.0 * measured_se_nt_peak,
                "expected_nominal_nt_peak": expected_nominal_nt_peak,
                "expected_if_highz_2x_nt_peak": expected_highz_nt_peak,
                "measured_over_highz_expected": measured_nt_peak / expected_highz_nt_peak
                if expected_highz_nt_peak
                else math.nan,
                "residual_rms_mv_after_fit": fit["residual_rms_mv"],
                "mains_60hz_mv_peak": fit["mains_60hz_mv_peak"],
                "phase_deg": fit["phase_deg"],
                "fft_bin_hz": fft["fft_bin_hz"],
                "fft_target_nt_peak": fft["fft_target_nt_peak"],
                "local_noise_nt_peak_bin": fft["local_noise_nt_peak_bin"],
                "local_fft_snr": fft["local_fft_snr"],
                "local_fft_three_sigma_nt_peak": 3.0 * fft["local_noise_nt_peak_bin"],
            }
        )

    out_csv = DATA_DIR / "sine_resolution_analysis.csv"
    with out_csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote {out_csv}")
    print()
    print(
        "file, measured nT peak, 1sigma nT peak, 3sigma nT peak, "
        "expected nT peak if high-Z voltage is 2x, lock-in SNR, local FFT 3sigma nT"
    )
    for row in rows:
        print(
            f"{row['file']}, "
            f"{row['measured_nt_peak']:.3f}, "
            f"{row['one_sigma_resolution_nt_peak']:.3f}, "
            f"{row['three_sigma_resolution_nt_peak']:.3f}, "
            f"{row['expected_if_highz_2x_nt_peak']:.3f}, "
            f"{row['snr_lockin']:.1f}, "
            f"{row['local_fft_three_sigma_nt_peak']:.3f}"
        )


if __name__ == "__main__":
    main()
