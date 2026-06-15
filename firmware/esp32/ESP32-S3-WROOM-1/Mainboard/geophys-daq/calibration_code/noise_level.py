import numpy as np
import matplotlib.pyplot as plt


sensitivity_mV_per_uT = 21.86

measurements = [
    {
        "label": "1.2 kSPS",
        "fs": 1200,
        "path": "calibration_data/board_mag_2/x/long/0mA_SR_4.csv",
        "color": "tab:blue",
    },
    {
        "label": "2.0 kSPS",
        "fs": 2000,
        "path": "calibration_data/board_mag_2/x/long/0mA_SR_3.csv",
        "color": "tab:red",
    },
]


def load_csv(path):
    return np.genfromtxt(path, delimiter=",", skip_header=1)


def compute_asd(data, fs, channel_index):
    t = data[:, 0]
    signal_mV = data[:, channel_index]

    signal_mV = signal_mV - np.mean(signal_mV)
    window = np.hanning(np.size(t))
    signal_windowed = signal_mV * window

    fft = np.fft.rfft(signal_windowed)
    psd = (np.abs(fft) ** 2) / (fs * np.sum(window**2))
    psd[1:-1] *= 2

    asd_mV = np.sqrt(psd)
    asd_nT = asd_mV / sensitivity_mV_per_uT * 1000
    freq = np.fft.rfftfreq(np.size(t), d=1 / fs)

    return freq, asd_nT


fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, sharey=True)
channels = [(2, "CH0"), (3, "CH1"), (4, "CH2")]

for measurement in measurements:
    data = load_csv(measurement["path"])

    for ax, (channel_index, channel_name) in zip(axes, channels):
        freq, asd_nT = compute_asd(data, measurement["fs"], channel_index)
        ax.plot(
            freq,
            asd_nT,
            color=measurement["color"],
            linewidth=1.1,
            alpha=0.9,
            label=measurement["label"],
        )

    # Mark the old suspicious binary subdivisions of the sample rate.
    for divisor in [64, 128]:
        peak_freq = measurement["fs"] / divisor
        for ax in axes:
            ax.axvline(
                peak_freq,
                color=measurement["color"],
                linestyle="--",
                linewidth=0.9,
                alpha=0.35,
            )

for peak_freq in [7.83, 15.6, 31.25, 44.0, 60.0]:
    for ax in axes:
        ax.axvline(peak_freq, color="0.75", linestyle=":", linewidth=0.8, alpha=0.8)

for ax, (_, channel_name) in zip(axes, channels):
    ax.set_yscale("log")
    ax.set_ylim(bottom=0.01)
    ax.set_ylabel(channel_name)
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(loc="upper right")

axes[-1].set_xlim(0, 80)
axes[-1].set_xlabel("Frequence (Hz)")
fig.supylabel("Densite de bruit (nT/sqrt Hz)")
plt.tight_layout()
plt.show()
