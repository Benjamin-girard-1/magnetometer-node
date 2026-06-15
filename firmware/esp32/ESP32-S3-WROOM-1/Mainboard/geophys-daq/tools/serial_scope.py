#!/usr/bin/env python3
"""
Fast serial scope for the magnetic expansion-card bring-up firmware.

Expected firmware frame:
    Binary packet:
      A5 5A len frame_u32 ch0_i32 ... ch7_i32 status_u8 checksum_u8

Also accepts older text firmware frames:
    /*<frame,ch0_mV,ch1_mV,ch2_mV,ch3_mV,ch4_mV,ch5_mV,ch6_mV,ch7_mV,status>*/
    /*<frame,y_mV,z_mV,status>*/
    /*<frame,x_v,y_v,z_v,status>*/

Install:
    python3 -m pip install pyserial pyqtgraph PyQt6 numpy

Run:
    python3 tools/serial_scope.py
"""

from __future__ import annotations

import argparse
import collections
import csv
import math
import re
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pyqtgraph as pg
import serial
from serial.tools import list_ports
from PyQt6 import QtCore, QtWidgets


FRAME_RE = re.compile(rb"/\*<\s*([^>]+?)\s*>\*/")
LSM6DSV_SAMPLE_RE = re.compile(
    r"LSM6DSV sample: acc\[g\]=([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)\s+"
    r"gyro\[dps\]=([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)\s+"
    r"temp=([+-]?\d+(?:\.\d+)?) C"
)
SCL3300_SAMPLE_RE = re.compile(
    r"SCL3300 sample: acc\[g\]=([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)\s+"
    r"angle\[deg\]=([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)\s+"
    r"temp=([+-]?\d+(?:\.\d+)?) C"
)
CHANNEL_COUNT = 8
ADC_FULL_SCALE_CODE = 8388608.0
ADC_REF_V = 2.5
ADC_GAINS = (1, 2, 4, 8)
ADC_SR_PINS = (
    ("3V3A LDO", 14, "Enable EN_LDO_3V3 for the +3V3A analog rail; this is not the main +3V3D digital rail."),
    ("MCLK EN", 3, "Enable the AD7779 master clock source."),
    ("~RESET high", 5, "Release the AD7779 reset pin. Unchecked asserts reset low."),
    ("START", 4, "Drive the AD7779 START pin."),
    ("CONVST SAR", 2, "Drive the AD7779 CONVST_SAR pin."),
)
BINARY_SYNC = b"\xA5\x5A"
BINARY_PAYLOAD_LEN = 4 + CHANNEL_COUNT * 4 + 1
BINARY_PACKET_LEN = 2 + 1 + BINARY_PAYLOAD_LEN + 1
BINARY_STRUCT = struct.Struct("<I" + "i" * CHANNEL_COUNT + "B")
DAC_CENTER_CODE = 2048
DAC_MIN_CODE = 1948
DAC_MAX_CODE = 2148
DAC_LIVE_INTERVAL_MS = 100
DAC_MAX_STEP_LSB = 64
DAC_TEST_SETTLE_MS = 1500
DAC_TEST_CAPTURE_MS = 1000
THERMAL_ISOLATION_UI = True
CHANNEL_COLORS = [
    "#ef4444",
    "#f97316",
    "#eab308",
    "#22c55e",
    "#06b6d4",
    "#3b82f6",
    "#8b5cf6",
    "#ec4899",
]
HELD_SAMPLE_STATUS = 0x80
HELD_SAMPLE_MAX_BATCH = 200
ADC_RATE_MIN_HZ = 501
ADC_RATE_MAX_HZ = 2000


@dataclass
class Sample:
    frame: int
    channels_mv: tuple[float, ...]
    status: int
    t_host: float


class SerialReader(threading.Thread):
    def __init__(
        self,
        port: str,
        baud: int,
        out_queue: collections.deque[Sample],
        accept_bad_checksum: bool = False,
    ):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_queue = out_queue
        self.accept_bad_checksum = accept_bad_checksum
        self.stop_event = threading.Event()
        self.error: str | None = None
        self.samples_seen = 0
        self.bytes_seen = 0
        self.raw_preview = ""
        self.binary_packets = 0
        self.text_packets = 0
        self.bad_checksums = 0
        self.sync_hits = 0
        self.bad_lengths = 0
        self.start_time: float | None = None
        self.rate_start_time: float | None = None
        self.rate_start_samples = 0
        self.serial_lock = threading.Lock()
        self.gain_lock = threading.Lock()
        self.ser: serial.Serial | None = None
        self.text_lines: collections.deque[str] = collections.deque(maxlen=200)
        self.channel_gains = [1.0] * CHANNEL_COUNT
        self.input_referred = False
        self.raw_codes = False

    @staticmethod
    def preview(chunk: bytes) -> str:
        tail = chunk[-40:]
        chars = []
        for b in tail:
            if b in (10, 13):
                chars.append("\\n")
            elif 32 <= b <= 126:
                chars.append(chr(b))
            else:
                chars.append(".")
        text = "".join(chars)
        hex_tail = " ".join(f"{b:02X}" for b in tail[-20:])
        return f"{text} | hex {hex_tail}"

    def run(self) -> None:
        buf = bytearray()
        try:
            with serial.Serial(self.port, self.baud, timeout=0.05) as ser:
                with self.serial_lock:
                    self.ser = ser
                ser.reset_input_buffer()
                self.start_time = time.monotonic()
                self.reset_average_sample_rate()
                while not self.stop_event.is_set():
                    chunk = ser.read(4096)
                    if not chunk:
                        continue
                    self.bytes_seen += len(chunk)
                    self.raw_preview = self.preview(chunk)
                    buf.extend(chunk)

                    while True:
                        if len(buf) >= BINARY_PACKET_LEN:
                            sync_pos = buf.find(BINARY_SYNC)
                            text_pos = buf.find(b"/*<")
                            if sync_pos >= 0 and (text_pos < 0 or sync_pos <= text_pos):
                                if sync_pos > 0:
                                    del buf[:sync_pos]
                                self.sync_hits += 1
                                if len(buf) < BINARY_PACKET_LEN:
                                    break
                                if buf[2] != BINARY_PAYLOAD_LEN:
                                    self.bad_lengths += 1
                                    del buf[0]
                                    continue
                                pkt = bytes(buf[:BINARY_PACKET_LEN])
                                if (sum(pkt[2:]) & 0xFF) != 0 and not self.accept_bad_checksum:
                                    self.bad_checksums += 1
                                    del buf[0]
                                    continue
                                payload = pkt[3 : 3 + BINARY_PAYLOAD_LEN]
                                unpacked = BINARY_STRUCT.unpack(payload)
                                frame = unpacked[0]
                                raw = unpacked[1 : 1 + CHANNEL_COUNT]
                                status = unpacked[-1]
                                with self.gain_lock:
                                    gains = tuple(self.channel_gains)
                                    input_referred = self.input_referred
                                    raw_codes = self.raw_codes
                                base_scale = (ADC_REF_V * 1000.0) / ADC_FULL_SCALE_CODE
                                scale = 1.0 if raw_codes else base_scale
                                sample = Sample(
                                    frame=frame,
                                    channels_mv=tuple(
                                        v * scale / (gains[ch] if input_referred and not raw_codes else 1.0)
                                        for ch, v in enumerate(raw)
                                    ),
                                    status=status,
                                    t_host=time.monotonic(),
                                )
                                self.out_queue.append(sample)
                                self.samples_seen += 1
                                self.binary_packets += 1
                                del buf[:BINARY_PACKET_LEN]
                                continue

                        match = FRAME_RE.search(buf)
                        if not match:
                            if BINARY_SYNC not in buf and b"/*<" not in buf:
                                newline = buf.find(b"\n")
                                if newline >= 0:
                                    line = bytes(buf[: newline + 1])
                                    del buf[: newline + 1]
                                    text = line.decode("utf-8", errors="replace").strip()
                                    if text:
                                        self.text_lines.append(text)
                                    continue
                            if len(buf) > 4096:
                                del buf[:-512]
                            break

                        del buf[: match.start()]
                        match = FRAME_RE.match(buf)
                        if not match:
                            continue

                        try:
                            fields = [f.strip() for f in match.group(1).split(b",")]
                            if len(fields) == 4:
                                frame = int(fields[0])
                                channels = [math.nan] * CHANNEL_COUNT
                                channels[1] = float(fields[1])
                                channels[2] = float(fields[2])
                                status = int(fields[3])
                            elif len(fields) == 5:
                                frame = int(fields[0])
                                channels = [math.nan] * CHANNEL_COUNT
                                channels[0] = float(fields[1]) * 1000.0
                                channels[1] = float(fields[2]) * 1000.0
                                channels[2] = float(fields[3]) * 1000.0
                                status = int(fields[4])
                            elif len(fields) == 10:
                                frame = int(fields[0])
                                channels = [float(f) for f in fields[1:9]]
                                status = int(fields[9])
                            else:
                                del buf[: match.end()]
                                continue

                            sample = Sample(
                                frame=frame,
                                channels_mv=tuple(channels),
                                status=status,
                                t_host=time.monotonic(),
                            )
                        except ValueError:
                            del buf[: match.end()]
                            continue

                        self.out_queue.append(sample)
                        self.samples_seen += 1
                        self.text_packets += 1
                        del buf[: match.end()]
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            with self.serial_lock:
                self.ser = None

    def stop(self) -> None:
        self.stop_event.set()

    def write_command(self, command: bytes) -> bool:
        with self.serial_lock:
            if self.ser is None:
                return False
            self.ser.write(command)
            return True

    def set_channel_gain(self, ch: int, gain: int) -> None:
        with self.gain_lock:
            self.channel_gains[ch] = float(gain)

    def set_input_referred(self, enabled: bool) -> None:
        with self.gain_lock:
            self.input_referred = enabled

    def set_raw_codes(self, enabled: bool) -> None:
        with self.gain_lock:
            self.raw_codes = enabled

    def drain_text_lines(self) -> list[str]:
        lines: list[str] = []
        while self.text_lines:
            lines.append(self.text_lines.popleft())
        return lines

    def reset_average_sample_rate(self) -> None:
        self.rate_start_time = time.monotonic()
        self.rate_start_samples = self.samples_seen

    def average_sample_rate(self) -> float:
        if self.rate_start_time is None:
            return float("nan")
        samples = self.samples_seen - self.rate_start_samples
        if samples < 1000:
            return float("nan")
        elapsed = time.monotonic() - self.rate_start_time
        if elapsed <= 1.0:
            return float("nan")
        return samples / elapsed


class SerialScope(QtWidgets.QMainWindow):
    def __init__(self, args: argparse.Namespace):
        super().__init__()
        self.args = args
        self.queue: collections.deque[Sample] = collections.deque(maxlen=args.queue)
        self.reader: SerialReader | None = None
        self.plot_hold_until = 0.0
        self.imu_test_running = False
        self.imu_test_started_at = 0.0
        self.imu_test_next_cycle = 0.0
        self.imu_test_rows: list[dict[str, float]] = []
        self.imu_test_pending_lsm: dict[str, float] | None = None
        self.imu_test_output_path: Path | None = None
        self.imu_test_current_segment: list[dict[str, float]] = []
        self.imu_test_segment_count = 0
        self.imu_test_last_event = ""
        self.imu_test_stable = False

        self.frames = np.empty(args.buffer, dtype=np.float64)
        self.host_times = np.empty(args.buffer, dtype=np.float64)
        self.channels = np.empty((CHANNEL_COUNT, args.buffer), dtype=np.float64)
        self.status = np.empty(args.buffer, dtype=np.float64)
        self.write_idx = 0
        self.count = 0
        self.last_queue_drained = 0
        self.channel_gains = [1] * CHANNEL_COUNT
        self.input_referred = False
        self.raw_codes = False
        self.auto_polarity_next_set = True
        self.held_samples_inserted = 0
        self.held_samples_replaced = 0
        self.next_fft_update_at = 0.0
        self.next_stats_update_at = 0.0
        self.last_stats_text = ""

        self.setWindowTitle("Magnetic Serial Scope")
        self.resize(1200, 800)

        pg.setConfigOptions(antialias=False)
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        self.setCentralWidget(central)

        self.info = QtWidgets.QLabel()
        self.info.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.info.setWordWrap(True)
        self.info.setMaximumHeight(48)
        self.info.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Ignored,
            QtWidgets.QSizePolicy.Policy.Fixed,
        )
        layout.addWidget(self.info)

        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)

        controls.addWidget(QtWidgets.QLabel("Port:"))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(260)
        self.port_combo.setEditable(True)
        controls.addWidget(self.port_combo)

        self.refresh_ports_btn = QtWidgets.QPushButton("Refresh")
        self.refresh_ports_btn.clicked.connect(self.refresh_ports)
        controls.addWidget(self.refresh_ports_btn)

        controls.addWidget(QtWidgets.QLabel("Baud:"))
        self.baud_box = QtWidgets.QSpinBox()
        self.baud_box.setRange(1200, 3000000)
        self.baud_box.setValue(args.baud)
        self.baud_box.setSingleStep(115200)
        controls.addWidget(self.baud_box)

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        controls.addWidget(self.connect_btn)

        controls.addWidget(QtWidgets.QLabel("ADC Hz:"))
        self.adc_rate_box = QtWidgets.QDoubleSpinBox()
        self.adc_rate_box.setRange(ADC_RATE_MIN_HZ, ADC_RATE_MAX_HZ)
        self.adc_rate_box.setValue(args.adc_rate)
        self.adc_rate_box.setDecimals(0)
        self.adc_rate_box.setSingleStep(10.0)
        self.adc_rate_box.setToolTip(
            f"Set the AD7779 output data rate ({ADC_RATE_MIN_HZ}-{ADC_RATE_MAX_HZ} Hz). "
            "The firmware briefly pauses streaming while changing it."
        )
        controls.addWidget(self.adc_rate_box)

        self.adc_rate_btn = QtWidgets.QPushButton("Set Hz")
        self.adc_rate_btn.setToolTip("Send ADC ODR and update scope timing.")
        self.adc_rate_btn.clicked.connect(self.send_adc_rate_command)
        self.adc_rate_btn.setEnabled(False)
        controls.addWidget(self.adc_rate_btn)

        self.clear_btn = QtWidgets.QPushButton("Clear")
        self.clear_btn.clicked.connect(self.clear_buffers)
        controls.addWidget(self.clear_btn)

        self.zero_btn = QtWidgets.QPushButton("Zero")
        self.zero_btn.setToolTip("Send ZERO command to start one firmware offset auto-zero pass.")
        self.zero_btn.clicked.connect(self.send_zero_command)
        self.zero_btn.setEnabled(False)
        controls.addWidget(self.zero_btn)

        self.set_reset_btn = QtWidgets.QPushButton("Set/Reset")
        self.set_reset_btn.setToolTip("Send one HMC100x set/reset strap sequence.")
        self.set_reset_btn.clicked.connect(self.send_set_reset_command)
        self.set_reset_btn.setEnabled(False)
        controls.addWidget(self.set_reset_btn)

        self.set_btn = QtWidgets.QPushButton("Set")
        self.set_btn.setToolTip("Send one HMC100x SET strap pulse.")
        self.set_btn.clicked.connect(self.send_set_command)
        self.set_btn.setEnabled(False)
        controls.addWidget(self.set_btn)

        self.reset_btn = QtWidgets.QPushButton("Reset")
        self.reset_btn.setToolTip("Send one HMC100x RESET strap pulse.")
        self.reset_btn.clicked.connect(self.send_reset_command)
        self.reset_btn.setEnabled(False)
        controls.addWidget(self.reset_btn)

        self.auto_polarity_btn = QtWidgets.QPushButton("Auto Polarity")
        self.auto_polarity_btn.setToolTip("Alternate HMC100x SET and RESET pulses once per second.")
        self.auto_polarity_btn.setCheckable(True)
        self.auto_polarity_btn.toggled.connect(self.toggle_auto_polarity)
        self.auto_polarity_btn.setEnabled(False)
        controls.addWidget(self.auto_polarity_btn)

        self.auto_polarity_timer = QtCore.QTimer(self)
        self.auto_polarity_timer.setInterval(1000)
        self.auto_polarity_timer.timeout.connect(self.send_next_auto_polarity_command)

        self.enable_9v = QtWidgets.QCheckBox("+9V enable")
        self.enable_9v.setToolTip("Toggle the shift-register enable for the +10V boost feeding +9VA.")
        self.enable_9v.setChecked(False)
        self.enable_9v.stateChanged.connect(self.send_9v_enable_command)
        self.enable_9v.setEnabled(False)
        controls.addWidget(self.enable_9v)

        self.enable_neg5v = QtWidgets.QCheckBox("-5V enable")
        self.enable_neg5v.setToolTip("Toggle the shift-register enable for the negative inverter.")
        self.enable_neg5v.setChecked(False)
        self.enable_neg5v.stateChanged.connect(self.send_neg5v_enable_command)
        self.enable_neg5v.setEnabled(False)
        controls.addWidget(self.enable_neg5v)

        self.dac_enable = QtWidgets.QPushButton("DAC Off")
        self.dac_enable.setToolTip("Enable or power down the MCP4728 DAC outputs. Leave off unless actively testing offset.")
        self.dac_enable.setCheckable(True)
        self.dac_enable.setChecked(False)
        self.dac_enable.toggled.connect(self.send_dac_enable_command)
        self.dac_enable.setEnabled(False)
        controls.addWidget(self.dac_enable)

        controls.addWidget(QtWidgets.QLabel("Window s:"))
        self.window_box = QtWidgets.QDoubleSpinBox()
        max_window_s = max(1.0, args.buffer / max(args.sample_rate, 1.0))
        self.window_box.setRange(1.0, max_window_s)
        self.window_box.setValue(min(args.window_s, max_window_s))
        self.window_box.setDecimals(1)
        self.window_box.setToolTip(
            f"Limited by the in-memory buffer: {args.buffer} samples at "
            f"{args.sample_rate:g} samples/s."
        )
        controls.addWidget(self.window_box)

        controls.addWidget(QtWidgets.QLabel("FFT max Hz:"))
        self.fft_max_box = QtWidgets.QDoubleSpinBox()
        self.fft_max_box.setRange(1.0, max(args.sample_rate / 2.0, args.fft_max_hz))
        self.fft_max_box.setValue(args.fft_max_hz)
        self.fft_max_box.setDecimals(1)
        self.fft_max_box.valueChanged.connect(self.update_fft_xrange)
        controls.addWidget(self.fft_max_box)

        controls.addStretch(1)

        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, stretch=1)

        self.scope_tab = QtWidgets.QWidget()
        scope_layout = QtWidgets.QVBoxLayout(self.scope_tab)
        self.tabs.addTab(self.scope_tab, "Scope")

        self.control_tab = QtWidgets.QWidget()
        control_layout = QtWidgets.QVBoxLayout(self.control_tab)
        self.tabs.addTab(self.control_tab, "Control")

        self.imu_test_tab = QtWidgets.QWidget()
        imu_test_layout = QtWidgets.QVBoxLayout(self.imu_test_tab)
        self.tabs.addTab(self.imu_test_tab, "IMU Test")

        channel_controls = QtWidgets.QHBoxLayout()
        scope_layout.addLayout(channel_controls)
        channel_controls.addWidget(QtWidgets.QLabel("Channels:"))
        self.channel_checks: list[QtWidgets.QCheckBox] = []
        for ch in range(CHANNEL_COUNT):
            cb = QtWidgets.QCheckBox(f"CH{ch}")
            cb.setChecked(ch in (1, 2, 6))
            cb.setStyleSheet(f"color: {CHANNEL_COLORS[ch]};")
            cb.stateChanged.connect(self.update_channel_visibility)
            self.channel_checks.append(cb)
            channel_controls.addWidget(cb)

        self.all_btn = QtWidgets.QPushButton("All")
        self.all_btn.clicked.connect(lambda: self.set_all_channels(True))
        channel_controls.addWidget(self.all_btn)

        self.none_btn = QtWidgets.QPushButton("None")
        self.none_btn.clicked.connect(lambda: self.set_all_channels(False))
        channel_controls.addWidget(self.none_btn)

        self.save_stats_btn = QtWidgets.QPushButton("Save Stats")
        self.save_stats_btn.setToolTip("Save mean/noise statistics for the current plot window.")
        self.save_stats_btn.clicked.connect(self.save_stats_csv)
        self.save_stats_btn.setEnabled(False)
        channel_controls.addWidget(self.save_stats_btn)

        self.save_scope_btn = QtWidgets.QPushButton("Save Scope CSV")
        self.save_scope_btn.setToolTip("Save the currently visible scope data as a simple CSV.")
        self.save_scope_btn.clicked.connect(self.save_scope_csv)
        self.save_scope_btn.setEnabled(False)
        channel_controls.addWidget(self.save_scope_btn)

        self.input_referred_check = QtWidgets.QCheckBox("Input-referred")
        self.input_referred_check.setToolTip(
            "Divide binary ADC values by the selected AD7779 PGA gain. "
            "Unchecked shows the ADC full-scale/post-PGA value."
        )
        self.input_referred_check.setChecked(False)
        self.input_referred_check.stateChanged.connect(self.set_voltage_scale_mode)
        channel_controls.addWidget(self.input_referred_check)

        self.raw_codes_check = QtWidgets.QCheckBox("Raw ADC codes")
        self.raw_codes_check.setToolTip("Bypass voltage/gain scaling and plot signed 24-bit ADC codes directly.")
        self.raw_codes_check.setChecked(False)
        self.raw_codes_check.stateChanged.connect(self.set_voltage_scale_mode)
        channel_controls.addWidget(self.raw_codes_check)

        channel_controls.addStretch(1)

        gain_controls = QtWidgets.QHBoxLayout()
        scope_layout.addLayout(gain_controls)
        gain_controls.addWidget(QtWidgets.QLabel("ADC gain:"))
        self.gain_combos: list[QtWidgets.QComboBox] = []
        for ch in range(CHANNEL_COUNT):
            gain_controls.addWidget(QtWidgets.QLabel(f"CH{ch}"))
            combo = QtWidgets.QComboBox()
            combo.addItems([f"x{gain}" for gain in ADC_GAINS])
            combo.setCurrentText("x1")
            combo.setEnabled(False)
            combo.setToolTip("Set this AD7779 channel PGA gain.")
            combo.currentIndexChanged.connect(lambda _idx, channel=ch: self.send_gain_command(channel))
            self.gain_combos.append(combo)
            gain_controls.addWidget(combo)
        gain_controls.addStretch(1)

        self.stats_label = QtWidgets.QLabel()
        self.stats_label.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.stats_label.setWordWrap(True)
        self.stats_label.setMaximumHeight(54)
        font = self.stats_label.font()
        font.setFamily("Menlo")
        self.stats_label.setFont(font)
        scope_layout.addWidget(self.stats_label)

        self.time_plot = pg.PlotWidget(title="Time Domain")
        self.time_plot.setLabel("left", "ADC/PGA voltage", units="mV")
        self.time_plot.setLabel("bottom", "time", units="s")
        self.time_plot.showGrid(x=True, y=True, alpha=0.25)
        scope_layout.addWidget(self.time_plot, stretch=3)

        self.time_curves = []
        for ch in range(CHANNEL_COUNT):
            curve = self.time_plot.plot(
                pen=pg.mkPen(CHANNEL_COLORS[ch], width=1),
                name=f"CH{ch}",
            )
            curve.setClipToView(True)
            curve.setDownsampling(auto=True, method="peak")
            self.time_curves.append(curve)
        self.time_plot.addLegend()

        self.fft_plot = pg.PlotWidget(title="Spectrum")
        self.fft_plot.setLabel("left", "ADC/PGA amplitude", units="mV")
        self.fft_plot.setLabel("bottom", "frequency", units="Hz")
        self.fft_plot.showGrid(x=True, y=True, alpha=0.25)
        self.update_fft_xrange()
        scope_layout.addWidget(self.fft_plot, stretch=2)

        self.fft_curves = []
        for ch in range(CHANNEL_COUNT):
            curve = self.fft_plot.plot(
                pen=pg.mkPen(CHANNEL_COLORS[ch], width=1),
                name=f"CH{ch} FFT",
            )
            curve.setClipToView(True)
            self.fft_curves.append(curve)
        self.fft_plot.addLegend()
        self.update_channel_visibility()

        adc_pin_group = QtWidgets.QGroupBox("ADC pins")
        adc_pin_layout = QtWidgets.QHBoxLayout(adc_pin_group)
        control_layout.addWidget(adc_pin_group)

        self.adc_pin_checks: list[QtWidgets.QCheckBox] = []
        for label, pin, tooltip in ADC_SR_PINS:
            cb = QtWidgets.QCheckBox(label)
            cb.setToolTip(f"{tooltip}  SR pin {pin}.")
            cb.setEnabled(False)
            cb.stateChanged.connect(lambda _state, sr_pin=pin, name=label: self.send_adc_pin_command(sr_pin, name))
            self.adc_pin_checks.append(cb)
            adc_pin_layout.addWidget(cb)

        self.adc_pins_off_btn = QtWidgets.QPushButton("ADC Pins Off")
        self.adc_pins_off_btn.setToolTip("Set ADC START, CONVST_SAR, ~RESET, MCLK_EN, and 3V3A LDO enable low/off.")
        self.adc_pins_off_btn.clicked.connect(self.send_adc_pins_off_command)
        self.adc_pins_off_btn.setEnabled(False)
        adc_pin_layout.addWidget(self.adc_pins_off_btn)
        adc_pin_layout.addStretch(1)

        mode_controls = QtWidgets.QHBoxLayout()
        control_layout.addLayout(mode_controls)

        self.control_mode_btn = QtWidgets.QPushButton("Control Mode")
        self.control_mode_btn.setToolTip("Stop ADC bytes on UART and use text commands/responses.")
        self.control_mode_btn.clicked.connect(self.send_control_mode_command)
        self.control_mode_btn.setEnabled(False)
        mode_controls.addWidget(self.control_mode_btn)

        self.adc_mode_btn = QtWidgets.QPushButton("ADC Stream")
        self.adc_mode_btn.setToolTip("Return UART output to the binary ADC stream.")
        self.adc_mode_btn.clicked.connect(self.send_adc_mode_command)
        self.adc_mode_btn.setEnabled(False)
        mode_controls.addWidget(self.adc_mode_btn)

        self.adc_reset_btn = QtWidgets.QPushButton("ADC Reset")
        self.adc_reset_btn.setToolTip("Force AD7779 hardware reset and SPI software reset, then leave it ready for MODE ADC.")
        self.adc_reset_btn.clicked.connect(self.send_adc_reset_command)
        self.adc_reset_btn.setEnabled(False)
        mode_controls.addWidget(self.adc_reset_btn)

        self.adc_diag_btn = QtWidgets.QPushButton("ADC Diag")
        self.adc_diag_btn.setToolTip("Run non-streaming AD7779 SPI/register diagnostic logging.")
        self.adc_diag_btn.clicked.connect(self.send_adc_diag_command)
        self.adc_diag_btn.setEnabled(False)
        mode_controls.addWidget(self.adc_diag_btn)

        self.clear_control_log_btn = QtWidgets.QPushButton("Clear Output")
        self.clear_control_log_btn.clicked.connect(self.clear_control_log)
        mode_controls.addWidget(self.clear_control_log_btn)
        mode_controls.addStretch(1)

        self.control_log = QtWidgets.QPlainTextEdit()
        self.control_log.setReadOnly(True)
        self.control_log.setMaximumBlockCount(500)
        self.control_log.setPlaceholderText("Control command log")
        control_layout.addWidget(self.control_log, stretch=1)

        imu_test_controls = QtWidgets.QHBoxLayout()
        imu_test_layout.addLayout(imu_test_controls)

        imu_test_controls.addWidget(QtWidgets.QLabel("Length min:"))
        self.imu_test_duration_min = QtWidgets.QDoubleSpinBox()
        self.imu_test_duration_min.setRange(0.1, 240.0)
        self.imu_test_duration_min.setValue(10.0)
        self.imu_test_duration_min.setDecimals(1)
        imu_test_controls.addWidget(self.imu_test_duration_min)

        imu_test_controls.addWidget(QtWidgets.QLabel("Sample s:"))
        self.imu_test_interval_s = QtWidgets.QDoubleSpinBox()
        self.imu_test_interval_s.setRange(0.5, 60.0)
        self.imu_test_interval_s.setValue(2.0)
        self.imu_test_interval_s.setDecimals(1)
        imu_test_controls.addWidget(self.imu_test_interval_s)

        imu_test_controls.addWidget(QtWidgets.QLabel("Stable s:"))
        self.imu_test_stable_window_s = QtWidgets.QDoubleSpinBox()
        self.imu_test_stable_window_s.setRange(2.0, 60.0)
        self.imu_test_stable_window_s.setValue(10.0)
        self.imu_test_stable_window_s.setDecimals(1)
        imu_test_controls.addWidget(self.imu_test_stable_window_s)

        imu_test_controls.addWidget(QtWidgets.QLabel("New avg deg:"))
        self.imu_test_new_avg_deg = QtWidgets.QDoubleSpinBox()
        self.imu_test_new_avg_deg.setRange(0.05, 5.0)
        self.imu_test_new_avg_deg.setValue(0.3)
        self.imu_test_new_avg_deg.setDecimals(2)
        imu_test_controls.addWidget(self.imu_test_new_avg_deg)

        self.imu_test_start_btn = QtWidgets.QPushButton("Start")
        self.imu_test_start_btn.clicked.connect(self.start_imu_test)
        self.imu_test_start_btn.setEnabled(False)
        imu_test_controls.addWidget(self.imu_test_start_btn)

        self.imu_test_stop_btn = QtWidgets.QPushButton("Stop")
        self.imu_test_stop_btn.clicked.connect(self.stop_imu_test)
        self.imu_test_stop_btn.setEnabled(False)
        imu_test_controls.addWidget(self.imu_test_stop_btn)

        self.imu_test_load_btn = QtWidgets.QPushButton("Load CSV")
        self.imu_test_load_btn.clicked.connect(self.load_imu_test_csv)
        imu_test_controls.addWidget(self.imu_test_load_btn)
        imu_test_controls.addStretch(1)

        self.imu_test_progress = QtWidgets.QProgressBar()
        self.imu_test_progress.setRange(0, 1000)
        self.imu_test_progress.setValue(0)
        imu_test_layout.addWidget(self.imu_test_progress)

        self.imu_test_summary = QtWidgets.QPlainTextEdit()
        self.imu_test_summary.setReadOnly(True)
        self.imu_test_summary.setPlaceholderText("IMU tilt test summary")
        imu_test_layout.addWidget(self.imu_test_summary, stretch=1)

        self.refresh_ports()
        if args.port:
            self.port_combo.setCurrentText(args.port)
            self.connect()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(int(1000 / args.refresh_hz))

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API name
        self.disconnect()
        event.accept()

    def refresh_ports(self) -> None:
        current = self.port_combo.currentText().strip() or self.args.port
        self.port_combo.blockSignals(True)
        self.port_combo.clear()

        ports = sorted(list_ports.comports(), key=lambda p: p.device)
        for p in ports:
            label = p.device
            if p.description and p.description != "n/a":
                label = f"{p.device}  ({p.description})"
            self.port_combo.addItem(label, p.device)

        if current:
            idx = self.port_combo.findData(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
            else:
                self.port_combo.setCurrentText(current)
        elif ports:
            preferred = next((i for i, p in enumerate(ports) if "/dev/cu." in p.device), 0)
            self.port_combo.setCurrentIndex(preferred)

        self.port_combo.blockSignals(False)

    def selected_port(self) -> str:
        data = self.port_combo.currentData()
        if isinstance(data, str) and data:
            return data
        text = self.port_combo.currentText().strip()
        if "  (" in text:
            return text.split("  (", 1)[0]
        return text

    def clear_buffers(self) -> None:
        self.queue.clear()
        self.write_idx = 0
        self.count = 0
        self.last_queue_drained = 0
        self.held_samples_inserted = 0
        self.held_samples_replaced = 0
        self.next_fft_update_at = 0.0
        self.next_stats_update_at = 0.0
        self.last_stats_text = ""
        for curve in self.time_curves + self.fft_curves:
            curve.setData([], [])

    def active_channels(self) -> list[int]:
        return [i for i, cb in enumerate(self.channel_checks) if cb.isChecked()]

    def channel_stats(self, channels: np.ndarray, active: list[int]) -> list[dict[str, float]]:
        stats = []
        for ch in active:
            values = channels[ch]
            values = values[np.isfinite(values)]
            if values.size < 2:
                continue
            stats.append(
                {
                    "channel": ch,
                    "samples": float(values.size),
                    "mean_mv": float(np.mean(values)),
                    "rms_noise_mv": float(np.std(values)),
                    "peak_to_peak_mv": float(np.max(values) - np.min(values)),
                    "min_mv": float(np.min(values)),
                    "max_mv": float(np.max(values)),
                }
            )
        return stats

    def format_stats(self, stats: list[dict[str, float]]) -> str:
        if not stats:
            return "Stats: select at least one channel with valid data."
        unit = self.value_unit()
        parts = []
        for row in stats:
            parts.append(
                f"CH{int(row['channel'])}: "
                f"mean {row['mean_mv']:+.3f} {unit}, "
                f"rms {row['rms_noise_mv']:.3f} {unit}, "
                f"p2p {row['peak_to_peak_mv']:.3f} {unit}"
            )
        return " | ".join(parts)

    def value_unit(self) -> str:
        return "codes" if self.raw_codes else "mV"

    @staticmethod
    def adc_status_values(status: np.ndarray) -> np.ndarray:
        return np.asarray(status, dtype=np.int64) & ~HELD_SAMPLE_STATUS

    @staticmethod
    def held_status_count(status: np.ndarray) -> int:
        return int(np.count_nonzero(np.asarray(status, dtype=np.int64) & HELD_SAMPLE_STATUS))

    def save_stats_csv(self) -> None:
        frames, host_times, channels, status = self.ordered_data()
        active = self.active_channels()
        stats = self.channel_stats(channels, active)
        if not stats:
            self.info.setText("No valid stats to save yet.")
            return

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        default_name = f"mag_scope_stats_{timestamp}.csv"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save channel stats",
            default_name,
            "CSV files (*.csv);;All files (*)",
        )
        if not path:
            return

        sample_rate, frame_step, missed = self.timing_stats(frames)
        adc_status = self.adc_status_values(status)
        status_max = int(np.max(adc_status)) if len(adc_status) else 0
        held_count = self.held_status_count(status)
        fieldnames = [
            "timestamp",
            "port",
            "baud",
            "window_s",
            "scale_mode",
            "sample_rate_hz",
            "frame_step",
            "missed_estimate",
            "adc_status_max_hex",
            "held_samples_in_window",
            "channel",
            "gain_x",
            "samples",
            "mean_mv",
            "rms_noise_mv",
            "peak_to_peak_mv",
            "min_mv",
            "max_mv",
        ]
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for row in stats:
                    writer.writerow(
                        {
                            "timestamp": timestamp,
                            "port": self.reader.port if self.reader else "",
                            "baud": self.reader.baud if self.reader else "",
                            "window_s": self.window_box.value(),
                            "scale_mode": (
                                "raw_codes" if self.raw_codes
                                else "input_referred" if self.input_referred
                                else "adc_pga"
                            ),
                            "sample_rate_hz": sample_rate,
                            "frame_step": frame_step,
                            "missed_estimate": missed,
                            "adc_status_max_hex": f"0x{status_max:02X}",
                            "held_samples_in_window": held_count,
                            "gain_x": self.channel_gains[int(row["channel"])],
                            **row,
                        }
                    )
        except OSError as exc:
            self.info.setText(f"Stats save failed: {exc}")
            return

        self.info.setText(f"Saved channel stats to {path}.")

    def save_scope_csv(self) -> None:
        frames, host_times, channels, status = self.ordered_data()
        if len(frames) == 0:
            self.info.setText("No scope data to save yet.")
            return
        active = self.active_channels()
        if not active:
            self.info.setText("Select at least one visible channel before saving scope data.")
            return

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        default_name = f"mag_scope_data_{timestamp}.csv"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save visible scope data",
            default_name,
            "CSV files (*.csv);;All files (*)",
        )
        if not path:
            return

        t = self.frame_times_s(frames)
        suffix = "raw" if self.raw_codes else "mv"
        fieldnames = ["time_s", "frame"] + [f"ch{ch}_{suffix}" for ch in active] + ["status"]
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for i in range(len(frames)):
                    row = {
                        "time_s": f"{t[i]:.9g}",
                        "frame": int(frames[i]),
                        "status": int(status[i]),
                    }
                    for ch in active:
                        row[f"ch{ch}_{suffix}"] = f"{channels[ch, i]:.9g}"
                    writer.writerow(row)
        except OSError as exc:
            self.info.setText(f"Scope CSV save failed: {exc}")
            return

        self.info.setText(f"Saved visible scope data to {path}.")

    def update_channel_visibility(self) -> None:
        for ch in range(CHANNEL_COUNT):
            visible = self.channel_checks[ch].isChecked()
            if hasattr(self, "time_curves"):
                self.time_curves[ch].setVisible(visible)
            if hasattr(self, "fft_curves"):
                self.fft_curves[ch].setVisible(visible)

    def set_all_channels(self, checked: bool) -> None:
        for cb in self.channel_checks:
            cb.setChecked(checked)

    def set_voltage_scale_mode(self) -> None:
        self.input_referred = self.input_referred_check.isChecked()
        self.raw_codes = self.raw_codes_check.isChecked()
        if self.reader:
            self.reader.set_input_referred(self.input_referred)
            self.reader.set_raw_codes(self.raw_codes)
        if self.raw_codes:
            self.time_plot.setLabel("left", "ADC raw code", units="")
            self.fft_plot.setLabel("left", "ADC raw amplitude", units="")
            self.append_control_log("Scale: raw ADC codes")
        elif self.input_referred:
            self.time_plot.setLabel("left", "ADC input voltage", units="mV")
            self.fft_plot.setLabel("left", "ADC input amplitude", units="mV")
            self.append_control_log("Scale: input-referred mV")
        else:
            self.time_plot.setLabel("left", "ADC/PGA voltage", units="mV")
            self.fft_plot.setLabel("left", "ADC/PGA amplitude", units="mV")
            self.append_control_log("Scale: ADC/PGA mV")
        self.clear_buffers()

    def update_fft_xrange(self) -> None:
        if hasattr(self, "fft_plot"):
            self.fft_plot.setXRange(0, self.fft_max_box.value(), padding=0.0)

    def set_nominal_adc_rate(self, rate_hz: float) -> None:
        rate_hz = max(float(ADC_RATE_MIN_HZ), min(float(ADC_RATE_MAX_HZ), float(rate_hz)))
        self.args.adc_rate = rate_hz
        self.args.sample_rate = rate_hz

        max_window_s = max(1.0, self.args.buffer / max(rate_hz, 1.0))
        self.window_box.setRange(1.0, max_window_s)
        if self.window_box.value() > max_window_s:
            self.window_box.setValue(max_window_s)
        self.window_box.setToolTip(
            f"Limited by the in-memory buffer: {self.args.buffer} samples at "
            f"{rate_hz:g} samples/s."
        )

        nyquist = max(1.0, rate_hz / 2.0)
        self.fft_max_box.setRange(1.0, nyquist)
        if self.fft_max_box.value() > nyquist:
            self.fft_max_box.setValue(nyquist)
        self.update_fft_xrange()

    def connect(self) -> None:
        port = self.selected_port()
        if not port:
            self.info.setText("Select a serial port, then click Connect.")
            return

        self.disconnect()
        self.clear_buffers()
        baud = int(self.baud_box.value())
        self.reader = SerialReader(
            port,
            baud,
            self.queue,
            accept_bad_checksum=False,
        )
        self.reader.start()
        self.connect_btn.setText("Disconnect")
        self.port_combo.setEnabled(False)
        self.refresh_ports_btn.setEnabled(False)
        self.baud_box.setEnabled(False)
        self.adc_rate_btn.setEnabled(True)
        self.zero_btn.setEnabled(True)
        self.set_reset_btn.setEnabled(True)
        self.set_btn.setEnabled(True)
        self.reset_btn.setEnabled(True)
        self.auto_polarity_btn.setEnabled(True)
        self.enable_9v.setEnabled(True)
        self.enable_neg5v.setEnabled(True)
        self.dac_enable.setEnabled(True)
        self.save_stats_btn.setEnabled(True)
        self.save_scope_btn.setEnabled(True)
        self.control_mode_btn.setEnabled(True)
        self.adc_mode_btn.setEnabled(True)
        self.adc_reset_btn.setEnabled(True)
        self.adc_diag_btn.setEnabled(True)
        for cb in self.adc_pin_checks:
            cb.setEnabled(True)
        self.adc_pins_off_btn.setEnabled(True)
        for combo in self.gain_combos:
            combo.setEnabled(True)
        self.imu_test_start_btn.setEnabled(True)
        self.reader.write_command(b"9V 0\n")
        self.sync_adc_rate_setting()
        self.append_control_log(f"Connected to {port} at {baud}.")
        if self.reader.write_command(b"MODE ADC\n"):
            self.append_control_log("> MODE ADC")
            self.info.setText(f"Connected to {port} at {baud}. Requested ADC stream...")
        else:
            self.info.setText(f"Connected to {port} at {baud}. Waiting for frames...")
        self.reader.set_input_referred(self.input_referred)
        self.reader.set_raw_codes(self.raw_codes)
        self.sync_adc_gain_settings()
        if THERMAL_ISOLATION_UI:
            self.dac_enable.setEnabled(False)

    def disconnect(self) -> None:
        if self.imu_test_running:
            self.finish_imu_test(manual=True)
        self.stop_auto_polarity()
        if self.reader:
            self.reader.stop()
            self.reader.join(timeout=1.0)
            self.reader = None
        self.connect_btn.setText("Connect")
        self.port_combo.setEnabled(True)
        self.refresh_ports_btn.setEnabled(True)
        self.baud_box.setEnabled(True)
        self.adc_rate_btn.setEnabled(False)
        self.zero_btn.setEnabled(False)
        self.set_reset_btn.setEnabled(False)
        self.set_btn.setEnabled(False)
        self.reset_btn.setEnabled(False)
        self.auto_polarity_btn.setEnabled(False)
        self.enable_9v.setEnabled(False)
        self.enable_neg5v.setEnabled(False)
        self.dac_enable.setEnabled(False)
        self.save_stats_btn.setEnabled(False)
        self.save_scope_btn.setEnabled(False)
        self.control_mode_btn.setEnabled(False)
        self.adc_mode_btn.setEnabled(False)
        self.adc_reset_btn.setEnabled(False)
        self.adc_diag_btn.setEnabled(False)
        for cb in self.adc_pin_checks:
            cb.setEnabled(False)
        self.adc_pins_off_btn.setEnabled(False)
        for combo in self.gain_combos:
            combo.setEnabled(False)
        self.imu_test_start_btn.setEnabled(False)
        self.imu_test_stop_btn.setEnabled(False)
        self.stats_label.setText("")

    def append_control_log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.control_log.appendPlainText(f"{stamp}  {text}")

    def clear_control_log(self) -> None:
        self.control_log.clear()

    def process_control_line(self, line: str) -> None:
        self.append_control_log(line)
        if self.imu_test_running:
            self.capture_imu_test_line(line)

    def start_imu_test(self) -> None:
        if not self.reader:
            self.info.setText("Connect before starting an IMU test.")
            return

        out_dir = Path(__file__).resolve().parent / "imu_tests"
        out_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.imu_test_output_path = out_dir / f"imu_tilt_test_{timestamp}.csv"
        self.imu_test_rows = []
        self.imu_test_pending_lsm = None
        self.imu_test_current_segment = []
        self.imu_test_segment_count = 0
        self.imu_test_last_event = "waiting for first stable average"
        self.imu_test_stable = False
        self.imu_test_started_at = time.monotonic()
        self.imu_test_next_cycle = 0.0
        self.imu_test_running = True
        self.imu_test_progress.setValue(0)
        self.imu_test_summary.setPlainText(f"Running IMU tilt test...\nSaving to {self.imu_test_output_path}")
        self.imu_test_start_btn.setEnabled(False)
        self.imu_test_stop_btn.setEnabled(True)
        self.tabs.setCurrentWidget(self.imu_test_tab)

        self.reader.write_command(b"MODE CTRL\n")
        self.append_control_log("> MODE CTRL")
        self.run_imu_test_cycle()

    def stop_imu_test(self) -> None:
        if not self.imu_test_running:
            return
        self.finish_imu_test(manual=True)

    def update_imu_test(self) -> None:
        if not self.imu_test_running:
            return

        elapsed = time.monotonic() - self.imu_test_started_at
        duration = self.imu_test_duration_min.value() * 60.0
        remaining = max(0.0, duration - elapsed)
        progress = int(min(1000.0, (elapsed / max(duration, 0.001)) * 1000.0))
        self.imu_test_progress.setValue(progress)
        self.imu_test_progress.setFormat(
            f"{int(remaining // 60):02d}:{int(remaining % 60):02d} remaining"
        )

        if elapsed >= duration:
            self.finish_imu_test(manual=False)
        elif time.monotonic() >= self.imu_test_next_cycle:
            self.run_imu_test_cycle()
        else:
            self.update_imu_test_live_summary()

    def run_imu_test_cycle(self) -> None:
        if not self.imu_test_running or not self.reader:
            return
        self.reader.write_command(b"LSM6DSV ON\n")
        QtCore.QTimer.singleShot(250, self.send_scl3300_imu_test_command)
        self.imu_test_next_cycle = time.monotonic() + self.imu_test_interval_s.value()

    def send_scl3300_imu_test_command(self) -> None:
        if self.imu_test_running and self.reader:
            self.reader.write_command(b"SCL3300 ON\n")

    @staticmethod
    def acc_tilt_x_deg(ax: float, ay: float, az: float) -> float:
        return math.degrees(math.atan2(ax, math.sqrt((ay * ay) + (az * az))))

    @staticmethod
    def acc_tilt_y_deg(ax: float, ay: float, az: float) -> float:
        return math.degrees(math.atan2(ay, math.sqrt((ax * ax) + (az * az))))

    def capture_imu_test_line(self, line: str) -> None:
        lsm = LSM6DSV_SAMPLE_RE.search(line)
        if lsm:
            ax, ay, az, gx, gy, gz, temp = [float(v) for v in lsm.groups()]
            self.imu_test_pending_lsm = {
                "elapsed_s": time.monotonic() - self.imu_test_started_at,
                "lsm_ax_g": ax,
                "lsm_ay_g": ay,
                "lsm_az_g": az,
                "lsm_acc_norm_g": math.sqrt((ax * ax) + (ay * ay) + (az * az)),
                "lsm_tilt_x_deg": self.acc_tilt_x_deg(ax, ay, az),
                "lsm_tilt_y_deg": self.acc_tilt_y_deg(ax, ay, az),
                "lsm_gx_dps": gx,
                "lsm_gy_dps": gy,
                "lsm_gz_dps": gz,
                "lsm_gyro_norm_dps": math.sqrt((gx * gx) + (gy * gy) + (gz * gz)),
                "lsm_temp_c": temp,
            }
            return

        scl = SCL3300_SAMPLE_RE.search(line)
        if scl and self.imu_test_pending_lsm:
            ax, ay, az, ang_x, ang_y, ang_z, temp = [float(v) for v in scl.groups()]
            row = dict(self.imu_test_pending_lsm)
            row.update(
                {
                    "elapsed_s": time.monotonic() - self.imu_test_started_at,
                    "scl_ax_g": ax,
                    "scl_ay_g": ay,
                    "scl_az_g": az,
                    "scl_acc_norm_g": math.sqrt((ax * ax) + (ay * ay) + (az * az)),
                    "scl_ang_x_deg": ang_x,
                    "scl_ang_y_deg": ang_y,
                    "scl_ang_z_deg": ang_z,
                    "scl_temp_c": temp,
                    "err_x_deg": row["lsm_tilt_x_deg"] - ang_x,
                    "err_y_deg": row["lsm_tilt_y_deg"] - ang_y,
                }
            )
            self.imu_test_rows.append(row)
            self.update_gated_average(row)
            self.imu_test_pending_lsm = None

    def recent_imu_rows(self) -> list[dict[str, float]]:
        if not self.imu_test_rows:
            return []
        cutoff = self.imu_test_rows[-1]["elapsed_s"] - self.imu_test_stable_window_s.value()
        return [row for row in self.imu_test_rows if row["elapsed_s"] >= cutoff]

    @staticmethod
    def values_from_rows(rows: list[dict[str, float]], key: str) -> np.ndarray:
        return np.array([row[key] for row in rows], dtype=np.float64)

    def recent_window_is_stable(self, rows: list[dict[str, float]]) -> bool:
        min_samples = max(5, int(math.ceil(self.imu_test_stable_window_s.value() / self.imu_test_interval_s.value())))
        if len(rows) < min_samples:
            return False

        lsm_x = self.values_from_rows(rows, "lsm_tilt_x_deg")
        lsm_y = self.values_from_rows(rows, "lsm_tilt_y_deg")
        lsm_norm = self.values_from_rows(rows, "lsm_acc_norm_g")
        gyro = self.values_from_rows(rows, "lsm_gyro_norm_dps")

        return (
            float(np.std(lsm_x)) < 0.20 and
            float(np.std(lsm_y)) < 0.20 and
            float(np.std(lsm_norm)) < 0.012 and
            float(np.percentile(gyro, 95)) < 1.5
        )

    def segment_mean(self, key: str) -> float:
        if not self.imu_test_current_segment:
            return float("nan")
        return float(np.mean(self.values_from_rows(self.imu_test_current_segment, key)))

    def update_gated_average(self, row: dict[str, float]) -> None:
        stable = self.recent_window_is_stable(self.recent_imu_rows())
        self.imu_test_stable = stable
        row["stable"] = 1.0 if stable else 0.0

        if not stable:
            row["segment_id"] = float(self.imu_test_segment_count)
            row["segment_samples"] = float(len(self.imu_test_current_segment))
            row["avg_lsm_tilt_x_deg"] = self.segment_mean("lsm_tilt_x_deg")
            row["avg_lsm_tilt_y_deg"] = self.segment_mean("lsm_tilt_y_deg")
            row["avg_scl_ang_x_deg"] = self.segment_mean("scl_ang_x_deg")
            row["avg_scl_ang_y_deg"] = self.segment_mean("scl_ang_y_deg")
            row["avg_err_x_deg"] = float("nan")
            row["avg_err_y_deg"] = float("nan")
            return

        if not self.imu_test_current_segment:
            self.imu_test_segment_count += 1
            self.imu_test_current_segment = [row]
            self.imu_test_last_event = f"started average {self.imu_test_segment_count}"
        else:
            avg_x = self.segment_mean("lsm_tilt_x_deg")
            avg_y = self.segment_mean("lsm_tilt_y_deg")
            moved = (
                abs(row["lsm_tilt_x_deg"] - avg_x) > self.imu_test_new_avg_deg.value() or
                abs(row["lsm_tilt_y_deg"] - avg_y) > self.imu_test_new_avg_deg.value()
            )
            if moved:
                self.imu_test_segment_count += 1
                self.imu_test_current_segment = [row]
                self.imu_test_last_event = f"tilt changed, started average {self.imu_test_segment_count}"
            else:
                self.imu_test_current_segment.append(row)

        row["segment_id"] = float(self.imu_test_segment_count)
        row["segment_samples"] = float(len(self.imu_test_current_segment))
        row["avg_lsm_tilt_x_deg"] = self.segment_mean("lsm_tilt_x_deg")
        row["avg_lsm_tilt_y_deg"] = self.segment_mean("lsm_tilt_y_deg")
        row["avg_scl_ang_x_deg"] = self.segment_mean("scl_ang_x_deg")
        row["avg_scl_ang_y_deg"] = self.segment_mean("scl_ang_y_deg")
        row["avg_err_x_deg"] = row["avg_lsm_tilt_x_deg"] - row["avg_scl_ang_x_deg"]
        row["avg_err_y_deg"] = row["avg_lsm_tilt_y_deg"] - row["avg_scl_ang_y_deg"]
        self.update_imu_test_live_summary()

    def update_imu_test_live_summary(self) -> None:
        if not self.imu_test_running:
            return

        elapsed = time.monotonic() - self.imu_test_started_at
        duration = self.imu_test_duration_min.value() * 60.0
        remaining = max(0.0, duration - elapsed)
        state = "stable" if self.imu_test_stable else "collecting / moving"
        avg_x = self.segment_mean("lsm_tilt_x_deg")
        avg_y = self.segment_mean("lsm_tilt_y_deg")
        seg_n = len(self.imu_test_current_segment)

        lines = [
            f"remaining: {int(remaining // 60):02d}:{int(remaining % 60):02d}",
            f"samples: {len(self.imu_test_rows)}",
            f"state: {state}",
            f"average segment: {self.imu_test_segment_count} ({seg_n} stable samples)",
            f"current LSM average: X={avg_x:+.4f} deg  Y={avg_y:+.4f} deg",
            f"last event: {self.imu_test_last_event}",
            f"saving to: {self.imu_test_output_path}",
        ]
        self.imu_test_summary.setPlainText("\n".join(lines))

    def finish_imu_test(self, manual: bool) -> None:
        self.imu_test_running = False
        self.imu_test_start_btn.setEnabled(self.reader is not None)
        self.imu_test_stop_btn.setEnabled(False)
        self.imu_test_progress.setValue(1000)

        if not self.imu_test_output_path or not self.imu_test_rows:
            self.imu_test_summary.setPlainText("IMU test stopped without paired samples.")
            return

        fieldnames = list(self.imu_test_rows[0].keys())
        with self.imu_test_output_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.imu_test_rows)

        prefix = "Stopped" if manual else "Completed"
        summary = self.analyze_imu_test_csv(self.imu_test_output_path)
        self.imu_test_summary.setPlainText(f"{prefix}: {self.imu_test_output_path}\n\n{summary}")

    def load_imu_test_csv(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Load IMU tilt test",
            str(Path(__file__).resolve().parent / "imu_tests"),
            "CSV files (*.csv);;All files (*)",
        )
        if not path:
            return
        summary = self.analyze_imu_test_csv(Path(path))
        self.imu_test_summary.setPlainText(f"Loaded: {path}\n\n{summary}")

    @staticmethod
    def rms(values: np.ndarray) -> float:
        return float(np.sqrt(np.mean(values * values))) if values.size else float("nan")

    def analyze_imu_test_csv(self, path: Path) -> str:
        try:
            with path.open("r", newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
        except OSError as exc:
            return f"Could not read CSV: {exc}"
        if not rows:
            return "No samples in CSV."

        cols: dict[str, np.ndarray] = {}
        for key in rows[0].keys():
            vals = []
            for row in rows:
                try:
                    vals.append(float(row[key]))
                except (TypeError, ValueError):
                    pass
            cols[key] = np.array(vals, dtype=np.float64)

        err_x = cols.get("err_x_deg", np.empty(0))
        err_y = cols.get("err_y_deg", np.empty(0))
        elapsed = cols.get("elapsed_s", np.empty(0))
        gyro = cols.get("lsm_gyro_norm_dps", np.empty(0))
        lsm_norm = cols.get("lsm_acc_norm_g", np.empty(0))
        scl_norm = cols.get("scl_acc_norm_g", np.empty(0))
        stable = cols.get("stable", np.empty(0))
        segment_id = cols.get("segment_id", np.empty(0))

        duration = float(np.max(elapsed) - np.min(elapsed)) if elapsed.size else 0.0
        stable_mask = stable > 0.5 if stable.size else np.zeros(len(rows), dtype=bool)
        stable_count = int(np.count_nonzero(stable_mask))
        segment_count = int(len(set(int(v) for v in segment_id[stable_mask] if v > 0))) if segment_id.size else 0
        lines = [
            f"samples: {len(rows)}",
            f"duration: {duration:.1f} s",
            f"stable samples: {stable_count} ({(100.0 * stable_count / max(len(rows), 1)):.1f}%), average segments: {segment_count}",
            f"LSM tilt X - SCL angle X: mean {np.mean(err_x):+.4f} deg, rms {self.rms(err_x):.4f} deg, p2p {np.ptp(err_x):.4f} deg",
            f"LSM tilt Y - SCL angle Y: mean {np.mean(err_y):+.4f} deg, rms {self.rms(err_y):.4f} deg, p2p {np.ptp(err_y):.4f} deg",
            f"LSM |acc|: mean {np.mean(lsm_norm):.5f} g, std {np.std(lsm_norm):.5f} g",
            f"SCL |acc|: mean {np.mean(scl_norm):.5f} g, std {np.std(scl_norm):.5f} g",
            f"LSM gyro norm: mean {np.mean(gyro):.4f} dps, p95 {np.percentile(gyro, 95):.4f} dps",
        ]
        return "\n".join(lines)

    def send_control_command(self, command: bytes, description: str) -> bool:
        if not self.reader:
            self.info.setText("Connect before sending a control command.")
            self.append_control_log(f"Not connected: {description}.")
            return False
        if self.reader.write_command(command):
            rendered = command.decode("ascii", errors="replace").strip()
            self.info.setText(f"Sent {description}.")
            self.append_control_log(f"> {rendered}")
            return True
        self.info.setText(f"{description} failed: serial port is not ready.")
        self.append_control_log(f"Failed: {description}.")
        return False

    def send_control_mode_command(self) -> None:
        if self.send_control_command(b"MODE CTRL\n", "control mode"):
            self.tabs.setCurrentWidget(self.control_tab)

    def send_adc_mode_command(self) -> None:
        if self.send_control_command(b"MODE ADC\n", "ADC stream mode"):
            if self.reader:
                self.reader.set_input_referred(self.input_referred)
                self.reader.set_raw_codes(self.raw_codes)
            self.sync_adc_gain_settings()
            self.tabs.setCurrentWidget(self.scope_tab)

    def send_adc_reset_command(self) -> None:
        if self.send_control_command(b"ADC RESET\n", "ADC reset/recovery"):
            self.clear_buffers()
            self.tabs.setCurrentWidget(self.control_tab)

    def send_adc_diag_command(self) -> None:
        if self.send_control_command(b"ADC DIAG\n", "ADC diagnostic"):
            self.clear_buffers()
            self.tabs.setCurrentWidget(self.control_tab)

    def send_adc_rate_command(self) -> None:
        rate = int(round(self.adc_rate_box.value()))
        cmd = f"ADC ODR {rate}\n".encode("ascii")
        if self.send_control_command(cmd, f"ADC ODR {rate} Hz"):
            self.set_nominal_adc_rate(float(rate))
            self.clear_buffers()
            if self.reader:
                self.reader.reset_average_sample_rate()
            self.plot_hold_until = time.monotonic() + 0.5
            self.info.setText(f"Requested ADC ODR {rate} Hz; scope timing updated.")

    def sync_adc_rate_setting(self) -> None:
        if not self.reader:
            return
        rate = int(round(self.adc_rate_box.value()))
        self.set_nominal_adc_rate(float(rate))
        cmd = f"ADC ODR {rate}\n".encode("ascii")
        if self.reader.write_command(cmd):
            self.reader.reset_average_sample_rate()
            self.append_control_log(f"> ADC ODR {rate}")

    def send_adc_pin_command(self, sr_pin: int, name: str) -> None:
        if not self.reader:
            return
        cb = self.sender()
        level = 1 if isinstance(cb, QtWidgets.QCheckBox) and cb.isChecked() else 0
        cmd = f"SR PIN {sr_pin} {level}\n".encode("ascii")
        self.send_control_command(cmd, f"{name} {'high/on' if level else 'low/off'}")

    def send_adc_pins_off_command(self) -> None:
        if not self.reader:
            return
        for cb in self.adc_pin_checks:
            cb.blockSignals(True)
            cb.setChecked(False)
            cb.blockSignals(False)
        for _label, pin, _tooltip in ADC_SR_PINS:
            cmd = f"SR PIN {pin} 0\n".encode("ascii")
            self.send_control_command(cmd, f"ADC SR pin {pin} low/off")

    def send_gain_command(self, ch: int) -> None:
        if not hasattr(self, "gain_combos"):
            return
        gain = ADC_GAINS[self.gain_combos[ch].currentIndex()]
        cmd = f"ADC GAIN {ch} {gain}\n".encode("ascii")
        if self.send_control_command(cmd, f"CH{ch} gain x{gain}"):
            self.channel_gains[ch] = gain
            if self.reader:
                self.reader.set_channel_gain(ch, gain)
            self.info.setText(f"CH{ch} gain set to x{gain}.")

    def sync_adc_gain_settings(self) -> None:
        if not self.reader or not hasattr(self, "gain_combos"):
            return
        for ch, combo in enumerate(self.gain_combos):
            gain = ADC_GAINS[combo.currentIndex()]
            self.channel_gains[ch] = gain
            self.reader.set_channel_gain(ch, gain)
            cmd = f"ADC GAIN {ch} {gain}\n".encode("ascii")
            if self.reader.write_command(cmd):
                self.append_control_log(f"> ADC GAIN {ch} {gain}")

    def send_zero_command(self) -> None:
        if not self.reader:
            self.info.setText("Connect before sending ZERO.")
            return
        if self.reader.write_command(b"ZERO\n"):
            self.info.setText("Sent ZERO command.")
        else:
            self.info.setText("ZERO command failed: serial port is not ready.")

    def send_set_reset_command(self) -> None:
        if not self.reader:
            self.info.setText("Connect before sending SETRESET.")
            return
        if self.reader.write_command(b"SETRESET\n"):
            self.info.setText("Sent SETRESET command.")
        else:
            self.info.setText("SETRESET command failed: serial port is not ready.")

    def send_set_command(self) -> None:
        if not self.reader:
            self.info.setText("Connect before sending SET.")
            return
        if self.reader.write_command(b"SET\n"):
            self.info.setText("Sent SET command.")
        else:
            self.info.setText("SET command failed: serial port is not ready.")

    def send_reset_command(self) -> None:
        if not self.reader:
            self.info.setText("Connect before sending RESET.")
            return
        if self.reader.write_command(b"RESET\n"):
            self.info.setText("Sent RESET command.")
        else:
            self.info.setText("RESET command failed: serial port is not ready.")

    def toggle_auto_polarity(self, enabled: bool) -> None:
        self.auto_polarity_btn.setText("Stop Polarity" if enabled else "Auto Polarity")
        if enabled:
            if not self.reader:
                self.info.setText("Connect before starting auto polarity.")
                self.stop_auto_polarity()
                return
            self.auto_polarity_next_set = True
            self.send_next_auto_polarity_command()
            self.auto_polarity_timer.start()
        else:
            self.auto_polarity_timer.stop()
            self.info.setText("Auto polarity stopped.")

    def stop_auto_polarity(self) -> None:
        self.auto_polarity_timer.stop()
        self.auto_polarity_next_set = True
        if self.auto_polarity_btn.isChecked():
            self.auto_polarity_btn.blockSignals(True)
            self.auto_polarity_btn.setChecked(False)
            self.auto_polarity_btn.blockSignals(False)
        self.auto_polarity_btn.setText("Auto Polarity")

    def send_next_auto_polarity_command(self) -> None:
        if not self.reader:
            self.stop_auto_polarity()
            self.info.setText("Auto polarity stopped: serial port is not connected.")
            return

        if self.auto_polarity_next_set:
            command = b"SET\n"
            description = "auto SET polarity pulse"
        else:
            command = b"RESET\n"
            description = "auto RESET polarity pulse"

        if self.send_control_command(command, description):
            self.auto_polarity_next_set = not self.auto_polarity_next_set
            next_pulse = "SET" if self.auto_polarity_next_set else "RESET"
            self.info.setText(f"Auto polarity sent {command.decode('ascii').strip()}; next {next_pulse} in 1 s.")
        else:
            self.stop_auto_polarity()

    def send_9v_enable_command(self) -> None:
        if not self.reader:
            return
        enabled = self.enable_9v.isChecked()
        cmd = b"9V 1\n" if enabled else b"9V 0\n"
        if self.reader.write_command(cmd):
            state = "enabled" if enabled else "disabled"
            self.info.setText(f"Sent +9V {state} command.")
        else:
            self.info.setText("+9V enable command failed: serial port is not ready.")

    def send_neg5v_enable_command(self) -> None:
        if not self.reader:
            return
        enabled = self.enable_neg5v.isChecked()
        cmd = b"NEG5V 1\n" if enabled else b"NEG5V 0\n"
        if self.reader.write_command(cmd):
            state = "enabled" if enabled else "disabled"
            self.info.setText(f"Sent -5V {state} command.")
            self.append_control_log(f"> {cmd.decode('ascii').strip()}")
        else:
            self.info.setText("-5V enable command failed: serial port is not ready.")

    def send_dac_enable_command(self) -> None:
        enabled = self.dac_enable.isChecked()
        self.dac_enable.setText("DAC On" if enabled else "DAC Off")
        if not self.reader:
            return
        cmd = b"DACEN 1\n" if enabled else b"DACEN 0\n"
        if self.reader.write_command(cmd):
            self.plot_hold_until = time.monotonic() + 1.2
            state = "enabled" if enabled else "disabled"
            self.info.setText(f"Sent DAC {state} command.")
        else:
            self.info.setText("DAC enable command failed: serial port is not ready.")

    def toggle_connection(self) -> None:
        if self.reader:
            self.disconnect()
            self.info.setText("Disconnected.")
        else:
            self.connect()

    def drain_queue(self) -> int:
        drained = 0
        while self.queue:
            s = self.queue.popleft()
            self.discard_trailing_held_samples(s.frame)
            self.append_plot_sample(
                float(s.frame),
                s.t_host,
                np.asarray(s.channels_mv, dtype=np.float64),
                float(s.status),
            )
            drained += 1
        self.last_queue_drained = drained
        return drained

    def discard_trailing_held_samples(self, next_real_frame: int) -> int:
        removed = 0
        while self.count > 0:
            last_idx = (self.write_idx - 1) % self.args.buffer
            last_status = int(self.status[last_idx])
            if (last_status & HELD_SAMPLE_STATUS) == 0:
                break
            if self.frames[last_idx] < float(next_real_frame):
                break
            self.write_idx = last_idx
            self.count -= 1
            removed += 1
        self.held_samples_replaced += removed
        return removed

    def append_plot_sample(
        self,
        frame: float,
        host_time: float,
        channels_mv: np.ndarray,
        status: float,
    ) -> None:
        i = self.write_idx
        self.frames[i] = frame
        self.host_times[i] = host_time
        self.channels[:, i] = channels_mv
        self.status[i] = status
        self.write_idx = (self.write_idx + 1) % self.args.buffer
        self.count = min(self.count + 1, self.args.buffer)

    def insert_held_samples(self) -> int:
        if not self.reader or self.count == 0:
            return 0

        last_idx = (self.write_idx - 1) % self.args.buffer
        last_host_time = float(self.host_times[last_idx])
        if not np.isfinite(last_host_time):
            return 0

        period_s = 1.0 / max(self.args.adc_rate, 1.0)
        now = time.monotonic()
        missing = int((now - last_host_time) / period_s)
        if missing <= 0:
            return 0
        missing = min(missing, HELD_SAMPLE_MAX_BATCH)

        last_frame = float(self.frames[last_idx])
        last_channels = self.channels[:, last_idx].copy()
        last_status = float(int(self.status[last_idx]) | HELD_SAMPLE_STATUS)

        for n in range(missing):
            self.append_plot_sample(
                last_frame + float(n + 1),
                last_host_time + period_s * float(n + 1),
                last_channels,
                last_status,
            )
        self.held_samples_inserted += missing
        return missing

    def ordered_data(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        n = self.count
        if n == 0:
            empty = np.empty(0)
            return empty, empty, np.empty((CHANNEL_COUNT, 0)), empty
        start = (self.write_idx - n) % self.args.buffer
        if start + n <= self.args.buffer:
            frames = self.frames[start : start + n]
            host_times = self.host_times[start : start + n]
            channels = self.channels[:, start : start + n]
            status = self.status[start : start + n]
        else:
            frames = np.concatenate((self.frames[start:], self.frames[: (start + n) % self.args.buffer]))
            host_times = np.concatenate(
                (self.host_times[start:], self.host_times[: (start + n) % self.args.buffer])
            )
            channels = np.concatenate(
                (
                    self.channels[:, start:],
                    self.channels[:, : (start + n) % self.args.buffer],
                ),
                axis=1,
            )
            status = np.concatenate((self.status[start:], self.status[: (start + n) % self.args.buffer]))

        frame_times = self.frame_times_s(frames)
        if len(frame_times):
            mask = frame_times >= -self.window_box.value()
            if np.count_nonzero(mask) >= 2:
                frames = frames[mask]
                host_times = host_times[mask]
                channels = channels[:, mask]
                status = status[mask]
            else:
                max_samples = max(2, int(self.window_box.value() * self.nominal_stream_rate()))
                frames = frames[-max_samples:]
                host_times = host_times[-max_samples:]
                channels = channels[:, -max_samples:]
                status = status[-max_samples:]
        return frames.copy(), host_times.copy(), channels.copy(), status.copy()

    def nominal_stream_rate(self) -> float:
        return self.args.sample_rate

    def frame_times_s(self, frames: np.ndarray) -> np.ndarray:
        if len(frames) == 0:
            return np.empty(0)
        return ((frames - frames[-1]) / self.args.adc_rate).astype(np.float64, copy=False)

    def timing_stats(self, frames: np.ndarray) -> tuple[float, float, int]:
        if len(frames) < 2:
            return self.args.sample_rate, 1.0, 0

        deltas = np.diff(frames)
        deltas = deltas[np.isfinite(deltas) & (deltas > 0)]
        if len(deltas) == 0:
            return self.args.sample_rate, 1.0, 0

        median_delta = float(np.median(deltas))
        sample_rate = self.args.adc_rate / median_delta
        missed = int(np.sum(np.maximum(deltas - median_delta, 0.0) / median_delta))
        return sample_rate, median_delta, missed

    def window_host_sample_rate(self, host_times: np.ndarray) -> float:
        host_times = host_times[np.isfinite(host_times)]
        if len(host_times) < 2:
            return float("nan")
        elapsed = float(host_times[-1] - host_times[0])
        if elapsed <= 0:
            return float("nan")
        return (len(host_times) - 1) / elapsed

    def update_plots(self) -> None:
        drained = self.drain_queue()
        held = 0
        if not drained and self.args.hold_samples:
            held = self.insert_held_samples()

        if not self.reader:
            self.info.setText("Disconnected. Select a port and click Connect.")
            return

        if self.reader.error:
            self.info.setText(f"Serial error: {self.reader.error}")
            self.disconnect()
            return

        for line in self.reader.drain_text_lines():
            self.process_control_line(line)
        self.update_imu_test()

        if time.monotonic() < self.plot_hold_until:
            return

        frames, host_times, channels, status = self.ordered_data()
        if len(frames) < 2:
            hint = ""
            if self.reader.bytes_seen > 1000 and self.reader.binary_packets == 0 and self.reader.text_packets == 0:
                hint = "  No valid frames yet: check baud, flashed firmware, or packet format."
            self.info.setText(
                f"Waiting for data on {self.reader.port} at {self.reader.baud} baud... "
                f"bytes={self.reader.bytes_seen} "
                f"bin={self.reader.binary_packets} text={self.reader.text_packets} "
                f"sync={self.reader.sync_hits} badlen={self.reader.bad_lengths} "
                f"badcrc={self.reader.bad_checksums} preview={self.reader.raw_preview!r}"
                f"{hint}"
            )
            return

        sample_rate, frame_step, missed = self.timing_stats(frames)
        window_host_rate = self.window_host_sample_rate(host_times)
        avg_rx_rate = self.reader.average_sample_rate() if self.reader else float("nan")
        plot_rate = sample_rate
        active = self.active_channels()
        t = self.frame_times_s(frames)
        for ch in range(CHANNEL_COUNT):
            if self.channel_checks[ch].isChecked():
                self.time_curves[ch].setData(t, channels[ch])
        self.time_plot.setXRange(-self.window_box.value(), 0.0, padding=0.0)

        visible = channels[active].reshape(-1) if active else np.empty(0)
        visible = visible[np.isfinite(visible)]
        if visible.size:
            y_min = float(np.min(visible))
            y_max = float(np.max(visible))
            span = max(y_max - y_min, 0.1)
            margin = max(span * 0.08, 0.05)
            self.time_plot.setYRange(y_min - margin, y_max + margin, padding=0.0)

        now = time.monotonic()
        if now >= self.next_fft_update_at:
            self.update_fft(channels, active, plot_rate)
            self.next_fft_update_at = now + (1.0 / max(self.args.fft_refresh_hz, 0.1))
        if now >= self.next_stats_update_at:
            self.last_stats_text = self.format_stats(self.channel_stats(channels, active))
            self.stats_label.setText(self.last_stats_text)
            self.next_stats_update_at = now + (1.0 / max(self.args.stats_refresh_hz, 0.1))

        adc_status = self.adc_status_values(status)
        status_max = int(np.max(adc_status)) if len(adc_status) else 0
        held_count = self.held_status_count(status)
        scale_mode = "raw codes" if self.raw_codes else "input-ref mV" if self.input_referred else "ADC/PGA mV"
        self.info.setText(
            f"{self.reader.port} @ {self.reader.baud} | "
            f"scale {scale_mode} | "
            f"rx {self.reader.samples_seen} samples | "
            f"held total {self.held_samples_inserted} (+{held}, repl {self.held_samples_replaced}, "
            f"window {held_count}) | "
            f"bin {self.reader.binary_packets} text {self.reader.text_packets} "
            f"sync {self.reader.sync_hits} badlen {self.reader.bad_lengths} "
            f"badcrc {self.reader.bad_checksums} | "
            f"frame fs {sample_rate:.2f} Hz | "
            f"avg rx fs {avg_rx_rate:.2f} Hz | "
            f"window host fs {window_host_rate:.2f} Hz | "
            f"plot/fft fs {plot_rate:.2f} Hz | "
            f"frame step {frame_step:.1f} | "
            f"missed~{missed} | "
            f"buffer {len(frames)}/{self.args.buffer} | "
            f"last drain {self.last_queue_drained} | "
            f"adc status max 0x{status_max:02X}"
        )

    def update_fft(self, channels: np.ndarray, active: list[int], sample_rate: float) -> None:
        sample_count = channels.shape[1]
        n = sample_count if self.args.fft_size <= 0 else min(sample_count, self.args.fft_size)
        if n < 32:
            return
        n = 1 << int(math.floor(math.log2(n)))
        window = np.hanning(n)
        scale = np.sum(window) / 2.0
        if scale <= 0:
            return

        freqs = np.fft.rfftfreq(n, d=1.0 / sample_rate)
        mask = freqs <= self.fft_max_box.value()
        active_set = set(active)
        for ch in range(CHANNEL_COUNT):
            if ch not in active_set:
                self.fft_curves[ch].setData([], [])
                continue
            seg = channels[ch, -n:]
            seg = seg[np.isfinite(seg)]
            if len(seg) != n:
                self.fft_curves[ch].setData([], [])
                continue
            seg = seg - np.mean(seg)
            mag = np.abs(np.fft.rfft(seg * window)) / scale
            self.fft_curves[ch].setData(freqs[mask], mag[mask])


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Live serial plotter for magnetic ADC data")
    p.add_argument("--port", default="", help="Serial port, e.g. /dev/cu.usbserial-110")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--sample-rate", type=float, default=1000.0, help="Nominal firmware output rate after decimation")
    p.add_argument("--adc-rate", type=float, default=1000.0, help="AD7779 frame rate used to timestamp frame_idx")
    p.add_argument("--buffer", type=int, default=0, help="Samples kept in memory; 0 keeps the full requested window")
    p.add_argument("--window-s", type=float, default=60.0, help="Visible rolling time window")
    p.add_argument("--queue", type=int, default=65536, help="Reader queue capacity")
    p.add_argument("--refresh-hz", type=float, default=30.0, help="UI redraw rate")
    p.add_argument("--fft-size", type=int, default=0, help="FFT samples; 0 uses the visible time window")
    p.add_argument("--fft-refresh-hz", type=float, default=30.0, help="Maximum FFT redraw rate")
    p.add_argument("--stats-refresh-hz", type=float, default=4.0, help="Maximum channel-stat redraw rate")
    p.add_argument("--fft-max-hz", type=float, default=128.0)
    p.add_argument("--no-held-samples", dest="hold_samples", action="store_false",
                   help="Do not synthesize repeated samples when serial packets pause")
    p.set_defaults(hold_samples=True)
    args = p.parse_args()
    args.adc_rate = max(float(ADC_RATE_MIN_HZ), min(float(ADC_RATE_MAX_HZ), args.adc_rate))
    args.sample_rate = max(float(ADC_RATE_MIN_HZ), min(float(ADC_RATE_MAX_HZ), args.sample_rate))
    min_buffer = max(2, int(math.ceil(args.window_s * max(args.sample_rate, args.adc_rate))))
    if args.buffer <= 0:
        args.buffer = min_buffer
    if args.buffer < min_buffer:
        args.buffer = min_buffer
    return args


def main() -> int:
    args = parse_args()
    app = QtWidgets.QApplication(sys.argv)
    win = SerialScope(args)
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
