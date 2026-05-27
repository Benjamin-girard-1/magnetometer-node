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
CHANNEL_COUNT = 8
ADC_FULL_SCALE_CODE = 8388608.0
ADC_REF_V = 2.5
ADC_GAIN = 1.0
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
        self.serial_lock = threading.Lock()
        self.ser: serial.Serial | None = None
        self.text_lines: collections.deque[str] = collections.deque(maxlen=200)

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
                                scale = (ADC_REF_V * 1000.0) / (ADC_FULL_SCALE_CODE * ADC_GAIN)
                                sample = Sample(
                                    frame=frame,
                                    channels_mv=tuple(v * scale for v in raw),
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

    def drain_text_lines(self) -> list[str]:
        lines: list[str] = []
        while self.text_lines:
            lines.append(self.text_lines.popleft())
        return lines

    def average_sample_rate(self) -> float:
        if self.start_time is None or self.samples_seen < 1000:
            return float("nan")
        elapsed = time.monotonic() - self.start_time
        if elapsed <= 1.0:
            return float("nan")
        return self.samples_seen / elapsed


class SerialScope(QtWidgets.QMainWindow):
    def __init__(self, args: argparse.Namespace):
        super().__init__()
        self.args = args
        self.queue: collections.deque[Sample] = collections.deque(maxlen=args.queue)
        self.reader: SerialReader | None = None
        self.plot_hold_until = 0.0
        self.dac_test_steps: list[tuple[str, list[int]]] = []
        self.dac_test_results: list[dict[str, float | int | str]] = []
        self.dac_test_axis = 0
        self.dac_test_step_idx = 0

        self.frames = np.empty(args.buffer, dtype=np.float64)
        self.host_times = np.empty(args.buffer, dtype=np.float64)
        self.channels = np.empty((CHANNEL_COUNT, args.buffer), dtype=np.float64)
        self.status = np.empty(args.buffer, dtype=np.float64)
        self.write_idx = 0
        self.count = 0
        self.last_queue_drained = 0

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

        self.accept_bad_checksum = QtWidgets.QCheckBox("Accept bad checksum")
        self.accept_bad_checksum.setToolTip("Debug only: parse binary packets even if checksum fails.")
        controls.addWidget(self.accept_bad_checksum)

        self.autorange = QtWidgets.QCheckBox("Auto range")
        self.autorange.setChecked(True)
        controls.addWidget(self.autorange)

        controls.addWidget(QtWidgets.QLabel("Manual span mV:"))
        self.span = QtWidgets.QDoubleSpinBox()
        self.span.setRange(0.01, 100000.0)
        self.span.setValue(args.span)
        self.span.setDecimals(3)
        controls.addWidget(self.span)

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
        self.tabs.addTab(self.control_tab, "Control / SD")

        dac_controls = QtWidgets.QHBoxLayout()
        scope_layout.addLayout(dac_controls)
        dac_controls.addWidget(QtWidgets.QLabel("DAC:"))
        self.last_dac_sent = [DAC_CENTER_CODE, DAC_CENTER_CODE, DAC_CENTER_CODE]
        self.dac_send_timer = QtCore.QTimer(self)
        self.dac_send_timer.setSingleShot(True)
        self.dac_send_timer.setInterval(DAC_LIVE_INTERVAL_MS)
        self.dac_send_timer.timeout.connect(self.send_dac_command)
        self.dac_sliders: list[QtWidgets.QSlider] = []
        self.dac_values: list[QtWidgets.QLabel] = []
        for label in ("X", "Y", "Z"):
            dac_controls.addWidget(QtWidgets.QLabel(label))
            slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
            slider.setRange(DAC_MIN_CODE, DAC_MAX_CODE)
            slider.setValue(DAC_CENTER_CODE)
            slider.setSingleStep(1)
            slider.setPageStep(1)
            slider.setMinimumWidth(160)
            value_label = QtWidgets.QLabel("2048")
            value_label.setMinimumWidth(42)
            slider.valueChanged.connect(self.on_dac_slider_changed)
            slider.sliderReleased.connect(self.send_dac_command)
            self.dac_sliders.append(slider)
            self.dac_values.append(value_label)
            dac_controls.addWidget(slider)
            dac_controls.addWidget(value_label)

        self.dac_center_btn = QtWidgets.QPushButton("Center")
        self.dac_center_btn.clicked.connect(self.center_dac_sliders)
        dac_controls.addWidget(self.dac_center_btn)

        dac_controls.addWidget(QtWidgets.QLabel("Test:"))
        self.dac_test_axis_box = QtWidgets.QComboBox()
        self.dac_test_axis_box.addItems(["X", "Y", "Z"])
        dac_controls.addWidget(self.dac_test_axis_box)

        self.dac_test_btn = QtWidgets.QPushButton("DAC Step Test")
        self.dac_test_btn.setToolTip("Step one DAC axis low/center/high and save per-channel response stats.")
        self.dac_test_btn.clicked.connect(self.start_dac_step_test)
        self.dac_test_btn.setEnabled(False)
        dac_controls.addWidget(self.dac_test_btn)
        dac_controls.addStretch(1)

        channel_controls = QtWidgets.QHBoxLayout()
        scope_layout.addLayout(channel_controls)
        channel_controls.addWidget(QtWidgets.QLabel("Channels:"))
        self.channel_checks: list[QtWidgets.QCheckBox] = []
        for ch in range(CHANNEL_COUNT):
            cb = QtWidgets.QCheckBox(f"CH{ch}")
            cb.setChecked(ch in (1, 2))
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
        channel_controls.addStretch(1)

        self.stats_label = QtWidgets.QLabel()
        self.stats_label.setTextFormat(QtCore.Qt.TextFormat.PlainText)
        self.stats_label.setWordWrap(True)
        self.stats_label.setMaximumHeight(54)
        font = self.stats_label.font()
        font.setFamily("Menlo")
        self.stats_label.setFont(font)
        scope_layout.addWidget(self.stats_label)

        self.time_plot = pg.PlotWidget(title="Time Domain")
        self.time_plot.setLabel("left", "field proxy", units="mV")
        self.time_plot.setLabel("bottom", "time", units="s")
        self.time_plot.showGrid(x=True, y=True, alpha=0.25)
        scope_layout.addWidget(self.time_plot, stretch=3)

        self.time_curves = []
        for ch in range(CHANNEL_COUNT):
            curve = self.time_plot.plot(
                pen=pg.mkPen(CHANNEL_COLORS[ch], width=1),
                name=f"CH{ch}",
            )
            self.time_curves.append(curve)
        self.time_plot.addLegend()

        self.fft_plot = pg.PlotWidget(title="Spectrum")
        self.fft_plot.setLabel("left", "amplitude", units="mV")
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
            self.fft_curves.append(curve)
        self.fft_plot.addLegend()
        self.update_channel_visibility()

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

        self.sd_test_btn = QtWidgets.QPushButton("Run SD Test")
        self.sd_test_btn.setToolTip("Mount the SD card from the ESP32, write the test file, read it back, then idle the mux.")
        self.sd_test_btn.clicked.connect(lambda: self.send_control_command(b"SD\n", "run SD write/read test"))
        self.sd_test_btn.setEnabled(False)
        mode_controls.addWidget(self.sd_test_btn)

        self.sd_probe_all_btn = QtWidgets.QPushButton("Probe Mux")
        self.sd_probe_all_btn.setToolTip("Try all EN_SD_MUX and SD_MUX_SEL combinations at 1-bit 400 kHz.")
        self.sd_probe_all_btn.clicked.connect(lambda: self.send_control_command(b"SD PROBEALL\n", "probe SD mux combinations"))
        self.sd_probe_all_btn.setEnabled(False)
        mode_controls.addWidget(self.sd_probe_all_btn)

        self.sd_spi_probe_btn = QtWidgets.QPushButton("SPI Probe")
        self.sd_spi_probe_btn.setToolTip("Try the SD-card path in SPI mode using CLK/CMD/D0/D3.")
        self.sd_spi_probe_btn.clicked.connect(lambda: self.send_control_command(b"SD SPIPROBE\n", "probe SD card in SPI mode"))
        self.sd_spi_probe_btn.setEnabled(False)
        mode_controls.addWidget(self.sd_spi_probe_btn)

        self.sd_line_probe_btn = QtWidgets.QPushButton("Line Probe")
        self.sd_line_probe_btn.setToolTip("Read SD line idle levels through all mux states.")
        self.sd_line_probe_btn.clicked.connect(lambda: self.send_control_command(b"SD LINEPROBE\n", "probe SD line levels"))
        self.sd_line_probe_btn.setEnabled(False)
        mode_controls.addWidget(self.sd_line_probe_btn)

        self.sd_write_btn = QtWidgets.QPushButton("Write Test File")
        self.sd_write_btn.clicked.connect(lambda: self.send_control_command(b"SD WRITE_TEST\n", "write SD test file"))
        self.sd_write_btn.setEnabled(False)
        mode_controls.addWidget(self.sd_write_btn)

        self.sd_read_btn = QtWidgets.QPushButton("Read Test File")
        self.sd_read_btn.clicked.connect(lambda: self.send_control_command(b"SD READ_TEST\n", "read SD test file"))
        self.sd_read_btn.setEnabled(False)
        mode_controls.addWidget(self.sd_read_btn)

        self.sd_usb_btn = QtWidgets.QPushButton("Mux Idle")
        self.sd_usb_btn.setToolTip("Assert USB2641 reset and disable the SD mux.")
        self.sd_usb_btn.clicked.connect(lambda: self.send_control_command(b"SD IDLE\n", "idle SD mux"))
        self.sd_usb_btn.setEnabled(False)
        mode_controls.addWidget(self.sd_usb_btn)

        self.sd_hiz_btn = QtWidgets.QPushButton("SD Hi-Z")
        self.sd_hiz_btn.setToolTip("Put ESP32 SD pins in input/no-pull and disable the mux.")
        self.sd_hiz_btn.clicked.connect(lambda: self.send_control_command(b"SD HIZ\n", "set SD pins high-Z"))
        self.sd_hiz_btn.setEnabled(False)
        mode_controls.addWidget(self.sd_hiz_btn)
        mode_controls.addStretch(1)

        command_controls = QtWidgets.QHBoxLayout()
        control_layout.addLayout(command_controls)
        self.custom_command = QtWidgets.QLineEdit()
        self.custom_command.setPlaceholderText("Command, for example: SD READ_TEST")
        self.custom_command.returnPressed.connect(self.send_custom_control_command)
        command_controls.addWidget(self.custom_command, stretch=1)

        self.custom_send_btn = QtWidgets.QPushButton("Send")
        self.custom_send_btn.clicked.connect(self.send_custom_control_command)
        self.custom_send_btn.setEnabled(False)
        command_controls.addWidget(self.custom_send_btn)

        self.clear_control_log_btn = QtWidgets.QPushButton("Clear Output")
        self.clear_control_log_btn.clicked.connect(self.clear_control_log)
        command_controls.addWidget(self.clear_control_log_btn)

        self.control_log = QtWidgets.QPlainTextEdit()
        self.control_log.setReadOnly(True)
        self.control_log.setMaximumBlockCount(500)
        self.control_log.setPlaceholderText("Control and SD command log")
        control_layout.addWidget(self.control_log, stretch=1)

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
        parts = []
        for row in stats:
            parts.append(
                f"CH{int(row['channel'])}: "
                f"mean {row['mean_mv']:+.3f} mV, "
                f"rms {row['rms_noise_mv']:.3f} mV, "
                f"p2p {row['peak_to_peak_mv']:.3f} mV"
            )
        return " | ".join(parts)

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
        status_max = int(np.max(status)) if len(status) else 0
        fieldnames = [
            "timestamp",
            "port",
            "baud",
            "window_s",
            "sample_rate_hz",
            "frame_step",
            "missed_estimate",
            "status_max_hex",
            "channel",
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
                            "sample_rate_hz": sample_rate,
                            "frame_step": frame_step,
                            "missed_estimate": missed,
                            "status_max_hex": f"0x{status_max:02X}",
                            **row,
                        }
                    )
        except OSError as exc:
            self.info.setText(f"Stats save failed: {exc}")
            return

        self.info.setText(f"Saved channel stats to {path}.")

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

    def update_fft_xrange(self) -> None:
        if hasattr(self, "fft_plot"):
            self.fft_plot.setXRange(0, self.fft_max_box.value(), padding=0.0)

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
            accept_bad_checksum=self.accept_bad_checksum.isChecked(),
        )
        self.reader.start()
        self.connect_btn.setText("Disconnect")
        self.port_combo.setEnabled(False)
        self.refresh_ports_btn.setEnabled(False)
        self.baud_box.setEnabled(False)
        self.zero_btn.setEnabled(True)
        self.set_reset_btn.setEnabled(True)
        self.set_btn.setEnabled(True)
        self.reset_btn.setEnabled(True)
        self.enable_9v.setEnabled(True)
        self.enable_neg5v.setEnabled(True)
        self.dac_enable.setEnabled(True)
        self.save_stats_btn.setEnabled(True)
        self.dac_test_btn.setEnabled(True)
        self.control_mode_btn.setEnabled(True)
        self.adc_mode_btn.setEnabled(True)
        self.sd_test_btn.setEnabled(True)
        self.sd_probe_all_btn.setEnabled(True)
        self.sd_spi_probe_btn.setEnabled(True)
        self.sd_line_probe_btn.setEnabled(True)
        self.sd_write_btn.setEnabled(True)
        self.sd_read_btn.setEnabled(True)
        self.sd_usb_btn.setEnabled(True)
        self.sd_hiz_btn.setEnabled(True)
        self.custom_command.setEnabled(True)
        self.custom_send_btn.setEnabled(True)
        self.clear_control_log_btn.setEnabled(True)
        self.reader.write_command(b"9V 0\n")
        self.append_control_log(f"Connected to {port} at {baud}.")
        if self.reader.write_command(b"MODE ADC\n"):
            self.append_control_log("> MODE ADC")
            self.info.setText(f"Connected to {port} at {baud}. Requested ADC stream...")
        else:
            self.info.setText(f"Connected to {port} at {baud}. Waiting for frames...")
        if THERMAL_ISOLATION_UI:
            self.dac_enable.setEnabled(False)
            self.dac_test_btn.setEnabled(False)

    def disconnect(self) -> None:
        if self.reader:
            self.reader.stop()
            self.reader.join(timeout=1.0)
            self.reader = None
        self.connect_btn.setText("Connect")
        self.port_combo.setEnabled(True)
        self.refresh_ports_btn.setEnabled(True)
        self.baud_box.setEnabled(True)
        self.zero_btn.setEnabled(False)
        self.set_reset_btn.setEnabled(False)
        self.set_btn.setEnabled(False)
        self.reset_btn.setEnabled(False)
        self.enable_9v.setEnabled(False)
        self.enable_neg5v.setEnabled(False)
        self.dac_enable.setEnabled(False)
        self.save_stats_btn.setEnabled(False)
        self.dac_test_btn.setEnabled(False)
        self.control_mode_btn.setEnabled(False)
        self.adc_mode_btn.setEnabled(False)
        self.sd_test_btn.setEnabled(False)
        self.sd_probe_all_btn.setEnabled(False)
        self.sd_spi_probe_btn.setEnabled(False)
        self.sd_line_probe_btn.setEnabled(False)
        self.sd_write_btn.setEnabled(False)
        self.sd_read_btn.setEnabled(False)
        self.sd_usb_btn.setEnabled(False)
        self.sd_hiz_btn.setEnabled(False)
        self.custom_command.setEnabled(False)
        self.custom_send_btn.setEnabled(False)
        self.clear_control_log_btn.setEnabled(False)
        self.stats_label.setText("")

    def append_control_log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.control_log.appendPlainText(f"{stamp}  {text}")

    def clear_control_log(self) -> None:
        self.control_log.clear()

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
            self.clear_buffers()
            self.tabs.setCurrentWidget(self.scope_tab)

    def send_custom_control_command(self) -> None:
        text = self.custom_command.text().strip()
        if not text:
            return
        command = f"{text}\n".encode("ascii", errors="replace")
        if self.send_control_command(command, f"command {text!r}"):
            self.custom_command.clear()

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

    def update_dac_labels(self) -> None:
        for slider, label in zip(self.dac_sliders, self.dac_values):
            label.setText(str(slider.value()))

    def on_dac_slider_changed(self) -> None:
        self.update_dac_labels()
        if self.reader:
            self.dac_send_timer.start()

    def center_dac_sliders(self) -> None:
        for slider in self.dac_sliders:
            slider.setValue(DAC_CENTER_CODE)
        self.send_dac_command()

    def set_dac_sliders_silent(self, codes: list[int]) -> None:
        for slider, code in zip(self.dac_sliders, codes):
            slider.blockSignals(True)
            slider.setValue(code)
            slider.blockSignals(False)
        self.update_dac_labels()

    def write_dac_codes_direct(self, codes: list[int]) -> bool:
        if not self.reader:
            return False
        x, y, z = codes
        cmd = f"DAC {x} {y} {z}\n".encode("ascii")
        if not self.reader.write_command(cmd):
            return False
        self.last_dac_sent = list(codes)
        self.set_dac_sliders_silent(list(codes))
        return True

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

    def send_dac_command(self) -> None:
        self.dac_send_timer.stop()
        if not self.reader:
            self.info.setText("Connect before sending DAC command.")
            return
        target = [slider.value() for slider in self.dac_sliders]
        next_codes = []
        at_target = True
        for current, wanted in zip(self.last_dac_sent, target):
            delta = wanted - current
            if abs(delta) > DAC_MAX_STEP_LSB:
                at_target = False
                delta = DAC_MAX_STEP_LSB if delta > 0 else -DAC_MAX_STEP_LSB
            next_codes.append(current + delta)

        x, y, z = next_codes
        cmd = f"DAC {x} {y} {z}\n".encode("ascii")
        if self.reader.write_command(cmd):
            self.last_dac_sent = next_codes
            suffix = "" if at_target else " slewing"
            disabled = "" if self.dac_enable.isChecked() else " (stored while disabled)"
            self.info.setText(f"Sent DAC X={x} Y={y} Z={z}.{suffix}{disabled}")
            if not at_target:
                self.dac_send_timer.start()
        else:
            self.info.setText("DAC command failed: serial port is not ready.")

    def recent_channel_stats(self, seconds: float) -> list[dict[str, float]]:
        self.drain_queue()
        frames, host_times, channels, status = self.ordered_data()
        if len(host_times) < 2:
            return []
        cutoff = host_times[-1] - seconds
        mask = host_times >= cutoff
        if np.count_nonzero(mask) < 2:
            return []
        return self.channel_stats(channels[:, mask], list(range(CHANNEL_COUNT)))

    def start_dac_step_test(self) -> None:
        if not self.reader:
            self.info.setText("Connect before running DAC step test.")
            return
        if not self.dac_enable.isChecked():
            self.info.setText("Turn DAC On before running DAC step test.")
            return

        self.dac_send_timer.stop()
        self.dac_test_axis = self.dac_test_axis_box.currentIndex()
        axis_name = self.dac_test_axis_box.currentText()
        self.dac_test_results = []
        self.dac_test_step_idx = 0
        self.dac_test_btn.setEnabled(False)

        center = [DAC_CENTER_CODE, DAC_CENTER_CODE, DAC_CENTER_CODE]
        low = list(center)
        high = list(center)
        low[self.dac_test_axis] = DAC_MIN_CODE
        high[self.dac_test_axis] = DAC_MAX_CODE
        self.dac_test_steps = [
            ("center_a", center),
            ("low", low),
            ("center_b", center),
            ("high", high),
            ("center_c", center),
        ]

        self.clear_buffers()
        self.info.setText(f"Starting DAC {axis_name} step test...")
        self.run_next_dac_test_step()

    def run_next_dac_test_step(self) -> None:
        if self.dac_test_step_idx >= len(self.dac_test_steps):
            self.finish_dac_step_test()
            return

        step_name, codes = self.dac_test_steps[self.dac_test_step_idx]
        if not self.write_dac_codes_direct(codes):
            self.info.setText("DAC step test failed: serial port is not ready.")
            self.dac_test_btn.setEnabled(True)
            return

        axis_name = self.dac_test_axis_box.currentText()
        self.info.setText(
            f"DAC {axis_name} test step {self.dac_test_step_idx + 1}/"
            f"{len(self.dac_test_steps)}: {step_name} X={codes[0]} Y={codes[1]} Z={codes[2]}"
        )
        QtCore.QTimer.singleShot(DAC_TEST_SETTLE_MS, self.capture_dac_test_step)

    def capture_dac_test_step(self) -> None:
        stats = self.recent_channel_stats(DAC_TEST_CAPTURE_MS / 1000.0)
        if not stats:
            self.info.setText("DAC step test waiting for data...")
            QtCore.QTimer.singleShot(DAC_TEST_SETTLE_MS, self.capture_dac_test_step)
            return

        step_name, codes = self.dac_test_steps[self.dac_test_step_idx]
        for row in stats:
            self.dac_test_results.append(
                {
                    "axis": self.dac_test_axis_box.currentText(),
                    "step": step_name,
                    "step_index": self.dac_test_step_idx,
                    "dac_x": codes[0],
                    "dac_y": codes[1],
                    "dac_z": codes[2],
                    **row,
                }
            )
        self.dac_test_step_idx += 1
        QtCore.QTimer.singleShot(250, self.run_next_dac_test_step)

    def finish_dac_step_test(self) -> None:
        self.write_dac_codes_direct([DAC_CENTER_CODE, DAC_CENTER_CODE, DAC_CENTER_CODE])
        self.dac_test_btn.setEnabled(True)

        if not self.dac_test_results:
            self.info.setText("DAC step test finished without valid samples.")
            return

        out_dir = Path(__file__).resolve().parent / "save stats"
        out_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        axis_name = self.dac_test_axis_box.currentText()
        out_path = out_dir / f"dac_{axis_name}_step_test_{timestamp}.csv"
        fieldnames = [
            "timestamp",
            "axis",
            "step",
            "step_index",
            "dac_x",
            "dac_y",
            "dac_z",
            "channel",
            "samples",
            "mean_mv",
            "rms_noise_mv",
            "peak_to_peak_mv",
            "min_mv",
            "max_mv",
        ]
        try:
            with out_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for row in self.dac_test_results:
                    writer.writerow({"timestamp": timestamp, **row})
        except OSError as exc:
            self.info.setText(f"DAC step test save failed: {exc}")
            return

        by_step_ch: dict[tuple[str, int], float] = {}
        for row in self.dac_test_results:
            by_step_ch[(str(row["step"]), int(row["channel"]))] = float(row["mean_mv"])
        responses = []
        for ch in range(CHANNEL_COUNT):
            low = by_step_ch.get(("low", ch))
            high = by_step_ch.get(("high", ch))
            if low is None or high is None:
                continue
            delta = high - low
            slope = delta / float(DAC_MAX_CODE - DAC_MIN_CODE)
            responses.append((abs(delta), delta, slope, ch))
        responses.sort(reverse=True)
        if responses:
            _, delta, slope, ch = responses[0]
            self.info.setText(
                f"DAC {axis_name} test saved to {out_path}. "
                f"Strongest response: CH{ch} high-low={delta:+.3f} mV "
                f"({slope:+.6f} mV/code)."
            )
        else:
            self.info.setText(f"DAC {axis_name} test saved to {out_path}. No channel response computed.")

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
            i = self.write_idx
            self.frames[i] = s.frame
            self.host_times[i] = s.t_host
            self.channels[:, i] = s.channels_mv
            self.status[i] = s.status
            self.write_idx = (self.write_idx + 1) % self.args.buffer
            self.count = min(self.count + 1, self.args.buffer)
            drained += 1
        self.last_queue_drained = drained
        return drained

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

        window_s = self.window_box.value()
        max_samples = max(2, int(window_s * self.nominal_stream_rate()))
        if len(frames) > max_samples:
            frames = frames[-max_samples:]
            host_times = host_times[-max_samples:]
            channels = channels[:, -max_samples:]
            status = status[-max_samples:]
        return frames.copy(), host_times.copy(), channels.copy(), status.copy()

    def nominal_stream_rate(self) -> float:
        return self.args.sample_rate

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
        self.drain_queue()

        if not self.reader:
            self.info.setText("Disconnected. Select a port and click Connect.")
            return

        if self.reader.error:
            self.info.setText(f"Serial error: {self.reader.error}")
            self.disconnect()
            return

        for line in self.reader.drain_text_lines():
            self.append_control_log(line)

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
        fft_rate = avg_rx_rate if math.isfinite(avg_rx_rate) else sample_rate
        t = (frames - frames[-1]) / fft_rate
        for ch in range(CHANNEL_COUNT):
            if self.channel_checks[ch].isChecked():
                self.time_curves[ch].setData(t, channels[ch])
        self.time_plot.setXRange(-self.window_box.value(), 0.0, padding=0.0)

        active = self.active_channels()
        if self.autorange.isChecked():
            visible = channels[active].reshape(-1) if active else np.empty(0)
            visible = visible[np.isfinite(visible)]
            if visible.size:
                y_min = float(np.min(visible))
                y_max = float(np.max(visible))
                span = max(y_max - y_min, 0.1)
                margin = max(span * 0.08, 0.05)
                self.time_plot.setYRange(y_min - margin, y_max + margin, padding=0.0)
        elif active:
            span = self.span.value()
            visible = channels[active].reshape(-1)
            visible = visible[np.isfinite(visible)]
            center = float(np.median(visible)) if visible.size else 0.0
            self.time_plot.setYRange(center - span, center + span, padding=0.0)

        self.update_fft(channels, active, fft_rate)
        self.stats_label.setText(self.format_stats(self.channel_stats(channels, active)))

        status_max = int(np.max(status)) if len(status) else 0
        self.info.setText(
            f"{self.reader.port} @ {self.reader.baud} | "
            f"rx {self.reader.samples_seen} samples | "
            f"bin {self.reader.binary_packets} text {self.reader.text_packets} "
            f"sync {self.reader.sync_hits} badlen {self.reader.bad_lengths} "
            f"badcrc {self.reader.bad_checksums} | "
            f"frame fs {sample_rate:.2f} Hz | "
            f"avg rx fs {avg_rx_rate:.2f} Hz | "
            f"window host fs {window_host_rate:.2f} Hz | "
            f"fft fs {fft_rate:.2f} Hz | "
            f"frame step {frame_step:.1f} | "
            f"missed~{missed} | "
            f"buffer {len(frames)}/{self.args.buffer} | "
            f"last drain {self.last_queue_drained} | "
            f"status max 0x{status_max:02X}"
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
        for ch in range(CHANNEL_COUNT):
            if ch not in active:
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
    p.add_argument("--buffer", type=int, default=20000, help="Samples kept in memory")
    p.add_argument("--window-s", type=float, default=60.0, help="Visible rolling time window")
    p.add_argument("--queue", type=int, default=65536, help="Reader queue capacity")
    p.add_argument("--refresh-hz", type=float, default=30.0, help="UI redraw rate")
    p.add_argument("--fft-size", type=int, default=0, help="FFT samples; 0 uses the visible time window")
    p.add_argument("--fft-max-hz", type=float, default=128.0)
    p.add_argument("--span", type=float, default=5.0, help="Manual half-span in mV")
    args = p.parse_args()
    min_buffer = max(2, int(math.ceil(args.window_s * args.sample_rate)))
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
