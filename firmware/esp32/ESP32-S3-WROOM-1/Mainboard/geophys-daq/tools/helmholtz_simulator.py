#!/usr/bin/env python3
"""
Interactive Helmholtz-pair magnetic field simulator.

Install:
    python3 -m pip install numpy matplotlib

Run:
    python3 tools/helmholtz_simulator.py

The model treats each circular coil as many straight current elements and
evaluates the Biot-Savart law on three central slices: XY, YZ, and XZ.
"""

from __future__ import annotations

import math
import tkinter as tk
from dataclasses import dataclass
from tkinter import ttk

import matplotlib
import numpy as np

matplotlib.use("TkAgg")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.colors import Normalize
from matplotlib.patches import Circle


MU0 = 4.0 * math.pi * 1e-7
EARTH_FIELD_UT = 50.0
ERROR_LEVELS_PCT = (0.01, 0.1, 1.0, 5.0)


@dataclass(frozen=True)
class CoilParams:
    diameter_mm: float = 250.0
    spacing_mm: float = 125.0
    turns: float = 15.0
    resistance_ohm: float = 1.0
    voltage_v: float = 0.46
    grid_points: int = 31
    segments: int = 96

    @property
    def radius_m(self) -> float:
        return self.diameter_mm / 2000.0

    @property
    def spacing_m(self) -> float:
        return self.spacing_mm / 1000.0

    @property
    def current_a(self) -> float:
        if self.resistance_ohm <= 0:
            return 0.0
        return self.voltage_v / self.resistance_ohm


def helmholtz_center_field_t(params: CoilParams, current_a: float | None = None) -> float:
    """Analytic center field for a Helmholtz pair."""
    radius = params.radius_m
    if radius <= 0:
        return 0.0
    current = params.current_a if current_a is None else current_a
    return (4.0 / 5.0) ** 1.5 * MU0 * params.turns * current / radius


def required_voltage_for_center_field(params: CoilParams, target_ut: float) -> float:
    radius = params.radius_m
    if params.turns <= 0 or radius <= 0:
        return 0.0
    target_t = target_ut * 1e-6
    current = target_t * radius / (((4.0 / 5.0) ** 1.5) * MU0 * params.turns)
    return current * max(params.resistance_ohm, 0.0)


def make_loop_segments(radius_m: float, z_m: float, count: int) -> tuple[np.ndarray, np.ndarray]:
    theta = np.linspace(0.0, 2.0 * math.pi, count + 1)
    points = np.column_stack(
        (
            radius_m * np.cos(theta),
            radius_m * np.sin(theta),
            np.full(theta.shape, z_m),
        )
    )
    starts = points[:-1]
    ends = points[1:]
    mids = 0.5 * (starts + ends)
    dl = ends - starts
    return mids, dl


def coil_pair_segments(params: CoilParams) -> tuple[np.ndarray, np.ndarray]:
    half_spacing = 0.5 * params.spacing_m
    top_mid, top_dl = make_loop_segments(params.radius_m, half_spacing, params.segments)
    bottom_mid, bottom_dl = make_loop_segments(params.radius_m, -half_spacing, params.segments)
    return np.vstack((top_mid, bottom_mid)), np.vstack((top_dl, bottom_dl))


def biot_savart_field(points: np.ndarray, params: CoilParams) -> np.ndarray:
    """Return B vectors in tesla for shape (N, 3) points."""
    mids, dl = coil_pair_segments(params)
    field = np.zeros_like(points, dtype=float)
    coeff = MU0 * params.current_a * params.turns / (4.0 * math.pi)

    for segment_mid, segment_dl in zip(mids, dl):
        r = points - segment_mid
        r_norm = np.linalg.norm(r, axis=1)
        mask = r_norm > 1e-9
        if not np.any(mask):
            continue
        cross = np.cross(segment_dl, r[mask])
        field[mask] += coeff * cross / (r_norm[mask] ** 3)[:, None]

    return field


def safe_circle_labels(contour_set: object) -> tuple[dict[float, str], dict[float, float]]:
    """Return largest centered circle diameter inside each error contour."""
    labels = {}
    radii_by_level = {}
    for level, segments in zip(contour_set.levels, contour_set.allsegs):
        radii = []
        for segment in segments:
            if len(segment) == 0:
                continue
            radii.extend(np.linalg.norm(segment, axis=1))
        if radii:
            radius_mm = float(np.min(radii))
            radii_by_level[level] = radius_mm
            labels[level] = f"{level:g}% safe circle: {2.0 * radius_mm:.1f} mm dia"
        else:
            labels[level] = f"{level:g}%"
    return labels, radii_by_level


def make_plane_points(
    plane: str,
    params: CoilParams,
    grid_points: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, str, str]:
    radius_mm = max(params.diameter_mm * 0.5, 1.0)
    half_xy_mm = radius_mm * 0.85
    half_z_mm = max(params.spacing_mm * 0.5 * 0.95, 1.0)
    half_yz_xz_mm = max(half_xy_mm, half_z_mm)

    if plane == "xy":
        a = np.linspace(-half_xy_mm, half_xy_mm, grid_points)
        b = np.linspace(-half_xy_mm, half_xy_mm, grid_points)
        aa, bb = np.meshgrid(a, b)
        points = np.column_stack((aa.ravel(), bb.ravel(), np.zeros(aa.size))) / 1000.0
        return aa, bb, points, "x (mm)", "y (mm)"
    if plane == "yz":
        a = np.linspace(-half_yz_xz_mm, half_yz_xz_mm, grid_points)
        b = np.linspace(-half_yz_xz_mm, half_yz_xz_mm, grid_points)
        aa, bb = np.meshgrid(a, b)
        points = np.column_stack((np.zeros(aa.size), aa.ravel(), bb.ravel())) / 1000.0
        return aa, bb, points, "y (mm)", "z (mm)"
    if plane == "xz":
        a = np.linspace(-half_yz_xz_mm, half_yz_xz_mm, grid_points)
        b = np.linspace(-half_yz_xz_mm, half_yz_xz_mm, grid_points)
        aa, bb = np.meshgrid(a, b)
        points = np.column_stack((aa.ravel(), np.zeros(aa.size), bb.ravel())) / 1000.0
        return aa, bb, points, "x (mm)", "z (mm)"
    raise ValueError(f"Unknown plane: {plane}")


class HelmholtzApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Helmholtz Pair Field Simulator")
        self.pending_update: str | None = None

        self.vars = {
            "diameter_mm": tk.DoubleVar(value=250.0),
            "spacing_mm": tk.DoubleVar(value=125.0),
            "turns": tk.DoubleVar(value=15.0),
            "resistance_ohm": tk.DoubleVar(value=1.0),
            "voltage_v": tk.DoubleVar(value=0.46),
            "grid_points": tk.IntVar(value=31),
            "segments": tk.IntVar(value=96),
        }
        self.component = tk.StringVar(value="magnitude")
        self.view_mode = tk.StringVar(value="all")
        self.show_isolines = tk.BooleanVar(value=True)
        self.status = tk.StringVar(value="")
        self.isoline_summary = tk.StringVar(value="")

        self._build_ui()
        self.schedule_update()

    def _build_ui(self) -> None:
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        controls = ttk.Frame(self.root, padding=12)
        controls.grid(row=0, column=0, sticky="ns")

        ttk.Label(controls, text="Coil Parameters", font=("", 14, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 8)
        )

        rows = [
            ("Diameter (mm)", "diameter_mm", 1.0, 1000.0, 1.0),
            ("Spacing (mm)", "spacing_mm", 1.0, 1000.0, 1.0),
            ("Turns", "turns", 1.0, 500.0, 1.0),
            ("Resistance (ohm)", "resistance_ohm", 0.001, 1000.0, 0.1),
            ("Voltage (V)", "voltage_v", -100.0, 100.0, 0.01),
            ("Grid points", "grid_points", 11, 81, 2),
            ("Loop segments", "segments", 24, 360, 12),
        ]
        for row, (label, key, low, high, step) in enumerate(rows, start=1):
            ttk.Label(controls, text=label).grid(row=row, column=0, sticky="w", pady=3)
            spin = ttk.Spinbox(
                controls,
                from_=low,
                to=high,
                increment=step,
                textvariable=self.vars[key],
                width=10,
                command=self.schedule_update,
            )
            spin.grid(row=row, column=1, sticky="ew", pady=3)
            spin.bind("<KeyRelease>", self.schedule_update)
            spin.bind("<Return>", self.schedule_update)

        ttk.Separator(controls).grid(row=8, column=0, columnspan=2, sticky="ew", pady=12)
        ttk.Label(controls, text="Color shows").grid(row=9, column=0, columnspan=2, sticky="w")
        component_rows = [
            ("|B| magnitude", "magnitude"),
            ("Bx", "bx"),
            ("By", "by"),
            ("Bz", "bz"),
        ]
        for offset, (label, value) in enumerate(component_rows, start=10):
            ttk.Radiobutton(
                controls,
                text=label,
                value=value,
                variable=self.component,
                command=self.schedule_update,
            ).grid(row=offset, column=0, columnspan=2, sticky="w")

        ttk.Separator(controls).grid(row=14, column=0, columnspan=2, sticky="ew", pady=12)
        ttk.Label(controls, text="View").grid(row=15, column=0, columnspan=2, sticky="w")
        view_rows = [
            ("All slices", "all"),
            ("XY large", "xy"),
            ("XZ large", "xz"),
        ]
        for offset, (label, value) in enumerate(view_rows, start=16):
            ttk.Radiobutton(
                controls,
                text=label,
                value=value,
                variable=self.view_mode,
                command=self.schedule_update,
            ).grid(row=offset, column=0, columnspan=2, sticky="w")

        ttk.Checkbutton(
            controls,
            text="Show error isolines",
            variable=self.show_isolines,
            command=self.schedule_update,
        ).grid(row=20, column=0, columnspan=2, sticky="w", pady=(10, 0))

        ttk.Button(
            controls,
            text=f"Set V for {EARTH_FIELD_UT:.0f} uT center",
            command=self.set_earth_voltage,
        ).grid(row=21, column=0, columnspan=2, sticky="ew", pady=(12, 4))
        ttk.Button(controls, text="Update", command=self.update_plots).grid(
            row=22, column=0, columnspan=2, sticky="ew"
        )

        ttk.Label(
            controls,
            textvariable=self.status,
            justify="left",
            wraplength=220,
        ).grid(row=23, column=0, columnspan=2, sticky="ew", pady=(12, 0))

        ttk.Label(
            controls,
            textvariable=self.isoline_summary,
            justify="left",
            wraplength=220,
        ).grid(row=24, column=0, columnspan=2, sticky="ew", pady=(12, 0))

        self.figure = Figure(figsize=(12, 4.4), dpi=100, constrained_layout=True)
        self.axes = []
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        self.canvas.get_tk_widget().grid(row=0, column=1, sticky="nsew")
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.root, pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.grid(row=1, column=1, sticky="ew")

    def read_params(self) -> CoilParams:
        grid_points = int(self.vars["grid_points"].get())
        if grid_points % 2 == 0:
            grid_points += 1
            self.vars["grid_points"].set(grid_points)

        return CoilParams(
            diameter_mm=float(self.vars["diameter_mm"].get()),
            spacing_mm=float(self.vars["spacing_mm"].get()),
            turns=float(self.vars["turns"].get()),
            resistance_ohm=float(self.vars["resistance_ohm"].get()),
            voltage_v=float(self.vars["voltage_v"].get()),
            grid_points=max(5, grid_points),
            segments=max(12, int(self.vars["segments"].get())),
        )

    def schedule_update(self, _event: object | None = None) -> None:
        if self.pending_update is not None:
            self.root.after_cancel(self.pending_update)
        self.pending_update = self.root.after(250, self.update_plots)

    def set_earth_voltage(self) -> None:
        params = self.read_params()
        self.vars["voltage_v"].set(round(required_voltage_for_center_field(params, EARTH_FIELD_UT), 4))
        self.schedule_update()

    def update_plots(self) -> None:
        self.pending_update = None
        try:
            params = self.read_params()
        except tk.TclError:
            return

        if params.diameter_mm <= 0 or params.spacing_mm <= 0 or params.turns <= 0:
            self.status.set("Diameter, spacing, and turns must be positive.")
            return

        all_planes = [("xy", "XY slice, z = 0"), ("xz", "XZ slice, y = 0")]
        view_mode = self.view_mode.get()
        planes = all_planes if view_mode == "all" else [item for item in all_planes if item[0] == view_mode]
        component_index = {"bx": 0, "by": 1, "bz": 2}
        selected = self.component.get()
        self.figure.clear()
        self.axes = [
            self.figure.add_subplot(1, len(planes), index)
            for index in range(1, len(planes) + 1)
        ]
        center = biot_savart_field(np.array([[0.0, 0.0, 0.0]]), params)[0] * 1e6
        center_bz = center[2]
        isoline_summaries = []
        plane_data = []
        color_min = math.inf
        color_max = -math.inf

        for plane, title in planes:
            aa, bb, points, xlabel, ylabel = make_plane_points(plane, params, params.grid_points)
            field_ut = biot_savart_field(points, params).reshape(aa.shape + (3,)) * 1e6

            if selected == "magnitude":
                color_data = np.linalg.norm(field_ut, axis=2)
                label = "|B| (uT)"
                cmap = "viridis"
            else:
                color_data = field_ut[:, :, component_index[selected]]
                label = selected.upper() + " (uT)"
                cmap = "coolwarm"

            color_min = min(color_min, float(np.nanmin(color_data)))
            color_max = max(color_max, float(np.nanmax(color_data)))
            plane_data.append((plane, title, aa, bb, field_ut, color_data, xlabel, ylabel, label, cmap))

        if math.isclose(color_min, color_max):
            color_min -= 1.0
            color_max += 1.0
        color_norm = Normalize(vmin=color_min, vmax=color_max)
        image = None

        for ax, (plane, title, aa, bb, field_ut, color_data, xlabel, ylabel, label, cmap) in zip(
            self.axes, plane_data
        ):
            image = ax.imshow(
                color_data,
                origin="lower",
                extent=[aa.min(), aa.max(), bb.min(), bb.max()],
                aspect="equal",
                cmap=cmap,
                norm=color_norm,
            )

            step = max(1, params.grid_points // 15)
            if plane == "xy":
                u, v = field_ut[:, :, 0], field_ut[:, :, 1]
            elif plane == "yz":
                u, v = field_ut[:, :, 1], field_ut[:, :, 2]
            else:
                u, v = field_ut[:, :, 0], field_ut[:, :, 2]
            ax.quiver(
                aa[::step, ::step],
                bb[::step, ::step],
                u[::step, ::step],
                v[::step, ::step],
                color="white",
                alpha=0.75,
                pivot="middle",
                scale_units="xy",
                scale=None,
                width=0.004,
            )

            ax.set_title(title)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.set_aspect("equal", adjustable="box")

            if self.show_isolines.get() and abs(center_bz) > 1e-12:
                error_pct = np.abs((field_ut[:, :, 2] - center_bz) / center_bz) * 100.0
                visible_levels = [
                    level
                    for level in ERROR_LEVELS_PCT
                    if np.nanmin(error_pct) <= level <= np.nanmax(error_pct)
                ]
                if visible_levels:
                    contours = ax.contour(
                        aa,
                        bb,
                        error_pct,
                        levels=visible_levels,
                        colors="white",
                        linewidths=1.1,
                        alpha=0.95,
                    )
                    label_map, safe_radii = safe_circle_labels(contours)
                    for level, radius_mm in safe_radii.items():
                        ax.add_patch(
                            Circle(
                                (0.0, 0.0),
                                radius_mm,
                                fill=False,
                                edgecolor="#ffd84d",
                                linewidth=1.4,
                                linestyle="--",
                                alpha=0.95,
                            )
                        )
                    proxy_lines = [
                        ax.plot([], [], color="#ffd84d", linewidth=1.4, linestyle="--", label=label_map[level])[0]
                        for level in visible_levels
                        if level in safe_radii
                    ]
                    proxy_lines.insert(
                        0,
                        ax.plot([], [], color="white", linewidth=1.1, label="white: error isolines")[0],
                    )
                    ax.legend(
                        handles=proxy_lines,
                        loc="upper right",
                        fontsize=8,
                        framealpha=0.75,
                        facecolor="black",
                        labelcolor="white",
                    )
                    isoline_summaries.append(
                        title.split()[0] + ":\n" + "\n".join(label_map[level] for level in visible_levels)
                    )
                else:
                    isoline_summaries.append(f"{title.split()[0]}:\nNo selected isolines in view")

        if image is not None:
            self.figure.colorbar(image, ax=self.axes, label=plane_data[0][8], shrink=0.82)

        center_formula = helmholtz_center_field_t(params) * 1e6
        self.status.set(
            f"Current: {params.current_a:.4g} A\n"
            f"Center B: Bx={center[0]:.3g} uT, By={center[1]:.3g} uT, Bz={center[2]:.3g} uT\n"
            f"Analytic center Bz: {center_formula:.3g} uT\n"
            f"White: axial Bz error isolines\n"
            f"Yellow: safe centered circles\n"
            f"Earth reference: about 25-65 uT"
        )
        if self.show_isolines.get():
            self.isoline_summary.set("Safe circle diameters\n" + "\n\n".join(isoline_summaries))
        else:
            self.isoline_summary.set("")
        self.canvas.draw_idle()


def main() -> None:
    root = tk.Tk()
    HelmholtzApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
