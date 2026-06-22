#!/usr/bin/env python3
"""Run and check the PGA855 functional-model validation netlists."""

from __future__ import annotations

import math
import re
import subprocess
from pathlib import Path


HERE = Path(__file__).resolve().parent


def run(netlist: str) -> str:
    result = subprocess.run(
        ["ngspice", "-b", netlist],
        cwd=HERE,
        check=True,
        text=True,
        capture_output=True,
    )
    return result.stdout + result.stderr


def value(text: str, name: str) -> float:
    match = re.search(rf"(?m)^{re.escape(name)}\s*=\s*([-+0-9.eE]+)", text)
    if not match:
        raise AssertionError(f"Missing result {name!r}\n{text}")
    return float(match.group(1))


def close(actual: float, expected: float, rel: float, label: str) -> None:
    error = abs(actual - expected) / max(abs(expected), 1e-30)
    if error > rel:
        raise AssertionError(
            f"{label}: got {actual:g}, expected {expected:g}, "
            f"relative error {error:.3%} > {rel:.3%}"
        )


matrix = run("pga855_gain_matrix.cir")
load_factor = 10_000 / (10_000 + 1.8)
for index, gain in enumerate((0.125, 0.25, 0.5, 1, 2, 4, 8, 16)):
    output_offset = 70e-6 if gain < 1 else gain * 70e-6
    expected = (gain * 1e-3 + output_offset) * load_factor
    close(
        value(matrix, f"v(o{index}p,o{index}n)"),
        expected,
        0.001,
        f"Gain {gain:g}",
    )

ac = run("pga855_ac.cir")
close(value(ac, "gain_dc"), 20 * math.log10(16), 0.001, "AC gain G=16")
close(value(ac, "f3db"), 10e6, 0.01, "Bandwidth")

tran = run("pga855_tran.cir")
close(value(tran, "slew_8v"), 35e6, 0.01, "Slew rate")

limits = run("pga855_output_limits.cir")
close(value(limits, "v(outp)"), 4.7964, 0.002, "Positive output limit")
close(value(limits, "v(outn)"), 0.2036, 0.002, "Negative output limit")

noise = run("pga855_noise.cir")
close(value(noise, "enoise_rti"), 7.8e-9, 0.01, "G=16 voltage noise")

print("PGA855 functional model validation: PASS")

