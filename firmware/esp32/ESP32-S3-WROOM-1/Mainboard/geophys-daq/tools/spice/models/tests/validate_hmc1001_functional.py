#!/usr/bin/env python3
"""Run and check the HMC1001 functional-model validation netlists."""

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


dc = run("hmc1001_dc.cir")
close(value(dc, "v(outp,outn)"), 17e-3, 0.001, "Sensitivity and offset")

offset = run("hmc1001_offset_physical.cir")
close(value(offset, "v(outp,outn)"), 11.25e-3, 0.001, "Physical bridge offset")

tran = run("hmc1001_tran.cir")
close(value(tran, "vpp"), 320e-6, 0.001, "Time-varying field")

noise = run("hmc1001_noise.cir")
expected_noise = {
    "n_01hz": math.sqrt((3.75362e-9) ** 2 + (30e-9) ** 2 / 0.1),
    "n_1hz": math.sqrt((3.75362e-9) ** 2 + (30e-9) ** 2),
    "n_10hz": math.sqrt((3.75362e-9) ** 2 + (30e-9) ** 2 / 10),
    "n_100hz": math.sqrt((3.75362e-9) ** 2 + (30e-9) ** 2 / 100),
    "n_1khz": math.sqrt((3.75362e-9) ** 2 + (30e-9) ** 2 / 1000),
}
for name, expected in expected_noise.items():
    close(value(noise, name), expected, 0.01, f"Noise {name}")

print("HMC1001 functional model validation: PASS")
