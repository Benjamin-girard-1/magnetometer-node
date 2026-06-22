#!/usr/bin/env python3
"""Run and check the INA851 functional-model validation netlists."""

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


dc = run("ina851_functional_dc.cir")
close(value(dc, "v(outp,outn)"), 2.02e-3 * 10_000 / 10_001.8, 0.002, "DC gain G=2")

ac = run("ina851_functional_ac.cir")
close(value(ac, "gain_dc"), 20 * math.log10(2), 0.003, "AC gain G=2")
close(value(ac, "f3db"), 15e6, 0.02, "Bandwidth")

tran = run("ina851_functional_tran.cir")
close(value(tran, "slew_16v"), 37e6, 0.01, "Slew rate")

matrix = run("ina851_functional_gain_matrix.cir")
for name, gain in (
    ("v(o1p,o1n)", 1),
    ("v(o2p,o2n)", 2),
    ("v(o10p,o10n)", 10),
    ("v(o100p,o100n)", 100),
):
    expected = 1.01e-3 * gain * 10_000 / 10_001.8
    close(value(matrix, name), expected, 0.003, f"Gain {gain}")

limits = run("ina851_functional_limits.cir")
close(value(limits, "v(freep)"), 7.6, 0.001, "Positive output swing")
close(value(limits, "v(freen)"), 1.4, 0.001, "Negative output swing")
close(value(limits, "v(clampp)"), 6.6, 0.001, "Positive clamp")
close(value(limits, "v(clampn)"), 2.4, 0.001, "Negative clamp")

bridge = run("ina851_functional_bridge.cir")
close(value(bridge, "out_pp"), 0.10588, 0.01, "Wheatstone bridge output")

print("INA851 functional model validation: PASS")
