#!/usr/bin/env python3
"""Validate the portable OPA1197_FUNC ngspice model."""

from __future__ import annotations

import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TEST_DIR = ROOT / "spice" / "tests"


def run(circuit: str) -> str:
    path = TEST_DIR / circuit
    result = subprocess.run(
        ["ngspice", "-b", str(path)],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    return result.stdout


def value(text: str, name: str) -> float:
    pattern = rf"(?m)^\s*{re.escape(name)}\s*=\s*([-+0-9.eE]+)"
    match = re.search(pattern, text)
    if not match:
        raise AssertionError(f"Could not find {name!r} in ngspice output")
    return float(match.group(1))


def between(name: str, got: float, lo: float, hi: float) -> None:
    if not (lo <= got <= hi):
        raise AssertionError(f"{name}: got {got:.6g}, expected {lo:.6g}..{hi:.6g}")


def main() -> None:
    dc = run("opa1197_dc.cir")
    out = value(dc, "v(out)")
    err = value(dc, "v(out)-v(inp)")
    between("unity follower output", out, 0.999, 1.001)
    between("unity follower error", abs(err), 0, 20e-6)

    ac = run("opa1197_gain_ac.cir")
    gain_10hz = value(ac, "gain_10hz")
    gain_1mhz = value(ac, "gain_1mhz")
    gain_10mhz = value(ac, "gain_10mhz")
    between("closed-loop gain at 10 Hz, dB", gain_10hz, 19.9, 20.1)
    between("closed-loop gain at 1 MHz, dB", gain_1mhz, 16.5, 18.2)
    between("closed-loop gain at 10 MHz, dB", gain_10mhz, -1.0, 1.0)

    slew = run("opa1197_slew.cir")
    slew_rate = value(slew, "slew")
    between("slew rate, V/s", slew_rate, 18e6, 22e6)

    limits = run("opa1197_output_limits.cir")
    headroom = value(limits, "v(vcc)-v(out)")
    between("10k loaded positive headroom", headroom, 90e-3, 160e-3)

    noise = run("opa1197_noise.cir")
    n_100hz = value(noise, "n_100hz")
    n_1khz = value(noise, "n_1khz")
    n_10khz = value(noise, "n_10khz")
    between("noise at 100 Hz", n_100hz, 9e-9, 12e-9)
    between("noise at 1 kHz", n_1khz, 5e-9, 7e-9)
    between("noise at 10 kHz", n_10khz, 5e-9, 6.5e-9)

    print("OPA1197 functional model validation: PASS")
    print(f"  follower output: {out:.9g} V, error {err:.3g} V")
    print(f"  gain: {gain_10hz:.3f} dB @10 Hz, {gain_1mhz:.3f} dB @1 MHz, {gain_10mhz:.3f} dB @10 MHz")
    print(f"  slew: {slew_rate/1e6:.3f} V/us")
    print(f"  +rail headroom, 10k load: {headroom*1e3:.2f} mV")
    print(f"  noise: {n_100hz*1e9:.2f}, {n_1khz*1e9:.2f}, {n_10khz*1e9:.2f} nV/sqrt(Hz)")


if __name__ == "__main__":
    main()
