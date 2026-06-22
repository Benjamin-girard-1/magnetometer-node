# OPA1197 functional ngspice model

This is a portable KiCad/ngspice functional model for one OPAx197 amplifier core,
intended for the single OPA1197-style device.

It is not the official TI PSpice macro-model.  It is a simplified model tuned to
the main datasheet values so it behaves well in KiCad.

## File

```text
tools/spice/OPA1197_functional.lib
```

## Subcircuit

```spice
.subckt OPA1197_FUNC IN+ IN- VCC VEE OUT
```

KiCad pin mapping:

| Symbol pin | Model pin |
|---|---|
| IN+ | IN+ |
| IN- | IN- |
| V+ / VCC | VCC |
| V- / VEE | VEE |
| OUT | OUT |

Example instance:

```spice
XU1 noninv inv vplus vminus out OPA1197_FUNC
```

## Main modeled specs

- Unity-gain bandwidth: about 10 MHz
- Slew rate: about 20 V/µs
- Quiescent current: 1 mA typical
- Input offset voltage: parameter `VOS=5u` by default
- Input offset drift: parameter `VOS_DRIFT=0.5u`, with `TEMP_C=27` by default
- Input bias current: `IBIAS=5p`
- Differential input impedance: `100Meg || 1.6p`
- Common-mode input impedance: `1e13 || 6.4p`
- Output swing:
  - about 25 mV from rail with no load
  - about 125 mV from rail with 10 kΩ load
  - about 500 mV from rail with 2 kΩ load
- Output current limit: about ±65 mA
- Input-referred voltage noise in `.noise`:
  - about 10.3 nV/√Hz at 100 Hz
  - about 6.1 nV/√Hz at 1 kHz
  - about 5.5 nV/√Hz above ~10 kHz

## Useful parameters

You can override parameters in the KiCad simulation model field or in a direct
SPICE instance:

```spice
XU1 inp inn vcc vee out OPA1197_FUNC PARAMS: VOS=20u IBIAS=5p
```

Most useful:

```text
VOS          input offset voltage
VOS_DRIFT    input offset drift in V/°C
TEMP_C       model temperature used for offset drift
IBIAS        input bias current
GBW          unity-gain bandwidth
SR           slew rate in V/s
IQ           quiescent current
ROUT         output resistance used for load droop
ISC          short-circuit current limit
```

## Validation

Run:

```sh
python3 tools/spice/tests/validate_opa1197_functional.py
```

Expected result:

```text
OPA1197 functional model validation: PASS
```

Current validation summary:

```text
follower output: 0.9999977 V
gain: 20.000 dB @10 Hz, 17.169 dB @1 MHz, -0.025 dB @10 MHz
slew: 19.996 V/us
+rail headroom, 10k load: 123.34 mV
noise: 10.34, 6.12, 5.52 nV/sqrt(Hz)
```

ngspice may print `gmin stepping failed` before recovering with source stepping
or transient operating point.  In these tests that warning is non-fatal; the
reported operating point and analyses are valid.

## Limitations

This model is meant for system-level simulation, not silicon-accurate analysis.

Not modeled in detail:

- package-to-package differences between OPA197, OPA2197, and OPA4197
- thermal shutdown
- overload recovery details
- detailed CMRR/PSRR frequency curves
- output supply-current realism under load
- distortion/THD
- EMI behavior
- exact phase margin versus arbitrary capacitive loads

For KiCad front-end work, filtering, gain staging, saturation checks, slew
checks, and approximate noise budgeting, this model should be much easier to use
than the TI PSpice macro-model.
