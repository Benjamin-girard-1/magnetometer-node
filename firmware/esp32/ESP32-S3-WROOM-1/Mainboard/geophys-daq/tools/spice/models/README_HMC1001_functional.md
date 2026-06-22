# HMC1001 functional ngspice model

This is an unofficial system-level HMC1001 AMR bridge model for ngspice and
KiCad.  The bridge is modeled as four slightly unbalanced resistors, so loading
or feedback applied to `OUT+` / `OUT-` affects the bridge nodes physically.

Three subcircuits are available:

- `HMC1001_SPICE`: simplified five-pin model matching the custom KiCad
  symbol: `Vb`, `OUT+`, `OUT-`, `GND`, `FIELD`.
- `HMC1001_FUNC`: eight physical package pins and a static `FIELD` parameter.
- `HMC1001_CTRL`: the same package pins plus a `BFIELD` control pin, where
  `1 V = 1 gauss`. Use this variant for transient magnetic-field simulations.

## Recommended HMC1001_SPICE mapping

Assign these pin numbers in the KiCad symbol:

| Symbol pin | Name | Model position |
|---:|---|---:|
| 1 | Vb | 1 |
| 2 | OUT+ | 2 |
| 3 | OUT- | 3 |
| 4 | GND | 4 |
| 5 | FIELD | 5 |

The current symbol file has blank pin numbers. KiCad cannot create a SPICE
mapping until each pin has a unique number.

Select:

```text
Model: HMC1001_SPICE
Compatibility mode: None
```

Drive `FIELD` relative to `GND`. One volt represents one gauss.

## Main parameters

```text
RBRIDGE = 850       ohms
SENS    = 3.2m      V/V/gauss
OFFSET  = 0         volts differential at VCAL
VCAL    = 8         bridge voltage where OFFSET is specified
FIELD   = 0         gauss, HMC1001_FUNC only
POL     = 1         use -1 to reverse magnetic polarity
```

For example, with a 5-V bridge supply, `SENS=3.2m`, `OFFSET=1m`, `VCAL=5`,
and a 1-gauss field:

```text
VOUT_DIFF = 1 mV + 5 * 0.0032 * 1 = 17 mV
```

For your offset-balancing circuit, this is the important case:

```text
OFFSET = 10m
VCAL   = 8
Vb     = 9
FIELD  = 0
```

The model creates a real resistor mismatch:

```text
delta_offset = OFFSET / VCAL = 10m / 8 = 0.00125
```

So the unloaded bridge offset at 9 V becomes:

```text
VOUT_DIFF = 9 * 0.00125 = 11.25 mV
```

## Physical-pin mapping for HMC1001_FUNC

| Physical pin | Signal | Model position |
|---:|---|---:|
| 1 | S/R+ | 1 |
| 2 | OFFSET+ | 2 |
| 3 | S/R- | 3 |
| 4 | GND / VBRIDGE- | 4 |
| 5 | OUT+ | 5 |
| 6 | OFFSET- | 6 |
| 7 | VBRIDGE | 7 |
| 8 | OUT- | 8 |

Select compatibility mode **None** in KiCad.

## Noise

The model targets:

```text
white floor = 3.8 nV/sqrt(Hz)
flicker term = 30 nV/sqrt(Hz) at 1 Hz
```

The resulting total density is approximately:

```text
sqrt((3.8 nV)^2 + (30 nV)^2/f)
```

The noise model is intended for `.noise` analysis. It does not create a
random waveform in ordinary transient analysis.

Validated densities with ngspice 46:

| Frequency | Model |
|---:|---:|
| 0.1 Hz | 94.94 nV/sqrt(Hz) |
| 1 Hz | 30.23 nV/sqrt(Hz) |
| 10 Hz | 10.20 nV/sqrt(Hz) |
| 100 Hz | 4.81 nV/sqrt(Hz) |
| 1 kHz | 3.87 nV/sqrt(Hz) |

Run all checks with:

```sh
python3 tools/spice/tests/validate_hmc1001_functional.py
```

## Time-varying field in KiCad

With `HMC1001_SPICE`, drive `FIELD` using a voltage source:

```spice
SIN(0 0.01 10)
```

This represents a magnetic field of +/-0.01 gauss at 10 Hz.
