# PGA855 functional ngspice model

`PGA855_functional.lib` is an unofficial system-level model based on the
PGA855 datasheet SBOSAE0B.  The subcircuit name is:

```spice
PGA855_FUNC
```

Pin order:

```text
A2 IN+ IN- A0 A1 VS+ LVDD FDA_IN- OUT+ OUT- FDA_IN+
VOCM LVSS VS- DGND
```

The package's NC pin 8 and thermal pad are not simulation pins.

## KiCad physical-pin mapping

| Physical pin | Signal | Model position |
|---:|---|---:|
| 1 | A2 | 1 |
| 2 | IN+ | 2 |
| 3 | IN- | 3 |
| 4 | A0 | 4 |
| 5 | A1 | 5 |
| 6 | VS+ | 6 |
| 7 | LVDD | 7 |
| 8 | NC | Not connected |
| 9 | FDA_IN- | 8 |
| 10 | OUT+ | 9 |
| 11 | OUT- | 10 |
| 12 | FDA_IN+ | 11 |
| 13 | VOCM | 12 |
| 14 | LVSS | 13 |
| 15 | VS- | 14 |
| 16 | DGND | 15 |

Select `PGA855_FUNC` in KiCad and use compatibility mode **None**.

Gain selection:

| A2:A0 | Gain |
|---|---:|
| 000 | 0.125 |
| 001 | 0.25 |
| 010 | 0.5 |
| 011 | 1 |
| 100 | 2 |
| 101 | 4 |
| 110 | 8 |
| 111 | 16 |

Use `VS+` and `VS-` for the input stage.  Use `LVDD` and `LVSS` for the
output stage.  Drive `VOCM` to the desired output common-mode voltage; if it
is left floating, the model biases it at the midpoint of `LVDD` and `LVSS`.

The model includes gain-dependent white voltage noise for `.noise` analysis.
Detailed 1/f noise is not included.

## Validation

Run:

```sh
python3 tools/spice/tests/validate_pga855_functional.py
```

Validated with ngspice 46:

| Check | Model result | Datasheet/expected |
|---|---:|---:|
| Gain range | 0.125 to 16 | 0.125 to 16 |
| Bandwidth at G=16 | 9.98 MHz | 10 MHz typical |
| Differential slew rate | 34.99 V/us | 35 V/us typical |
| Output rails, LVSS/LVDD=0/5 V | 0.204 V to 4.796 V | approximately 0.2 V from rails |
| Input voltage noise at G=16 | 7.799 nV/sqrt(Hz) | 7.8 nV/sqrt(Hz) |
| Input-stage quiescent current | 3 mA | 3 mA typical |
| Output-stage quiescent current | 2.3 mA | 2.3 mA typical |

The model uses a nominal gain error of zero and a deterministic +70-uV
typical input offset. Production distributions and temperature drift are not
simulated.
