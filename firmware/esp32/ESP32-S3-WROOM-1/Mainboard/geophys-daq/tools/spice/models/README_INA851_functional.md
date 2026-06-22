# INA851 functional ngspice model

`INA851_functional.lib` is a system-level ngspice model for use when the
official TI PSpice/TINA model cannot be parsed by ngspice.

The subcircuit name is:

```spice
INA851_FUNC
```

Its 15-pin order is identical to the official TI model:

```text
IN+ IN- RG+ RG- VS+ VS- OUT+ OUT- VOCM
FDA_IN+ FDA_IN- G02+ G02- VCLAMP+ VCLAMP-
```

The model assumes the output-stage gain is 1. Connect an external resistor
between `RG+` and `RG-` to set:

```text
G = 1 + 6000 / RG
```

For unclamped operation, connect `VCLAMP+` to `VS+` and `VCLAMP-` to `VS-`.
The outputs are centered on `VOCM`.

In KiCad, select `INA851_FUNC` from this library. Keep the same pin mapping
used for the official `INA851` subcircuit.

## Validation

The validation netlists are in `tools/spice/tests`. Run all checks with:

```sh
python3 tools/spice/tests/validate_ina851_functional.py
```

Validated with ngspice 46:

| Check | Model result | Datasheet/expected |
|---|---:|---:|
| Gain with `RG=6 kOhm` | 2.000 nominal | 2.000 |
| Loaded AC gain, `RL=10 kOhm` | 6.019 dB | 6.021 dB |
| -3 dB bandwidth | 14.96 MHz | 15 MHz at gain 1 |
| Differential slew rate | 36.99 V/us | 37 V/us typical |
| Output swing, +/-15 V supplies | +/-13.6 V | about 1.4 V from rails |
| Wheatstone bridge, 850 Ohm +/-10 Ohm, gain 2 | 105.87 mVpp | 105.88 mVpp calculated |

## Important limitations

The bandwidth is fixed at 15 MHz. The real device bandwidth decreases at
higher gains. Detailed noise, CMRR, PSRR, output-current limiting, temperature
drift, and the `GOUT=0.2` mode are not included.
