# edrv-spl06

Rust driver for the Goertek SPL06-001 and the register-compatible SPL06-007
barometric pressure sensor.

## Overview

This crate provides a `no_std`, HAL-agnostic driver for the SPL06-001 /
SPL06-007 digital barometer. It exposes both an asynchronous API
(`edrv_spl06::SPL06`) and a blocking one (`edrv_spl06::blocking::SPL06`) built
on `embedded-hal` v1, and gives direct access to the sensor's registers when the
high-level API is not enough.

The two parts are the same device as far as software is concerned: the
SPL06-001 V1.0 and SPL06-007 V1.0 datasheets specify the same register map, the
same calibration coefficient layout and the same compensation formulas, and both
give `0x10` as the reset value of the product/revision ID register, so this
driver accepts either part and cannot tell them apart.

The compensation is evaluated in integer Q24 fixed-point arithmetic, so the
driver itself does no floating point. The integer results (`Measurements::pressure`
in pascal and `Measurements::temperature` in hundredths of a degree Celsius) are
the primary output; the `f32` accessors are a convenience and are the only place
a conversion happens, which keeps software floating point out of a target
without an FPU (such as `riscv32imc`) unless it is actually used.

## Maintenance

This project is maintained by the embedded-drivers team. Our organization's goal is to provide consistent driver access interfaces for embedded Rust and to maintain these drivers collectively to avoid orphaned crates.

## Features

- Temperature and pressure measurements.
- Both `embedded-hal-async` and blocking (`embedded-hal`) flavours with identical APIs.
- Integer (Q24 fixed-point) compensation; the `f32` accessors are a convenience on top.
- Raw 24-bit ADC values (`raw_temperature()`, `raw_pressure()`) exposed alongside the compensated ones.
- Configurable oversampling, with the mandatory result bit-shift handled automatically.
- Raw register access: `read_reg`, `read_regs` and `write_reg`.
- `reset(delay)` is a separate step; `init(delay)` never soft resets.

## Wiring

The SPL06-001 / SPL06-007 speaks I2C when `CSB` is tied high, and its address is
selected by the `SDO` pin:

| `SDO` | Address | Constant |
| ----- | ------- | -------- |
| High (VDDIO) - **default** | `0x77` | `edrv_spl06::ADDRESS` |
| Low (GND) | `0x76` | `edrv_spl06::ADDRESS_ALT` |

Use `SPL06::new(i2c, addr)` for a specific address, or `SPL06::new_primary(i2c)`
for the default `0x77`. Pull-ups are required on both `SDA` and `SCL`, as usual
for I2C. Note the default is `0x77`, the opposite of Bosch's barometers, whose
primary address is `0x76`.

## Usage

```rust,ignore
use edrv_spl06::SPL06;

let mut sensor = SPL06::new(i2c, edrv_spl06::ADDRESS);
sensor.init(&mut delay).await?; // product ID, start-up, calibration, default oversampling

let measurement = sensor.measure(&mut delay).await?;
let pressure_pa = measurement.pressure;          // integer pascal
let temperature_centi_c = measurement.temperature; // 0.01 degC
let pressure_hpa = measurement.pressure_hpa();   // f32 convenience
```

The blocking driver has the same shape:

```rust,ignore
use edrv_spl06::blocking::SPL06;

let mut sensor = SPL06::new_primary(i2c);
sensor.init(&mut delay)?;
let measurement = sensor.measure(&mut delay)?;
```

`init` validates the product/revision ID (`0x10`), waits for the part to report
that its start-up sequence has finished, reads the 18-byte calibration block at
`0x10` and programs 8 times oversampling on both channels. `measure` performs a
single command-mode temperature conversion followed by a pressure conversion,
polling the ready flags with a timeout, so the sensor stays in standby between
readings. Unlike the vendor's start-up sequence, `init` does not soft reset; call
`reset(&mut delay)` first for a full power-on sequence, then `init` again.

The start-up wait matters. The calibration coefficients are not available for
`TCoef_rdy` (40 ms) after power-on, and the part answers a read in that window
with an all-zero block instead of an error. Every term of the compensation
polynomial is multiplied by a coefficient, so that block decodes to exactly 0 Pa
and 0.00 degC and the readings never look obviously broken. `init` therefore
polls `MEAS_CFG` for `COEF_RDY` (bit 7, "calibration coefficients valid") and
returns an error if it does not appear, rather than accepting a blank block. Only
that bit is required: `SENSOR_RDY` (bit 6) reports the sensor's own
initialisation, which is not a precondition for reading the coefficient block.
`reset(&mut delay)` waits the same 40 ms before returning.

Use `set_oversampling(pressure, temperature)` to change the precision; it writes
the configuration registers immediately. `init`, `measure` and `reset` all need a
delay, exactly like `edrv-bme280`'s and `edrv-bme680`'s.

## Oversampling and the `kP` / `kT` scale factors

The raw 24-bit conversion results are **not** used directly. The datasheet
defines

```text
Traw_sc = Traw / kT        Praw_sc = Praw / kP
Tcomp   = c0 * 0.5 + c1 * Traw_sc
Pcomp   = c00 + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * c30))
                + Traw_sc * c01 + Traw_sc * Praw_sc * (c11 + Praw_sc * c21)
```

where `kP` and `kT` depend on the configured oversampling rate:

| Oversampling | `kP` / `kT` |
| ------------ | ----------- |
| 1 (single) | 524288 |
| 2 (Low Power) | 1572864 |
| 4 | 3670016 |
| 8 | 7864320 |
| 16 (Standard) | 253952 |
| 32 | 516096 |
| 64 (High Precision) | 1040384 |
| 128 | 2088960 |

For oversampling above 8 times the corresponding result bit-shift in `CFG_REG`
is mandatory; `Config::control_writes` sets it automatically. Feeding raw data
through the wrong scale factor produces a plausible-looking but wrong reading
(the 8-times and single-oversampling factors differ by a factor of 15), so the
driver always derives both factors from the configuration it programmed.

## Documentation

[Docs.rs link](https://docs.rs/edrv-spl06/)

## Reference

The authority for all behaviour in this crate is the vendor datasheet:

- **Goertek SPL06-001 V1.0** ("SPL06-001 Digital pressure sensor"): the register
  map (section 7/8), the calibration coefficient layout (Table 13) and the
  compensation formulas and scale factors (sections 5.7.1/5.7.2, Table 7).
- **Goertek SPL06-007 V1.0** (also distributed as the Infineon SPL06-007): the
  register map, coefficient layout, formulas and scale factors are identical,
  and its ID reset value is also `0x10`.

## Contributing & License

Please refer to [embedded-drivers](https://github.com/embedded-drivers/embedded-drivers)

This crate is distributed as `MIT OR Apache-2.0`.
