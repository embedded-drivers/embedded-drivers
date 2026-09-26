# edrv-bme680

Rust driver for the BME680 temperature, humidity, pressure and gas sensor.

## Overview

This crate provides a `no_std`, HAL-agnostic driver for the Bosch BME680. It
exposes both an asynchronous API (`edrv_bme680::BME680`) and a blocking one
(`edrv_bme680::blocking::BME680`) built on `embedded-hal` v1, and gives direct
access to the sensor's registers when the high-level API is not enough.

The compensation maths follows Bosch Sensortec's official BME68x
SensorAPI. The integer ("no FPU") Bosch compensation is used, so the driver's
own arithmetic is integer-only and does not pull in software floating point on
targets without an FPU.

## Maintenance

This project is maintained by the embedded-drivers team. Our organization's goal is to provide consistent driver access interfaces for embedded Rust and to maintain these drivers collectively to avoid orphaned crates.

## Features

- Temperature, pressure, humidity and gas resistance measurements.
- Both `embedded-hal-async` and blocking (`embedded-hal`) flavours with identical APIs.
- Integer compensation; the `f32` accessors on `Measurements` are a convenience on top.
- Raw ADC values (`raw_temperature()`, `raw_pressure()`, `raw_humidity()`, `raw_gas_resistance()`, `gas_range()`) exposed alongside the compensated ones.
- Configurable gas heater profile via `set_gas_heater(target_celsius, duration_ms)`.
- Raw register access: `read_reg`, `read_regs` and `write_reg`.

## Wiring

The BME680 speaks I2C when `CSB` is tied high, and its address is selected by the
`SDO` pin:

| `SDO` | Address | Constant |
| ----- | ------- | -------- |
| Low (GND) | `0x76` | `edrv_bme680::ADDRESS` |
| High (VDDIO) | `0x77` | `edrv_bme680::ADDRESS_ALT` |

Use `BME680::new(i2c, addr)` for the alternative address, or
`BME680::new_primary(i2c)` for `0x76`. Pull-ups are required on both `SDA` and
`SCL`, as usual for I2C.

## Usage

```rust,ignore
use edrv_bme680::BME680;

let mut sensor = BME680::new(i2c, edrv_bme680::ADDRESS);
sensor.reset(&mut delay).await?; // soft reset, then wait 10 ms
sensor.init().await?;            // chip id, calibration, default settings

let measurement = sensor.measure(&mut delay).await?;
let temperature = measurement.temperature_celsius();
let pressure_hpa = measurement.pressure_hpa();
let humidity = measurement.humidity_percent();
let gas_ohms = measurement.gas_resistance_ohms();
```

The blocking driver has the same shape:

```rust,ignore
use edrv_bme680::blocking::BME680;

let mut sensor = BME680::new_primary(i2c);
sensor.reset(&mut delay)?;
sensor.init()?;
let measurement = sensor.measure(&mut delay)?;
```

`init` validates the chip ID (`0x61`), reads the calibration data and programs
temperature oversampling x2, pressure x16, humidity x1, IIR filter size 3 and the
gas heater (320 C for 150 ms), leaving the part in sleep mode. Unlike Bosch's own
`bme68x_init`, it does not soft reset; call `reset` first for the full Bosch
sequence. `measure` performs a single forced-mode conversion, so the sensor stays
in sleep mode between readings.

`reset` and `measure` need a delay, exactly like `edrv-bme280`'s `reset`. The
ambient temperature used to compute the heater resistance defaults to 25 C
because the BME680 cannot measure it before its first conversion; override
`BME680::ambient_temperature_celsius` if needed.

## What the gas resistance means

The gas reading is the resistance of a metal-oxide (MOX) sensing element, in
ohms. Clean air gives a high resistance; reducing gases (VOCs) lower it. It is
**not** a calibrated air-quality number:

- It drifts with temperature, humidity and the sensor's own history, so absolute
  values are not comparable between devices.
- A new sensor needs a burn-in period (Bosch suggests running the heater for
  tens of minutes to hours) before readings stabilise.
- Bosch's VOC *index* and equivalent-CO2 numbers are produced by the closed-source
  BSEC library, which runs a stateful calibration model over many samples. This
  driver deliberately stops at resistance and does not attempt to reproduce BSEC.
- Check `Measurements::gas_valid` and `Measurements::heat_stable` before using a
  reading; `heat_stable` says the heater reached its set point.

Only the BME680 (`BME68X_VARIANT_GAS_LOW`) path is implemented. The BME688 shares
the die and the chip ID but uses a different gas-resistance formula
(`BME68X_VARIANT_GAS_HIGH`); `BME680::variant_id` reports `0x01` on such a part
and its gas readings should be discarded.

## Documentation

[Docs.rs link](https://docs.rs/edrv-bme680/)

## Reference

[Bosch Sensortec's BME68x SensorAPI](https://github.com/BoschSensortec/BME68x_SensorAPI):
the register map, the calibration register layout, the temperature / pressure /
humidity compensation algorithms, the gas-resistance compensation and its two
lookup tables, and the heater-resistance and heater-duration calculations. Only
the BME680 (`BME68X_VARIANT_GAS_LOW`) gas path is implemented.

## Contributing & License

Please refer to [embedded-drivers](https://github.com/embedded-drivers/embedded-drivers)

This crate is distributed as `MIT OR Apache-2.0`. It also references Bosch's
BSD-3-Clause licensed driver; the retained notice is in
[`LICENSE-BOSCH`](LICENSE-BOSCH).
