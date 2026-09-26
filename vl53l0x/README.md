# edrv-vl53l0x

Rust driver for the ST VL53L0X time-of-flight distance sensor.

## Overview

This project provides a Rust driver for the ST VL53L0X time-of-flight (ToF) distance sensor. It aims to offer a consistent and reliable interface for interacting with the VL53L0X in embedded Rust applications. Both blocking and asynchronous interfaces are provided.

## Maintenance

This project is maintained by the embedded-drivers team. Our organization's goal is to provide consistent driver access interfaces for embedded Rust and to maintain these drivers collectively to avoid orphaned crates.

## Features

- Single-shot and continuous distance measurements, in millimetres
- Full ST initialisation sequence (2V8 mode, SPAD discovery, reference calibration, default tuning settings)
- Measurement timing budget configuration
- Blocking (`blocking::VL53L0X`) and asynchronous (`VL53L0X`) drivers

## Usage

[Provide basic usage examples]

## Documentation

[Docs.rs link](https://docs.rs/edrv-vl53l0x/)

## Contributing & License

Please refer to [embedded-drivers](https://github.com/embedded-drivers/embedded-drivers)
