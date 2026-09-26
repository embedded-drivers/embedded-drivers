# edrv-mhz19

Rust driver for the Winsen MH-Z19 / MH-Z19B NDIR CO2 sensor.

## Overview

The MH-Z19 is a CO2 sensor with a request/response UART protocol at 9600 8N1.
Both directions use 9-byte packets; the driver validates the start byte, the
command echo and the checksum of every response.

Both an async and a blocking API are provided:

| module | traits required |
|---|---|
| `edrv_mhz19::MHZ19` | `embedded_io_async::Read + Write` |
| `edrv_mhz19::blocking::MHZ19` | `embedded_io::Read + Write` |

Being generic over the standard `embedded-io` traits means the driver works with
any HAL that implements them.

## Calibration

`calibrate_zero` assumes the sensor has been sitting in **fresh air of about
400 ppm for at least 20 minutes** - "zero" is 400 ppm, not 0 ppm. `calibrate_span`
needs a known reference gas and should follow the zero calibration.

## Timeouts

`embedded-io` has no timeout concept, so `read_co2` blocks until the transport
delivers nine bytes. Wrap the transport or the call in a timeout in the
application if a missing sensor must not hang the caller.

## Maintenance

This project is maintained by the embedded-drivers team. Our organization's goal
is to provide consistent driver access interfaces for embedded Rust and to
maintain these drivers collectively to avoid orphaned crates.

## License

Licensed under either of MIT or Apache-2.0, at your option.
