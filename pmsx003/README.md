# edrv-pmsx003

Rust driver for Plantower PMSx003 particulate sensors (PMS1003, PMS3003,
PMS5003, PMS7003, PMS9003).

## Overview

These sensors stream 32-byte frames over a 9600 8N1 UART without needing any
command. The driver validates the `0x42 0x4D` header, the payload length and the
checksum of every frame, and resynchronises on the header when the stream is
misaligned.

Both an async and a blocking API are provided:

| module | traits required |
|---|---|
| `edrv_pmsx003::Pmsx003` | `embedded_io_async::Read` |
| `edrv_pmsx003::blocking::Pmsx003` | `embedded_io::Read` |

Being generic over the standard `embedded-io` traits means the driver works with
any HAL that implements them.

## Field reliability

`pm1_0`, `pm2_5` and `pm10` are the measurements to use. The six particle-count
bins are cumulative, so each must be less than or equal to the bin for the next
smaller diameter; `Frame::bins_monotonic` checks that. Compatible/clone modules
are known to report plausible-looking garbage in those bins.

## Timeouts

`embedded-io` has no timeout concept, so `read_frame` blocks until a valid frame
arrives. Wrap the transport or the call in a timeout in the application.

## Maintenance

This project is maintained by the embedded-drivers team. Our organization's goal
is to provide consistent driver access interfaces for embedded Rust and to
maintain these drivers collectively to avoid orphaned crates.

## License

Licensed under either of MIT or Apache-2.0, at your option.
