//! Driver for the Winsen MH-Z19 / MH-Z19B NDIR CO2 sensor.
//!
//! This is the **async** driver. See [`blocking`] for the blocking one.
//!
//! The sensor speaks a request/response protocol over a 9600 8N1 UART. Both
//! directions are 9-byte packets:
//!
//! ```text
//!   [0] 0xFF  start byte
//!   [1] 0x01  sensor address (command) / command echo (response)
//!   [2] command
//!   [3..8] data, checksum in [8]
//! ```
//!
//! Checksum = `255 - sum(bytes[1..8]) + 1`.
//!
//! # Transport
//!
//! The driver is generic over [`embedded_io_async::Read`] +
//! [`embedded_io_async::Write`], so it works with any HAL that implements the
//! standard serial traits.
//!
//! # No timeouts here
//!
//! `embedded-io` has no notion of a timeout, so every read blocks until the
//! transport delivers. `read_co2` on a sensor that is unpowered or mis-wired
//! therefore blocks forever rather than returning an error: wrap the transport
//! (or the future) with a timeout in the application if that matters.
//!
//! For the same reason the calibration commands, which the reference driver
//! sends without reading a reply, are send-only here. A sensor that does not
//! answer them would otherwise hang the caller.
//!
//! # Resynchronising
//!
//! `read_co2` reads exactly nine bytes. If a read is interrupted mid-frame the
//! next attempt will be offset by a byte or two and fail the start-byte or
//! checksum check. There is no way to drain the input through `embedded-io`
//! without a readiness trait, so the application should reset the transport
//! (or discard input) after such a failure.

#![cfg_attr(not(test), no_std)]

pub mod blocking;

/// The sensor's address byte. MH-Z19 sensors are point-to-point, so this is a
/// constant rather than something to configure.
pub const ADDRESS: u8 = 0x01;

/// Packet framing.
pub const START_BYTE: u8 = 0xFF;
pub const PACKET_LEN: usize = 9;

/// Command bytes.
pub mod cmds {
    /// Read the CO2 concentration.
    pub const READ_CO2: u8 = 0x86;
    /// Zero point calibration (assumes the sensor sits in ~400 ppm air).
    pub const CALIBRATE_ZERO: u8 = 0x87;
    /// Span point calibration.
    pub const CALIBRATE_SPAN: u8 = 0x88;
    /// Automatic baseline correction on/off.
    pub const ABC: u8 = 0x79;
    /// Detection range.
    pub const RANGE: u8 = 0x99;
}

/// Detection range values the sensor accepts.
pub const RANGE_MIN_PPM: u16 = 1000;
pub const RANGE_MAX_PPM: u16 = 10000;

/// Driver error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// The transport failed.
    Transport(E),
    /// The stream ended before a full packet arrived.
    UnexpectedEof,
    /// The packet did not begin with `0xFF`.
    BadStartByte(u8),
    /// The packet did not echo the command that was sent.
    WrongCommand { expected: u8, got: u8 },
    /// The checksum did not match.
    BadChecksum { expected: u8, got: u8 },
    /// An argument was outside the range the sensor accepts.
    InvalidArgument,
}

/// Checksum over bytes 1..=7, matching the reference implementation
/// (`255 - sum; sum++`).
pub fn checksum(packet: &[u8; PACKET_LEN]) -> u8 {
    let sum = packet[1..PACKET_LEN - 1]
        .iter()
        .fold(0u8, |acc, byte| acc.wrapping_add(*byte));
    0xFFu8.wrapping_sub(sum).wrapping_add(1)
}

/// Build a command packet with the correct header and checksum.
pub fn build_command(command: u8, b3: u8, b4: u8, b5: u8, b6: u8, b7: u8) -> [u8; PACKET_LEN] {
    let mut packet = [START_BYTE, ADDRESS, command, b3, b4, b5, b6, b7, 0x00];
    packet[PACKET_LEN - 1] = checksum(&packet);
    packet
}

/// A decoded `0x86` response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reading {
    /// CO2 concentration in ppm.
    pub co2_ppm: u16,
    /// The sensor's internal temperature, in degrees Celsius.
    ///
    /// This is `response[4] - 40`; on several MH-Z19B firmware revisions that
    /// byte is zero or unused, which shows up as a constant -40 C.
    pub temperature_c: i16,
    /// The status/accuracy byte, `response[5]`.
    pub status: u8,
    /// `response[6..8]`. The reference driver notes this needs further
    /// calculation before it is usable, so treat it as diagnostic only.
    pub min_co2_ppm: u16,
}

impl Reading {
    /// Decode the fields of a `0x86` response. The packet is assumed to have
    /// been validated already.
    pub fn from_packet(packet: &[u8; PACKET_LEN]) -> Self {
        Self {
            co2_ppm: u16::from_be_bytes([packet[2], packet[3]]),
            temperature_c: packet[4] as i16 - 40,
            status: packet[5],
            min_co2_ppm: u16::from_be_bytes([packet[6], packet[7]]),
        }
    }
}

/// MH-Z19 driver.
pub struct MHZ19<UART> {
    uart: UART,
    /// Last command sent, checked against the response's echo byte.
    command: u8,
    /// Last response packet.
    response: [u8; PACKET_LEN],
}

impl<UART> MHZ19<UART>
where
    UART: embedded_io_async::Read + embedded_io_async::Write,
{
    /// Create a driver from any byte-stream transport.
    pub fn new(uart: UART) -> Self {
        Self {
            uart,
            command: 0,
            response: [0; PACKET_LEN],
        }
    }

    /// Consume the driver and return the transport.
    pub fn release(self) -> UART {
        self.uart
    }

    /// The raw packet from the last exchange.
    pub fn last_response(&self) -> &[u8; PACKET_LEN] {
        &self.response
    }

    /// Send a command and read its response, validating framing.
    async fn exchange(
        &mut self,
        command: u8,
        b3: u8,
        b4: u8,
        b5: u8,
        b6: u8,
        b7: u8,
    ) -> Result<(), Error<UART::Error>> {
        self.command = command;
        let packet = build_command(command, b3, b4, b5, b6, b7);
        self.uart.write_all(&packet).await.map_err(Error::Transport)?;
        self.receive().await
    }

    /// Send a command that the sensor does not acknowledge.
    async fn send_only(
        &mut self,
        command: u8,
        b3: u8,
        b4: u8,
        b5: u8,
        b6: u8,
        b7: u8,
    ) -> Result<(), Error<UART::Error>> {
        let packet = build_command(command, b3, b4, b5, b6, b7);
        self.uart.write_all(&packet).await.map_err(Error::Transport)
    }

    /// Read one response packet and validate it.
    async fn receive(&mut self) -> Result<(), Error<UART::Error>> {
        use embedded_io_async::ReadExactError;

        self.uart.read_exact(&mut self.response).await.map_err(|e| match e {
            ReadExactError::UnexpectedEof => Error::UnexpectedEof,
            ReadExactError::Other(e) => Error::Transport(e),
        })?;

        self.validate()
    }

    fn validate(&self) -> Result<(), Error<UART::Error>> {
        let packet = &self.response;

        if packet[0] != START_BYTE {
            return Err(Error::BadStartByte(packet[0]));
        }
        if packet[1] != self.command {
            return Err(Error::WrongCommand {
                expected: self.command,
                got: packet[1],
            });
        }

        let expected = checksum(packet);
        if packet[PACKET_LEN - 1] != expected {
            return Err(Error::BadChecksum {
                expected,
                got: packet[PACKET_LEN - 1],
            });
        }

        Ok(())
    }

    /// Read the CO2 concentration (`0x86`).
    pub async fn read_co2(&mut self) -> Result<Reading, Error<UART::Error>> {
        self.exchange(cmds::READ_CO2, 0x00, 0x00, 0x00, 0x00, 0x00).await?;
        Ok(Reading::from_packet(&self.response))
    }

    /// Zero point calibration (`0x87`).
    ///
    /// The datasheet requires the sensor to have sat in **fresh air at about
    /// 400 ppm for at least 20 minutes** first. "Zero" means 400 ppm, not
    /// 0 ppm; running this in an occupied room bakes a wrong baseline into the
    /// sensor. Send-only, because the sensor does not acknowledge it.
    pub async fn calibrate_zero(&mut self) -> Result<(), Error<UART::Error>> {
        self.send_only(cmds::CALIBRATE_ZERO, 0x00, 0x00, 0x00, 0x00, 0x00).await
    }

    /// Span point calibration (`0x88`) against a known reference gas.
    ///
    /// Do the zero calibration first. Rejects spans below 1000 ppm, matching the
    /// reference driver. Send-only.
    pub async fn calibrate_span(&mut self, span_ppm: u16) -> Result<(), Error<UART::Error>> {
        if span_ppm < RANGE_MIN_PPM {
            return Err(Error::InvalidArgument);
        }
        let [msb, lsb] = span_ppm.to_be_bytes();
        self.send_only(cmds::CALIBRATE_SPAN, msb, lsb, 0x00, 0x00, 0x00).await
    }

    /// Turn automatic baseline correction on or off (`0x79`). Send-only.
    pub async fn set_auto_calibration(&mut self, enabled: bool) -> Result<(), Error<UART::Error>> {
        let value = if enabled { 0xA0 } else { 0x00 };
        self.send_only(cmds::ABC, value, 0x00, 0x00, 0x00, 0x00).await
    }

    /// Set the detection range (`0x99`). Send-only.
    ///
    /// The value goes big-endian in bytes 6..8; 2000 ppm is what the sensor is
    /// most accurate at.
    pub async fn set_detection_range(&mut self, range_ppm: u16) -> Result<(), Error<UART::Error>> {
        if !(RANGE_MIN_PPM..=RANGE_MAX_PPM).contains(&range_ppm) {
            return Err(Error::InvalidArgument);
        }
        let [msb, lsb] = range_ppm.to_be_bytes();
        self.send_only(cmds::RANGE, 0x00, 0x00, 0x00, msb, lsb).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_co2_command_matches_the_known_good_packet() {
        // 0xFF 01 86 00 00 00 00 00 79 is the packet every MH-Z19 datasheet and
        // driver implementation agrees on.
        assert_eq!(
            build_command(cmds::READ_CO2, 0, 0, 0, 0, 0),
            [0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79]
        );
    }

    #[test]
    fn checksum_matches_reference_vectors() {
        assert_eq!(checksum(&[0xFF, 0x01, 0x86, 0, 0, 0, 0, 0, 0]), 0x79);
        assert_eq!(checksum(&[0xFF, 0x01, 0x87, 0, 0, 0, 0, 0, 0]), 0x78);
        // Span 2000 ppm: FF 01 88 07 D0 00 00 00 A0
        assert_eq!(checksum(&[0xFF, 0x01, 0x88, 0x07, 0xD0, 0, 0, 0, 0]), 0xA0);
    }

    #[test]
    fn decodes_a_reading() {
        // 0x019A = 410 ppm, byte[4] = 0x48 -> 32 C.
        let mut packet = [0xFF, 0x86, 0x01, 0x9A, 0x48, 0x00, 0x00, 0x00, 0x00];
        packet[8] = checksum(&packet);
        assert_eq!(packet[8], 0x97);

        let reading = Reading::from_packet(&packet);
        assert_eq!(reading.co2_ppm, 410);
        assert_eq!(reading.temperature_c, 32);
        assert_eq!(reading.status, 0);
    }
}
