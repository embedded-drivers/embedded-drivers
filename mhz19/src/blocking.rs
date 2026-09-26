//! Blocking-mode driver for the Winsen MH-Z19 / MH-Z19B NDIR CO2 sensor.
//!
//! See the [crate documentation](crate) for the protocol and the calibration
//! preconditions.
//!
//! The driver is generic over [`embedded_io::Read`] + [`embedded_io::Write`],
//! so it works with any HAL that implements the standard serial traits.

use embedded_io::{Read, ReadExactError, Write};

use crate::{Error, PACKET_LEN, Reading, START_BYTE, build_command, checksum, cmds};

/// MH-Z19 driver, blocking.
pub struct MHZ19<UART> {
    uart: UART,
    /// Last command sent, checked against the response's echo byte.
    command: u8,
    /// Last response packet.
    response: [u8; PACKET_LEN],
}

impl<UART> MHZ19<UART>
where
    UART: Read + Write,
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
    fn exchange(&mut self, command: u8, b3: u8, b4: u8, b5: u8, b6: u8, b7: u8) -> Result<(), Error<UART::Error>> {
        self.command = command;
        let packet = build_command(command, b3, b4, b5, b6, b7);
        self.uart.write_all(&packet).map_err(Error::Transport)?;
        self.receive()
    }

    /// Send a command that the sensor does not acknowledge.
    fn send_only(&mut self, command: u8, b3: u8, b4: u8, b5: u8, b6: u8, b7: u8) -> Result<(), Error<UART::Error>> {
        let packet = build_command(command, b3, b4, b5, b6, b7);
        self.uart.write_all(&packet).map_err(Error::Transport)
    }

    /// Read one response packet and validate it.
    fn receive(&mut self) -> Result<(), Error<UART::Error>> {
        self.uart.read_exact(&mut self.response).map_err(|e| match e {
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
    pub fn read_co2(&mut self) -> Result<Reading, Error<UART::Error>> {
        self.exchange(cmds::READ_CO2, 0x00, 0x00, 0x00, 0x00, 0x00)?;
        Ok(Reading::from_packet(&self.response))
    }

    /// Zero point calibration (`0x87`).
    ///
    /// The datasheet requires the sensor to have sat in **fresh air at about
    /// 400 ppm for at least 20 minutes** first. "Zero" means 400 ppm, not
    /// 0 ppm; running this in an occupied room bakes a wrong baseline into the
    /// sensor. Send-only, because the sensor does not acknowledge it.
    pub fn calibrate_zero(&mut self) -> Result<(), Error<UART::Error>> {
        self.send_only(cmds::CALIBRATE_ZERO, 0x00, 0x00, 0x00, 0x00, 0x00)
    }

    /// Span point calibration (`0x88`) against a known reference gas.
    ///
    /// Do the zero calibration first. Rejects spans below 1000 ppm, matching the
    /// reference driver. Send-only.
    pub fn calibrate_span(&mut self, span_ppm: u16) -> Result<(), Error<UART::Error>> {
        if span_ppm < crate::RANGE_MIN_PPM {
            return Err(Error::InvalidArgument);
        }
        let [msb, lsb] = span_ppm.to_be_bytes();
        self.send_only(cmds::CALIBRATE_SPAN, msb, lsb, 0x00, 0x00, 0x00)
    }

    /// Turn automatic baseline correction on or off (`0x79`). Send-only.
    pub fn set_auto_calibration(&mut self, enabled: bool) -> Result<(), Error<UART::Error>> {
        let value = if enabled { 0xA0 } else { 0x00 };
        self.send_only(cmds::ABC, value, 0x00, 0x00, 0x00, 0x00)
    }

    /// Set the detection range (`0x99`). Send-only.
    ///
    /// The value goes big-endian in bytes 6..8; 2000 ppm is what the sensor is
    /// most accurate at.
    pub fn set_detection_range(&mut self, range_ppm: u16) -> Result<(), Error<UART::Error>> {
        if !(crate::RANGE_MIN_PPM..=crate::RANGE_MAX_PPM).contains(&range_ppm) {
            return Err(Error::InvalidArgument);
        }
        let [msb, lsb] = range_ppm.to_be_bytes();
        self.send_only(cmds::RANGE, 0x00, 0x00, 0x00, msb, lsb)
    }
}
