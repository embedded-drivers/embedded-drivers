//! Blocking-mode driver for Plantower PMSx003 particulate sensors.
//!
//! See the [crate documentation](crate) for the frame layout and the notes on
//! field reliability.

use embedded_io::{Read, ReadExactError};

use crate::{Error, FRAME_LEN, Frame, HEADER, HeaderSync};

/// PMSx003 driver, blocking.
pub struct Pmsx003<UART> {
    uart: UART,
    buffer: [u8; FRAME_LEN],
}

impl<UART> Pmsx003<UART>
where
    UART: Read,
{
    /// Create a driver from any byte-stream transport.
    pub fn new(uart: UART) -> Self {
        Self {
            uart,
            buffer: [0; FRAME_LEN],
        }
    }

    /// Consume the driver and return the transport.
    pub fn release(self) -> UART {
        self.uart
    }

    /// Read the next valid frame, resynchronising on the header as needed.
    ///
    /// Frames that fail validation are skipped rather than returned, so this
    /// keeps reading until it has a good one. There is no timeout: a sensor that
    /// never sends anything blocks forever.
    pub fn read_frame(&mut self) -> Result<Frame, Error<UART::Error>> {
        loop {
            self.sync()?;
            self.buffer[0] = HEADER[0];
            self.buffer[1] = HEADER[1];

            self.uart.read_exact(&mut self.buffer[2..]).map_err(|e| match e {
                ReadExactError::UnexpectedEof => Error::UnexpectedEof,
                ReadExactError::Other(e) => Error::Transport(e),
            })?;

            if let Some(frame) = Frame::parse(&self.buffer) {
                return Ok(frame);
            }
            // Bad frame: drop sync and hunt for the next header.
        }
    }

    /// Consume bytes until the `0x42 0x4D` header has been seen.
    fn sync(&mut self) -> Result<(), Error<UART::Error>> {
        let mut sync = HeaderSync::default();
        let mut byte = [0u8; 1];

        loop {
            let read = self.uart.read(&mut byte).map_err(Error::Transport)?;
            if read == 0 {
                continue;
            }
            if sync.push(byte[0]) {
                return Ok(());
            }
        }
    }
}
