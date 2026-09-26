//! Driver for Plantower PMSx003 particulate sensors.
//!
//! Covers the PMS1003, PMS3003, PMS5003, PMS7003 and PMS9003, which all use the
//! same protocol. This is the **async** driver; see [`blocking`] for the
//! blocking one.
//!
//! The sensor only ever transmits. It streams 32-byte frames continuously (one
//! every ~0.2-1 s) and needs no command:
//!
//! ```text
//!   [0..2]   0x42 0x4D  header
//!   [2..4]   frame length, 28
//!   [4..6]   PM1.0  (CF=1)
//!   [6..8]   PM2.5  (CF=1)
//!   [8..10]  PM10   (CF=1)
//!   [10..16] the same three under the atmospheric-environment calibration
//!   [16..28] particle counts per 0.1 L above 0.3/0.5/1.0/2.5/5.0/10 um
//!   [28..30] version / error code
//!   [30..32] checksum = sum of the first 30 bytes, big-endian
//! ```
//!
//! # Resynchronisation
//!
//! Because this is a stream, [`Pmsx003::read_frame`] hunts for the `0x42 0x4D`
//! header instead of assuming it is frame-aligned. Reading a fixed 32 bytes and
//! trusting it would latch a permanent byte offset after a single dropped byte,
//! and then report every frame as invalid forever.
//!
//! # Field reliability
//!
//! The mass concentrations (`pm1_0`, `pm2_5`, `pm10`) are what these sensors
//! measure directly. The six particle-count bins are cumulative, so each must be
//! less than or equal to the bin for the next smaller diameter;
//! [`Frame::bins_monotonic`] checks that, and compatible/clone modules are known
//! to report garbage there. Use it rather than trusting the bins.

#![cfg_attr(not(test), no_std)]

pub mod blocking;

/// Frame header.
pub const HEADER: [u8; 2] = [0x42, 0x4D];

/// Fixed frame size, header included.
pub const FRAME_LEN: usize = 32;

/// The payload-length field of a well-formed frame.
pub const PAYLOAD_LEN: u16 = 28;

/// Driver error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// The transport failed.
    Transport(E),
    /// The stream ended before a full frame arrived.
    UnexpectedEof,
}

/// Header synchroniser.
///
/// Feed it one byte at a time; `push` returns `true` when the `0x42 0x4D`
/// header has just been matched. A repeated `0x42` (`0x42 0x42 0x4D`) is handled
/// by keeping it as a possible new start rather than discarding it.
#[derive(Default)]
pub(crate) struct HeaderSync {
    seen_start: bool,
}

impl HeaderSync {
    pub(crate) fn push(&mut self, byte: u8) -> bool {
        if !self.seen_start {
            self.seen_start = byte == HEADER[0];
            return false;
        }

        if byte == HEADER[1] {
            self.seen_start = false;
            return true;
        }

        // Not a continuation: either this byte starts a new header or we lose
        // sync entirely.
        self.seen_start = byte == HEADER[0];
        false
    }
}

/// One decoded 32-byte frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame([u8; FRAME_LEN]);

impl Frame {
    /// Wrap a raw frame. The caller is responsible for having validated it;
    /// use [`Frame::parse`] to validate.
    pub const fn from_bytes(bytes: [u8; FRAME_LEN]) -> Self {
        Self(bytes)
    }

    /// Validate header, payload length and checksum.
    pub fn parse(bytes: &[u8; FRAME_LEN]) -> Option<Self> {
        if bytes[0..2] != HEADER {
            return None;
        }
        if u16::from_be_bytes([bytes[2], bytes[3]]) != PAYLOAD_LEN {
            return None;
        }
        if !Self::checksum_ok(bytes) {
            return None;
        }
        Some(Self(*bytes))
    }

    /// Sum of the first 30 bytes must equal the big-endian value in the last 2.
    pub fn checksum_ok(bytes: &[u8; FRAME_LEN]) -> bool {
        let sum = bytes[..FRAME_LEN - 2]
            .iter()
            .fold(0u16, |acc, byte| acc.wrapping_add(*byte as u16));
        sum == u16::from_be_bytes([bytes[FRAME_LEN - 2], bytes[FRAME_LEN - 1]])
    }

    /// The raw frame bytes.
    pub const fn as_bytes(&self) -> &[u8; FRAME_LEN] {
        &self.0
    }

    fn be16(&self, at: usize) -> u16 {
        u16::from_be_bytes([self.0[at], self.0[at + 1]])
    }

    /// PM1.0 under the factory (CF=1) calibration, ug/m3.
    pub fn pm1_0(&self) -> u16 {
        self.be16(4)
    }

    /// PM2.5 under the factory (CF=1) calibration, ug/m3.
    pub fn pm2_5(&self) -> u16 {
        self.be16(6)
    }

    /// PM10 under the factory (CF=1) calibration, ug/m3.
    pub fn pm10(&self) -> u16 {
        self.be16(8)
    }

    /// PM1.0 under the atmospheric-environment calibration, ug/m3.
    pub fn pm1_0_atm(&self) -> u16 {
        self.be16(10)
    }

    /// PM2.5 under the atmospheric-environment calibration, ug/m3.
    pub fn pm2_5_atm(&self) -> u16 {
        self.be16(12)
    }

    /// PM10 under the atmospheric-environment calibration, ug/m3.
    pub fn pm10_atm(&self) -> u16 {
        self.be16(14)
    }

    /// The six cumulative particle bins: >0.3, >0.5, >1.0, >2.5, >5.0, >10 um,
    /// in particles per 0.1 L.
    pub fn bins(&self) -> [u16; 6] {
        [
            self.be16(16),
            self.be16(18),
            self.be16(20),
            self.be16(22),
            self.be16(24),
            self.be16(26),
        ]
    }

    /// The bins are cumulative, so each must be <= the one for the next smaller
    /// diameter. A violation means the count fields are not real measurements,
    /// which is how compatible/clone modules get caught.
    pub fn bins_monotonic(&self) -> bool {
        self.bins().windows(2).all(|pair| pair[0] >= pair[1])
    }

    /// Version / error code field.
    pub fn version(&self) -> u16 {
        self.be16(28)
    }
}

/// PMSx003 driver.
pub struct Pmsx003<UART> {
    uart: UART,
    buffer: [u8; FRAME_LEN],
}

impl<UART> Pmsx003<UART>
where
    UART: embedded_io_async::Read,
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
    /// never sends anything blocks forever, so wrap the transport or the future
    /// in one if that matters.
    pub async fn read_frame(&mut self) -> Result<Frame, Error<UART::Error>> {
        use embedded_io_async::ReadExactError;

        loop {
            self.sync().await?;
            self.buffer[0] = HEADER[0];
            self.buffer[1] = HEADER[1];

            self.uart.read_exact(&mut self.buffer[2..]).await.map_err(|e| match e {
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
    async fn sync(&mut self) -> Result<(), Error<UART::Error>> {
        let mut sync = HeaderSync::default();
        let mut byte = [0u8; 1];

        loop {
            let read = self.uart.read(&mut byte).await.map_err(Error::Transport)?;
            if read == 0 {
                continue;
            }
            if sync.push(byte[0]) {
                return Ok(());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a frame with the correct length and checksum for the given mass
    /// concentrations.
    fn frame(pm1: u16, pm25: u16, pm10: u16, bins: [u16; 6]) -> [u8; FRAME_LEN] {
        let mut f = [0u8; FRAME_LEN];
        f[0] = 0x42;
        f[1] = 0x4D;
        f[2..4].copy_from_slice(&PAYLOAD_LEN.to_be_bytes());
        f[4..6].copy_from_slice(&pm1.to_be_bytes());
        f[6..8].copy_from_slice(&pm25.to_be_bytes());
        f[8..10].copy_from_slice(&pm10.to_be_bytes());
        f[10..12].copy_from_slice(&pm1.to_be_bytes());
        f[12..14].copy_from_slice(&pm25.to_be_bytes());
        f[14..16].copy_from_slice(&pm10.to_be_bytes());
        for (i, value) in bins.iter().enumerate() {
            let at = 16 + i * 2;
            f[at..at + 2].copy_from_slice(&value.to_be_bytes());
        }
        let sum = f[..FRAME_LEN - 2]
            .iter()
            .fold(0u16, |acc, byte| acc.wrapping_add(*byte as u16));
        f[FRAME_LEN - 2..].copy_from_slice(&sum.to_be_bytes());
        f
    }

    #[test]
    fn parses_a_well_formed_frame() {
        let raw = frame(4, 7, 9, [1234, 100, 50, 10, 5, 1]);
        let parsed = Frame::parse(&raw).expect("valid frame");
        assert_eq!(parsed.pm1_0(), 4);
        assert_eq!(parsed.pm2_5(), 7);
        assert_eq!(parsed.pm10(), 9);
        assert_eq!(parsed.pm1_0_atm(), 4);
        assert_eq!(parsed.bins(), [1234, 100, 50, 10, 5, 1]);
        assert!(parsed.bins_monotonic());
    }

    #[test]
    fn rejects_a_flipped_byte() {
        let mut raw = frame(4, 7, 9, [1234, 100, 50, 10, 5, 1]);
        raw[7] ^= 0x01;
        assert!(Frame::parse(&raw).is_none());
    }

    #[test]
    fn rejects_a_wrong_payload_length() {
        let mut raw = frame(4, 7, 9, [1234, 100, 50, 10, 5, 1]);
        raw[3] = 0x24;
        // The checksum still covers the mutated byte, so this must be caught by
        // the length check, not the checksum.
        let sum = raw[..FRAME_LEN - 2]
            .iter()
            .fold(0u16, |acc, byte| acc.wrapping_add(*byte as u16));
        raw[FRAME_LEN - 2..].copy_from_slice(&sum.to_be_bytes());
        assert!(Frame::parse(&raw).is_none());
    }

    #[test]
    fn detects_non_monotonic_bins() {
        // >2.5um larger than >1.0um is physically impossible for cumulative bins.
        let raw = frame(4, 7, 9, [18890, 1162, 107, 136, 0, 124]);
        let parsed = Frame::parse(&raw).expect("valid frame, bogus counts");
        assert!(!parsed.bins_monotonic());
    }

    #[test]
    fn header_sync_handles_noise_and_repeated_start() {
        let mut sync = HeaderSync::default();
        for byte in [0x00, 0x42] {
            assert!(!sync.push(byte));
        }
        assert!(sync.push(0x4D));

        // 0x42 0x42 0x4D: the first 0x42 must be kept as a possible start.
        let mut sync = HeaderSync::default();
        assert!(!sync.push(0x42));
        assert!(!sync.push(0x42));
        assert!(sync.push(0x4D));

        // A false start followed by the real header.
        let mut sync = HeaderSync::default();
        assert!(!sync.push(0x42));
        assert!(!sync.push(0x00));
        assert!(!sync.push(0x42));
        assert!(sync.push(0x4D));
    }
}
