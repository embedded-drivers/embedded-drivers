//! Driver for ADXL345.

#![cfg_attr(not(test), no_std)]

use embedded_hal_async::delay::DelayNs;

pub mod blocking;

/// Register map
pub mod regs {
    pub const DEVID: u8 = 0x00;
    pub const THRESH_TAP: u8 = 0x1D;
    pub const OFSX: u8 = 0x1E;
    pub const OFSY: u8 = 0x1F;
    pub const OFSZ: u8 = 0x20;
    pub const DUR: u8 = 0x21;
    pub const LATENT: u8 = 0x22;
    pub const WINDOW: u8 = 0x23;
    pub const THRESH_ACT: u8 = 0x24;
    pub const THRESH_INACT: u8 = 0x25;
    pub const TIME_INACT: u8 = 0x26;
    pub const ACT_INACT_CTL: u8 = 0x27;
    pub const THRESH_FF: u8 = 0x28;
    pub const TIME_FF: u8 = 0x29;
    pub const TAP_AXES: u8 = 0x2A;
    pub const ACT_TAP_STATUS: u8 = 0x2B;
    pub const BW_RATE: u8 = 0x2C;
    pub const POWER_CTL: u8 = 0x2D;
    pub const INT_ENABLE: u8 = 0x2E;
    pub const INT_MAP: u8 = 0x2F;
    pub const INT_SOURCE: u8 = 0x30;
    pub const DATA_FORMAT: u8 = 0x31;
    pub const DATAX0: u8 = 0x32;
    pub const DATAX1: u8 = 0x33;
    pub const DATAY0: u8 = 0x34;
    pub const DATAY1: u8 = 0x35;
    pub const DATAZ0: u8 = 0x36;
    pub const DATAZ1: u8 = 0x37;
    pub const FIFO_CTL: u8 = 0x38;
    pub const FIFO_STATUS: u8 = 0x39;

    /// `INT_SOURCE` bit 7: a new sample is available.
    ///
    /// Datasheet Rev. G, Register 0x30: "The DATA_READY bit is set when new data
    /// is available and is cleared when no new data is available." The register
    /// description adds that it is "always set if the corresponding events
    /// occur, regardless of the INT_ENABLE register settings", so it can be
    /// polled without routing the interrupt to a pin.
    pub const INT_SOURCE_DATA_READY: u8 = 0x80;
}

/// `POWER_CTL` with the Measure bit (D3) set: measurement mode.
pub const POWER_CTL_MEASURE: u8 = 0x08;

/// `POWER_CTL` with every bit clear: standby mode.
pub const POWER_CTL_STANDBY: u8 = 0x00;

/// Extra polling budget added to the datasheet turn-on time before `init`
/// gives up waiting for the first sample.
pub(crate) const DATA_READY_POLL_MARGIN_MS: u32 = 20;

pub const PRIMARY_ADDRESS: u8 = 0x53;
pub const SECONDARY_ADDRESS: u8 = 0x1D;

#[derive(Debug)]
pub enum Error<IE> {
    Bus(IE),
    InvalidDevice,
    /// The device did not report a new sample within the driver's polling
    /// limit. `init` waits for the first sample, and returns this rather than
    /// handing a caller data that was never measured.
    Timeout,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Bus(e)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Range {
    G2 = 0b00,
    G4 = 0b01,
    G8 = 0b10,
    G16 = 0b11,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Rate {
    Hz3200 = 0b1111,
    Hz1600 = 0b1110,
    Hz800 = 0b1101,
    Hz400 = 0b1100,
    Hz200 = 0b1011,
    Hz100 = 0b1010,
    Hz50 = 0b1001,
    Hz25 = 0b1000,
    Hz12_5 = 0b0111,
    Hz6_25 = 0b0110,
    Hz3_13 = 0b0101,
    Hz1_56 = 0b0100,
    Hz0_78 = 0b0011,
    Hz0_39 = 0b0010,
    Hz0_20 = 0b0001,
    Hz0_10 = 0b0000,
}

impl Rate {
    /// Nominal sample period in milliseconds, rounded up.
    pub const fn period_ms(self) -> u32 {
        match self {
            Rate::Hz3200 => 1, // 0.3125 ms
            Rate::Hz1600 => 1, // 0.625 ms
            Rate::Hz800 => 2,  // 1.25 ms
            Rate::Hz400 => 3,  // 2.5 ms
            Rate::Hz200 => 5,  // 5 ms
            Rate::Hz100 => 10, // 10 ms
            Rate::Hz50 => 20,
            Rate::Hz25 => 40,
            Rate::Hz12_5 => 80,
            Rate::Hz6_25 => 160,
            Rate::Hz3_13 => 320,
            Rate::Hz1_56 => 641,
            Rate::Hz0_78 => 1282,
            Rate::Hz0_39 => 2564,
            Rate::Hz0_20 => 5000,
            Rate::Hz0_10 => 10000,
        }
    }

    /// Datasheet turn-on / wake-up time in milliseconds, rounded up.
    ///
    /// ADXL345 datasheet Rev. G, Table 1, note 7: "Turn-on and wake-up times are
    /// determined by the user-defined bandwidth. At a 100 Hz data rate, the
    /// turn-on and wake-up times are each approximately 11.1 ms. For other data
    /// rates, the turn-on and wake-up times are each approximately
    /// `1/(data rate) + 1.1` in milliseconds". The first sample after the
    /// Measure bit is set is not available before this, so the data registers
    /// still hold their reset value until then.
    pub const fn turn_on_time_ms(self) -> u32 {
        self.period_ms() + 2
    }
}

/// Configuration settings for the ADXL345 accelerometer.
///
/// This struct holds the range and data rate settings for the accelerometer.
/// The `range` field determines the measurement range of the accelerometer,
/// and the `rate` field determines the data output rate.

#[derive(Clone, Copy)]
pub struct Config {
    pub range: Range,
    pub rate: Rate,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            range: Range::G2,
            rate: Rate::Hz100,
        }
    }
}
/// A struct representing the ADXL345 accelerometer.
//
// This struct encapsulates the I2C interface, the device address, and the least significant bit (LSB) scale factor.
// It provides methods to initialize the accelerometer, read raw acceleration data, and read acceleration values in g-force.
pub struct ADXL345<I2C: embedded_hal_async::i2c::I2c> {
    i2c: I2C,
    addr: u8,
    lsb_scale: f32,
}

impl<I2C: embedded_hal_async::i2c::I2c> ADXL345<I2C> {
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            lsb_scale: 0.0,
        }
    }

    /// Creates a new instance of the ADXL345 accelerometer with the primary address.
    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, PRIMARY_ADDRESS)
    }

    pub fn new_secondary(i2c: I2C) -> Self {
        Self::new(i2c, SECONDARY_ADDRESS)
    }

    /// Configure the part and wait until it has produced its first sample.
    ///
    /// The data registers hold their reset value (zero) from power-up until the
    /// first conversion completes, which takes the datasheet's turn-on time
    /// after the Measure bit is set (`1/data rate + 1.1` ms; about 11.1 ms at
    /// the default 100 Hz). Returning from `init` before then makes a caller's
    /// first read return `(0, 0, 0)` with no error.
    ///
    /// The configuration registers are therefore written while the part is
    /// still in standby, and `init` only returns once `INT_SOURCE` reports
    /// `DATA_READY` (or the turn-on budget expires with [`Error::Timeout`]).
    /// Afterwards the part free-runs at the configured rate, so later reads
    /// return the most recent sample without an additional wait.
    pub async fn init(&mut self, config: Config, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        let id = self.read_reg(regs::DEVID).await?;
        if id != 0xE5 {
            return Err(Error::InvalidDevice);
        }

        // The datasheet recommends clearing SLEEP/AUTO_SLEEP by passing through
        // standby before re-entering measurement mode, and configuring the part
        // while it is in standby.
        self.write_reg(regs::POWER_CTL, POWER_CTL_STANDBY).await?; // Standby
        self.write_reg(regs::POWER_CTL, 16).await?; // AUTO_SLEEP, cleared by the transition below

        // Set data rate and range
        let mut data_format = (config.range as u8) & 0x03;
        data_format |= 0b100; // Set bit 2 to enable left justified mode
        self.write_reg(regs::DATA_FORMAT, data_format).await?;

        let bw_rate = (config.rate as u8) & 0x0F; // Set rate
        self.write_reg(regs::BW_RATE, bw_rate).await?;

        // Enter measurement mode last: the turn-on time is measured from this
        // write, so the output data rate must already be programmed.
        self.write_reg(regs::POWER_CTL, POWER_CTL_MEASURE).await?;

        // Set scale factor based on the range
        self.lsb_scale = match config.range {
            Range::G2 => 4.0 / 65536.0,
            Range::G4 => 8.0 / 65536.0,
            Range::G8 => 16.0 / 65536.0,
            Range::G16 => 32.0 / 65536.0,
        };

        self.wait_for_data_ready(config.rate, &mut delay).await?;

        Ok(())
    }

    /// Wait until `INT_SOURCE.DATA_READY` is set, or give up after the
    /// datasheet's turn-on time for the configured rate plus a small margin.
    async fn wait_for_data_ready(&mut self, rate: Rate, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        let budget_ms = rate.turn_on_time_ms() + DATA_READY_POLL_MARGIN_MS;

        for _ in 0..=budget_ms {
            if self.read_reg(regs::INT_SOURCE).await? & regs::INT_SOURCE_DATA_READY != 0 {
                return Ok(());
            }
            delay.delay_ms(1).await;
        }

        Err(Error::Timeout)
    }

    /// Reads the raw acceleration data from the sensor.
    ///
    /// This method reads the raw 16-bit acceleration values for the X, Y, and Z axes
    /// from the sensor's data registers. The values are returned as a tuple of three
    /// 16-bit integers representing the acceleration in each axis.
    ///
    /// # Returns
    ///
    /// A `Result` containing a tuple of three 16-bit integers `(x, y, z)` representing
    /// the raw acceleration values for the X, Y, and Z axes, or an `Error` if the read
    /// operation fails.
    ///
    /// # Errors
    ///
    /// Returns an `Error` if the I2C read operation fails.
    pub async fn read_raw(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0; 6];

        self.i2c.write_read(self.addr, &[regs::DATAX0], &mut buf).await?;

        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }

    /// Reads the acceleration values from the sensor and converts them to g-force.
    /// The scaling factor is set during initialization based on the configured range.
    ///
    /// Returns a tuple of (x, y, z) acceleration values in g-force.
    pub async fn read_accel(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x_raw, y_raw, z_raw) = self.read_raw().await?;

        // Convert raw values to g-force
        // The scaling factor is set during initialization based on the configured range
        let x = x_raw as f32 * self.lsb_scale;
        let y = y_raw as f32 * self.lsb_scale;
        let z = z_raw as f32 * self.lsb_scale;

        Ok((x, y, z))
    }

    pub async fn read_reg(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0];
        self.i2c.write_read(self.addr, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    // Add this new method to write to registers
    pub async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, &[reg, value]).await?;
        Ok(())
    }
}
