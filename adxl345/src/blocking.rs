use super::{Config, Error, PRIMARY_ADDRESS, Range, SECONDARY_ADDRESS, regs};

/// A struct representing the ADXL345 accelerometer.
//
// This struct encapsulates the I2C interface, the device address, and the least significant bit (LSB) scale factor.
// It provides methods to initialize the accelerometer, read raw acceleration data, and read acceleration values in g-force.
pub struct ADXL345<I2C> {
    i2c: I2C,
    addr: u8,
    lsb_scale: f32,
}

impl<I2C> ADXL345<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
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

    pub fn init(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        let id = self.read_reg(regs::DEVID)?;
        if id != 0xE5 {
            return Err(Error::InvalidDevice);
        }

        self.write_reg(regs::POWER_CTL, 0)?; // Wake up
        self.write_reg(regs::POWER_CTL, 16)?; // Auto-sleep
        self.write_reg(regs::POWER_CTL, 8)?; // Measure

        // Set data rate and range
        let mut data_format = (config.range as u8) & 0x03;
        data_format |= 0b100; // Set bit 2 to enable left justified mode
        self.write_reg(regs::DATA_FORMAT, data_format)?;

        let bw_rate = (config.rate as u8) & 0x0F; // Set rate
        self.write_reg(regs::BW_RATE, bw_rate)?;

        // Set scale factor based on the range
        self.lsb_scale = match config.range {
            Range::G2 => 4.0 / 65536.0,
            Range::G4 => 8.0 / 65536.0,
            Range::G8 => 16.0 / 65536.0,
            Range::G16 => 32.0 / 65536.0,
        };

        Ok(())
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
    pub fn read_raw(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0; 6];

        self.i2c.write_read(self.addr, &[regs::DATAX0], &mut buf)?;

        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }

    /// Reads the acceleration values from the sensor and converts them to g-force.
    /// The scaling factor is set during initialization based on the configured range.
    ///
    /// Returns a tuple of (x, y, z) acceleration values in g-force.
    pub fn read_accel(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x_raw, y_raw, z_raw) = self.read_raw()?;

        // Convert raw values to g-force
        // The scaling factor is set during initialization based on the configured range
        let x = x_raw as f32 * self.lsb_scale;
        let y = y_raw as f32 * self.lsb_scale;
        let z = z_raw as f32 * self.lsb_scale;

        Ok((x, y, z))
    }

    pub fn read_reg(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    // Add this new method to write to registers
    pub fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[reg, value])
    }
}
