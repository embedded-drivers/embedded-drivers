use super::{ADDRESS, Config, Error, Scale, regs};

pub struct L3G4200D<I2C: embedded_hal::i2c::I2c> {
    i2c: I2C,
    addr: u8,
    dps_per_digit: f32,
}

impl<I2C: embedded_hal::i2c::I2c> L3G4200D<I2C> {
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            dps_per_digit: 0.00875,
        }
    }

    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, ADDRESS)
    }

    pub fn init(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        if self.read_reg(regs::WHO_AM_I)? != 0xD3 {
            return Err(Error::InvalidDevice);
        }

        // Enable all axis and setup normal mode + Output Data Range & Bandwidth
        let mut reg1 = 0x0F; // Enable all axis and setup normal mode
        reg1 |= (config.data_rate as u8) << 4; // Set output data rate & bandwidth
        self.write_reg(regs::CTRL_REG1, reg1)?;

        // Disable high pass filter
        self.write_reg(regs::CTRL_REG2, 0x00)?;

        // Generate data ready interrupt on INT2
        self.write_reg(regs::CTRL_REG3, 0x08)?;

        // Set full scale selection in continuous mode
        self.write_reg(regs::CTRL_REG4, (config.scale as u8) << 4)?;

        // Set dpsPerDigit based on scale
        self.dps_per_digit = match config.scale {
            Scale::Dps250 => 0.00875,
            Scale::Dps500 => 0.0175,
            Scale::Dps2000 => 0.07,
        };

        // Boot in normal mode, disable FIFO, HPF disabled
        self.write_reg(regs::CTRL_REG5, 0x00)?;

        Ok(())
    }

    pub fn read_raw(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0u8; 6];

        // Read 6 bytes starting from OUT_X_L register (0x28 | 0x80 for auto-increment)
        self.i2c.write_read(self.addr, &[regs::OUT_X_L | 0x80], &mut buf)?;

        // Combine high and low bytes into 16-bit integers
        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }

    /// Reads the normalized angular rate data from the sensor.
    pub fn read_gyro(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw()?;

        // Apply normalization using dps_per_digit
        let x_norm = (x as f32 * self.dps_per_digit) as i16;
        let y_norm = (y as f32 * self.dps_per_digit) as i16;
        let z_norm = (z as f32 * self.dps_per_digit) as i16;

        Ok((x_norm, y_norm, z_norm))
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
