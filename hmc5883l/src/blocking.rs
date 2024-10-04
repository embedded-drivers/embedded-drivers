//! Driver implementation in blocking mode

use crate::{regs, Config, Error, Gain, ADDRESS};

pub struct HMC5883L<I2C: embedded_hal::i2c::I2c> {
    i2c: I2C,
    addr: u8,
    gain: Gain,
}

impl<I2C> HMC5883L<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            gain: Gain::Gain1090,
        }
    }

    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, ADDRESS)
    }

    pub fn init(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        let id_a = self.read_reg(regs::IDENT_A)?;
        let id_b = self.read_reg(regs::IDENT_B)?;
        let id_c = self.read_reg(regs::IDENT_C)?;

        if id_a != 0x48 || id_b != 0x34 || id_c != 0x33 {
            return Err(Error::InvalidDevice);
        }

        let mut cra = 0; // Initialize CRA with all bits set to 0
        cra |= (config.samples as u8) << 5; // Set MA (CRA6 to CRA5) based on config.samples
        cra |= (config.data_rate as u8) << 2; // Set DO (CRA4 to CRA2) based on config.data_rate
        self.write_reg(regs::CONFIG_A, cra)?;

        let crb = (config.gain as u8) << 5; // Set GN (CRB7 to CRB5) based on config.gain
        self.write_reg(regs::CONFIG_B, crb)?;

        self.write_reg(regs::MODE, 0x00)?; // Continuous-measurement mode

        self.gain = config.gain;

        Ok(())
    }

    pub fn read_raw_measurement(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0u8; 6];

        // Read 6 bytes starting from DATA_X_MSB
        self.i2c.write_read(self.addr, &[regs::DATA_X_MSB], &mut buf)?;
        self.i2c.write(self.addr, &[regs::DATA_X_MSB])?;

        let x = ((buf[0] as i16) << 8) | buf[1] as i16;
        let y = ((buf[4] as i16) << 8) | buf[5] as i16;
        let z = ((buf[2] as i16) << 8) | buf[3] as i16;

        Ok((x, y, z))
    }

    pub fn read_measurement(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw_measurement()?;

        let resolution = self.gain.resolution();

        let x_norm = x as f32 * resolution;
        let y_norm = y as f32 * resolution;
        let z_norm = z as f32 * resolution;

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
