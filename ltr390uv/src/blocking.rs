use embedded_hal::delay::DelayNs;

use crate::{ADDRESS, Config, Error, regs};

pub struct LTR390UV<I2C: embedded_hal::i2c::I2c> {
    i2c: I2C,
    addr: u8,
    config: Config,
}

impl<I2C: embedded_hal::i2c::I2c> LTR390UV<I2C> {
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            config: Config::default(),
        }
    }

    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, ADDRESS)
    }

    pub fn init(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        if self.read_reg(regs::PART_ID)? & 0xF0 != 0b1011_0000 {
            return Err(Error::InvalidDevice);
        }

        self.write_reg(regs::MAIN_CTRL, 0x00)?; // standby mode

        self.write_reg(
            regs::ALS_UVS_MEAS_RATE,
            ((config.resolution as u8) << 4) | config.rate as u8,
        )?;
        self.write_reg(regs::ALS_UVS_GAIN, config.gain as u8)?;

        Ok(())
    }

    pub fn soft_reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_reg(regs::MAIN_CTRL, 0b10000)?; // software reset

        Ok(())
    }

    pub fn read_als_data(&mut self, mut delay: impl DelayNs) -> Result<u32, Error<I2C::Error>> {
        self.write_reg(regs::MAIN_CTRL, 0b0010)?; // active sensor

        delay.delay_ms(10 + self.config.resolution.conversion_time_ms() as u32 + 1);

        //if self.read_reg(regs::MAIN_CTRL)? & 0b10 == ! {
        //   return Err(Error::InvalidDevice);
        //}
        if self.read_reg(regs::MAIN_STATUS)? & 0b1000 == 0 {
            return Err(Error::OldData);
        }

        let mut buf = [0u8; 3];
        self.read_regs(regs::ALS_DATA_0, &mut buf)?;

        Ok(u32::from(buf[0]) | (u32::from(buf[1]) << 8) | (u32::from(buf[2]) << 16))
    }

    pub fn read_uvs_data(&mut self, mut delay: impl DelayNs) -> Result<u32, Error<I2C::Error>> {
        self.write_reg(regs::MAIN_CTRL, 0b1010)?; // active sensor

        delay.delay_ms(10 + self.config.resolution.conversion_time_ms() as u32 + 1);

        //if self.read_reg(regs::MAIN_CTRL)? & 0b10 == ! {
        //   return Err(Error::InvalidDevice);
        //}
        if self.read_reg(regs::MAIN_STATUS)? & 0b1000 == 0 {
            return Err(Error::OldData);
        }

        let mut buf = [0u8; 3];
        self.read_regs(regs::UVS_DATA_0, &mut buf)?;

        Ok(u32::from(buf[0]) | (u32::from(buf[1]) << 8) | (u32::from(buf[2]) << 16))
    }

    pub fn read_lux(&mut self, mut delay: impl DelayNs) -> Result<u32, Error<I2C::Error>> {
        let als_data = self.read_als_data(&mut delay)? as f32;

        let gain = self.config.gain.value() as f32;
        let int = (self.config.rate.rate_ms() as f32) / 100.0;

        const W_FAC: f32 = 1.0;

        let lux = 0.6 * als_data / (gain * int) * W_FAC;

        Ok(lux as u32)
    }

    /// Read UV index
    pub fn read_uvi(&mut self, mut delay: impl DelayNs) -> Result<u32, Error<I2C::Error>> {
        let uvs_data = self.read_uvs_data(&mut delay)? as f32;

        const UV_SENSITIVITY: f32 = 2300.0;

        let uvi = uvs_data / UV_SENSITIVITY;

        Ok(uvi as u32)
    }

    pub fn read_reg(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c.write_read(self.addr, &[reg], buf)
    }

    // Add this new method to write to registers
    pub fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[reg, value])
    }
}
