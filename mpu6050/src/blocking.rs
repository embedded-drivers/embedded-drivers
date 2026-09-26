use crate::{AccelRange, Config, Error, GyroRange, PRIMARY_ADDRESS, consts, regs};

pub struct MPU6050<I2C: embedded_hal::i2c::I2c> {
    addr: u8,
    i2c: I2C,
    gyro_range: GyroRange,
    accel_range: AccelRange,
}

impl<I2C: embedded_hal::i2c::I2c> MPU6050<I2C> {
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            addr,
            i2c,
            gyro_range: GyroRange::Deg1000,
            accel_range: AccelRange::G2,
        }
    }

    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, PRIMARY_ADDRESS)
    }

    pub fn init(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        let who_am_i = self.read_reg(regs::WHO_AM_I)?;
        if who_am_i != consts::DEV_ID_MPU6050
            && who_am_i != consts::DEV_ID_MPU6500
            && who_am_i != consts::DEV_ID_MPU9250
            && who_am_i != consts::DEV_ID_MPU9255
        {
            return Err(Error::InvalidDevice);
        }

        // exit sleep mode
        self.write_reg(regs::PWR_MGMT_1, 0x00)?;

        // LPF
        self.write_reg(regs::CONFIG, config.lpf as u8)?;

        // gyro ADC scale
        self.write_reg(regs::GYRO_CONFIG, config.gyro_range.config_bits())?;

        // accel ADC scale
        self.write_reg(regs::ACCEL_CONFIG, config.accel_range.config_bits())?;

        self.gyro_range = config.gyro_range;
        self.accel_range = config.accel_range;

        Ok(())
    }

    pub fn read_raw_accel(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.read_regs(regs::ACCEL_XOUT_H, &mut buf)?;

        let x = i16::from_be_bytes([buf[0], buf[1]]);
        let y = i16::from_be_bytes([buf[2], buf[3]]);
        let z = i16::from_be_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }

    pub fn read_raw_gyro(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.read_regs(regs::GYRO_XOUT_H, &mut buf)?;

        let x = i16::from_be_bytes([buf[0], buf[1]]);
        let y = i16::from_be_bytes([buf[2], buf[3]]);
        let z = i16::from_be_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }

    /// Read accelerometer data in g
    pub fn read_accel(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw_accel()?;
        let lsb_sensitivity = self.accel_range.lsb_sensitivity();
        Ok((
            x as f32 / lsb_sensitivity,
            y as f32 / lsb_sensitivity,
            z as f32 / lsb_sensitivity,
        ))
    }

    /// Read gyroscope data in degrees per second
    pub fn read_gyro(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw_gyro()?;
        let lsb_sensitivity = self.gyro_range.lsb_sensitivity();
        Ok((
            x as f32 / lsb_sensitivity,
            y as f32 / lsb_sensitivity,
            z as f32 / lsb_sensitivity,
        ))
    }

    /// Read temperature in degrees Celsius
    pub fn read_temperature(&mut self) -> Result<f32, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.read_regs(regs::TEMP_OUT_H, &mut buf)?;

        let temp = i16::from_be_bytes([buf[0], buf[1]]);
        Ok((temp as f32) / 340.0 + 36.53)
    }

    pub fn read_reg(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    pub fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write_read(self.addr, &[reg], buf)?;
        Ok(())
    }

    pub fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, &[reg, value])?;
        Ok(())
    }
}
