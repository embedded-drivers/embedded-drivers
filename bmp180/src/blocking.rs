use embedded_hal::delay::DelayNs;
use micromath::F32Ext;

use super::{cmds, regs, CalibrationData, Config, Error, Measurement, Mode, ADDRESS};

/// BMP180, or BMP085.
pub struct BMP180<I2C: embedded_hal::i2c::I2c> {
    i2c: I2C,
    addr: u8,
    // oversampling settings
    mode: Mode,
    calib: CalibrationData,
}

impl<I2C: embedded_hal::i2c::I2c> BMP180<I2C> {
    /// Create device driver instance.
    pub fn new(i2c: I2C, addr: u8) -> Self {
        BMP180 {
            i2c,
            addr,
            mode: Mode::UltraHighResolution,
            calib: unsafe { core::mem::zeroed() },
        }
    }

    /// Create a new instance of BMP180 with the default address.
    pub fn new_primary(i2c: I2C) -> Self {
        BMP180::new(i2c, ADDRESS)
    }

    /// Read calibration data from the EEPROM of BMP180.
    pub fn init(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        let mut buf = [0u8; 22];
        self.i2c.write_read(self.addr, &[regs::CAL_AC1], &mut buf[..])?;

        self.calib = CalibrationData::from_raw(&buf);

        self.mode = config.mode;

        Ok(())
    }

    /// read uncompensated temperature value
    pub fn read_raw_temperature<D: DelayNs>(&mut self, delay: &mut D) -> Result<i32, Error<I2C::Error>> {
        self.write_reg(regs::CONTROL, cmds::READTEMPCMD)?;
        delay.delay_ms(5);

        let mut buf = [0u8; 2];
        self.i2c.write_read(self.addr, &[regs::TEMPDATA], &mut buf)?;

        Ok(((buf[0] as i32) << 8) + (buf[1] as i32))
    }

    /// read uncompensated pressure value
    #[inline]
    pub fn read_raw_pressure<D: DelayNs>(&mut self, delay: &mut D) -> Result<i32, Error<I2C::Error>> {
        let oss = self.mode.oversampling_settings();
        self.write_reg(regs::CONTROL, cmds::READPRESSURECMD + (oss << 6))?;
        delay.delay_ms(self.mode.conversion_delay_in_ms() as u32);

        let mut buf = [0u8; 3];
        self.i2c.write_read(self.addr, &[regs::PRESSUREDATA], &mut buf)?;

        let up = ((buf[0] as i32) << 16) + ((buf[1] as i32) << 8) + (buf[2] as i32) >> (8 - oss);

        Ok(up)
    }

    /// Calculate true temperature, resolution is 0.1C
    pub fn read_temperature<D: DelayNs>(&mut self, delay: &mut D) -> Result<f32, Error<I2C::Error>> {
        let ut = self.read_raw_temperature(delay)?;

        Ok(super::convert_temperature(ut, &self.calib))
    }

    /// Read temperature and pressure at once
    pub fn read_measurement<D: DelayNs>(&mut self, delay: &mut D) -> Result<Measurement, Error<I2C::Error>> {
        let ut = self.read_raw_temperature(delay)?;
        let up = self.read_raw_pressure(delay)?;

        let measure = super::convert_measurement(up, ut, &self.calib, self.mode);

        Ok(measure)
    }

    /// Calculate true pressure, in Pa
    pub fn read_pressure<D: DelayNs>(&mut self, delay: &mut D) -> Result<i32, Error<I2C::Error>> {
        self.read_measurement(delay).map(|m| m.pressure)
    }

    /// Calculate absolute altitude
    pub fn calculate_altitude<D: DelayNs>(
        &mut self,
        delay: &mut D,
        sealevel_pressure: f32,
    ) -> Result<f32, Error<I2C::Error>> {
        let pa = self.read_pressure(delay)? as f32;
        Ok(44330.0 * (1.0 - (pa / sealevel_pressure).powf(1.0 / 5.255)))
    }

    /// Calculate pressure at sea level
    pub fn calculate_sealevel_pressure<D: DelayNs>(
        &mut self,
        delay: &mut D,
        altitude_m: f32,
    ) -> Result<u32, Error<I2C::Error>> {
        let pressure = self.read_pressure(delay)? as f32;
        let p0 = pressure / (1.0 - altitude_m / 44330.0).powf(5.255);
        Ok(p0 as u32)
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, &[reg, value])?;
        Ok(())
    }
}
