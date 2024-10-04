//! Blocking API

use embedded_hal::delay::DelayNs;

use crate::{regs, CalibrationData, Error, Measurements, ADDRESS, CHIP_ID_BME280, CHIP_ID_BMP280};

pub struct BME280<I2C: embedded_hal::i2c::I2c> {
    addr: u8,
    i2c: I2C,
    pub is_bme280: bool,
    pub calib: CalibrationData,
}

impl<I2C: embedded_hal::i2c::I2c> BME280<I2C> {
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            addr,
            i2c,
            is_bme280: true,
            calib: CalibrationData::new(),
        }
    }
    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, ADDRESS)
    }

    pub fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        let chip_id = self.read_reg(regs::CHIP_ID)?;

        if chip_id == CHIP_ID_BME280 || chip_id == CHIP_ID_BMP280 {
            // BME280 or BMP280
            self.is_bme280 = chip_id == CHIP_ID_BME280;
        } else {
            return Err(Error::InvalidDevice);
        }

        // set normal mode, temp and pressure oversampling x1
        self.write_reg(regs::CTRL_MEAS, 0b001_001_11)?;
        self.write_reg(regs::CTRL_HUM, 0b001)?;

        let mut raw = [0u8; 38];
        self.read_regs(regs::CALIB_00, &mut raw)?;

        self.calib = CalibrationData::from_raw(&raw);

        Ok(())
    }

    /// soft reset
    pub fn reset(&mut self, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        self.write_reg(regs::RESET, 0xB6)?;

        delay.delay_ms(10);
        Ok(())
    }

    pub fn read_raw_temperature(&mut self) -> Result<i32, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        self.read_regs(regs::TEMP_MSB, &mut buf)?;

        let temp_msb = buf[0] as i32;
        let temp_lsb = buf[1] as i32;
        let temp_xlsb = buf[2] as i32;

        Ok((temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4))
    }

    pub fn read_raw_pressure(&mut self) -> Result<i32, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        self.read_regs(regs::PRESS_MSB, &mut buf)?;

        let press_msb = buf[0] as i32;
        let press_lsb = buf[1] as i32;
        let press_xlsb = buf[2] as i32;

        Ok((press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4))
    }

    pub fn read_raw_humidity(&mut self) -> Result<i32, Error<I2C::Error>> {
        if !self.is_bme280 {
            return Err(Error::UnsupportedMeasurement);
        }

        let mut buf = [0u8; 2];
        self.read_regs(regs::HUM_MSB, &mut buf)?;

        let hum_msb = buf[0] as i32;
        let hum_lsb = buf[1] as i32;

        Ok((hum_msb << 8) | hum_lsb)
    }

    pub fn read_measurement(&mut self) -> Result<Measurements, Error<I2C::Error>> {
        // Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
        // t_fine carries fine temperature as global value
        let adc_t = self.read_raw_temperature()?;
        let (t_fine, t) = crate::convert_temperature(adc_t, &self.calib);

        // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
        // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
        let adc_p = self.read_raw_pressure()?;
        let p = crate::convert_pressure(adc_p, t_fine, &self.calib);

        // convert Q24.8 to centi
        let p = p * 100 / 256;

        // BME280 only
        let mut h = 0;
        if self.is_bme280 {
            let adc_h = self.read_raw_humidity()?;

            let h0 = super::convert_humidity(adc_h, t_fine, &self.calib);

            // convert Q22.10 to centi
            h = h0 * 100 / 1024;
        };

        Ok(Measurements {
            temperature: t,
            pressure: p as u32,
            humidity: h,
        })
    }

    pub fn read_reg(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    pub fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, &[reg, val])?;
        Ok(())
    }

    pub fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write_read(self.addr, &[reg], buf)?;
        Ok(())
    }
}
