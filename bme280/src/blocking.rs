//! Blocking API

use embedded_hal::delay::DelayNs;

use crate::{regs, CalibrationData, Error, Measurements, CHIP_ID_BME280, CHIP_ID_BMP280};

pub struct BME280<I2C: embedded_hal::i2c::I2c> {
    addr: u8,
    i2c: I2C,
    pub is_bme280: bool,
    pub calib: CalibrationData,
}

impl<I2C: embedded_hal::i2c::I2c> BME280<I2C> {
    pub fn new_primary(i2c: I2C) -> Self {
        Self {
            addr: 0x76,
            i2c,
            is_bme280: true,
            calib: CalibrationData::new(),
        }
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

    pub fn measure_raw_temperature(&mut self) -> Result<i32, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        self.read_regs(regs::TEMP_MSB, &mut buf)?;

        let temp_msb = buf[0] as i32;
        let temp_lsb = buf[1] as i32;
        let temp_xlsb = buf[2] as i32;

        Ok((temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4))
    }

    pub fn measure_raw_pressure(&mut self) -> Result<i32, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        self.read_regs(regs::PRESS_MSB, &mut buf)?;

        let press_msb = buf[0] as i32;
        let press_lsb = buf[1] as i32;
        let press_xlsb = buf[2] as i32;

        Ok((press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4))
    }

    pub fn measure_raw_humidity(&mut self) -> Result<i32, Error<I2C::Error>> {
        if !self.is_bme280 {
            return Err(Error::UnsupportedMeasurement);
        }

        let mut buf = [0u8; 2];
        self.read_regs(regs::HUM_MSB, &mut buf)?;

        let hum_msb = buf[0] as i32;
        let hum_lsb = buf[1] as i32;

        Ok((hum_msb << 8) | hum_lsb)
    }

    pub fn mesaure(&mut self, calib_data: &CalibrationData) -> Result<Measurements, Error<I2C::Error>> {
        // Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
        // t_fine carries fine temperature as global value
        let adc_t = self.measure_raw_temperature()?;

        let var1 = (((adc_t >> 3) - ((calib_data.dig_t1 as i32) << 1)) * (calib_data.dig_t2 as i32)) >> 11;
        let var2 = (((((adc_t >> 4) - (calib_data.dig_t1 as i32)) * ((adc_t >> 4) - (calib_data.dig_t1 as i32)))
            >> 12)
            * (calib_data.dig_t3 as i32))
            >> 14;

        let t_fine = var1 + var2;

        let t = (t_fine * 5 + 128) >> 8;

        // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
        // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
        let adc_p = self.measure_raw_pressure()?;

        let mut var1 = t_fine as i64 - 128000;
        let mut var2 = var1 * var1 as i64 * calib_data.dig_p6 as i64;
        var2 = var2 + ((var1 * calib_data.dig_p5 as i64) << 17);
        var2 = var2 + ((calib_data.dig_p4 as i64) << 35);
        var1 = ((var1 * var1 as i64 * calib_data.dig_p3 as i64) >> 8) + ((var1 * calib_data.dig_p2 as i64) << 12);
        var1 = (((1i64 << 47) + var1) * (calib_data.dig_p1 as i64)) >> 33;

        let p = if var1 == 0 {
            0
        } else {
            let mut p = 1048576 - adc_p as i64;
            p = (((p << 31) - var2) * 3125) / var1;
            var1 = (calib_data.dig_p9 as i64 * (p >> 13) * (p >> 13)) >> 25;
            var2 = (calib_data.dig_p8 as i64 * p) >> 19;

            p = ((p + var1 + var2) >> 8) + ((calib_data.dig_p7 as i64) << 4);
            p
        };

        // convert Q24.8 to centi
        let p = p * 100 / 256;

        // BME280 only
        let h = if self.is_bme280 {
            let adc_h = self.measure_raw_humidity()?;
            let h = {
                let dig_h1 = calib_data.dig_h1 as i32;
                let dig_h2 = calib_data.dig_h2 as i32;
                let dig_h3 = calib_data.dig_h3 as i32;
                let dig_h4 = calib_data.dig_h4 as i32;
                let dig_h5 = calib_data.dig_h5 as i32;
                let dig_h6 = calib_data.dig_h6 as i32;

                let v_x1_u32r = (t_fine - 76800) as i32;
                let v_x1_u32r = (((((adc_h << 14) - (dig_h4 << 20) - (dig_h5 * v_x1_u32r)) + 16384) >> 15)
                    * (((((((v_x1_u32r * (dig_h6)) >> 10) * (((v_x1_u32r * dig_h3) >> 11) + 32768)) >> 10)
                        + 2097152)
                        * dig_h2
                        + 8192)
                        >> 14));
                let v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * dig_h1) >> 4));

                // limit check

                let v_x1_u32r = i32::min(v_x1_u32r, 419430400);
                let v_x1_u32r = i32::max(v_x1_u32r, 0);

                (v_x1_u32r >> 12) as u32
            };

            // convert Q22.10 to centi
            h * 100 / 1024
        } else {
            0
        };

        Ok(Measurements {
            temperature_centidegree: t,
            pressure_centipascal: p as u32,
            humidity_centipercent: h,
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
