//! Driver for BME280 and BMP280.
//!
//! The only difference between the two sensors is that the BME280 also includes a humidity sensor.

#![no_std]

use embedded_hal_async::delay::DelayNs;

pub const ADDRESS: u8 = 0x76;

pub const CHIP_ID_BME280: u8 = 0x60;
pub const CHIP_ID_BMP280: u8 = 0x58;

pub mod blocking;

pub mod regs {
    pub const TEMP_MSB: u8 = 0xFA;
    pub const TEMP_LSB: u8 = 0xFB;
    pub const TEMP_XLSB: u8 = 0xFC;

    pub const PRESS_MSB: u8 = 0xF7;
    pub const PRESS_LSB: u8 = 0xF8;
    pub const PRESS_XLSB: u8 = 0xF9;

    pub const HUM_MSB: u8 = 0xFD;
    pub const HUM_LSB: u8 = 0xFE;
    pub const HUM_XLSB: u8 = 0xFD;

    pub const CHIP_ID: u8 = 0xD0;

    pub const CALIB_00: u8 = 0x88;

    pub const CONFIG: u8 = 0xF5;
    pub const CTRL_MEAS: u8 = 0xF4;
    pub const CTRL_HUM: u8 = 0xF2;

    pub const STATUS: u8 = 0xF3;
    pub const RESET: u8 = 0xE0;
}

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    CalibrationDataError,
    ConversionError,
    InvalidDevice,
    UnsupportedMeasurement,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::Bus(error)
    }
}

#[derive(Debug)]
pub struct CalibrationData {
    pub dig_t1: u16,
    pub dig_t2: i16,
    pub dig_t3: i16,
    pub dig_p1: u16,
    pub dig_p2: i16,
    pub dig_p3: i16,
    pub dig_p4: i16,
    pub dig_p5: i16,
    pub dig_p6: i16,
    pub dig_p7: i16,
    pub dig_p8: i16,
    pub dig_p9: i16,
    // BME280 only
    pub dig_h1: u8,
    pub dig_h2: i16,
    pub dig_h3: u8,
    pub dig_h4: i16,
    pub dig_h5: i16,
    pub dig_h6: i8,
}

impl CalibrationData {
    pub const fn new() -> Self {
        unsafe { core::mem::zeroed() }
    }

    fn from_raw(raw: &[u8]) -> Self {
        CalibrationData {
            dig_t1: u16::from_le_bytes([raw[0], raw[1]]),
            dig_t2: i16::from_le_bytes([raw[2], raw[3]]),
            dig_t3: i16::from_le_bytes([raw[4], raw[5]]),
            dig_p1: u16::from_le_bytes([raw[6], raw[7]]),
            dig_p2: i16::from_le_bytes([raw[8], raw[9]]),
            dig_p3: i16::from_le_bytes([raw[10], raw[11]]),
            dig_p4: i16::from_le_bytes([raw[12], raw[13]]),
            dig_p5: i16::from_le_bytes([raw[14], raw[15]]),
            dig_p6: i16::from_le_bytes([raw[16], raw[17]]),
            dig_p7: i16::from_le_bytes([raw[18], raw[19]]),
            dig_p8: i16::from_le_bytes([raw[20], raw[21]]),
            dig_p9: i16::from_le_bytes([raw[22], raw[23]]),
            // BME280 only
            dig_h1: raw[25],
            dig_h2: i16::from_le_bytes([raw[26], raw[27]]),
            dig_h3: raw[28],
            dig_h4: i16::from_le_bytes([raw[29], raw[30] & 0x0F]),
            dig_h5: i16::from_le_bytes([raw[30] >> 4, raw[31]]),
            dig_h6: raw[32] as i8,
        }
    }
}

#[derive(Debug)]
pub struct Measurements {
    /// in 1/100 DegC
    pub temperature: i32,
    /// in 1/100 pascal
    pub pressure: u32,
    /// in 1/100 %RH
    pub humidity: u32,
}

// - MARK: Async driver

/// Async BME280 driver, compatible with BMP280
pub struct BME280<I2C: embedded_hal_async::i2c::I2c> {
    addr: u8,
    i2c: I2C,
    pub is_bme280: bool,
    pub calib: CalibrationData,
}

impl<I2C: embedded_hal_async::i2c::I2c> BME280<I2C> {
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

    pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        let chip_id = self.read_reg(regs::CHIP_ID).await?;

        if chip_id == CHIP_ID_BME280 || chip_id == CHIP_ID_BMP280 {
            // BME280 or BMP280
            self.is_bme280 = chip_id == CHIP_ID_BME280;
        } else {
            return Err(Error::InvalidDevice);
        }

        // set normal mode, temp and pressure oversampling x1
        self.write_reg(regs::CTRL_MEAS, 0b001_001_11).await?;
        self.write_reg(regs::CTRL_HUM, 0b001).await?;

        let mut raw = [0u8; 38];
        self.read_regs(regs::CALIB_00, &mut raw).await?;

        self.calib = CalibrationData::from_raw(&raw);

        Ok(())
    }

    /// soft reset
    pub async fn reset(&mut self, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        self.write_reg(regs::RESET, 0xB6).await?;

        delay.delay_ms(10).await;
        Ok(())
    }

    pub async fn read_raw_temperature(&mut self) -> Result<i32, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        self.read_regs(regs::TEMP_MSB, &mut buf).await?;

        let temp_msb = buf[0] as i32;
        let temp_lsb = buf[1] as i32;
        let temp_xlsb = buf[2] as i32;

        Ok((temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4))
    }

    pub async fn read_raw_pressure(&mut self) -> Result<i32, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        self.read_regs(regs::PRESS_MSB, &mut buf).await?;

        let press_msb = buf[0] as i32;
        let press_lsb = buf[1] as i32;
        let press_xlsb = buf[2] as i32;

        Ok((press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4))
    }

    pub async fn read_raw_humidity(&mut self) -> Result<i32, Error<I2C::Error>> {
        if !self.is_bme280 {
            return Err(Error::UnsupportedMeasurement);
        }

        let mut buf = [0u8; 2];
        self.read_regs(regs::HUM_MSB, &mut buf).await?;

        let hum_msb = buf[0] as i32;
        let hum_lsb = buf[1] as i32;

        Ok((hum_msb << 8) | hum_lsb)
    }

    pub async fn read_measurement(&mut self) -> Result<Measurements, Error<I2C::Error>> {
        // Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
        // t_fine carries fine temperature as global value
        let adc_t = self.read_raw_temperature().await?;
        let (t_fine, t) = convert_temperature(adc_t, &self.calib);

        // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
        // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
        let adc_p = self.read_raw_pressure().await?;
        let p = convert_pressure(adc_p, t_fine, &self.calib);

        // convert Q24.8 to centi
        let p = p * 100 / 256;

        // BME280 only
        let mut h = 0;
        if self.is_bme280 {
            let adc_h = self.read_raw_humidity().await?;
            let h0 = convert_humidity(adc_h, t_fine, &self.calib);

            // convert Q22.10 to centi
            h = h0 * 100 / 1024;
        };

        Ok(Measurements {
            temperature: t,
            pressure: p as u32,
            humidity: h,
        })
    }

    pub async fn read_reg(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    pub async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, &[reg, val]).await?;
        Ok(())
    }

    pub async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write_read(self.addr, &[reg], buf).await?;
        Ok(())
    }
}

// - MARK: Helper functions

/// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
/// t_fine carries fine temperature as global value
/// Returns (t_fine, t)
#[inline]
fn convert_temperature(adc_t: i32, calib_data: &CalibrationData) -> (i32, i32) {
    let var1 = (((adc_t >> 3) - ((calib_data.dig_t1 as i32) << 1)) * (calib_data.dig_t2 as i32)) >> 11;
    let var2 = (((((adc_t >> 4) - (calib_data.dig_t1 as i32)) * ((adc_t >> 4) - (calib_data.dig_t1 as i32))) >> 12)
        * (calib_data.dig_t3 as i32))
        >> 14;

    let t_fine = var1 + var2;

    let t = (t_fine * 5 + 128) >> 8;

    (t_fine, t)
}

/// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
/// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
#[inline]
fn convert_pressure(adc_p: i32, t_fine: i32, calib_data: &CalibrationData) -> i32 {
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

    p as i32
}

/// Returns Q22.10 format
#[inline]
fn convert_humidity(adc_h: i32, t_fine: i32, calib_data: &CalibrationData) -> u32 {
    let dig_h1 = calib_data.dig_h1 as i32;
    let dig_h2 = calib_data.dig_h2 as i32;
    let dig_h3 = calib_data.dig_h3 as i32;
    let dig_h4 = calib_data.dig_h4 as i32;
    let dig_h5 = calib_data.dig_h5 as i32;
    let dig_h6 = calib_data.dig_h6 as i32;

    let v_x1_u32r = (t_fine - 76800) as i32;
    let v_x1_u32r = (((((adc_h << 14) - (dig_h4 << 20) - (dig_h5 * v_x1_u32r)) + 16384) >> 15)
        * (((((((v_x1_u32r * (dig_h6)) >> 10) * (((v_x1_u32r * dig_h3) >> 11) + 32768)) >> 10) + 2097152) * dig_h2
            + 8192)
            >> 14));
    let v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * dig_h1) >> 4);

    // limit check

    let v_x1_u32r = i32::min(v_x1_u32r, 419430400);
    let v_x1_u32r = i32::max(v_x1_u32r, 0);

    (v_x1_u32r >> 12) as u32
}
