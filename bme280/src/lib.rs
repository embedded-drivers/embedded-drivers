//! Driver for BME280 and BMP280.
//!
//! The only difference between the two sensors is that the BME280 also includes a humidity sensor.
//!
//! # Licence
//!
//! This crate is distributed as `MIT OR Apache-2.0`. It also references Bosch's
//! BSD-3-Clause driver; the retained notice is in `LICENSE-BOSCH`.

#![cfg_attr(not(test), no_std)]

use embedded_hal_async::delay::DelayNs;

pub const ADDRESS: u8 = 0x76;

pub const CHIP_ID_BME280: u8 = 0x60;
pub const CHIP_ID_BMP280: u8 = 0x58;

/// Control-register writes performed by `init`, **in this exact order**.
///
/// The order is a hardware requirement, not a style choice. Bosch's BME280 API
/// documents that "humidity related changes will be only effective after a write
/// operation to ctrl_meas register", so `CTRL_HUM` must be written *before*
/// `CTRL_MEAS`. Written the other way round, the humidity oversampling setting is
/// silently ignored and `osrs_h` stays at its reset default, which is `0`
/// (skipped) - so humidity would never be measured at all.
///
/// These are two single-byte writes rather than one burst over `0xF2..=0xF5`,
/// because `0xF3` is the read-only `status` register. (The BME680 can burst its
/// equivalent `0x71..=0x75` block, but only because all five of those registers
/// are writable.)
#[allow(clippy::unusual_byte_groupings)]
pub const CONTROL_WRITES: [(u8, u8); 2] = [
    // osrs_h = x1.
    (regs::CTRL_HUM, 0b001),
    // osrs_t = x1, osrs_p = x1, mode = sleep. The 3/3/2 grouping mirrors
    // CTRL_MEAS exactly: osrs_t[7:5], osrs_p[4:2], mode[1:0]. The part is left
    // idle deliberately: `read_measurement` triggers a forced conversion and
    // waits for it, which is what makes a returned measurement trustworthy.
    (regs::CTRL_MEAS, 0b001_001_00),
];

/// `CTRL_MEAS` with x1 oversampling on temperature and pressure, and
/// `mode = forced`: triggers exactly one conversion and then returns to sleep.
///
/// Same 3/3/2 field grouping as [`CONTROL_WRITES`], hence the allow.
#[allow(clippy::unusual_byte_groupings)]
pub(crate) const CTRL_MEAS_FORCED: u8 = 0b001_001_01;

/// Datasheet measurement time at x1 oversampling on all three channels.
///
/// The formula is `1.25 + 2.3*osrs_t + (2.3*osrs_p + 0.575) + (2.3*osrs_h + 0.575)`
/// ms, which is about 9.3 ms at x1/x1/x1. Rounded up.
const MEASUREMENT_TIME_MS: u32 = 10;

/// Extra 1 ms polling budget after the fixed wait, before giving up.
const MEASUREMENT_POLL_LIMIT_MS: u32 = 20;

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

    /// `status`: a conversion is currently running.
    pub const STATUS_MEASURING: u8 = 0x08;
    /// `status`: the NVM calibration data is being copied into the image
    /// registers. Reading calibration before this clears gives garbage.
    pub const STATUS_IM_UPDATE: u8 = 0x01;

    /// `press_msb`: start of the 8-byte pressure/temperature/humidity block.
    pub const DATA_MSB: u8 = 0xF7;
    /// Length of that block: `press(3) + temp(3) + hum(2)`.
    pub const DATA_LEN: usize = 8;
    pub const RESET: u8 = 0xE0;
}

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    CalibrationDataError,
    ConversionError,
    InvalidDevice,
    UnsupportedMeasurement,
    /// The device did not report a completed conversion within the driver's
    /// polling limit. Added in 0.1.1; the enum is not `#[non_exhaustive]`, so a
    /// caller matching exhaustively must handle it.
    Timeout,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::Bus(error)
    }
}

#[derive(Debug, Default)]
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
    /// Placeholder used until `init` reads the real values from the device.
    pub fn new() -> Self {
        Self::default()
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

impl Measurements {
    pub fn temperature_celsius(&self) -> f32 {
        self.temperature as f32 / 100.0
    }

    pub fn pressure_hpa(&self) -> f32 {
        self.pressure as f32 / 100.0 / 100.0
    }

    pub fn pressure_pa(&self) -> u32 {
        self.pressure / 100
    }

    pub fn humidity_percent(&self) -> f32 {
        self.humidity as f32 / 100.0
    }
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

        // `CONTROL_WRITES` carries the required order and the reason for it.
        for (reg, value) in CONTROL_WRITES {
            self.write_reg(reg, value).await?;
        }

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

    /// Trigger a measurement, wait for the device to finish it, then read the
    /// compensated result.
    ///
    /// **Waiting here is the whole point.** Until a conversion completes, the
    /// BME280's data registers hold `0x80000` - its 20-bit "measurement skipped"
    /// marker - and those placeholder bytes decode through the compensation
    /// maths to values that look *plausible* rather than obviously wrong: about
    /// 2.4 degC low, and a pressure about a **third** low, because `t_fine` is
    /// derived from the bogus temperature. Versions before 0.1.1 read the
    /// registers straight after `init` and returned exactly that, with no error
    /// and no other symptom.
    ///
    /// The sequence is therefore:
    ///
    /// 1. write `mode = forced`, so exactly one conversion runs;
    /// 2. wait the datasheet's measurement time;
    /// 3. confirm `status.measuring` has cleared, as a belt-and-braces check;
    /// 4. read the whole pressure/temperature/humidity block in **one burst**,
    ///    as Bosch's `bme280_get_sensor_data` does, so the three values always
    ///    come from the same conversion.
    ///
    /// Step 2 is a fixed wait rather than an immediate poll on purpose: right
    /// after the write the device may not have set `measuring` yet, so polling
    /// straight away can see "not measuring" and read the *previous* result.
    ///
    /// `delay` paces steps 2 and 3. The driver programs x1 oversampling on all
    /// three channels, which the datasheet's measurement-time formula puts at
    /// about 9.3 ms.
    pub async fn read_measurement(&mut self, mut delay: impl DelayNs) -> Result<Measurements, Error<I2C::Error>> {
        self.start_forced_measurement().await?;
        self.wait_for_measurement(&mut delay).await?;

        let mut raw = [0u8; regs::DATA_LEN];
        self.read_regs(regs::DATA_MSB, &mut raw).await?;

        Ok(self.compensate(&raw))
    }

    /// Write `mode = forced`, keeping the x1 oversampling programmed by `init`.
    async fn start_forced_measurement(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_reg(regs::CTRL_MEAS, CTRL_MEAS_FORCED).await
    }

    /// Wait out the conversion, then verify it really finished.
    async fn wait_for_measurement(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        // The datasheet's measurement-time formula at x1/x1/x1 is ~9.3 ms.
        delay.delay_ms(MEASUREMENT_TIME_MS).await;

        let mut extra = 0;
        while self.read_reg(regs::STATUS).await? & regs::STATUS_MEASURING != 0 {
            if extra >= MEASUREMENT_POLL_LIMIT_MS {
                return Err(Error::Timeout);
            }
            delay.delay_ms(1).await;
            extra += 1;
        }
        Ok(())
    }

    /// Decode the 8-byte `press | temp | hum` block and compensate it.
    ///
    /// Field order follows Bosch's `parse_sensor_data`.
    fn compensate(&self, raw: &[u8; regs::DATA_LEN]) -> Measurements {
        let adc_p = ((raw[0] as i32) << 12) | ((raw[1] as i32) << 4) | ((raw[2] as i32) >> 4);
        let adc_t = ((raw[3] as i32) << 12) | ((raw[4] as i32) << 4) | ((raw[5] as i32) >> 4);
        let adc_h = ((raw[6] as i32) << 8) | (raw[7] as i32);

        // Returns temperature in DegC, resolution is 0.01 DegC.
        // t_fine carries fine temperature as global value
        let (t_fine, t) = convert_temperature(adc_t, &self.calib);

        // Pressure in Q24.8 Pa; converted to centi-pascal below.
        let p = convert_pressure(adc_p, t_fine, &self.calib);
        let p = p * 100 / 256;

        // BME280 only
        let mut h = 0;
        if self.is_bme280 {
            let h0 = convert_humidity(adc_h, t_fine, &self.calib);

            // convert Q22.10 to centi
            h = h0 * 100 / 1024;
        }

        Measurements {
            temperature: t,
            pressure: p as u32,
            humidity: h,
        }
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
// NOTE: i32 overflows
#[inline]
fn convert_pressure(adc_p: i32, t_fine: i32, calib_data: &CalibrationData) -> i64 {
    let mut var1 = t_fine as i64 - 128000;
    let mut var2 = var1 * var1 * calib_data.dig_p6 as i64;
    var2 += (var1 * calib_data.dig_p5 as i64) << 17;
    var2 += (calib_data.dig_p4 as i64) << 35;
    var1 = ((var1 * var1 * calib_data.dig_p3 as i64) >> 8) + ((var1 * calib_data.dig_p2 as i64) << 12);
    var1 = (((1i64 << 47) + var1) * (calib_data.dig_p1 as i64)) >> 33;

    if var1 == 0 {
        0
    } else {
        let mut p = 1048576 - adc_p as i64;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (calib_data.dig_p9 as i64 * (p >> 13) * (p >> 13)) >> 25;
        var2 = (calib_data.dig_p8 as i64 * p) >> 19;

        p = ((p + var1 + var2) >> 8) + ((calib_data.dig_p7 as i64) << 4);
        p
    }
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

    let v_x1_u32r = t_fine - 76800;
    let v_x1_u32r = ((((adc_h << 14) - (dig_h4 << 20) - (dig_h5 * v_x1_u32r)) + 16384) >> 15)
        * (((((((v_x1_u32r * (dig_h6)) >> 10) * (((v_x1_u32r * dig_h3) >> 11) + 32768)) >> 10) + 2097152) * dig_h2
            + 8192)
            >> 14);
    let v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * dig_h1) >> 4);

    // limit check

    let v_x1_u32r = i32::min(v_x1_u32r, 419430400);
    let v_x1_u32r = i32::max(v_x1_u32r, 0);

    (v_x1_u32r >> 12) as u32
}
