//! Driver for the BME680 temperature, humidity, pressure and gas sensor.
//!
//! The compensation maths follows Bosch Sensortec's official BME68x SensorAPI
//! (`bme68x.c`, BSD-3-Clause), which is also the reference for the register map.
//! The sensor's own state machine is deliberately not reproduced: this driver
//! exposes a small, idiomatic `embedded-hal` API instead. Raw register access is
//! available through [`BME680::read_reg`], [`BME680::read_regs`] and
//! [`BME680::write_reg`] for anything the high-level API does not cover.
//!
//! Only the **BME680** (Bosch `BME68X_VARIANT_GAS_LOW`) gas path is implemented.
//! The integer ("no FPU") Bosch compensation is used, so the driver itself does
//! no floating point work; the `f32` accessors on [`Measurements`] perform a
//! single final conversion. The BME688 shares the die and answers the same chip
//! ID (`0x61`), but uses the `BME68X_VARIANT_GAS_HIGH` gas-resistance formula,
//! so [`BME680::variant_id`] is exposed and gas readings on a BME688 should be
//! treated as invalid.
//!
//! This module contains the asynchronous driver. See [`blocking`] for the
//! blocking variant.
//!
//! # Licence
//!
//! This crate is distributed under `MIT OR Apache-2.0`, but it derives from
//! Bosch's BSD-3-Clause BME68x SensorAPI. The retained upstream notice is in
//! `LICENSE-BOSCH`.

#![cfg_attr(not(test), no_std)]

use embedded_hal_async::delay::DelayNs;

pub mod blocking;

/// Default 7-bit I2C address, selected when SDO is tied low.
pub const ADDRESS: u8 = 0x76;
/// Alternative 7-bit I2C address, selected when SDO is tied high.
pub const ADDRESS_ALT: u8 = 0x77;

/// Value expected in the chip ID register (`0xD0`).
pub const CHIP_ID: u8 = 0x61;

/// Ambient temperature, in degrees Celsius, used to compute the heater
/// resistance. Bosch's `calc_res_heat` is the only compensation step that needs
/// an ambient temperature; 25 C is used by default because the BME680 cannot
/// measure it before the first conversion. Override
/// [`BME680::ambient_temperature_celsius`] if the board runs hot or cold.
pub const DEFAULT_AMBIENT_CELSIUS: i8 = 25;

/// Default gas heater target, in degrees Celsius. This is Bosch's recommended
/// set point for the BME680.
pub const DEFAULT_HEATER_TARGET_CELSIUS: u16 = 320;

/// Default gas heater duration, in milliseconds. This is Bosch's recommended
/// duration for the BME680.
pub const DEFAULT_HEATER_DURATION_MS: u16 = 150;

/// BME680 register addresses and register fields.
pub mod regs {
    /// `res_heat_val`: heater resistance correction, start of calibration block 3.
    pub const RES_HEAT_VAL: u8 = 0x00;
    /// `res_heat_range`: heater resistance range, start of the calibration block
    /// read that also covers [`RES_HEAT_VAL`].
    pub const RES_HEAT_RANGE: u8 = 0x02;
    /// `range_sw_err`: gas range switching error.
    pub const RANGE_SW_ERR: u8 = 0x04;

    /// `field0`: start of the 17-byte measurement field.
    pub const FIELD0: u8 = 0x1D;
    /// `field0` bit 7: a new measurement is ready.
    pub const FIELD_NEW_DATA: u8 = 0x80;
    /// `field14` bit 5: the gas measurement is valid (`gasm_valid`).
    pub const FIELD_GAS_VALID: u8 = 0x20;
    /// `field14` bit 4: the heater reached its set point (`heat_stab`).
    pub const FIELD_HEAT_STABLE: u8 = 0x10;
    /// `field14` bits 3:0: gas range index.
    pub const FIELD_GAS_RANGE: u8 = 0x0F;

    /// `res_heat0`: heater resistance for gas profile 0.
    pub const RES_HEAT0: u8 = 0x5A;
    /// `gas_wait0`: heater duration for gas profile 0.
    pub const GAS_WAIT0: u8 = 0x64;

    /// `ctrl_gas_0`: heater on/off, bit 3.
    pub const CTRL_GAS_0: u8 = 0x70;
    /// `ctrl_gas_1`: `run_gas` (bit 4) and `nb_conv` (bits 3:0).
    pub const CTRL_GAS_1: u8 = 0x71;
    /// `ctrl_hum`: humidity oversampling, bits 2:0.
    pub const CTRL_HUM: u8 = 0x72;
    /// `ctrl_meas`: temperature/pressure oversampling and power mode.
    pub const CTRL_MEAS: u8 = 0x74;
    /// `config`: IIR filter (bits 4:2) and SPI mode.
    pub const CONFIG: u8 = 0x75;

    /// Start of calibration block 1 (`0x8A..=0xA0`).
    pub const COEFF1: u8 = 0x8A;
    /// `chip_id`: chip identification register.
    pub const CHIP_ID: u8 = 0xD0;
    /// `reset`: write `0xB6` to trigger a soft reset.
    pub const SOFT_RESET: u8 = 0xE0;
    /// Start of calibration block 2 (`0xE1..=0xEE`).
    pub const COEFF2: u8 = 0xE1;
    /// `variant_id`: `0x00` for the BME680, `0x01` for the BME688.
    pub const VARIANT_ID: u8 = 0xF0;

    /// Length of calibration block 1.
    pub const COEFF1_LEN: usize = 23;
    /// Length of calibration block 2.
    pub const COEFF2_LEN: usize = 14;
    /// Length of calibration block 3 (`0x00..=0x04`).
    pub const COEFF3_LEN: usize = 5;
    /// Length of the concatenated calibration image.
    pub const CALIBRATION_LEN: usize = COEFF1_LEN + COEFF2_LEN + COEFF3_LEN;
    /// Length of one measurement field.
    pub const FIELD_LEN: usize = 17;
}

/// Soft reset command written to [`regs::SOFT_RESET`].
pub const SOFT_RESET_COMMAND: u8 = 0xB6;

/// Errors returned by the BME680 driver.
#[derive(Debug)]
pub enum Error<E> {
    /// Error from the underlying I2C bus.
    Bus(E),
    /// The chip ID register (`0xD0`) did not read [`CHIP_ID`]. The payload is
    /// the value that was read.
    InvalidDevice(u8),
    /// A measurement was requested before the calibration data had been read.
    /// Call `init` first.
    NotCalibrated,
    /// The sensor did not report a new measurement within the driver's polling
    /// limit.
    Timeout,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::Bus(error)
    }
}

/// Calibration coefficients, named after Bosch's `struct bme68x_calib_data`.
///
/// All fields are public so that a calibration image read elsewhere can be fed
/// to the compensation functions, but normal use only requires [`BME680::init`]
/// to read them from the device.
#[derive(Debug, Default, Clone, Copy)]
pub struct CalibrationData {
    /// Temperature coefficient 1.
    pub par_t1: u16,
    /// Temperature coefficient 2.
    pub par_t2: i16,
    /// Temperature coefficient 3.
    pub par_t3: i8,
    /// Pressure coefficient 1.
    pub par_p1: u16,
    /// Pressure coefficient 2.
    pub par_p2: i16,
    /// Pressure coefficient 3.
    pub par_p3: i8,
    /// Pressure coefficient 4.
    pub par_p4: i16,
    /// Pressure coefficient 5.
    pub par_p5: i16,
    /// Pressure coefficient 6.
    pub par_p6: i8,
    /// Pressure coefficient 7.
    pub par_p7: i8,
    /// Pressure coefficient 8.
    pub par_p8: i16,
    /// Pressure coefficient 9.
    pub par_p9: i16,
    /// Pressure coefficient 10.
    pub par_p10: u8,
    /// Humidity coefficient 1.
    pub par_h1: u16,
    /// Humidity coefficient 2.
    pub par_h2: u16,
    /// Humidity coefficient 3.
    pub par_h3: i8,
    /// Humidity coefficient 4.
    pub par_h4: i8,
    /// Humidity coefficient 5.
    pub par_h5: i8,
    /// Humidity coefficient 6.
    pub par_h6: u8,
    /// Humidity coefficient 7.
    pub par_h7: i8,
    /// Gas heater coefficient 1.
    pub par_gh1: i8,
    /// Gas heater coefficient 2.
    pub par_gh2: i16,
    /// Gas heater coefficient 3.
    pub par_gh3: i8,
    /// Heater resistance range.
    pub res_heat_range: u8,
    /// Heater resistance correction value.
    pub res_heat_val: i8,
    /// Gas range switching error. Bosch parses this as a **signed** value.
    pub range_sw_err: i8,
}

impl CalibrationData {
    /// Placeholder used until `init` reads the real values from the device.
    pub fn new() -> Self {
        Self::default()
    }

    /// Decode the three calibration blocks exactly as Bosch's `get_calib_data`.
    ///
    /// `raw` is the concatenation of block 1 (`0x8A..=0xA0`), block 2
    /// (`0xE1..=0xEE`) and block 3 (`0x00..=0x04`).
    fn from_raw(raw: &[u8; regs::CALIBRATION_LEN]) -> Self {
        Self {
            par_t1: u16::from_le_bytes([raw[31], raw[32]]),
            par_t2: i16::from_le_bytes([raw[0], raw[1]]),
            par_t3: raw[2] as i8,
            par_p1: u16::from_le_bytes([raw[4], raw[5]]),
            par_p2: i16::from_le_bytes([raw[6], raw[7]]),
            par_p3: raw[8] as i8,
            par_p4: i16::from_le_bytes([raw[10], raw[11]]),
            par_p5: i16::from_le_bytes([raw[12], raw[13]]),
            par_p6: raw[15] as i8,
            par_p7: raw[14] as i8,
            par_p8: i16::from_le_bytes([raw[18], raw[19]]),
            par_p9: i16::from_le_bytes([raw[20], raw[21]]),
            par_p10: raw[22],
            // par_h1 lives in the low nibble of 0xE2 and the whole of 0xE3,
            // par_h2 in the high nibble of 0xE2 and the whole of 0xE1.
            par_h1: ((raw[25] as u16) << 4) | (raw[24] as u16 & 0x0F),
            par_h2: ((raw[23] as u16) << 4) | (raw[24] as u16 >> 4),
            par_h3: raw[26] as i8,
            par_h4: raw[27] as i8,
            par_h5: raw[28] as i8,
            par_h6: raw[29],
            par_h7: raw[30] as i8,
            par_gh1: raw[35] as i8,
            par_gh2: i16::from_le_bytes([raw[33], raw[34]]),
            par_gh3: raw[36] as i8,
            res_heat_range: (raw[39] & 0x30) / 16,
            res_heat_val: raw[37] as i8,
            range_sw_err: ((raw[41] & 0xF0) as i8) / 16,
        }
    }
}

/// One compensated measurement.
///
/// The `raw_*` accessors return the ADC values exactly as they appeared in the
/// sensor's measurement field. The public fields hold Bosch's integer
/// compensation results, so integer-only users never have to touch floating
/// point; the `f32` accessors are a convenience with a single conversion at the
/// end.
#[derive(Debug, Clone, Copy)]
pub struct Measurements {
    raw_temperature: u32,
    raw_pressure: u32,
    raw_humidity: u16,
    raw_gas_resistance: u16,
    gas_range: u8,
    /// Compensated temperature in hundredths of a degree Celsius.
    pub temperature: i32,
    /// Compensated pressure in pascal.
    pub pressure: u32,
    /// Compensated relative humidity in thousandths of a percent.
    pub humidity: u32,
    /// Compensated gas resistance in ohms. Bosch's `gasm_valid` flag says
    /// whether this value is meaningful; the resistance also drifts and needs
    /// burn-in, so treat it as a relative air-quality signal, not a calibrated
    /// reading.
    pub gas_resistance: u32,
    /// Bosch `gasm_valid`: the gas measurement completed.
    pub gas_valid: bool,
    /// Bosch `heat_stab`: the gas heater reached its set point, so
    /// `gas_resistance` is trustworthy.
    pub heat_stable: bool,
}

impl Measurements {
    /// Compensated temperature in degrees Celsius.
    pub fn temperature_celsius(&self) -> f32 {
        self.temperature as f32 / 100.0
    }

    /// Compensated pressure in pascal.
    pub fn pressure_pa(&self) -> u32 {
        self.pressure
    }

    /// Compensated pressure in hectopascal.
    pub fn pressure_hpa(&self) -> f32 {
        self.pressure as f32 / 100.0
    }

    /// Compensated relative humidity in percent.
    pub fn humidity_percent(&self) -> f32 {
        self.humidity as f32 / 1000.0
    }

    /// Compensated gas resistance in ohms.
    pub fn gas_resistance_ohms(&self) -> f32 {
        self.gas_resistance as f32
    }

    /// Raw 20-bit temperature ADC value.
    pub fn raw_temperature(&self) -> u32 {
        self.raw_temperature
    }

    /// Raw 20-bit pressure ADC value.
    pub fn raw_pressure(&self) -> u32 {
        self.raw_pressure
    }

    /// Raw 16-bit humidity ADC value.
    pub fn raw_humidity(&self) -> u16 {
        self.raw_humidity
    }

    /// Raw 10-bit gas resistance ADC value.
    pub fn raw_gas_resistance(&self) -> u16 {
        self.raw_gas_resistance
    }

    /// Gas range index (0..=15) used to interpret [`Self::raw_gas_resistance`].
    pub fn gas_range(&self) -> u8 {
        self.gas_range
    }
}

// - MARK: Bosch compensation maths

/// Lookup table 1 from Bosch's `calc_gas_resistance_low`.
const GAS_RANGE_LOOKUP_1: [i64; 16] = [
    2_147_483_647,
    2_147_483_647,
    2_147_483_647,
    2_147_483_647,
    2_147_483_647,
    2_126_008_810,
    2_147_483_647,
    2_130_303_777,
    2_147_483_647,
    2_147_483_647,
    2_143_188_679,
    2_136_746_228,
    2_147_483_647,
    2_126_008_810,
    2_147_483_647,
    2_147_483_647,
];

/// Lookup table 2 from Bosch's `calc_gas_resistance_low`.
const GAS_RANGE_LOOKUP_2: [i64; 16] = [
    4_096_000_000,
    2_048_000_000,
    1_024_000_000,
    512_000_000,
    255_744_255,
    127_110_228,
    64_000_000,
    32_258_064,
    16_016_016,
    8_000_000,
    4_000_000,
    2_000_000,
    1_000_000,
    500_000,
    250_000,
    125_000,
];

/// Bosch `calc_temperature`. Returns `(t_fine, temperature)` where the
/// temperature is in hundredths of a degree Celsius and `t_fine` is the
/// intermediate used by pressure and humidity.
#[inline]
fn calc_temperature(adc_temp: u32, calib: &CalibrationData) -> (i32, i32) {
    let var1 = ((adc_temp as i32 >> 3) - ((calib.par_t1 as i32) << 1)) as i64;
    let var2 = (var1 * (calib.par_t2 as i64)) >> 11;
    let var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    let var3 = (var3 * ((calib.par_t3 as i64) << 4)) >> 14;

    let t_fine = (var2 + var3) as i32;
    let temperature = ((t_fine * 5) + 128) >> 8;

    (t_fine, temperature)
}

/// Bosch `calc_pressure`. Returns pressure in pascal.
///
/// Bosch performs some of these operations in `int32_t` where they are allowed
/// to wrap; the `wrapping_*` calls reproduce that two's-complement behaviour
/// instead of panicking in a debug build. The `var1 == 0` guard is ours: Bosch
/// would divide by zero on a device whose `par_p1` read back as zero.
#[inline]
fn calc_pressure(adc_pres: u32, t_fine: i32, calib: &CalibrationData) -> u32 {
    /// Bosch's `BME68X_OVERFLOW_CHECK` (`1 << 30`).
    const OVERFLOW_CHECK: i32 = 0x4000_0000;

    let mut var1 = (t_fine >> 1).wrapping_sub(64_000);
    let mut var2 = (((var1 >> 2).wrapping_mul(var1 >> 2)) >> 11).wrapping_mul(calib.par_p6 as i32) >> 2;
    var2 = var2.wrapping_add((var1.wrapping_mul(calib.par_p5 as i32)) << 1);
    var2 = (var2 >> 2).wrapping_add((calib.par_p4 as i32) << 16);
    var1 = (((((var1 >> 2).wrapping_mul(var1 >> 2)) >> 13).wrapping_mul((calib.par_p3 as i32) << 5)) >> 3)
        .wrapping_add((calib.par_p2 as i32).wrapping_mul(var1) >> 1);
    var1 >>= 18;
    var1 = (32_768_i32.wrapping_add(var1).wrapping_mul(calib.par_p1 as i32)) >> 15;

    if var1 == 0 {
        return 0;
    }

    let mut pressure_comp = (1_048_576_i32.wrapping_sub(adc_pres as i32))
        .wrapping_sub(var2 >> 12)
        .wrapping_mul(3125);

    pressure_comp = if pressure_comp >= OVERFLOW_CHECK {
        (pressure_comp / var1) << 1
    } else {
        (pressure_comp << 1) / var1
    };

    var1 = (calib.par_p9 as i32).wrapping_mul(((pressure_comp >> 3).wrapping_mul(pressure_comp >> 3)) >> 13) >> 12;
    var2 = ((pressure_comp >> 2).wrapping_mul(calib.par_p8 as i32)) >> 13;
    let var3 = ((pressure_comp >> 8)
        .wrapping_mul(pressure_comp >> 8)
        .wrapping_mul(pressure_comp >> 8))
    .wrapping_mul(calib.par_p10 as i32)
        >> 17;

    pressure_comp = pressure_comp.wrapping_add(
        (var1
            .wrapping_add(var2)
            .wrapping_add(var3)
            .wrapping_add((calib.par_p7 as i32) << 7))
            >> 4,
    );

    pressure_comp as u32
}

/// Bosch `calc_humidity`. Returns relative humidity in thousandths of a
/// percent, capped to `0..=100_000`.
///
/// As in [`calc_pressure`], `wrapping_*` reproduces the wrapping of Bosch's
/// `int32_t` arithmetic.
#[inline]
fn calc_humidity(adc_hum: u16, t_fine: i32, calib: &CalibrationData) -> u32 {
    let temp_scaled = ((t_fine.wrapping_mul(5)).wrapping_add(128)) >> 8;

    let var1 = (adc_hum as i32)
        .wrapping_sub((calib.par_h1 as i32).wrapping_mul(16))
        .wrapping_sub((temp_scaled.wrapping_mul(calib.par_h3 as i32) / 100) >> 1);

    let var2 = (calib.par_h2 as i32).wrapping_mul(
        (temp_scaled.wrapping_mul(calib.par_h4 as i32) / 100)
            .wrapping_add((temp_scaled.wrapping_mul(temp_scaled.wrapping_mul(calib.par_h5 as i32) / 100) >> 6) / 100)
            .wrapping_add(1 << 14),
    ) >> 10;

    let var3 = var1.wrapping_mul(var2);
    let var4 = ((calib.par_h6 as i32) << 7).wrapping_add(temp_scaled.wrapping_mul(calib.par_h7 as i32) / 100) >> 4;
    let var5 = ((var3 >> 14).wrapping_mul(var3 >> 14)) >> 10;
    let var6 = var4.wrapping_mul(var5) >> 1;
    let humidity = ((var3.wrapping_add(var6) >> 10).wrapping_mul(1000)) >> 12;

    humidity.clamp(0, 100_000) as u32
}

/// Bosch `calc_gas_resistance_low`, the **BME680** (`BME68X_VARIANT_GAS_LOW`)
/// gas path. Returns gas resistance in ohms.
#[inline]
fn calc_gas_resistance(adc_gas: u16, gas_range: u8, calib: &CalibrationData) -> u32 {
    let var1 = ((1340 + 5 * (calib.range_sw_err as i64)) * GAS_RANGE_LOOKUP_1[gas_range as usize]) >> 16;
    let var2 = ((adc_gas as i64) << 15) - 16_777_216 + var1;
    let var3 = (GAS_RANGE_LOOKUP_2[gas_range as usize] * var1) >> 9;

    if var2 == 0 {
        return 0;
    }

    ((var3 + (var2 >> 1)) / var2) as u32
}

/// Bosch `calc_res_heat`. Returns the value for the `res_heat0` register.
///
/// `target_celsius` is capped at 400 C, exactly as Bosch does. `ambient_celsius`
/// is the only place the driver needs an ambient temperature.
#[inline]
fn calc_res_heat(target_celsius: u16, ambient_celsius: i8, calib: &CalibrationData) -> u8 {
    let temp = target_celsius.min(400) as i32;

    let var1 = (((ambient_celsius as i32) * (calib.par_gh3 as i32)) / 1000) * 256;
    let var2 =
        (calib.par_gh1 as i32 + 784) * (((((calib.par_gh2 as i32 + 154_009) * temp * 5) / 100) + 3_276_800) / 10);
    let var3 = var1 + (var2 / 2);
    let var4 = var3 / (calib.res_heat_range as i32 + 4);
    let var5 = (131 * (calib.res_heat_val as i32)) + 65_536;
    let heatr_res_x100 = ((var4 / var5) - 250) * 34;

    ((heatr_res_x100 + 50) / 100) as u8
}

/// Bosch `calc_gas_wait`. Encodes a heater duration in milliseconds for the
/// `gas_wait0` register (strictly, it is a uint16 duration with an exponent and
/// a mantissa, in units of the sensor's frame timing).
#[inline]
fn calc_gas_wait(mut duration_ms: u16) -> u8 {
    if duration_ms >= 0x0FC0 {
        return 0xFF;
    }

    let mut factor = 0u8;
    while duration_ms > 0x3F {
        duration_ms /= 4;
        factor += 1;
    }

    (duration_ms as u8) + (factor * 64)
}

/// Decode a raw measurement field and compensate every channel.
fn compensate_field(calib: &CalibrationData, field: &[u8; regs::FIELD_LEN]) -> Measurements {
    let raw_pressure = ((field[2] as u32) << 12) | ((field[3] as u32) << 4) | ((field[4] as u32) >> 4);
    let raw_temperature = ((field[5] as u32) << 12) | ((field[6] as u32) << 4) | ((field[7] as u32) >> 4);
    let raw_humidity = ((field[8] as u16) << 8) | (field[9] as u16);
    let raw_gas_resistance = ((field[13] as u16) << 2) | ((field[14] as u16) >> 6);
    let gas_range = field[14] & regs::FIELD_GAS_RANGE;

    let (t_fine, temperature) = calc_temperature(raw_temperature, calib);
    let pressure = calc_pressure(raw_pressure, t_fine, calib);
    let humidity = calc_humidity(raw_humidity, t_fine, calib);
    let gas_resistance = calc_gas_resistance(raw_gas_resistance, gas_range, calib);

    Measurements {
        raw_temperature,
        raw_pressure,
        raw_humidity,
        raw_gas_resistance,
        gas_range,
        temperature,
        pressure,
        humidity,
        gas_resistance,
        gas_valid: field[14] & regs::FIELD_GAS_VALID != 0,
        heat_stable: field[14] & regs::FIELD_HEAT_STABLE != 0,
    }
}

// - MARK: Async driver

/// Async BME680 driver.
pub struct BME680<I2C: embedded_hal_async::i2c::I2c> {
    addr: u8,
    i2c: I2C,
    calibrated: bool,
    /// Ambient temperature used by [`BME680::set_gas_heater`]; defaults to
    /// [`DEFAULT_AMBIENT_CELSIUS`].
    pub ambient_temperature_celsius: i8,
    /// Variant ID read from register `0xF0`: `0x00` is a BME680, `0x01` a
    /// BME688. Gas compensation is only valid for `0x00`.
    pub variant_id: u8,
    /// Calibration data read by [`BME680::init`].
    pub calib: CalibrationData,
    gas_heater_target_celsius: u16,
    gas_heater_duration_ms: u16,
}

impl<I2C: embedded_hal_async::i2c::I2c> BME680<I2C> {
    /// Create a driver for a sensor at `addr`.
    ///
    /// This does not touch the bus. Call [`BME680::init`] before measuring.
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            addr,
            i2c,
            calibrated: false,
            ambient_temperature_celsius: DEFAULT_AMBIENT_CELSIUS,
            variant_id: 0,
            calib: CalibrationData::new(),
            gas_heater_target_celsius: DEFAULT_HEATER_TARGET_CELSIUS,
            gas_heater_duration_ms: DEFAULT_HEATER_DURATION_MS,
        }
    }

    /// Create a driver for a sensor at the default address ([`ADDRESS`]).
    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, ADDRESS)
    }

    /// Validate the chip, read the calibration data and program the defaults.
    ///
    /// Bosch's own `bme68x_init` starts with a soft reset, but this driver
    /// deliberately splits the reset into [`BME680::reset`] to match
    /// [`bme280`](https://docs.rs/edrv-bme280): the two crates share one shape.
    /// `init` is safe without a preceding reset because it programs every
    /// control register explicitly.
    ///
    /// Unlike `bme280`, the control registers are written as one burst in
    /// Bosch's order, `CTRL_GAS_1 (0x71)` through `CONFIG (0x75)`. This matters:
    /// `CTRL_HUM (0x72)` must be written *before* `CTRL_MEAS (0x74)`, otherwise
    /// the humidity oversampling setting never takes effect.
    ///
    /// Defaults: temperature oversampling x2, pressure x16, humidity x1, IIR
    /// filter size 3, gas heater enabled at 320 C for 150 ms. The part is left
    /// in sleep mode; use [`BME680::measure`] for a one-shot forced-mode
    /// measurement.
    pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        let chip_id = self.read_reg(regs::CHIP_ID).await?;
        if chip_id != CHIP_ID {
            return Err(Error::InvalidDevice(chip_id));
        }

        self.variant_id = self.read_reg(regs::VARIANT_ID).await?;

        self.read_calibration().await?;
        self.configure().await?;
        self.write_heater_config().await?;

        Ok(())
    }

    /// Soft reset the sensor and wait for it to come back.
    ///
    /// Writes `0xB6` to register `0xE0` and waits 10 ms, which is Bosch's
    /// `BME68X_PERIOD_RESET` (10000 microseconds). The calibration data is kept,
    /// but the driver is marked uncalibrated so the next [`BME680::measure`]
    /// fails with [`Error::NotCalibrated`] until `init` runs again.
    pub async fn reset(&mut self, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        self.write_reg(regs::SOFT_RESET, SOFT_RESET_COMMAND).await?;

        delay.delay_ms(10).await;

        self.calibrated = false;

        Ok(())
    }

    /// Read and store the calibration data. Also called by [`BME680::init`].
    async fn read_calibration(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut raw = [0u8; regs::CALIBRATION_LEN];
        let split = regs::COEFF1_LEN + regs::COEFF2_LEN;
        self.read_regs(regs::COEFF1, &mut raw[..regs::COEFF1_LEN]).await?;
        self.read_regs(regs::COEFF2, &mut raw[regs::COEFF1_LEN..split]).await?;
        self.read_regs(regs::RES_HEAT_VAL, &mut raw[split..]).await?;

        self.calib = CalibrationData::from_raw(&raw);
        self.calibrated = true;

        Ok(())
    }

    /// Program the default oversampling, filter and gas settings.
    async fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
        // Read and write the whole 0x71..=0x75 block, as Bosch's
        // `bme68x_set_conf` does. Bit fields have to be merged into the values
        // that are already there so reserved bits are preserved.
        let mut conf = [0u8; 5];
        self.read_regs(regs::CTRL_GAS_1, &mut conf).await?;
        let [ctrl_gas_1, ctrl_hum, _reserved, ctrl_meas, config] = &mut conf;

        // ctrl_gas_1: nb_conv = 0, run_gas = 1, ODR3 = 1. Bosch's
        // `bme68x_set_conf` sets ODR3 when the ODR is disabled.
        *ctrl_gas_1 = (*ctrl_gas_1 & !0x0F) | (1 << 4) | 0x80;
        // ctrl_hum: osrs_h = x1.
        *ctrl_hum = (*ctrl_hum & !0x07) | 0b001;
        // ctrl_meas: osrs_t = x2, osrs_p = x16, mode = sleep.
        *ctrl_meas = (*ctrl_meas & !0xE0) | (0b010 << 5);
        *ctrl_meas = (*ctrl_meas & !0x1C) | (0b101 << 2);
        *ctrl_meas &= !0x03;
        // config: ODR20 = 0, IIR filter coefficient 3.
        *config = (*config & !0xE0) | (0b010 << 2);

        self.write_config_block(&conf).await?;

        // ctrl_gas_0 bit 3 low enables the heater.
        let ctrl_gas_0 = self.read_reg(regs::CTRL_GAS_0).await?;
        self.write_reg(regs::CTRL_GAS_0, ctrl_gas_0 & !0x08).await?;

        Ok(())
    }

    /// Write `res_heat0` and `gas_wait0` for the configured heater profile.
    async fn write_heater_config(&mut self) -> Result<(), Error<I2C::Error>> {
        let res_heat = calc_res_heat(
            self.gas_heater_target_celsius,
            self.ambient_temperature_celsius,
            &self.calib,
        );
        let gas_wait = calc_gas_wait(self.gas_heater_duration_ms);

        self.write_reg(regs::RES_HEAT0, res_heat).await?;
        self.write_reg(regs::GAS_WAIT0, gas_wait).await?;

        Ok(())
    }

    /// Perform a single forced-mode measurement.
    ///
    /// Triggers one conversion and polls the measurement field for the new-data
    /// flag, sleeping 1 ms between polls. The `delay` is borrowed exactly like
    /// [`bme280::reset`](https://docs.rs/edrv-bme280) borrows its delay. Returns
    /// [`Error::Timeout`] if no new data appears within the polling limit
    /// (roughly one second, which is far longer than the 150 ms gas heater plus
    /// a TPH conversion).
    ///
    /// [`Error::NotCalibrated`] is returned if [`BME680::init`] has not run (or
    /// [`BME680::reset`] has run since).
    pub async fn measure(&mut self, mut delay: impl DelayNs) -> Result<Measurements, Error<I2C::Error>> {
        /// Number of 1 ms polls before giving up.
        const POLL_LIMIT: u32 = 1000;

        if !self.calibrated {
            return Err(Error::NotCalibrated);
        }

        // The heater profile can be changed between measurements, so re-apply it
        // while the part is still in sleep mode.
        self.write_heater_config().await?;

        // Trigger a single forced-mode conversion.
        let ctrl_meas = self.read_reg(regs::CTRL_MEAS).await?;
        self.write_reg(regs::CTRL_MEAS, (ctrl_meas & !0x03) | 0x01).await?;

        let mut polls = 0;
        while self.read_reg(regs::FIELD0).await? & regs::FIELD_NEW_DATA == 0 {
            polls += 1;
            if polls >= POLL_LIMIT {
                return Err(Error::Timeout);
            }
            delay.delay_ms(1).await;
        }

        let mut field = [0u8; regs::FIELD_LEN];
        self.read_regs(regs::FIELD0, &mut field).await?;

        Ok(compensate_field(&self.calib, &field))
    }

    /// Set the gas heater target and duration used by later
    /// [`BME680::measure`] calls.
    ///
    /// The values are stored, not written immediately; `measure` programs
    /// `res_heat0`/`gas_wait0` before triggering each conversion. `target_celsius`
    /// is capped at 400 C by the compensation maths, and `duration_ms` saturates
    /// at the sensor's maximum encodable value.
    pub fn set_gas_heater(&mut self, target_celsius: u16, duration_ms: u16) {
        self.gas_heater_target_celsius = target_celsius;
        self.gas_heater_duration_ms = duration_ms;
    }

    /// Write the 5-byte `0x71..=0x75` configuration block in one transaction.
    async fn write_config_block(&mut self, conf: &[u8; 5]) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(
                self.addr,
                &[regs::CTRL_GAS_1, conf[0], conf[1], conf[2], conf[3], conf[4]],
            )
            .await?;

        Ok(())
    }

    /// Read one register.
    pub async fn read_reg(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    /// Read consecutive registers into `buf`.
    pub async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write_read(self.addr, &[reg], buf).await?;
        Ok(())
    }

    /// Write one register.
    pub async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, &[reg, val]).await?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Real-sensor calibration dump, used by Pimoroni's own compensation tests
    /// (bme680-python issue #11), encoded into Bosch's 42-byte register image.
    const CALIBRATION_A: [u8; regs::CALIBRATION_LEN] = [
        0x65, 0x67, 0x03, 0x00, 0x41, 0x8f, 0xed, 0xd6, 0x58, 0x00, 0x8e, 0x1c, 0x7f, 0xff, 0x2e, 0x1e, 0x00, 0x00,
        0x97, 0xf3, 0xb5, 0xf6, 0x1e, 0x40, 0x54, 0x2a, 0x00, 0x2d, 0x14, 0x78, 0x9c, 0xb9, 0x65, 0x4e, 0x9f, 0xe2,
        0x12, 0x30, 0x00, 0x10, 0x00, 0x00,
    ];

    /// Calibration used by the `bme68x` crate's own compensation tests.
    const CALIBRATION_B: [u8; regs::CALIBRATION_LEN] = [
        0x66, 0x67, 0x03, 0x00, 0x30, 0x8e, 0x43, 0xd6, 0x58, 0x00, 0x10, 0x27, 0x38, 0xff, 0xce, 0x1e, 0x00, 0x00,
        0xa8, 0xe4, 0x70, 0x17, 0x1e, 0x3f, 0xb8, 0x33, 0x00, 0x2d, 0x14, 0x78, 0x9c, 0x90, 0x65, 0x3c, 0xf6, 0xe2,
        0x04, 0x28, 0x00, 0x10, 0x00, 0xe0,
    ];

    /// Build a measurement field the way Bosch's `read_field_data` parses it.
    fn field(
        temperature: u32,
        pressure: u32,
        humidity: u16,
        gas_resistance: u16,
        gas_range: u8,
        gas_valid: bool,
        heat_stable: bool,
    ) -> [u8; regs::FIELD_LEN] {
        let mut field = [0u8; regs::FIELD_LEN];
        field[0] = regs::FIELD_NEW_DATA;
        field[2] = (pressure >> 12) as u8;
        field[3] = (pressure >> 4) as u8;
        field[4] = ((pressure & 0x0F) << 4) as u8;
        field[5] = (temperature >> 12) as u8;
        field[6] = (temperature >> 4) as u8;
        field[7] = ((temperature & 0x0F) << 4) as u8;
        field[8] = (humidity >> 8) as u8;
        field[9] = humidity as u8;
        field[13] = (gas_resistance >> 2) as u8;
        field[14] = (((gas_resistance & 0x03) << 6) as u8) | (gas_range & regs::FIELD_GAS_RANGE);
        if gas_valid {
            field[14] |= regs::FIELD_GAS_VALID;
        }
        if heat_stable {
            field[14] |= regs::FIELD_HEAT_STABLE;
        }
        field
    }

    #[test]
    fn calibration_decoding_matches_bosch_reference() {
        let a = CalibrationData::from_raw(&CALIBRATION_A);
        assert_eq!(a.par_t1, 26041);
        assert_eq!(a.par_t2, 26469);
        assert_eq!(a.par_t3, 3);
        assert_eq!(a.par_p1, 36673);
        assert_eq!(a.par_p2, -10515);
        assert_eq!(a.par_p3, 88);
        assert_eq!(a.par_p4, 7310);
        assert_eq!(a.par_p5, -129);
        assert_eq!(a.par_p6, 30);
        assert_eq!(a.par_p7, 46);
        assert_eq!(a.par_p8, -3177);
        assert_eq!(a.par_p9, -2379);
        assert_eq!(a.par_p10, 30);
        assert_eq!(a.par_h1, 676);
        assert_eq!(a.par_h2, 1029);
        assert_eq!(a.par_h3, 0);
        assert_eq!(a.par_h4, 45);
        assert_eq!(a.par_h5, 20);
        assert_eq!(a.par_h6, 120);
        assert_eq!(a.par_h7, -100);
        assert_eq!(a.par_gh1, -30);
        assert_eq!(a.par_gh2, -24754);
        assert_eq!(a.par_gh3, 18);
        assert_eq!(a.res_heat_range, 1);
        assert_eq!(a.res_heat_val, 48);
        assert_eq!(a.range_sw_err, 0);

        // Two's-complement packing of the shared H1/H2 byte and the signed
        // range-switching error (0xE0 decodes to -2).
        let b = CalibrationData::from_raw(&CALIBRATION_B);
        assert_eq!(b.par_h1, 824);
        assert_eq!(b.par_h2, 1019);
        assert_eq!(b.par_gh2, -2500);
        assert_eq!(b.range_sw_err, -2);
    }

    #[test]
    fn temperature_matches_bosch_reference() {
        let a = CalibrationData::from_raw(&CALIBRATION_A);
        let (t_fine, temperature) = calc_temperature(501240, &a);
        assert_eq!(t_fine, 136667);
        assert_eq!(temperature, 2669);

        let b = CalibrationData::from_raw(&CALIBRATION_B);
        let (t_fine, temperature) = calc_temperature(519888, &b);
        assert_eq!(t_fine, 167871);
        assert_eq!(temperature, 3279);
    }

    #[test]
    fn pressure_matches_bosch_reference() {
        let a = CalibrationData::from_raw(&CALIBRATION_A);
        let (t_fine, _) = calc_temperature(501240, &a);
        assert_eq!(calc_pressure(353485, t_fine, &a), 98711);

        let b = CalibrationData::from_raw(&CALIBRATION_B);
        let (t_fine, _) = calc_temperature(519888, &b);
        assert_eq!(calc_pressure(364576, t_fine, &b), 91655);
    }

    #[test]
    fn humidity_matches_bosch_reference() {
        let a = CalibrationData::from_raw(&CALIBRATION_A);
        let (t_fine, _) = calc_temperature(501240, &a);
        assert_eq!(calc_humidity(19019, t_fine, &a), 42402);

        // Bosch caps humidity at 100 %RH.
        let b = CalibrationData::from_raw(&CALIBRATION_B);
        let (t_fine, _) = calc_temperature(519888, &b);
        assert_eq!(calc_humidity(30000, t_fine, &b), 100000);
    }

    #[test]
    fn gas_resistance_matches_bosch_reference() {
        let a = CalibrationData::from_raw(&CALIBRATION_A);
        assert_eq!(calc_gas_resistance(0, 0, &a), 12946860);

        // Range 12 is the entry that several third-party ports get wrong; the
        // Bosch value is 1711 ohms.
        let b = CalibrationData::from_raw(&CALIBRATION_B);
        assert_eq!(calc_gas_resistance(700, 8, &b), 27407);
        assert_eq!(calc_gas_resistance(700, 12, &b), 1711);
    }

    #[test]
    fn heater_resistance_matches_bosch_reference() {
        let a = CalibrationData::from_raw(&CALIBRATION_A);
        assert_eq!(calc_res_heat(200, 25, &a), 78);
        assert_eq!(calc_res_heat(300, 25, &a), 101);
        assert_eq!(calc_res_heat(320, 25, &a), 106);
        assert_eq!(calc_res_heat(400, 25, &a), 124);
        // Targets above 400 C are capped, not rejected.
        assert_eq!(calc_res_heat(500, 25, &a), calc_res_heat(400, 25, &a));

        let b = CalibrationData::from_raw(&CALIBRATION_B);
        assert_eq!(calc_res_heat(320, 25, &b), 121);
    }

    #[test]
    fn gas_wait_encoding_matches_bosch_reference() {
        assert_eq!(calc_gas_wait(0), 0x00);
        assert_eq!(calc_gas_wait(63), 0x3F);
        assert_eq!(calc_gas_wait(64), 0x50);
        assert_eq!(calc_gas_wait(150), 0x65);
        assert_eq!(calc_gas_wait(252), 0x7F);
        assert_eq!(calc_gas_wait(1008), 0xBF);
        assert_eq!(calc_gas_wait(4031), 0xFE);
        assert_eq!(calc_gas_wait(4032), 0xFF);
        assert_eq!(calc_gas_wait(u16::MAX), 0xFF);
    }

    #[test]
    fn field_decoding_and_compensation_match_reference() {
        let a = CalibrationData::from_raw(&CALIBRATION_A);
        let raw_field = field(501240, 353485, 19019, 0, 0, true, true);
        let m = compensate_field(&a, &raw_field);

        // Raw values survive the register bit packing untouched.
        assert_eq!(m.raw_temperature(), 501240);
        assert_eq!(m.raw_pressure(), 353485);
        assert_eq!(m.raw_humidity(), 19019);
        assert_eq!(m.raw_gas_resistance(), 0);
        assert_eq!(m.gas_range(), 0);

        // Compensated values match the Bosch C reference.
        assert_eq!(m.temperature, 2669);
        assert_eq!(m.temperature_celsius(), 26.69);
        assert_eq!(m.pressure_pa(), 98711);
        assert_eq!(m.pressure_hpa(), 987.11);
        assert_eq!(m.humidity, 42402);
        assert_eq!(m.humidity_percent(), 42.402);
        assert_eq!(m.gas_resistance, 12946860);
        assert_eq!(m.gas_resistance_ohms(), 12946860.0);
        assert!(m.gas_valid);
        assert!(m.heat_stable);

        // The flags come from bits 5 and 4 of field byte 14.
        let m = compensate_field(&a, &field(501240, 353485, 19019, 0, 0, false, false));
        assert!(!m.gas_valid);
        assert!(!m.heat_stable);
    }

    #[test]
    fn constants_are_as_documented() {
        assert_eq!(ADDRESS, 0x76);
        assert_eq!(ADDRESS_ALT, 0x77);
        assert_eq!(CHIP_ID, 0x61);
        assert_eq!(SOFT_RESET_COMMAND, 0xB6);
        assert_eq!(regs::CALIBRATION_LEN, 42);
        assert_eq!(regs::FIELD_LEN, 17);
    }
}
