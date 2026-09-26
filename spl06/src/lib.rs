//! Driver for the Goertek SPL06-001 and the register-compatible SPL06-007
//! digital barometric pressure sensor.
//!
//! The SPL06-001 / SPL06-007 is a calibrated digital barometer with 24-bit
//! pressure and temperature conversion results. This crate provides both an
//! asynchronous driver ([`SPL06`]) and a blocking one ([`blocking::SPL06`])
//! with identical APIs, built on `embedded-hal` v1.
//!
//! # Compensation
//!
//! The datasheet defines the compensation as a real-number polynomial
//! (SPL06-001 V1.0 sections 5.7.1/5.7.2, SPL06-007 V1.0 sections 5.6.1/5.6.2):
//!
//! ```text
//! Traw_sc = Traw / kT
//! Praw_sc = Praw / kP
//!
//! Tcomp(degC) = c0 * 0.5 + c1 * Traw_sc
//! Pcomp(Pa)   = c00 + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * c30))
//!                    + Traw_sc * c01
//!                    + Traw_sc * Praw_sc * (c11 + Praw_sc * c21)
//! ```
//!
//! `kP` and `kT` are **not** constants. Each depends on the configured
//! oversampling rate (Table 7 of the SPL06-001 datasheet), so the raw 24-bit
//! values must be divided by the factor that belongs to the oversampling the
//! driver is actually using. Using the wrong factor silently produces a
//! plausible-looking but wrong reading.
//!
//! The driver evaluates the polynomial in integer Q24 fixed-point arithmetic,
//! so it does no floating point itself; the `f32` accessors on [`Measurements`]
//! are the only conversion and stay out of the generated code for a target
//! without an FPU unless they are actually called.
//!
//! # Init and reset
//!
//! [`SPL06::init`] reads the product/revision ID register (`0x0D`, expected
//! value [`PRODUCT_ID`] = `0x10`) and the 18-byte calibration block at `0x10`,
//! then programs the default oversampling. Like `edrv-bme280` and
//! `edrv-bme680`, `init` deliberately does **not** soft reset;
//! [`SPL06::reset`] is a separate, explicit step.
//!
//! # Licence
//!
//! This crate is distributed under `MIT OR Apache-2.0`.

#![cfg_attr(not(test), no_std)]

use embedded_hal_async::delay::DelayNs;

pub mod blocking;

/// I2C address with `SDO` high - the sensor's **default** address.
///
/// SPL06-001/-007 datasheet, section 5.3.1: "The sensor's address is `0x77`
/// (default) or `0x76` (if the `SDO` pin is pulled-down to GND)."
///
/// Note this is the **opposite** convention to Bosch's barometers, where the
/// primary address is `0x76`. Do not carry that habit across: an SPL06 at
/// `0x76` and one at `0x77` are the same part with a different `SDO` strap.
pub const ADDRESS: u8 = 0x77;

/// I2C address with `SDO` pulled down to GND.
///
/// The address is a property of your board's `SDO` strap, not of the part, so
/// confirm which one you have by reading register `0x0D` rather than assuming.
pub const ADDRESS_ALT: u8 = 0x76;

/// Value expected in the product/revision ID register ([`regs::ID`]).
///
/// The register holds `PROD_ID[7:4]` and `REV_ID[3:0]`, and both the SPL06-001
/// V1.0 and the SPL06-007 V1.0 datasheets give `0x10` as its reset value
/// (`PROD_ID = 0x1`, `REV_ID = 0x0`). The two parts therefore cannot be told
/// apart by this register.
pub const PRODUCT_ID: u8 = 0x10;

/// Value written to [`regs::RESET`] to trigger a soft reset.
///
/// The datasheet specifies `SOFT_RST[3:0] = 0b1001`; the part then runs the
/// same sequence as a power-on reset, so the calibration data has to be read
/// again before the next measurement.
pub const SOFT_RESET_COMMAND: u8 = 0x09;

/// Worst-case time from power-on (or soft reset) until the calibration
/// coefficients can be read, `TCoef_rdy` in the datasheet.
///
/// Both [`SPL06::init`] and [`SPL06::reset`] wait this long.
pub(crate) const STARTUP_TIME_MS: u32 = 40;

/// Register addresses, register field masks and block lengths.
///
/// Addresses and bit positions are from the SPL06-001 V1.0 datasheet,
/// section 7 ("Register Map") and section 8 ("Register Description"); the
/// SPL06-007 V1.0 map is identical.
pub mod regs {
    /// `PRS_B2`: pressure result, bits 23:16.
    pub const PRS_B2: u8 = 0x00;
    /// `PRS_B1`: pressure result, bits 15:8.
    pub const PRS_B1: u8 = 0x01;
    /// `PRS_B0`: pressure result, bits 7:0.
    pub const PRS_B0: u8 = 0x02;

    /// `TMP_B2`: temperature result, bits 23:16.
    pub const TMP_B2: u8 = 0x03;
    /// `TMP_B1`: temperature result, bits 15:8.
    pub const TMP_B1: u8 = 0x04;
    /// `TMP_B0`: temperature result, bits 7:0.
    pub const TMP_B0: u8 = 0x05;

    /// `PRS_CFG`: pressure measurement rate and oversampling.
    pub const PRS_CFG: u8 = 0x06;
    /// `TMP_CFG`: temperature measurement rate, source and oversampling.
    pub const TMP_CFG: u8 = 0x07;
    /// `MEAS_CFG`: operating mode, plus the sensor/coefficient/data-ready flags.
    pub const MEAS_CFG: u8 = 0x08;
    /// `CFG_REG`: interrupt, result-shift and FIFO configuration.
    pub const CFG_REG: u8 = 0x09;
    /// `INT_STS`: interrupt status, cleared on read.
    pub const INT_STS: u8 = 0x0A;
    /// `FIFO_STS`: FIFO full/empty status.
    pub const FIFO_STS: u8 = 0x0B;
    /// `RESET`: soft reset and FIFO flush.
    pub const RESET: u8 = 0x0C;
    /// `ID`: product and revision ID.
    pub const ID: u8 = 0x0D;

    /// Start of the 18-byte calibration coefficient block (`0x10..=0x21`).
    pub const COEF: u8 = 0x10;

    /// `PRS_CFG[6:4]`: pressure measurement rate (background mode only).
    pub const PM_RATE_MASK: u8 = 0x70;
    /// `PRS_CFG[3:0]`: pressure oversampling rate.
    pub const PM_PRC_MASK: u8 = 0x0F;

    /// `TMP_CFG` bit 7: temperature source, `1` = external MEMS sensor.
    pub const TMP_CFG_EXT: u8 = 0x80;
    /// `TMP_CFG[6:4]`: temperature measurement rate (background mode only).
    pub const TMP_RATE_MASK: u8 = 0x70;
    /// `TMP_CFG[2:0]`: temperature oversampling rate.
    pub const TMP_PRC_MASK: u8 = 0x07;

    /// `MEAS_CFG` bit 7: calibration coefficients are available.
    pub const COEF_RDY: u8 = 0x80;
    /// `MEAS_CFG` bit 6: the pressure sensor finished self-initialisation.
    pub const SENSOR_RDY: u8 = 0x40;
    /// `MEAS_CFG` bit 5: a new temperature result is available.
    pub const TMP_RDY: u8 = 0x20;
    /// `MEAS_CFG` bit 4: a new pressure result is available.
    pub const PRS_RDY: u8 = 0x10;
    /// `MEAS_CFG[2:0]`: operating mode and measurement type.
    pub const MEAS_CTRL_MASK: u8 = 0x07;

    /// `MEAS_CTRL = 0b001`: one pressure measurement, then standby.
    pub const MEAS_PRESSURE: u8 = 0b001;
    /// `MEAS_CTRL = 0b010`: one temperature measurement, then standby.
    pub const MEAS_TEMPERATURE: u8 = 0b010;

    /// `CFG_REG` bit 3: shift the temperature result.
    ///
    /// Mandatory when the temperature oversampling is greater than 8 times.
    pub const TMP_SHIFT_EN: u8 = 0x08;
    /// `CFG_REG` bit 2: shift the pressure result.
    ///
    /// Mandatory when the pressure oversampling is greater than 8 times.
    pub const PRS_SHIFT_EN: u8 = 0x04;
    /// `CFG_REG` bit 1: enable the FIFO. This driver keeps the FIFO disabled.
    pub const FIFO_EN: u8 = 0x02;
    /// `CFG_REG` bit 0: `0` = 4-wire SPI, `1` = 3-wire SPI. Unused over I2C.
    pub const SPI_MODE: u8 = 0x01;

    /// `INT_STS` bit 2: FIFO-full interrupt is active.
    pub const INT_FIFO_FULL: u8 = 0x04;
    /// `INT_STS` bit 1: temperature-ready interrupt is active.
    pub const INT_TMP: u8 = 0x02;
    /// `INT_STS` bit 0: pressure-ready interrupt is active.
    pub const INT_PRS: u8 = 0x01;

    /// `FIFO_STS` bit 1: the FIFO is full.
    pub const FIFO_FULL: u8 = 0x02;
    /// `FIFO_STS` bit 0: the FIFO is empty.
    pub const FIFO_EMPTY: u8 = 0x01;

    /// `RESET` bit 7: flush the FIFO.
    pub const FIFO_FLUSH: u8 = 0x80;
    /// `RESET[3:0]`: the soft reset command field.
    pub const SOFT_RST_MASK: u8 = 0x0F;

    /// `ID[7:4]`: product ID.
    pub const PROD_ID_MASK: u8 = 0xF0;
    /// `ID[3:0]`: revision ID.
    pub const REV_ID_MASK: u8 = 0x0F;

    /// Length of one 24-bit conversion result.
    pub const DATA_LEN: usize = 3;
    /// Length of the calibration coefficient block (`0x10..=0x21`).
    pub const CALIBRATION_LEN: usize = 18;
}

/// Errors returned by the SPL06 driver.
#[derive(Debug)]
pub enum Error<E> {
    /// Error from the underlying I2C bus.
    Bus(E),
    /// The ID register ([`regs::ID`]) did not read [`PRODUCT_ID`]. The payload is
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

/// Oversampling (precision) setting for one measurement channel.
///
/// The variants map directly onto `PM_PRC[3:0]` and `TMP_PRC[2:0]` in
/// [`regs::PRS_CFG`] / [`regs::TMP_CFG`]. 16 times and above also require the
/// corresponding result bit-shift in [`regs::CFG_REG`]; [`Config`] takes care
/// of that.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Oversampling {
    /// Single conversion, 3.6 ms.
    Single = 0b000,
    /// 2 times (datasheet "Low Power"), 5.2 ms.
    X2 = 0b001,
    /// 4 times, 8.4 ms.
    X4 = 0b010,
    /// 8 times, 14.8 ms.
    #[default]
    X8 = 0b011,
    /// 16 times (datasheet "Standard"), 27.6 ms. Needs a result shift.
    X16 = 0b100,
    /// 32 times, 53.2 ms. Needs a result shift.
    X32 = 0b101,
    /// 64 times (datasheet "High Precision"), 104.4 ms. Needs a result shift.
    X64 = 0b110,
    /// 128 times, 206.8 ms. Needs a result shift.
    X128 = 0b111,
}

impl Oversampling {
    /// The `PM_PRC` / `TMP_PRC` register value for this setting.
    pub const fn bits(self) -> u8 {
        self as u8
    }

    /// The datasheet's `kP` / `kT` scale factor for this setting (Table 7).
    ///
    /// The raw 24-bit result must be divided by this before compensation:
    /// `Traw_sc = Traw / kT`, `Praw_sc = Praw / kP`.
    pub const fn scale_factor(self) -> i32 {
        match self {
            Oversampling::Single => 524_288,
            Oversampling::X2 => 1_572_864,
            Oversampling::X4 => 3_670_016,
            Oversampling::X8 => 7_864_320,
            Oversampling::X16 => 253_952,
            Oversampling::X32 => 516_096,
            Oversampling::X64 => 1_040_384,
            Oversampling::X128 => 2_088_960,
        }
    }

    /// Whether the result bit-shift in [`regs::CFG_REG`] is mandatory.
    ///
    /// True for every setting above 8 times.
    pub const fn requires_shift(self) -> bool {
        (self as u8) > (Oversampling::X8 as u8)
    }

    /// Worst-case measurement time in milliseconds, rounded up.
    ///
    /// From the "Pressure measurement time (ms)" row of the SPL06-001
    /// datasheet Table 11.
    pub const fn measurement_time_ms(self) -> u32 {
        match self {
            Oversampling::Single => 4,
            Oversampling::X2 => 6,
            Oversampling::X4 => 9,
            Oversampling::X8 => 15,
            Oversampling::X16 => 28,
            Oversampling::X32 => 54,
            Oversampling::X64 => 105,
            Oversampling::X128 => 207,
        }
    }
}

/// Oversampling configuration, with the register writes it implies.
///
/// `kP` / `kT` are derived from this configuration, so changing the
/// oversampling after a measurement means the next measurement has to be
/// compensated with the new factors. [`SPL06::set_oversampling`] writes the
/// configuration registers and updates the driver's configuration together.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Pressure oversampling.
    pub pressure: Oversampling,
    /// Temperature oversampling. Pressure compensation depends on a fresh
    /// temperature result, so this should normally be greater than or equal to
    /// [`Config::pressure`].
    pub temperature: Oversampling,
}

impl Default for Config {
    /// 8 times oversampling on both channels.
    ///
    /// 8 times is the highest setting that needs no result bit-shift, which
    /// keeps the default configuration self-contained at a moderate conversion
    /// time (14.8 ms per channel).
    fn default() -> Self {
        Config {
            pressure: Oversampling::X8,
            temperature: Oversampling::X8,
        }
    }
}

impl Config {
    /// The register writes implied by this configuration, **in this exact
    /// order**.
    ///
    /// [`regs::PRS_CFG`] and [`regs::TMP_CFG`] select the oversampling, and
    /// [`regs::CFG_REG`] carries the result bit-shifts that the datasheet
    /// requires for any oversampling above 8 times. Writing `CFG_REG` after the
    /// two oversampling registers keeps every shift enable next to the precision
    /// it belongs to, and a single source of truth
    /// ([`Config::control_writes`]) drives `init`, `set_oversampling` and the
    /// regression test that guards the sequence, so the shift and the
    /// oversampling can never be programmed from two places that drift apart.
    ///
    /// `CFG_REG`'s interrupt and FIFO enable bits are written as zero because
    /// this driver uses neither; the measurement rate fields are left at their
    /// default 1 measurement per second because they only apply in background
    /// mode, which this single-shot driver does not use.
    pub const fn control_writes(&self) -> [(u8, u8); 3] {
        [
            // PM_RATE = 0, PM_PRC = oversampling.
            (regs::PRS_CFG, self.pressure.bits() & regs::PM_PRC_MASK),
            // TMP_EXT = 1 (the datasheet asks for the external MEMS sensor),
            // TMP_RATE = 0, TMP_PRC = oversampling.
            (
                regs::TMP_CFG,
                regs::TMP_CFG_EXT | (self.temperature.bits() & regs::TMP_PRC_MASK),
            ),
            // Result bit-shifts, mandatory above 8 times.
            (regs::CFG_REG, self.shift_enables()),
        ]
    }

    /// The [`regs::CFG_REG`] shift bits this configuration requires.
    pub const fn shift_enables(&self) -> u8 {
        let mut value = 0;
        if self.pressure.requires_shift() {
            value |= regs::PRS_SHIFT_EN;
        }
        if self.temperature.requires_shift() {
            value |= regs::TMP_SHIFT_EN;
        }
        value
    }
}

/// Calibration coefficients, named after the datasheet's `COEF` registers.
///
/// All fields are public so that a calibration image read elsewhere can be
/// inspected, but normal use only requires [`SPL06::init`] to read them from
/// the device.
///
/// The coefficients are two's complement values of mixed width: `c0` and `c1`
/// are 12-bit, `c00` and `c10` are 20-bit, and `c01`, `c11`, `c20`, `c21` and
/// `c30` are 16-bit (SPL06-001 V1.0 Table 13).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct CalibrationData {
    /// Temperature coefficient 0 (12-bit).
    pub c0: i16,
    /// Temperature coefficient 1 (12-bit).
    pub c1: i16,
    /// Pressure coefficient 00 (20-bit), the pressure offset in pascal.
    pub c00: i32,
    /// Pressure coefficient 10 (20-bit).
    pub c10: i32,
    /// Pressure coefficient 01 (16-bit).
    pub c01: i16,
    /// Pressure coefficient 11 (16-bit).
    pub c11: i16,
    /// Pressure coefficient 20 (16-bit).
    pub c20: i16,
    /// Pressure coefficient 21 (16-bit).
    pub c21: i16,
    /// Pressure coefficient 30 (16-bit).
    pub c30: i16,
}

impl CalibrationData {
    /// Placeholder used until `init` reads the real values from the device.
    pub const fn new() -> Self {
        Self {
            c0: 0,
            c1: 0,
            c00: 0,
            c10: 0,
            c01: 0,
            c11: 0,
            c20: 0,
            c21: 0,
            c30: 0,
        }
    }

    /// Decode the 18-byte calibration block starting at [`regs::COEF`].
    ///
    /// Every coefficient is a two's complement value spanning a nibble
    /// boundary, so each one has to be sign-extended from its own bit width;
    /// missing sign extension is a classic way to get plausible but wrong
    /// readings.
    pub fn from_raw(raw: &[u8; regs::CALIBRATION_LEN]) -> Self {
        // c0 = raw[0] and the high nibble of raw[1].
        // c1 = the low nibble of raw[1] and raw[2].
        // c00 = raw[3], raw[4] and the high nibble of raw[5].
        // c10 = the low nibble of raw[5], raw[6] and raw[7].
        // The rest are plain big-endian 16-bit values.
        Self {
            c0: sign_extend(u32::from(raw[0]) << 4 | u32::from(raw[1]) >> 4, 12) as i16,
            c1: sign_extend(u32::from(raw[1] & 0x0F) << 8 | u32::from(raw[2]), 12) as i16,
            c00: sign_extend(
                u32::from(raw[3]) << 12 | u32::from(raw[4]) << 4 | u32::from(raw[5]) >> 4,
                20,
            ),
            c10: sign_extend(
                u32::from(raw[5] & 0x0F) << 16 | u32::from(raw[6]) << 8 | u32::from(raw[7]),
                20,
            ),
            c01: i16::from_be_bytes([raw[8], raw[9]]),
            c11: i16::from_be_bytes([raw[10], raw[11]]),
            c20: i16::from_be_bytes([raw[12], raw[13]]),
            c21: i16::from_be_bytes([raw[14], raw[15]]),
            c30: i16::from_be_bytes([raw[16], raw[17]]),
        }
    }
}

/// One compensated measurement.
///
/// The `raw_*` accessors return the 24-bit two's complement ADC values exactly
/// as they appeared in the sensor's data registers. The public fields hold the
/// integer compensation results, so an integer-only user never has to touch
/// floating point; the `f32` accessors are a convenience on top.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurements {
    raw_temperature: i32,
    raw_pressure: i32,
    /// Compensated temperature in hundredths of a degree Celsius.
    pub temperature: i32,
    /// Compensated pressure in pascal.
    pub pressure: i32,
}

impl Measurements {
    /// Compensated temperature in degrees Celsius.
    pub fn temperature_celsius(&self) -> f32 {
        self.temperature as f32 / 100.0
    }

    /// Compensated pressure in pascal.
    pub fn pressure_pa(&self) -> f32 {
        self.pressure as f32
    }

    /// Compensated pressure in hectopascal (millibar).
    pub fn pressure_hpa(&self) -> f32 {
        self.pressure as f32 / 100.0
    }

    /// Raw 24-bit two's complement temperature ADC value.
    pub fn raw_temperature(&self) -> i32 {
        self.raw_temperature
    }

    /// Raw 24-bit two's complement pressure ADC value.
    pub fn raw_pressure(&self) -> i32 {
        self.raw_pressure
    }
}

// - MARK: Fixed-point compensation

/// Fractional bits used for the scaled raw values and the polynomial.
///
/// 24 bits leave a wide safety margin against i64 overflow even for a
/// saturated 24-bit raw value at single oversampling, where the scaled value
/// can reach 16 (`|raw| <= 2^23`, `kP = 2^19`). See [`compensate_pressure`] for
/// the worst-case products.
const FRACTIONAL_BITS: u32 = 24;
/// `1 << FRACTIONAL_BITS`.
const FRACTIONAL_SCALE: i64 = 1 << FRACTIONAL_BITS;

/// Sign-extend the low `bits` of `value` to a full `i32`.
#[inline]
const fn sign_extend(value: u32, bits: u32) -> i32 {
    let shift = 32 - bits;
    ((value << shift) as i32) >> shift
}

/// Divide, rounding half away from zero. `denominator` must be positive.
#[inline]
fn div_round(numerator: i64, denominator: i64) -> i64 {
    if numerator >= 0 {
        (numerator + denominator / 2) / denominator
    } else {
        -((-numerator + denominator / 2) / denominator)
    }
}

/// Divide toward zero. `denominator` must be positive.
#[inline]
fn div_trunc(numerator: i64, denominator: i64) -> i64 {
    numerator / denominator
}

/// Scale a raw 24-bit result by its oversampling-dependent `kP` / `kT` factor.
///
/// The returned value is Q24: it represents `raw / k`.
#[inline]
fn scale_raw(raw: i32, oversampling: Oversampling) -> i64 {
    div_round(
        i64::from(raw) * FRACTIONAL_SCALE,
        i64::from(oversampling.scale_factor()),
    )
}

/// Decode a 24-bit big-endian two's complement value.
#[inline]
fn decode_i24(raw: &[u8; regs::DATA_LEN]) -> i32 {
    sign_extend(u32::from(raw[0]) << 16 | u32::from(raw[1]) << 8 | u32::from(raw[2]), 24)
}

/// Datasheet 5.7.2: `Tcomp = c0 * 0.5 + c1 * Traw_sc`, in hundredths of a
/// degree Celsius.
#[inline]
fn compensate_temperature(
    raw_temperature: i32,
    temperature_oversampling: Oversampling,
    calib: &CalibrationData,
) -> i32 {
    let traw_sc = scale_raw(raw_temperature, temperature_oversampling);

    // Q24 degrees Celsius.
    let tcomp = i64::from(calib.c0) * (FRACTIONAL_SCALE / 2) + i64::from(calib.c1) * traw_sc;

    div_round(tcomp * 100, FRACTIONAL_SCALE) as i32
}

/// Datasheet 5.7.1: the pressure polynomial, in pascal.
///
/// The polynomial is expanded term by term rather than evaluated in its nested
/// form. With `Praw_sc` in Q24 the nested form squares and cubes `Praw_sc`
/// before dividing, and `Praw_sc` can reach 16 for a saturated 24-bit raw value
/// at single oversampling, which overflows `i64`. Expanding it lets every
/// division happen before the next multiplication, so the largest intermediate
/// is `Praw_sc^3 * c30` at about `2^51` - six orders of magnitude below
/// `i64::MAX`. Q24 also keeps the rounding error of the scaled values well
/// below the sensor's own noise: over the physical range, one Q24 step of error
/// in `Praw_sc` moves the result by well under 0.1 Pa.
#[inline]
fn compensate_pressure(
    raw_pressure: i32,
    raw_temperature: i32,
    pressure_oversampling: Oversampling,
    temperature_oversampling: Oversampling,
    calib: &CalibrationData,
) -> i32 {
    let praw_sc = scale_raw(raw_pressure, pressure_oversampling);
    let traw_sc = scale_raw(raw_temperature, temperature_oversampling);

    // c00 + Praw_sc*(c10 + Praw_sc*(c20 + Praw_sc*c30))
    //     + Traw_sc*c01 + Traw_sc*Praw_sc*(c11 + Praw_sc*c21)
    let praw_sq = div_trunc(praw_sc * praw_sc, FRACTIONAL_SCALE);
    let praw_cu = div_trunc(praw_sq * praw_sc, FRACTIONAL_SCALE);
    let traw_praw = div_trunc(traw_sc * praw_sc, FRACTIONAL_SCALE);
    let traw_praw_sq = div_trunc(traw_praw * praw_sc, FRACTIONAL_SCALE);

    let mut pressure = i64::from(calib.c00) * FRACTIONAL_SCALE;
    pressure += praw_sc * i64::from(calib.c10);
    pressure += praw_sq * i64::from(calib.c20);
    pressure += praw_cu * i64::from(calib.c30);
    pressure += traw_sc * i64::from(calib.c01);
    pressure += traw_praw * i64::from(calib.c11);
    pressure += traw_praw_sq * i64::from(calib.c21);

    div_round(pressure, FRACTIONAL_SCALE) as i32
}

/// Compensate one pressure / temperature pair.
#[inline]
fn compensate(raw_pressure: i32, raw_temperature: i32, config: &Config, calib: &CalibrationData) -> Measurements {
    Measurements {
        raw_temperature,
        raw_pressure,
        temperature: compensate_temperature(raw_temperature, config.temperature, calib),
        pressure: compensate_pressure(
            raw_pressure,
            raw_temperature,
            config.pressure,
            config.temperature,
            calib,
        ),
    }
}

// - MARK: Async driver

/// Async SPL06-001 / SPL06-007 driver.
pub struct SPL06<I2C: embedded_hal_async::i2c::I2c> {
    addr: u8,
    i2c: I2C,
    calibrated: bool,
    /// Oversampling configuration used by [`SPL06::configure`] and
    /// [`SPL06::measure`]. Starts at [`Config::default`].
    pub config: Config,
    /// Calibration data read by [`SPL06::init`].
    pub calib: CalibrationData,
}

impl<I2C: embedded_hal_async::i2c::I2c> SPL06<I2C> {
    /// Create a driver for a sensor at `addr`.
    ///
    /// This does not touch the bus. Call [`SPL06::init`] before measuring.
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            addr,
            i2c,
            calibrated: false,
            config: Config::default(),
            calib: CalibrationData::new(),
        }
    }

    /// Create a driver for a sensor at the default address ([`ADDRESS`]).
    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, ADDRESS)
    }

    /// Validate the device, wait for its start-up sequence, read the
    /// calibration data and program the default oversampling.
    ///
    /// Reads [`regs::ID`] and rejects anything other than [`PRODUCT_ID`], then
    /// polls `MEAS_CFG` until `COEF_RDY` is set, reads the 18-byte calibration
    /// block at [`regs::COEF`] and applies [`Config::default`].
    ///
    /// Unlike the vendor's own start-up sequence this does **not** soft reset:
    /// [`SPL06::reset`] is an explicit, separate step, matching `edrv-bme280`
    /// and `edrv-bme680`. `init` is safe without a preceding reset because it
    /// programs every writable configuration register explicitly.
    ///
    /// The wait is not optional. The coefficients are unavailable for
    /// `TCoef_rdy` (40 ms) after power-on, and the part answers a read in that
    /// window with an all-zero block rather than an error. Every term of the
    /// compensation polynomial is multiplied by a coefficient, so that block
    /// decodes to exactly 0 Pa and 0.00 degC and no measurement ever complains
    /// about it. `init` therefore returns [`Error::Timeout`] if the flag does not
    /// appear within twice `TCoef_rdy`, rather than accepting a blank block.
    pub async fn init(&mut self, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        let id = self.read_reg(regs::ID).await?;
        if id != PRODUCT_ID {
            return Err(Error::InvalidDevice(id));
        }

        self.wait_for_startup(&mut delay).await?;

        self.read_calibration().await?;
        self.configure().await?;

        Ok(())
    }

    /// Soft reset the sensor and wait for it to come back.
    ///
    /// Writes [`SOFT_RESET_COMMAND`] to [`regs::RESET`] and waits 40 ms, the
    /// datasheet's worst-case time for the calibration coefficients to become
    /// available again (`TCoef_rdy`). The whole configuration returns to its
    /// reset defaults, so `init` has to run again before the next measurement;
    /// the driver is marked uncalibrated so a premature [`SPL06::measure`]
    /// fails with [`Error::NotCalibrated`] instead of using stale data.
    pub async fn reset(&mut self, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        self.write_reg(regs::RESET, SOFT_RESET_COMMAND).await?;

        delay.delay_ms(STARTUP_TIME_MS).await;

        self.calibrated = false;

        Ok(())
    }

    /// Write the current [`SPL06::config`] to the sensor.
    ///
    /// The writes and their order come from [`Config::control_writes`]; see
    /// that method for why the order matters.
    pub async fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
        for (reg, value) in self.config.control_writes() {
            self.write_reg(reg, value).await?;
        }

        Ok(())
    }

    /// Change the oversampling and write the new configuration immediately.
    pub async fn set_oversampling(
        &mut self,
        pressure: Oversampling,
        temperature: Oversampling,
    ) -> Result<(), Error<I2C::Error>> {
        self.config = Config { pressure, temperature };

        self.configure().await
    }

    /// Read and store the calibration block. Also called by [`SPL06::init`].
    pub async fn read_calibration(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut raw = [0u8; regs::CALIBRATION_LEN];
        self.read_regs(regs::COEF, &mut raw).await?;

        self.calib = CalibrationData::from_raw(&raw);
        self.calibrated = true;

        Ok(())
    }

    /// Trigger a single temperature conversion and return the raw 24-bit value.
    pub async fn read_raw_temperature(&mut self, mut delay: impl DelayNs) -> Result<i32, Error<I2C::Error>> {
        self.trigger(regs::MEAS_TEMPERATURE).await?;
        self.wait_for_flag(regs::TMP_RDY, self.config.temperature.measurement_time_ms(), &mut delay)
            .await?;

        let mut raw = [0u8; regs::DATA_LEN];
        self.read_regs(regs::TMP_B2, &mut raw).await?;

        Ok(decode_i24(&raw))
    }

    /// Trigger a single pressure conversion and return the raw 24-bit value.
    pub async fn read_raw_pressure(&mut self, mut delay: impl DelayNs) -> Result<i32, Error<I2C::Error>> {
        self.trigger(regs::MEAS_PRESSURE).await?;
        self.wait_for_flag(regs::PRS_RDY, self.config.pressure.measurement_time_ms(), &mut delay)
            .await?;

        let mut raw = [0u8; regs::DATA_LEN];
        self.read_regs(regs::PRS_B2, &mut raw).await?;

        Ok(decode_i24(&raw))
    }

    /// Perform one single-shot temperature and pressure measurement.
    ///
    /// Temperature is measured first because the pressure compensation depends
    /// on it. Each conversion is started through [`regs::MEAS_CFG`] and then
    /// polled for its ready flag, sleeping 1 ms between polls; the poll limit is
    /// twice the datasheet's worst-case conversion time plus 10 ms, after which
    /// [`Error::Timeout`] is returned.
    ///
    /// [`Error::NotCalibrated`] is returned if [`SPL06::init`] has not run (or
    /// [`SPL06::reset`] has run since).
    pub async fn measure(&mut self, mut delay: impl DelayNs) -> Result<Measurements, Error<I2C::Error>> {
        if !self.calibrated {
            return Err(Error::NotCalibrated);
        }

        let raw_temperature = self.read_raw_temperature(&mut delay).await?;
        let raw_pressure = self.read_raw_pressure(&mut delay).await?;

        Ok(compensate(raw_pressure, raw_temperature, &self.config, &self.calib))
    }

    /// Start one command-mode conversion.
    async fn trigger(&mut self, command: u8) -> Result<(), Error<I2C::Error>> {
        self.write_reg(regs::MEAS_CFG, command).await
    }

    /// Poll [`regs::MEAS_CFG`] until `flag` is set.
    ///
    /// Gives up after twice `worst_case_ms` plus 10 ms, sleeping 1 ms between
    /// polls.
    async fn wait_for_flag<D: DelayNs>(
        &mut self,
        flag: u8,
        worst_case_ms: u32,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        let limit = worst_case_ms * 2 + 10;
        let mut polls = 0;

        while self.read_reg(regs::MEAS_CFG).await? & flag == 0 {
            polls += 1;
            if polls >= limit {
                return Err(Error::Timeout);
            }
            delay.delay_ms(1).await;
        }

        Ok(())
    }

    /// Poll [`regs::MEAS_CFG`] until the part reports that the calibration
    /// coefficients can be read.
    ///
    /// [`regs::COEF_RDY`] (bit 7) means "calibration coefficients valid", which
    /// the datasheet places about 40 ms after power-up. That is exactly the
    /// precondition for the read that follows, so waiting on it alone is both
    /// sufficient and minimal.
    ///
    /// [`regs::SENSOR_RDY`] (bit 6) is deliberately *not* required: it reports
    /// that the sensor finished its own initialisation, which is not a
    /// precondition for reading the coefficient block. Requiring it would make
    /// `init` fail on a part that publishes its coefficients before, or without,
    /// that flag. Paparazzi's `spa06.c` waits for both bits, iNav's
    /// `barometer_spl06.c` reads the block on `COEFFS_RDY` alone.
    async fn wait_for_startup(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        self.wait_for_flag(regs::COEF_RDY, STARTUP_TIME_MS, delay).await
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

    /// A calibration image read from a real SPL06-001, with the coefficient
    /// values it decodes to asserted alongside it. Using bytes from a real part
    /// rather than a synthetic pattern is what makes the signedness and scaling
    /// mistakes visible.
    const CALIBRATION_REAL: [u8; regs::CALIBRATION_LEN] = [
        0x0c, 0xbe, 0xfc, 0x13, 0xd9, 0xaf, 0x2b, 0x34, 0xf3, 0xf7, 0x04, 0xff, 0xda, 0x5a, 0x00, 0x0a, 0xfb, 0x1b,
    ];

    /// Every coefficient at its most negative value: `c0`/`c1` = -2048,
    /// `c00`/`c10` = -524288, the rest = -32768.
    const CALIBRATION_SIGN_BITS: [u8; regs::CALIBRATION_LEN] = [
        0x80, 0x08, 0x00, 0x80, 0x00, 0x08, 0x00, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
    ];

    /// Hand-built, physically plausible calibration used to check the integer
    /// path against the datasheet's real-number formula.
    const CALIBRATION_PHYSICAL: CalibrationData = CalibrationData {
        c0: 50,
        c1: 100,
        c00: 100_000,
        c10: -2_000,
        c01: 120,
        c11: 250,
        c20: 1_500,
        c21: -30,
        c30: -200,
    };

    #[test]
    fn calibration_decoding_matches_mit_reference() {
        let calib = CalibrationData::from_raw(&CALIBRATION_REAL);

        assert_eq!(calib.c0, 203);
        assert_eq!(calib.c1, -260);
        assert_eq!(calib.c00, 81306);
        assert_eq!(calib.c10, -54476);
        assert_eq!(calib.c01, -3081);
        assert_eq!(calib.c11, 1279);
        assert_eq!(calib.c20, -9638);
        assert_eq!(calib.c21, 10);
        assert_eq!(calib.c30, -1253);
    }

    #[test]
    fn calibration_sign_extends_every_coefficient() {
        let calib = CalibrationData::from_raw(&CALIBRATION_SIGN_BITS);

        assert_eq!(calib.c0, -2048);
        assert_eq!(calib.c1, -2048);
        assert_eq!(calib.c00, -524_288);
        assert_eq!(calib.c10, -524_288);
        assert_eq!(calib.c01, -32_768);
        assert_eq!(calib.c11, -32_768);
        assert_eq!(calib.c20, -32_768);
        assert_eq!(calib.c21, -32_768);
        assert_eq!(calib.c30, -32_768);

        // ... and a positive coefficient stays positive, i.e. the sign bits are
        // not simply copied into every value.
        let mut raw = CALIBRATION_SIGN_BITS;
        raw[0] = 0x7F;
        raw[1] = 0xF0;
        raw[3] = 0x7F;
        raw[4] = 0xFF;
        raw[5] = 0xF0;
        raw[8] = 0x7F;
        raw[9] = 0xFF;
        let calib = CalibrationData::from_raw(&raw);
        assert_eq!(calib.c0, 2047);
        assert_eq!(calib.c1, 0);
        assert_eq!(calib.c00, 524_287);
        assert_eq!(calib.c10, 0);
        assert_eq!(calib.c01, 32_767);
    }

    #[test]
    fn raw_24_bit_values_are_sign_extended() {
        assert_eq!(decode_i24(&[0x00, 0x00, 0x01]), 1);
        assert_eq!(decode_i24(&[0x7F, 0xFF, 0xFF]), 8_388_607);
        assert_eq!(decode_i24(&[0x80, 0x00, 0x00]), -8_388_608);
        assert_eq!(decode_i24(&[0xFF, 0xFF, 0xFF]), -1);
    }

    #[test]
    fn temperature_matches_the_datasheet_formula() {
        let calib = CalibrationData::from_raw(&CALIBRATION_REAL);

        // float reference: 66.8333 degC at 8 times oversampling.
        assert_eq!(compensate_temperature(0x100000, Oversampling::X8, &calib), 6683);
        assert_eq!(compensate_temperature(-0x100000, Oversampling::X8, &calib), 13617);
        // The kT factor belongs to the configured oversampling: at single
        // oversampling the same raw value is 1000 times smaller.
        assert_eq!(compensate_temperature(0x000800, Oversampling::Single, &calib), 10048);

        assert_eq!(
            compensate_temperature(0x040000, Oversampling::X8, &CALIBRATION_PHYSICAL),
            2833
        );
        assert_eq!(
            compensate_temperature(0x000400, Oversampling::Single, &CALIBRATION_PHYSICAL),
            2520
        );
    }

    #[test]
    fn pressure_matches_the_datasheet_formula() {
        let calib = CalibrationData::from_raw(&CALIBRATION_REAL);

        let pressure = |raw_pressure: i32, raw_temperature: i32, pressure_os, temperature_os| {
            compensate_pressure(raw_pressure, raw_temperature, pressure_os, temperature_os, &calib)
        };

        // Float references (see the crate's test-vector cross-check): the
        // integer path agrees with the C implementations to well under 1 Pa.
        assert_eq!(pressure(0x300000, 0x100000, Oversampling::X8, Oversampling::X8), 57551);
        assert_eq!(
            pressure(-0x300000, -0x100000, Oversampling::X8, Oversampling::X8),
            102113
        );
        assert_eq!(
            pressure(0x001000, 0x000800, Oversampling::Single, Oversampling::Single),
            80868
        );
        assert_eq!(
            pressure(0x400000, 0x100000, Oversampling::X64, Oversampling::X8),
            -376763
        );

        let physical = |raw_pressure: i32, raw_temperature: i32, os| {
            compensate_pressure(raw_pressure, raw_temperature, os, os, &CALIBRATION_PHYSICAL)
        };
        assert_eq!(physical(0x100000, 0x040000, Oversampling::X8), 99765);
        assert_eq!(physical(0x000800, 0x000400, Oversampling::Single), 99992);
    }

    #[test]
    fn pressure_uses_each_channels_own_scale_factor() {
        let calib = CalibrationData::from_raw(&CALIBRATION_REAL);

        // Temperature and pressure oversampling are independent, so the two
        // scale factors must be looked up separately.
        assert_eq!(
            compensate_pressure(0x300000, 0x100000, Oversampling::X8, Oversampling::X8, &calib),
            57551
        );
        assert_eq!(
            compensate_pressure(0x300000, 0x100000, Oversampling::X64, Oversampling::X8, &calib),
            -206042
        );

        // The trap: feeding 8-times raw data through the single-oversampling kP
        // is a factor of 15 out and gives a completely different pressure.
        let wrong = compensate_pressure(0x300000, 0x100000, Oversampling::Single, Oversampling::X8, &calib);
        assert_eq!(wrong, -862_506);
        assert!((wrong - 57551).abs() > 100_000);
    }

    #[test]
    fn compensate_uses_the_configured_oversampling() {
        let calib = CalibrationData::from_raw(&CALIBRATION_REAL);

        // An asymmetric configuration makes sure the two scale factors are not
        // swapped somewhere in the plumbing.
        let config = Config {
            pressure: Oversampling::X64,
            temperature: Oversampling::X8,
        };
        let measurements = compensate(0x300000, 0x100000, &config, &calib);

        assert_eq!(measurements.pressure, -206042);
        assert_eq!(measurements.temperature, 6683);
    }

    #[test]
    fn scale_factors_match_the_datasheet_table() {
        // (setting, kP/kT, needs a result shift, worst-case time in ms)
        let expected = [
            (Oversampling::Single, 524_288, false, 4),
            (Oversampling::X2, 1_572_864, false, 6),
            (Oversampling::X4, 3_670_016, false, 9),
            (Oversampling::X8, 7_864_320, false, 15),
            (Oversampling::X16, 253_952, true, 28),
            (Oversampling::X32, 516_096, true, 54),
            (Oversampling::X64, 1_040_384, true, 105),
            (Oversampling::X128, 2_088_960, true, 207),
        ];

        for (setting, factor, shift, time_ms) in expected {
            assert_eq!(setting.scale_factor(), factor);
            assert_eq!(setting.requires_shift(), shift);
            assert_eq!(setting.measurement_time_ms(), time_ms);
        }
    }

    #[test]
    fn control_writes_encode_the_oversampling() {
        assert_eq!(
            Config::default().control_writes(),
            [
                (regs::PRS_CFG, 0b0011),
                (regs::TMP_CFG, regs::TMP_CFG_EXT | 0b0011),
                (regs::CFG_REG, 0),
            ]
        );

        let high = Config {
            pressure: Oversampling::X128,
            temperature: Oversampling::X16,
        };
        assert_eq!(
            high.control_writes(),
            [
                (regs::PRS_CFG, 0b0111),
                (regs::TMP_CFG, regs::TMP_CFG_EXT | 0b0100),
                (regs::CFG_REG, regs::PRS_SHIFT_EN | regs::TMP_SHIFT_EN),
            ]
        );
    }

    #[test]
    fn measurements_expose_both_integer_and_float_results() {
        let calib = CalibrationData::from_raw(&CALIBRATION_REAL);
        let measurements = compensate(0x300000, 0x100000, &Config::default(), &calib);

        assert_eq!(measurements.raw_pressure(), 0x300000);
        assert_eq!(measurements.raw_temperature(), 0x100000);
        assert_eq!(measurements.pressure, 57551);
        assert_eq!(measurements.temperature, 6683);

        assert_eq!(measurements.pressure_pa(), 57551.0);
        assert_eq!(measurements.pressure_hpa(), 575.51);
        assert_eq!(measurements.temperature_celsius(), 66.83);
    }

    #[test]
    fn constants_are_as_documented() {
        assert_eq!(ADDRESS, 0x77);
        assert_eq!(ADDRESS_ALT, 0x76);
        assert_eq!(PRODUCT_ID, 0x10);
        assert_eq!(SOFT_RESET_COMMAND, 0b1001);

        // Register addresses (datasheet section 7).
        assert_eq!(regs::PRS_B2, 0x00);
        assert_eq!(regs::PRS_B1, 0x01);
        assert_eq!(regs::PRS_B0, 0x02);
        assert_eq!(regs::TMP_B2, 0x03);
        assert_eq!(regs::TMP_B1, 0x04);
        assert_eq!(regs::TMP_B0, 0x05);
        assert_eq!(regs::PRS_CFG, 0x06);
        assert_eq!(regs::TMP_CFG, 0x07);
        assert_eq!(regs::MEAS_CFG, 0x08);
        assert_eq!(regs::CFG_REG, 0x09);
        assert_eq!(regs::INT_STS, 0x0A);
        assert_eq!(regs::FIFO_STS, 0x0B);
        assert_eq!(regs::RESET, 0x0C);
        assert_eq!(regs::ID, 0x0D);
        assert_eq!(regs::COEF, 0x10);
        assert_eq!(regs::CALIBRATION_LEN, 18);
        assert_eq!(regs::DATA_LEN, 3);

        // Register fields (datasheet section 8).
        assert_eq!(regs::PM_RATE_MASK, 0x70);
        assert_eq!(regs::PM_PRC_MASK, 0x0F);
        assert_eq!(regs::TMP_CFG_EXT, 0x80);
        assert_eq!(regs::TMP_RATE_MASK, 0x70);
        assert_eq!(regs::TMP_PRC_MASK, 0x07);
        assert_eq!(regs::COEF_RDY, 0x80);
        assert_eq!(regs::SENSOR_RDY, 0x40);
        assert_eq!(regs::TMP_RDY, 0x20);
        assert_eq!(regs::PRS_RDY, 0x10);
        assert_eq!(regs::MEAS_CTRL_MASK, 0x07);
        assert_eq!(regs::MEAS_PRESSURE, 0b001);
        assert_eq!(regs::MEAS_TEMPERATURE, 0b010);
        assert_eq!(regs::TMP_SHIFT_EN, 0x08);
        assert_eq!(regs::PRS_SHIFT_EN, 0x04);
        assert_eq!(regs::FIFO_EN, 0x02);
        assert_eq!(regs::SPI_MODE, 0x01);
        assert_eq!(regs::FIFO_FLUSH, 0x80);
        assert_eq!(regs::SOFT_RST_MASK, 0x0F);
        assert_eq!(regs::PROD_ID_MASK, 0xF0);
        assert_eq!(regs::REV_ID_MASK, 0x0F);

        assert_eq!(FRACTIONAL_BITS, 24);
    }
}
