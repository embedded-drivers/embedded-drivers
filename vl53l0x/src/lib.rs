//! Driver for the VL53L0X time-of-flight distance sensor.
//!
//! The VL53L0X reports a distance in millimetres. The sensor uses 16-bit
//! register addresses on the wire, but every register used by this driver lives
//! below `0x100`, so the address is transmitted as a single byte exactly like
//! the proven reference driver this code was ported from.
//!
//! This module contains the asynchronous driver. See [`blocking`] for the
//! blocking variant.

#![cfg_attr(not(test), no_std)]

pub mod blocking;

/// Default (factory) 7-bit I2C address of the VL53L0X.
pub const PRIMARY_ADDRESS: u8 = 0x29;

/// Model ID reported by register `0xC0` on a VL53L0X.
pub const MODEL_ID_VL53L0X: u8 = 0xEE;

/// Model ID reported by register `0xC0` on a VL53L1X. The two parts are
/// pin-compatible but not register compatible, so this is worth failing on
/// loudly.
pub const MODEL_ID_VL53L1X: u8 = 0xEA;

/// Reading returned when the sensor detects no target, or the target is out of
/// range. It is the largest representable reading and is *not* a distance.
pub const OUT_OF_RANGE_MILLIMETERS: u16 = 8190;

/// VL53L0X register addresses.
///
/// All of these are below `0x100`, so they are written as a single address
/// byte.
pub mod regs {
    /// `SYSRANGE_START`
    pub const SYSRANGE_START: u8 = 0x00;
    /// `SYSTEM_SEQUENCE_CONFIG`
    pub const SYSTEM_SEQUENCE_CONFIG: u8 = 0x01;
    /// `SYSTEM_INTERMEASUREMENT_PERIOD`
    pub const SYSTEM_INTERMEASUREMENT_PERIOD: u8 = 0x04;
    /// `SYSTEM_INTERRUPT_CONFIG_GPIO`
    pub const SYSTEM_INTERRUPT_CONFIG_GPIO: u8 = 0x0A;
    /// `SYSTEM_INTERRUPT_CLEAR`
    pub const SYSTEM_INTERRUPT_CLEAR: u8 = 0x0B;
    /// `RESULT_INTERRUPT_STATUS`
    pub const RESULT_INTERRUPT_STATUS: u8 = 0x13;
    /// `RESULT_RANGE_STATUS`
    pub const RESULT_RANGE_STATUS: u8 = 0x14;
    /// `RESULT_RANGE_STATUS + 10`, the start of the 16-bit range result.
    pub const RESULT_RANGE_STATUS_PLUS_10: u8 = 0x1E;
    /// `CROSSTALK_COMPENSATION_PEAK_RATE_MCPS`
    pub const CROSSTALK_COMPENSATION_PEAK_RATE_MCPS: u8 = 0x20;
    /// `FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT`
    pub const FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT: u8 = 0x44;
    /// `MSRC_CONFIG_TIMEOUT_MACROP`
    pub const MSRC_CONFIG_TIMEOUT_MACROP: u8 = 0x46;
    /// `DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD`
    pub const DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD: u8 = 0x4E;
    /// `DYNAMIC_SPAD_REF_EN_START_OFFSET`
    pub const DYNAMIC_SPAD_REF_EN_START_OFFSET: u8 = 0x4F;
    /// `PRE_RANGE_CONFIG_VCSEL_PERIOD`
    pub const PRE_RANGE_CONFIG_VCSEL_PERIOD: u8 = 0x50;
    /// `PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI`
    pub const PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI: u8 = 0x51;
    /// `PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO`
    pub const PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO: u8 = 0x52;
    /// `MSRC_CONFIG_CONTROL`
    pub const MSRC_CONFIG_CONTROL: u8 = 0x60;
    /// `FINAL_RANGE_CONFIG_VCSEL_PERIOD`
    pub const FINAL_RANGE_CONFIG_VCSEL_PERIOD: u8 = 0x70;
    /// `FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI`
    pub const FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI: u8 = 0x71;
    /// `FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO`
    pub const FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO: u8 = 0x72;
    /// `GPIO_HV_MUX_ACTIVE_HIGH`
    pub const GPIO_HV_MUX_ACTIVE_HIGH: u8 = 0x84;
    /// `VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV`
    pub const VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV: u8 = 0x89;
    /// `I2C_SLAVE_DEVICE_ADDRESS`
    pub const I2C_SLAVE_DEVICE_ADDRESS: u8 = 0x8A;
    /// `GLOBAL_CONFIG_SPAD_ENABLES_REF_0`, the first of six bytes.
    pub const GLOBAL_CONFIG_SPAD_ENABLES_REF_0: u8 = 0xB0;
    /// `GLOBAL_CONFIG_REF_EN_START_SELECT`
    pub const GLOBAL_CONFIG_REF_EN_START_SELECT: u8 = 0xB6;
    /// `WHO_AM_I`, the model ID register.
    pub const WHO_AM_I: u8 = 0xC0;
    /// `OSC_CALIBRATE_VAL`
    pub const OSC_CALIBRATE_VAL: u8 = 0xF8;
}

/// Errors returned by the VL53L0X driver.
#[derive(Debug, Clone, Copy)]
pub enum Error<E> {
    /// Register `0xC0` did not read `0xEE`. The payload is the value that was
    /// read; `0xEA` means a VL53L1X, which is not register compatible.
    InvalidDevice(u8),
    /// Error from the underlying I2C bus.
    Bus(E),
    /// The sensor did not signal a result within the driver's polling limit.
    Timeout,
    /// The requested address is outside the usable 7-bit range `0x08..=0x77`.
    InvalidAddress(u8),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Bus(e)
    }
}

/// Which sequence steps are enabled in `SYSTEM_SEQUENCE_CONFIG`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SeqStepEnables {
    pub(crate) tcc: bool,
    pub(crate) dss: bool,
    pub(crate) msrc: bool,
    pub(crate) pre_range: bool,
    pub(crate) final_range: bool,
}

/// Decoded pre-range/final-range timeouts used by the timing-budget maths.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SeqStepTimeouts {
    pub(crate) final_range_vcsel_period_pclks: u8,
    pub(crate) pre_range_mclks: u16,
    pub(crate) msrc_dss_tcc_microseconds: u32,
    pub(crate) pre_range_microseconds: u32,
    pub(crate) final_range_microseconds: u32,
}

/// Selects the pre-range or final-range VCSEL period register.
#[derive(Debug, Clone, Copy)]
pub(crate) enum VcselPeriodType {
    PreRange,
    FinalRange,
}

/// `DefaultTuningSettings` from ST's `vl53l0x_tuning.h`, in write order.
///
/// The reference driver writes these inline during init. Keeping them in one
/// table means the async and blocking drivers cannot drift apart.
pub(crate) const DEFAULT_TUNING_SETTINGS: &[(u8, u8)] = &[
    (0xFF, 0x01),
    (0x00, 0x00),
    (0xFF, 0x00),
    (0x09, 0x00),
    (0x10, 0x00),
    (0x11, 0x00),
    (0x24, 0x01),
    (0x25, 0xFF),
    (0x75, 0x00),
    (0xFF, 0x01),
    (0x4E, 0x2C),
    (0x48, 0x00),
    (0x30, 0x20),
    (0xFF, 0x00),
    (0x30, 0x09),
    (0x54, 0x00),
    (0x31, 0x04),
    (0x32, 0x03),
    (0x40, 0x83),
    (0x46, 0x25),
    (0x60, 0x00),
    (0x27, 0x00),
    (0x50, 0x06),
    (0x51, 0x00),
    (0x52, 0x96),
    (0x56, 0x08),
    (0x57, 0x30),
    (0x61, 0x00),
    (0x62, 0x00),
    (0x64, 0x00),
    (0x65, 0x00),
    (0x66, 0xA0),
    (0xFF, 0x01),
    (0x22, 0x32),
    (0x47, 0x14),
    (0x49, 0xFF),
    (0x4A, 0x00),
    (0xFF, 0x00),
    (0x7A, 0x0A),
    (0x7B, 0x00),
    (0x78, 0x21),
    (0xFF, 0x01),
    (0x23, 0x34),
    (0x42, 0x00),
    (0x44, 0xFF),
    (0x45, 0x26),
    (0x46, 0x05),
    (0x40, 0x40),
    (0x0E, 0x06),
    (0x20, 0x1A),
    (0x43, 0x40),
    (0xFF, 0x00),
    (0x34, 0x03),
    (0x35, 0x44),
    (0xFF, 0x01),
    (0x31, 0x04),
    (0x4B, 0x09),
    (0x4C, 0x05),
    (0x4D, 0x04),
    (0xFF, 0x00),
    (0x44, 0x00),
    (0x45, 0x20),
    (0x47, 0x08),
    (0x48, 0x28),
    (0x67, 0x00),
    (0x70, 0x04),
    (0x71, 0x01),
    (0x72, 0xFE),
    (0x76, 0x00),
    (0x77, 0x00),
    (0xFF, 0x01),
    (0x0D, 0x01),
    (0xFF, 0x00),
    (0x80, 0x01),
    (0x01, 0xF8),
    (0xFF, 0x01),
    (0x8E, 0x01),
    (0x00, 0x01),
    (0xFF, 0x00),
    (0x80, 0x00),
];

/// Decode a `TIMEOUT_MACROP` register value. Format: `(LSByte * 2^MSByte) + 1`.
pub fn decode_timeout(register_value: u16) -> u16 {
    ((register_value & 0x00FF) << ((register_value & 0xFF00) >> 8)) + 1
}

/// Encode a timeout in macro periods into `TIMEOUT_MACROP` register format.
pub fn encode_timeout(timeout_mclks: u16) -> u16 {
    if timeout_mclks == 0 {
        return 0;
    }

    let mut ls_byte = (timeout_mclks as u32) - 1;
    let mut ms_byte: u16 = 0;

    while (ls_byte & 0xFFFFFF00) > 0 {
        ls_byte >>= 1;
        ms_byte += 1;
    }

    (ms_byte << 8) | ((ls_byte & 0xFF) as u16)
}

/// Macro period in nanoseconds for a VCSEL period given in PCLKs.
pub fn calc_macro_period(vcsel_period_pclks: u8) -> u32 {
    ((2304 * (vcsel_period_pclks as u32) * 1655) + 500) / 1000
}

/// Convert a timeout in macro periods to microseconds.
pub fn timeout_mclks_to_microseconds(timeout_period_mclks: u16, vcsel_period_pclks: u8) -> u32 {
    let macro_period_nanoseconds = calc_macro_period(vcsel_period_pclks);

    (((timeout_period_mclks as u32) * macro_period_nanoseconds) + (macro_period_nanoseconds / 2)) / 1000
}

/// Convert a timeout in microseconds to macro periods.
pub fn timeout_microseconds_to_mclks(timeout_period_microseconds: u32, vcsel_period_pclks: u8) -> u32 {
    let macro_period_nanoseconds = calc_macro_period(vcsel_period_pclks);

    ((timeout_period_microseconds * 1000) + (macro_period_nanoseconds / 2)) / macro_period_nanoseconds
}

/// Decode a VCSEL period in PCLKs from its register value.
pub const fn decode_vcsel_period(register_value: u8) -> u8 {
    (register_value + 1) << 1
}

/// Encode a VCSEL period in PCLKs into its register value.
pub const fn encode_vcsel_period(period_pclks: u8) -> u8 {
    (period_pclks >> 1) - 1
}

pub struct VL53L0X<I2C: embedded_hal_async::i2c::I2c> {
    i2c: I2C,
    addr: u8,
    io_mode2v8: bool,
    stop_variable: u8,
    measurement_timing_budget_microseconds: u32,
}

impl<I2C: embedded_hal_async::i2c::I2c> VL53L0X<I2C> {
    /// Create a driver for a sensor at `addr`.
    ///
    /// This does not touch the bus. Call [`VL53L0X::init`] before ranging.
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            io_mode2v8: true,
            stop_variable: 0,
            measurement_timing_budget_microseconds: 0,
        }
    }

    /// Create a driver for a sensor at the default address.
    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, PRIMARY_ADDRESS)
    }

    /// Validate the model ID and run the full ST initialisation sequence.
    ///
    /// Returns [`Error::InvalidDevice`] with the value read from register
    /// `0xC0` if the part is not a VL53L0X (`0xEE`); a VL53L1X answers `0xEA`.
    pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        let model_id = self.read_reg(regs::WHO_AM_I).await?;
        if model_id != MODEL_ID_VL53L0X {
            return Err(Error::InvalidDevice(model_id));
        }

        self.init_hardware().await
    }

    /// Change the I2C address of the sensor.
    ///
    /// The address resets when the device is powered off. Only `0x08..=0x77` is
    /// accepted; `0x00..=0x07` and `0x78..=0x7F` are reserved 7-bit addresses.
    pub async fn set_address(&mut self, new_address: u8) -> Result<(), Error<I2C::Error>> {
        if !(0x08..=0x77).contains(&new_address) {
            return Err(Error::InvalidAddress(new_address));
        }
        self.write_reg(regs::I2C_SLAVE_DEVICE_ADDRESS, new_address).await?;
        self.addr = new_address;

        Ok(())
    }

    /// Read the model ID register (`0xC0`).
    pub async fn who_am_i(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_reg(regs::WHO_AM_I).await
    }

    /// Start continuous ranging.
    ///
    /// `period_millis` of 0 selects back-to-back mode; any other value selects
    /// timed mode with that inter-measurement period.
    pub async fn start_continuous(&mut self, period_millis: u32) -> Result<(), Error<I2C::Error>> {
        self.write_reg(0x80, 0x01).await?;
        self.write_reg(0xFF, 0x01).await?;
        self.write_reg(0x00, 0x00).await?;
        let sv = self.stop_variable;
        self.write_reg(0x91, sv).await?;
        self.write_reg(0x00, 0x01).await?;
        self.write_reg(0xFF, 0x00).await?;
        self.write_reg(0x80, 0x00).await?;

        let mut period_millis = period_millis;
        if period_millis != 0 {
            // Continuous timed mode.
            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
            let osc_calibrate_value = self.read_reg16(regs::OSC_CALIBRATE_VAL).await?;

            if osc_calibrate_value != 0 {
                period_millis *= osc_calibrate_value as u32;
            }

            self.write_reg32(regs::SYSTEM_INTERMEASUREMENT_PERIOD, period_millis)
                .await?;
            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end
            // VL53L0X_REG_SYSRANGE_MODE_TIMED
            self.write_reg(regs::SYSRANGE_START, 0x04).await?;
        } else {
            // Continuous back-to-back mode.
            // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
            self.write_reg(regs::SYSRANGE_START, 0x02).await?;
        }

        Ok(())
    }

    /// Stop continuous ranging.
    pub async fn stop_continuous(&mut self) -> Result<(), Error<I2C::Error>> {
        // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
        self.write_reg(regs::SYSRANGE_START, 0x01).await?;
        self.write_reg(0xFF, 0x01).await?;
        self.write_reg(0x00, 0x00).await?;
        self.write_reg(0x91, 0x00).await?;
        self.write_reg(0x00, 0x01).await?;
        self.write_reg(0xFF, 0x00).await?;

        Ok(())
    }

    /// Perform a single-shot measurement and return the distance in
    /// millimetres.
    ///
    /// A return value of [`OUT_OF_RANGE_MILLIMETERS`] (8190) means no target was
    /// detected or the target is out of range; it is not a distance.
    pub async fn read_range_single_millimeters(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.write_reg(0x80, 0x01).await?;
        self.write_reg(0xFF, 0x01).await?;
        self.write_reg(0x00, 0x00).await?;
        let sv = self.stop_variable;
        self.write_reg(0x91, sv).await?;
        self.write_reg(0x00, 0x01).await?;
        self.write_reg(0xFF, 0x00).await?;
        self.write_reg(0x80, 0x00).await?;

        self.write_reg(regs::SYSRANGE_START, 0x01).await?;

        // "Wait until start bit has been cleared"
        let mut c = 0;
        while (self.read_reg(regs::SYSRANGE_START).await? & 0x01) != 0 {
            c += 1;
            if c == 10000 {
                return Err(Error::Timeout);
            }
        }

        self.read_range_continuous_millimeters().await
    }

    /// Read the result of the current continuous measurement, in millimetres.
    ///
    /// A return value of [`OUT_OF_RANGE_MILLIMETERS`] (8190) means no target was
    /// detected or the target is out of range; it is not a distance.
    pub async fn read_range_continuous_millimeters(&mut self) -> Result<u16, Error<I2C::Error>> {
        let mut c = 0;
        while (self.read_reg(regs::RESULT_INTERRUPT_STATUS).await? & 0x07) == 0 {
            c += 1;
            if c == 10000 {
                return Err(Error::Timeout);
            }
        }

        let range = self.read_reg16(regs::RESULT_RANGE_STATUS_PLUS_10).await?;
        // Clear the interrupt even if the result read above failed.
        self.write_reg(regs::SYSTEM_INTERRUPT_CLEAR, 0x01).await?;

        Ok(range)
    }

    /// Get the measurement timing budget in microseconds.
    pub async fn get_measurement_timing_budget(&mut self) -> Result<u32, Error<I2C::Error>> {
        let start_overhead: u32 = 1910;
        let end_overhead: u32 = 960;
        let msrc_overhead: u32 = 660;
        let tcc_overhead: u32 = 590;
        let dss_overhead: u32 = 690;
        let pre_range_overhead: u32 = 660;
        let final_range_overhead: u32 = 550;

        let enables = self.get_sequence_step_enables().await?;
        let timeouts = self.get_sequence_step_timeouts(&enables).await?;

        // "Start and end overhead times always present"
        let mut budget_microseconds = start_overhead + end_overhead;
        if enables.tcc {
            budget_microseconds += timeouts.msrc_dss_tcc_microseconds + tcc_overhead;
        }
        if enables.dss {
            budget_microseconds += 2 * (timeouts.msrc_dss_tcc_microseconds + dss_overhead);
        } else if enables.msrc {
            budget_microseconds += timeouts.msrc_dss_tcc_microseconds + msrc_overhead;
        }
        if enables.pre_range {
            budget_microseconds += timeouts.pre_range_microseconds + pre_range_overhead;
        }
        if enables.final_range {
            budget_microseconds += timeouts.final_range_microseconds + final_range_overhead;
        }

        Ok(budget_microseconds)
    }

    /// Set the measurement timing budget in microseconds.
    ///
    /// Returns `Ok(false)` if the requested budget is below the 20 ms minimum
    /// or too small for the currently enabled sequence steps.
    pub async fn set_measurement_timing_budget(&mut self, budget_microseconds: u32) -> Result<bool, Error<I2C::Error>> {
        // Note that these overheads differ from the ones in get_.
        let start_overhead: u32 = 1320;
        let end_overhead: u32 = 960;
        let msrc_overhead: u32 = 660;
        let tcc_overhead: u32 = 590;
        let dss_overhead: u32 = 690;
        let pre_range_overhead: u32 = 660;
        let final_range_overhead: u32 = 550;
        let min_timing_budget: u32 = 20000;

        if budget_microseconds < min_timing_budget {
            return Ok(false);
        }

        let enables = self.get_sequence_step_enables().await?;
        let timeouts = self.get_sequence_step_timeouts(&enables).await?;

        let mut use_budget_microseconds: u32 = start_overhead + end_overhead;
        if enables.tcc {
            use_budget_microseconds += timeouts.msrc_dss_tcc_microseconds + tcc_overhead;
        }
        if enables.dss {
            use_budget_microseconds += 2 * timeouts.msrc_dss_tcc_microseconds + dss_overhead;
        } else if enables.msrc {
            use_budget_microseconds += timeouts.msrc_dss_tcc_microseconds + msrc_overhead;
        }
        if enables.pre_range {
            use_budget_microseconds += timeouts.pre_range_microseconds + pre_range_overhead;
        }
        if enables.final_range {
            use_budget_microseconds += final_range_overhead;
        }

        // "Note that the final range timeout is determined by the timing budget
        // and the sum of all other timeouts within the sequence. If there is no
        // room for the final range timeout, then an error will be set.
        // Otherwise the remaining time will be applied to the final range."

        if use_budget_microseconds > budget_microseconds {
            // "Requested timeout too small."
            return Ok(false);
        }

        let final_range_timeout_microseconds: u32 = budget_microseconds - use_budget_microseconds;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)
        // "For the final range timeout, the pre-range timeout must be added. To
        // do this both final and pre-range timeouts must be expressed in macro
        // periods MClks because they have different vcsel periods."
        let mut final_range_timeout_mclks: u16 = timeout_microseconds_to_mclks(
            final_range_timeout_microseconds,
            timeouts.final_range_vcsel_period_pclks,
        ) as u16;

        if enables.pre_range {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        self.write_reg16(
            regs::FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
            encode_timeout(final_range_timeout_mclks),
        )
        .await?;

        // set_sequence_step_timeout() end
        // Store for internal reuse.
        self.measurement_timing_budget_microseconds = budget_microseconds;
        Ok(true)
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
    pub async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, &[reg, value]).await?;
        Ok(())
    }

    async fn read_reg16(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.read_regs(reg, &mut buf).await?;
        Ok(u16::from_be_bytes([buf[0], buf[1]]))
    }

    async fn write_reg16(&mut self, reg: u8, word: u16) -> Result<(), Error<I2C::Error>> {
        let [hi, lo] = word.to_be_bytes();
        self.i2c.write(self.addr, &[reg, hi, lo]).await?;
        Ok(())
    }

    async fn write_reg32(&mut self, reg: u8, word: u32) -> Result<(), Error<I2C::Error>> {
        // Big-endian, matching ST's `VL53L0X_WrDWord`, Pololu's `writeReg32Bit`
        // and every other multi-byte value in this driver.
        //
        // NOTE: the `vl53l0x` crate this was ported from serialises this one
        // value the other way round, which contradicts its own big-endian
        // register reads. The effect is limited to `start_continuous` with a
        // non-zero period: the common back-to-back case writes all zeros, where
        // the byte order cannot matter, which is presumably why it went
        // unnoticed there.
        let bytes = word.to_be_bytes();
        self.i2c
            .write(self.addr, &[reg, bytes[0], bytes[1], bytes[2], bytes[3]])
            .await?;
        Ok(())
    }

    async fn set_signal_rate_limit(&mut self, limit: f32) -> Result<bool, Error<I2C::Error>> {
        if !(0.0..=511.99).contains(&limit) {
            Ok(false)
        } else {
            // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
            self.write_reg16(
                regs::FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                (limit * ((1 << 7) as f32)) as u16,
            )
            .await?;
            Ok(true)
        }
    }

    async fn get_spad_info(&mut self) -> Result<(u8, u8), Error<I2C::Error>> {
        self.write_reg(0x80, 0x01).await?;
        self.write_reg(0xFF, 0x01).await?;
        self.write_reg(0x00, 0x00).await?;

        self.write_reg(0xFF, 0x06).await?;
        let mut tmp83 = self.read_reg(0x83).await?;
        self.write_reg(0x83, tmp83 | 0x04).await?;
        self.write_reg(0xFF, 0x07).await?;
        self.write_reg(0x81, 0x01).await?;

        self.write_reg(0x80, 0x01).await?;

        self.write_reg(0x94, 0x6b).await?;
        self.write_reg(0x83, 0x00).await?;

        let mut c = 0;
        while self.read_reg(0x83).await? == 0x00 {
            c += 1;
            if c == 65535 {
                return Err(Error::Timeout);
            }
        }

        self.write_reg(0x83, 0x01).await?;
        let tmp = self.read_reg(0x92).await?;

        let count: u8 = tmp & 0x7f;
        let type_is_aperture: u8 = (tmp >> 7) & 0x01;

        self.write_reg(0x81, 0x00).await?;
        self.write_reg(0xFF, 0x06).await?;
        tmp83 = self.read_reg(0x83).await?;
        self.write_reg(0x83, tmp83 & !0x04).await?;
        self.write_reg(0xFF, 0x01).await?;
        self.write_reg(0x00, 0x01).await?;

        self.write_reg(0xFF, 0x00).await?;
        self.write_reg(0x80, 0x00).await?;

        Ok((count, type_is_aperture))
    }

    /// `performSingleRefCalibration(uint8_t vhvInitByte)`
    async fn perform_single_ref_calibration(&mut self, vhv_init_byte: u8) -> Result<(), Error<I2C::Error>> {
        // VL53L0X_REG_SYSRANGE_MODE_START_STOP
        self.write_reg(regs::SYSRANGE_START, 0x01 | vhv_init_byte).await?;

        let mut c = 0;
        while (self.read_reg(regs::RESULT_INTERRUPT_STATUS).await? & 0x07) == 0 {
            c += 1;
            if c == 10000 {
                return Err(Error::Timeout);
            }
        }
        self.write_reg(regs::SYSTEM_INTERRUPT_CLEAR, 0x01).await?;
        self.write_reg(regs::SYSRANGE_START, 0x00).await?;

        Ok(())
    }

    async fn get_vcsel_pulse_period(&mut self, ty: VcselPeriodType) -> Result<u8, Error<I2C::Error>> {
        match ty {
            VcselPeriodType::PreRange => Ok(decode_vcsel_period(
                self.read_reg(regs::PRE_RANGE_CONFIG_VCSEL_PERIOD).await?,
            )),
            VcselPeriodType::FinalRange => Ok(decode_vcsel_period(
                self.read_reg(regs::FINAL_RANGE_CONFIG_VCSEL_PERIOD).await?,
            )),
        }
    }

    /// `getSequenceStepEnables(VL53L0XSequenceStepEnables* enables)`
    async fn get_sequence_step_enables(&mut self) -> Result<SeqStepEnables, Error<I2C::Error>> {
        let sequence_config = self.read_reg(regs::SYSTEM_SEQUENCE_CONFIG).await?;
        Ok(SeqStepEnables {
            tcc: (sequence_config & 0x10) != 0,
            dss: (sequence_config & 0x08) != 0,
            msrc: (sequence_config & 0x04) != 0,
            pre_range: (sequence_config & 0x40) != 0,
            final_range: (sequence_config & 0x80) != 0,
        })
    }

    /// `getSequenceStepTimeouts(timeouts)`
    async fn get_sequence_step_timeouts(
        &mut self,
        enables: &SeqStepEnables,
    ) -> Result<SeqStepTimeouts, Error<I2C::Error>> {
        let pre_range_mclks = decode_timeout(self.read_reg16(regs::PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI).await?);
        let mut final_range_mclks = decode_timeout(self.read_reg16(regs::FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI).await?);
        if enables.pre_range {
            final_range_mclks -= pre_range_mclks;
        }

        let pre_range_vcselperiod_pclks = self.get_vcsel_pulse_period(VcselPeriodType::PreRange).await?;
        let msrc_dss_tcc_mclks = self.read_reg(regs::MSRC_CONFIG_TIMEOUT_MACROP).await? + 1;
        let final_range_vcsel_period_pclks = self.get_vcsel_pulse_period(VcselPeriodType::FinalRange).await?;

        Ok(SeqStepTimeouts {
            msrc_dss_tcc_microseconds: timeout_mclks_to_microseconds(
                msrc_dss_tcc_mclks as u16,
                pre_range_vcselperiod_pclks,
            ),
            pre_range_microseconds: timeout_mclks_to_microseconds(pre_range_mclks, pre_range_vcselperiod_pclks),
            final_range_vcsel_period_pclks,
            pre_range_mclks,
            final_range_microseconds: timeout_mclks_to_microseconds(final_range_mclks, final_range_vcsel_period_pclks),
        })
    }

    async fn load_tuning_settings(&mut self) -> Result<(), Error<I2C::Error>> {
        // -- VL53L0X_load_tuning_settings() begin
        for (reg, value) in DEFAULT_TUNING_SETTINGS {
            self.write_reg(*reg, *value).await?;
        }

        Ok(())
        // -- VL53L0X_load_tuning_settings() end
    }

    async fn power_on(&mut self) -> Result<(), Error<I2C::Error>> {
        // TODO: drive XSHUT to power the sensor on/off.
        Ok(())
    }

    async fn init_hardware(&mut self) -> Result<(), Error<I2C::Error>> {
        self.power_on().await?;

        // VL53L0X_DataInit() begin

        // The sensor uses 1V8 mode for I/O by default; switch to 2V8 mode.
        if self.io_mode2v8 {
            // set bit 0
            let ext_sup_hv = self.read_reg(regs::VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV).await?;
            self.write_reg(regs::VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, ext_sup_hv | 0x01)
                .await?;
        }

        // "Set I2C standard mode"
        self.write_reg(0x88, 0x00).await?;
        self.write_reg(0x80, 0x01).await?;
        self.write_reg(0xFF, 0x01).await?;
        self.write_reg(0x00, 0x00).await?;
        self.stop_variable = self.read_reg(0x91).await?;
        self.write_reg(0x00, 0x01).await?;
        self.write_reg(0xFF, 0x00).await?;
        self.write_reg(0x80, 0x00).await?;

        // Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4)
        // limit checks.
        let config = self.read_reg(regs::MSRC_CONFIG_CONTROL).await?;
        self.write_reg(regs::MSRC_CONFIG_CONTROL, config | 0x12).await?;

        // Set the final range signal rate limit to 0.25 MCPS.
        self.set_signal_rate_limit(0.25).await?;

        self.write_reg(regs::SYSTEM_SEQUENCE_CONFIG, 0xFF).await?;

        // VL53L0X_DataInit() end

        // VL53L0X_StaticInit() begin

        let (spad_count, spad_type_is_aperture) = self.get_spad_info().await?;

        // The SPAD map (RefGoodSpadMap) is read by
        // VL53L0X_get_info_from_device() in the API, but the same data is more
        // easily readable from GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so
        // read it from there.
        let mut ref_spad_map = [0u8; 6];
        self.read_regs(regs::GLOBAL_CONFIG_SPAD_ENABLES_REF_0, &mut ref_spad_map)
            .await?;

        // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

        self.write_reg(0xFF, 0x01).await?;
        self.write_reg(regs::DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00).await?;
        self.write_reg(regs::DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C).await?;
        self.write_reg(0xFF, 0x00).await?;
        self.write_reg(regs::GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4).await?;

        // 12 is the first aperture SPAD.
        let first_spad_to_enable = if spad_type_is_aperture != 0 { 12 } else { 0 };
        let mut spads_enabled: u8 = 0;

        for i in 0..48u8 {
            if i < first_spad_to_enable || spads_enabled == spad_count {
                // This bit is lower than the first one that should be enabled,
                // or spad_count bits have already been enabled, so zero it.
                ref_spad_map[(i / 8) as usize] &= !(1 << (i % 8));
            } else if (ref_spad_map[(i / 8) as usize] >> (i % 8)) & 0x1 > 0 {
                spads_enabled += 1;
            }
        }

        // The six SPAD enable bytes are written in one auto-incrementing
        // transaction, with the address as a single byte like every other
        // register access.
        let mut spad_write = [0u8; 7];
        spad_write[0] = regs::GLOBAL_CONFIG_SPAD_ENABLES_REF_0;
        spad_write[1..].copy_from_slice(&ref_spad_map);
        self.i2c.write(self.addr, &spad_write).await?;

        // -- VL53L0X_set_reference_spads() end

        self.load_tuning_settings().await?;

        // "Set interrupt config to new sample ready"
        // -- VL53L0X_SetGpioConfig() begin

        self.write_reg(regs::SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04).await?;
        // active low
        let high = self.read_reg(regs::GPIO_HV_MUX_ACTIVE_HIGH).await?;
        self.write_reg(regs::GPIO_HV_MUX_ACTIVE_HIGH, high & !0x10).await?;
        self.write_reg(regs::SYSTEM_INTERRUPT_CLEAR, 0x01).await?;

        // -- VL53L0X_SetGpioConfig() end
        // "Disable MSRC and TCC by default"
        // MSRC = Minimum Signal Rate Check
        // TCC = Target Centre Check
        // -- VL53L0X_SetSequenceStepEnable() begin
        self.measurement_timing_budget_microseconds = self.get_measurement_timing_budget().await?;
        self.write_reg(regs::SYSTEM_SEQUENCE_CONFIG, 0xE8).await?;

        // -- VL53L0X_SetSequenceStepEnable() end

        // "Recalculate timing budget"
        let mtbm = self.measurement_timing_budget_microseconds;
        self.set_measurement_timing_budget(mtbm).await?;

        // VL53L0X_StaticInit() end

        // VL53L0X_PerformRefCalibration() begin

        // -- VL53L0X_perform_vhv_calibration() begin
        self.write_reg(regs::SYSTEM_SEQUENCE_CONFIG, 0x01).await?;
        self.perform_single_ref_calibration(0x40).await?;
        // -- VL53L0X_perform_vhv_calibration() end
        // -- VL53L0X_perform_phase_calibration() begin

        self.write_reg(regs::SYSTEM_SEQUENCE_CONFIG, 0x02).await?;
        self.perform_single_ref_calibration(0x00).await?;

        // -- VL53L0X_perform_phase_calibration() end

        // "restore the previous Sequence Config"
        self.write_reg(regs::SYSTEM_SEQUENCE_CONFIG, 0xE8).await?;

        // VL53L0X_PerformRefCalibration() end

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timeout_codec_matches_st_formula() {
        // format: (LSByte * 2^MSByte) + 1
        assert_eq!(decode_timeout(0x0000), 1);
        assert_eq!(decode_timeout(0x0063), 100);
        assert_eq!(decode_timeout(0x0195), 299);
        assert_eq!(encode_timeout(0), 0);
        assert_eq!(encode_timeout(100), 0x0063);
        assert_eq!(encode_timeout(300), 0x0195);
    }

    #[test]
    fn vcsel_period_codec_round_trips() {
        for pclks in [14u8, 18, 38] {
            assert_eq!(decode_vcsel_period(encode_vcsel_period(pclks)), pclks);
        }
        assert_eq!(decode_vcsel_period(0x12), 38);
        assert_eq!(encode_vcsel_period(38), 0x12);
    }

    #[test]
    fn macro_period_matches_st_formula() {
        // (2304 * pclks * 1655 + 500) / 1000
        assert_eq!(calc_macro_period(12), 45757);
    }

    #[test]
    fn constants_are_as_documented() {
        assert_eq!(PRIMARY_ADDRESS, 0x29);
        assert_eq!(MODEL_ID_VL53L0X, 0xEE);
        assert_ne!(MODEL_ID_VL53L0X, MODEL_ID_VL53L1X);
        assert_eq!(OUT_OF_RANGE_MILLIMETERS, 8190);
    }
}
