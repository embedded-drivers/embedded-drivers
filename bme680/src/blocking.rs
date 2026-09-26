//! Blocking API.
//!
//! This mirrors [`crate::BME680`] exactly, but takes `embedded_hal`'s blocking
//! I2C and delay traits and has no `async` methods.

use embedded_hal::delay::DelayNs;

use crate::{
    ADDRESS, CHIP_ID, CalibrationData, Error, Measurements, calc_gas_wait, calc_res_heat, compensate_field, regs,
};

/// Blocking BME680 driver.
pub struct BME680<I2C: embedded_hal::i2c::I2c> {
    addr: u8,
    i2c: I2C,
    calibrated: bool,
    /// Ambient temperature used by [`BME680::set_gas_heater`]; defaults to
    /// [`crate::DEFAULT_AMBIENT_CELSIUS`].
    pub ambient_temperature_celsius: i8,
    /// Variant ID read from register `0xF0`: `0x00` is a BME680, `0x01` a
    /// BME688. Gas compensation is only valid for `0x00`.
    pub variant_id: u8,
    /// Calibration data read by [`BME680::init`].
    pub calib: CalibrationData,
    gas_heater_target_celsius: u16,
    gas_heater_duration_ms: u16,
}

impl<I2C: embedded_hal::i2c::I2c> BME680<I2C> {
    /// Create a driver for a sensor at `addr`.
    ///
    /// This does not touch the bus. Call [`BME680::init`] before measuring.
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            addr,
            i2c,
            calibrated: false,
            ambient_temperature_celsius: crate::DEFAULT_AMBIENT_CELSIUS,
            variant_id: 0,
            calib: CalibrationData::new(),
            gas_heater_target_celsius: crate::DEFAULT_HEATER_TARGET_CELSIUS,
            gas_heater_duration_ms: crate::DEFAULT_HEATER_DURATION_MS,
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
    /// `edrv-bme280`; `init` is safe without a preceding reset because it
    /// programs every control register explicitly.
    ///
    /// The control registers are written as one burst in Bosch's order,
    /// `CTRL_GAS_1 (0x71)` through `CONFIG (0x75)`, so `CTRL_HUM (0x72)` lands
    /// before `CTRL_MEAS (0x74)` and the humidity oversampling setting takes
    /// effect.
    ///
    /// Defaults: temperature oversampling x2, pressure x16, humidity x1, IIR
    /// filter size 3, gas heater enabled at 320 C for 150 ms. The part is left
    /// in sleep mode; use [`BME680::measure`] for a one-shot forced-mode
    /// measurement.
    pub fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        let chip_id = self.read_reg(regs::CHIP_ID)?;
        if chip_id != CHIP_ID {
            return Err(Error::InvalidDevice(chip_id));
        }

        self.variant_id = self.read_reg(regs::VARIANT_ID)?;

        self.read_calibration()?;
        self.configure()?;
        self.write_heater_config()?;

        Ok(())
    }

    /// Soft reset the sensor and wait for it to come back.
    ///
    /// Writes `0xB6` to register `0xE0` and waits 10 ms, which is Bosch's
    /// `BME68X_PERIOD_RESET` (10000 microseconds). The calibration data is kept,
    /// but the driver is marked uncalibrated so the next [`BME680::measure`]
    /// fails with [`Error::NotCalibrated`] until `init` runs again.
    pub fn reset(&mut self, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        self.write_reg(regs::SOFT_RESET, crate::SOFT_RESET_COMMAND)?;

        delay.delay_ms(10);

        self.calibrated = false;

        Ok(())
    }

    /// Read and store the calibration data. Also called by [`BME680::init`].
    fn read_calibration(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut raw = [0u8; regs::CALIBRATION_LEN];
        let split = regs::COEFF1_LEN + regs::COEFF2_LEN;
        self.read_regs(regs::COEFF1, &mut raw[..regs::COEFF1_LEN])?;
        self.read_regs(regs::COEFF2, &mut raw[regs::COEFF1_LEN..split])?;
        self.read_regs(regs::RES_HEAT_VAL, &mut raw[split..])?;

        self.calib = CalibrationData::from_raw(&raw);
        self.calibrated = true;

        Ok(())
    }

    /// Program the default oversampling, filter and gas settings.
    fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
        // Read and write the whole 0x71..=0x75 block, as Bosch's
        // `bme68x_set_conf` does.
        let mut conf = [0u8; 5];
        self.read_regs(regs::CTRL_GAS_1, &mut conf)?;
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

        self.write_config_block(&conf)?;

        // ctrl_gas_0 bit 3 low enables the heater.
        let ctrl_gas_0 = self.read_reg(regs::CTRL_GAS_0)?;
        self.write_reg(regs::CTRL_GAS_0, ctrl_gas_0 & !0x08)?;

        Ok(())
    }

    /// Write `res_heat0` and `gas_wait0` for the configured heater profile.
    fn write_heater_config(&mut self) -> Result<(), Error<I2C::Error>> {
        let res_heat = calc_res_heat(
            self.gas_heater_target_celsius,
            self.ambient_temperature_celsius,
            &self.calib,
        );
        let gas_wait = calc_gas_wait(self.gas_heater_duration_ms);

        self.write_reg(regs::RES_HEAT0, res_heat)?;
        self.write_reg(regs::GAS_WAIT0, gas_wait)?;

        Ok(())
    }

    /// Perform a single forced-mode measurement.
    ///
    /// Triggers one conversion and polls the measurement field for the new-data
    /// flag, sleeping 1 ms between polls. Returns [`Error::Timeout`] if no new
    /// data appears within the polling limit (roughly one second, far longer
    /// than the 150 ms gas heater plus a TPH conversion).
    ///
    /// [`Error::NotCalibrated`] is returned if [`BME680::init`] has not run (or
    /// [`BME680::reset`] has run since).
    pub fn measure(&mut self, mut delay: impl DelayNs) -> Result<Measurements, Error<I2C::Error>> {
        /// Number of 1 ms polls before giving up.
        const POLL_LIMIT: u32 = 1000;

        if !self.calibrated {
            return Err(Error::NotCalibrated);
        }

        // The heater profile can be changed between measurements, so re-apply it
        // while the part is still in sleep mode.
        self.write_heater_config()?;

        // Trigger a single forced-mode conversion.
        let ctrl_meas = self.read_reg(regs::CTRL_MEAS)?;
        self.write_reg(regs::CTRL_MEAS, (ctrl_meas & !0x03) | 0x01)?;

        let mut polls = 0;
        while self.read_reg(regs::FIELD0)? & regs::FIELD_NEW_DATA == 0 {
            polls += 1;
            if polls >= POLL_LIMIT {
                return Err(Error::Timeout);
            }
            delay.delay_ms(1);
        }

        let mut field = [0u8; regs::FIELD_LEN];
        self.read_regs(regs::FIELD0, &mut field)?;

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
    fn write_config_block(&mut self, conf: &[u8; 5]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(
            self.addr,
            &[regs::CTRL_GAS_1, conf[0], conf[1], conf[2], conf[3], conf[4]],
        )?;

        Ok(())
    }

    /// Read one register.
    pub fn read_reg(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    /// Read consecutive registers into `buf`.
    pub fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write_read(self.addr, &[reg], buf)?;
        Ok(())
    }

    /// Write one register.
    pub fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, &[reg, val])?;
        Ok(())
    }
}
