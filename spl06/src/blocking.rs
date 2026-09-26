//! Blocking API.
//!
//! This mirrors [`crate::SPL06`] exactly, but takes `embedded_hal`'s blocking
//! I2C and delay traits and has no `async` methods.

use embedded_hal::delay::DelayNs;

use crate::{
    ADDRESS, CalibrationData, Config, Error, Measurements, Oversampling, PRODUCT_ID, SOFT_RESET_COMMAND,
    STARTUP_TIME_MS, compensate, decode_i24, regs,
};

/// Blocking SPL06-001 / SPL06-007 driver.
pub struct SPL06<I2C: embedded_hal::i2c::I2c> {
    addr: u8,
    i2c: I2C,
    calibrated: bool,
    /// Oversampling configuration used by [`SPL06::configure`] and
    /// [`SPL06::measure`]. Starts at [`Config::default`].
    pub config: Config,
    /// Calibration data read by [`SPL06::init`].
    pub calib: CalibrationData,
}

impl<I2C: embedded_hal::i2c::I2c> SPL06<I2C> {
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
    /// about it. `init` therefore returns [`Error::Timeout`] if the flags do not
    /// appear within twice `TCoef_rdy`, rather than accepting a blank block.
    pub fn init(&mut self, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        let id = self.read_reg(regs::ID)?;
        if id != PRODUCT_ID {
            return Err(Error::InvalidDevice(id));
        }

        self.wait_for_startup(&mut delay)?;

        self.read_calibration()?;
        self.configure()?;

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
    pub fn reset(&mut self, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        self.write_reg(regs::RESET, SOFT_RESET_COMMAND)?;

        delay.delay_ms(STARTUP_TIME_MS);

        self.calibrated = false;

        Ok(())
    }

    /// Write the current [`SPL06::config`] to the sensor.
    ///
    /// The writes and their order come from [`Config::control_writes`]; see
    /// that method for why the order matters.
    pub fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
        for (reg, value) in self.config.control_writes() {
            self.write_reg(reg, value)?;
        }

        Ok(())
    }

    /// Change the oversampling and write the new configuration immediately.
    pub fn set_oversampling(
        &mut self,
        pressure: Oversampling,
        temperature: Oversampling,
    ) -> Result<(), Error<I2C::Error>> {
        self.config = Config { pressure, temperature };

        self.configure()
    }

    /// Read and store the calibration block. Also called by [`SPL06::init`].
    pub fn read_calibration(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut raw = [0u8; regs::CALIBRATION_LEN];
        self.read_regs(regs::COEF, &mut raw)?;

        self.calib = CalibrationData::from_raw(&raw);
        self.calibrated = true;

        Ok(())
    }

    /// Trigger a single temperature conversion and return the raw 24-bit value.
    pub fn read_raw_temperature(&mut self, mut delay: impl DelayNs) -> Result<i32, Error<I2C::Error>> {
        self.trigger(regs::MEAS_TEMPERATURE)?;
        self.wait_for_flag(regs::TMP_RDY, self.config.temperature.measurement_time_ms(), &mut delay)?;

        let mut raw = [0u8; regs::DATA_LEN];
        self.read_regs(regs::TMP_B2, &mut raw)?;

        Ok(decode_i24(&raw))
    }

    /// Trigger a single pressure conversion and return the raw 24-bit value.
    pub fn read_raw_pressure(&mut self, mut delay: impl DelayNs) -> Result<i32, Error<I2C::Error>> {
        self.trigger(regs::MEAS_PRESSURE)?;
        self.wait_for_flag(regs::PRS_RDY, self.config.pressure.measurement_time_ms(), &mut delay)?;

        let mut raw = [0u8; regs::DATA_LEN];
        self.read_regs(regs::PRS_B2, &mut raw)?;

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
    pub fn measure(&mut self, mut delay: impl DelayNs) -> Result<Measurements, Error<I2C::Error>> {
        if !self.calibrated {
            return Err(Error::NotCalibrated);
        }

        let raw_temperature = self.read_raw_temperature(&mut delay)?;
        let raw_pressure = self.read_raw_pressure(&mut delay)?;

        Ok(compensate(raw_pressure, raw_temperature, &self.config, &self.calib))
    }

    /// Start one command-mode conversion.
    fn trigger(&mut self, command: u8) -> Result<(), Error<I2C::Error>> {
        self.write_reg(regs::MEAS_CFG, command)
    }

    /// Poll [`regs::MEAS_CFG`] until `flag` is set.
    ///
    /// Gives up after twice `worst_case_ms` plus 10 ms, sleeping 1 ms between
    /// polls.
    fn wait_for_flag<D: DelayNs>(
        &mut self,
        flag: u8,
        worst_case_ms: u32,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        let limit = worst_case_ms * 2 + 10;
        let mut polls = 0;

        while self.read_reg(regs::MEAS_CFG)? & flag == 0 {
            polls += 1;
            if polls >= limit {
                return Err(Error::Timeout);
            }
            delay.delay_ms(1);
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
    fn wait_for_startup(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        self.wait_for_flag(regs::COEF_RDY, STARTUP_TIME_MS, delay)
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

#[cfg(test)]
mod tests {
    use core::cell::{Cell, RefCell};
    use std::vec::Vec;

    use embedded_hal::i2c::ErrorKind;

    use super::*;
    use crate::regs;

    #[derive(Debug)]
    struct MockError;

    impl embedded_hal::i2c::Error for MockError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    /// A fake SPL06 that records every register write and answers reads from a
    /// programmable register image, so `init` / `measure` can run without
    /// hardware.
    struct FakeSpl06 {
        writes: RefCell<Vec<(u8, u8)>>,
        reads: RefCell<Vec<u8>>,
        /// Value returned for [`regs::MEAS_CFG`], once start-up has finished.
        status: u8,
        /// Number of `MEAS_CFG` reads that still report [`regs::COEF_RDY`] as
        /// clear, used to model the roughly 40 ms a real part needs before its
        /// coefficients are valid. Zero by default.
        startup_polls_left: Cell<u32>,
        calibration: [u8; regs::CALIBRATION_LEN],
        pressure: [u8; regs::DATA_LEN],
        temperature: [u8; regs::DATA_LEN],
    }

    impl Default for FakeSpl06 {
        fn default() -> Self {
            Self {
                writes: RefCell::new(Vec::new()),
                reads: RefCell::new(Vec::new()),
                // Coefficients and raw values from the cross-check test vector.
                status: regs::COEF_RDY | regs::SENSOR_RDY | regs::TMP_RDY | regs::PRS_RDY,
                startup_polls_left: Cell::new(0),
                calibration: [
                    0x0c, 0xbe, 0xfc, 0x13, 0xd9, 0xaf, 0x2b, 0x34, 0xf3, 0xf7, 0x04, 0xff, 0xda, 0x5a, 0x00, 0x0a,
                    0xfb, 0x1b,
                ],
                pressure: [0x30, 0x00, 0x00],
                temperature: [0x10, 0x00, 0x00],
            }
        }
    }

    impl embedded_hal::i2c::ErrorType for FakeSpl06 {
        type Error = MockError;
    }

    impl embedded_hal::i2c::I2c for FakeSpl06 {
        fn read(&mut self, _address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
            read.fill(0);
            Ok(())
        }

        fn write(&mut self, _address: u8, write: &[u8]) -> Result<(), Self::Error> {
            if let [reg, value] = write {
                self.writes.borrow_mut().push((*reg, *value));
            }
            Ok(())
        }

        fn write_read(&mut self, _address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
            let reg = write[0];
            self.reads.borrow_mut().push(reg);
            read.fill(0);

            match reg {
                regs::ID => read[0] = PRODUCT_ID,
                regs::MEAS_CFG => {
                    // Model the ~40 ms window before the coefficients are valid:
                    // report `COEF_RDY` as clear for the first few reads.
                    let left = self.startup_polls_left.get();
                    if left > 0 {
                        self.startup_polls_left.set(left - 1);
                        read[0] = self.status & !regs::COEF_RDY;
                    } else {
                        read[0] = self.status;
                    }
                }
                regs::COEF => read.copy_from_slice(&self.calibration),
                regs::PRS_B2 => read[..regs::DATA_LEN].copy_from_slice(&self.pressure),
                regs::TMP_B2 => read[..regs::DATA_LEN].copy_from_slice(&self.temperature),
                _ => {}
            }

            Ok(())
        }

        fn transaction(
            &mut self,
            _address: u8,
            _operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            unimplemented!("the driver does not use transactions")
        }
    }

    /// A delay that does not actually sleep.
    struct NoDelay;

    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    /// `init` must issue exactly the writes [`Config::control_writes`]
    /// advertises, in that order.
    ///
    /// The order is a hardware requirement: `PRS_CFG` and `TMP_CFG` select the
    /// oversampling and `CFG_REG` carries the result bit-shift that is
    /// mandatory for any oversampling above 8 times. Writing `CFG_REG` first
    /// would be harmless, but splitting the sequence across two call sites (as
    /// this driver used to do) means the shift and the oversampling can drift
    /// apart - and a missing shift silently scales the pressure by 15.
    #[test]
    fn configuration_is_written_in_the_documented_order() {
        let mut sensor = SPL06::new(FakeSpl06::default(), ADDRESS);
        sensor.init(&mut NoDelay).expect("init should succeed against the fake");

        let writes = sensor.i2c.writes.borrow().clone();

        assert_eq!(writes, Config::default().control_writes().to_vec());
        assert_eq!(
            writes.iter().map(|(reg, _)| *reg).collect::<Vec<_>>(),
            vec![regs::PRS_CFG, regs::TMP_CFG, regs::CFG_REG]
        );

        // `init` reads the product ID, waits for the start-up flags, and only
        // then reads the calibration block.
        assert_eq!(
            sensor.i2c.reads.borrow().clone(),
            vec![regs::ID, regs::MEAS_CFG, regs::COEF]
        );
    }

    #[test]
    fn default_oversampling_needs_no_result_shift() {
        let mut sensor = SPL06::new(FakeSpl06::default(), ADDRESS);
        sensor.init(&mut NoDelay).expect("init should succeed against the fake");

        let writes = sensor.i2c.writes.borrow().clone();
        let (_, cfg_reg) = writes
            .iter()
            .find(|(reg, _)| *reg == regs::CFG_REG)
            .expect("CFG_REG must be written");

        // Bits 3:2 of CFG_REG are T_SHIFT and P_SHIFT; neither is needed at 8x.
        assert_eq!(*cfg_reg & 0x0C, 0);
    }

    #[test]
    fn oversampling_above_eight_enables_both_result_shifts() {
        let mut sensor = SPL06::new(FakeSpl06::default(), ADDRESS);
        sensor
            .set_oversampling(Oversampling::X128, Oversampling::X16)
            .expect("configuration should succeed");

        let writes = sensor.i2c.writes.borrow().clone();
        let (_, cfg_reg) = writes
            .iter()
            .find(|(reg, _)| *reg == regs::CFG_REG)
            .expect("CFG_REG must be written");

        // T_SHIFT is bit 3 and P_SHIFT is bit 2.
        assert_eq!(*cfg_reg & 0x0C, 0x0C);
    }

    #[test]
    fn init_rejects_a_wrong_product_id() {
        let mut sensor = SPL06::new(WrongIdBus, ADDRESS);
        let error = sensor.init(&mut NoDelay).expect_err("wrong chip ID must be rejected");
        assert!(matches!(error, Error::InvalidDevice(0x11)));
    }

    /// A bus whose ID register always reads `0x11`.
    struct WrongIdBus;

    impl embedded_hal::i2c::ErrorType for WrongIdBus {
        type Error = MockError;
    }

    impl embedded_hal::i2c::I2c for WrongIdBus {
        fn read(&mut self, _address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
            read.fill(0);
            Ok(())
        }

        fn write(&mut self, _address: u8, _write: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn write_read(&mut self, _address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
            read.fill(0);
            if write[0] == regs::ID {
                read[0] = 0x11;
            }
            Ok(())
        }

        fn transaction(
            &mut self,
            _address: u8,
            _operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            unimplemented!("the driver does not use transactions")
        }
    }

    #[test]
    fn init_waits_for_the_coefficients_to_become_valid() {
        let bus = FakeSpl06 {
            startup_polls_left: Cell::new(3),
            ..FakeSpl06::default()
        };
        let mut sensor = SPL06::new(bus, ADDRESS);

        sensor
            .init(&mut NoDelay)
            .expect("init must wait out the window before the coefficients are valid");

        // The calibration block may only be read once the part reports its
        // coefficients as valid. Reading it earlier is what produced a silent
        // all-zero calibration and, with it, 0 Pa and 0.00 degC forever.
        let reads = sensor.i2c.reads.borrow().clone();
        let first_coef = reads
            .iter()
            .position(|reg| *reg == regs::COEF)
            .expect("init must read the calibration block");
        let status_polls = reads[..first_coef].iter().filter(|reg| **reg == regs::MEAS_CFG).count();

        assert_eq!(
            status_polls, 4,
            "expected 3 not-yet-valid polls plus the one that reports valid before COEF: {reads:02X?}"
        );

        // And the coefficients it waited for are the real ones, not a blank block.
        assert_eq!(sensor.calib.c0, 203);
        assert_eq!(sensor.calib.c00, 81306);
    }

    #[test]
    fn init_times_out_when_the_coefficients_never_become_valid() {
        let bus = FakeSpl06 {
            status: regs::TMP_RDY | regs::PRS_RDY,
            ..FakeSpl06::default()
        };
        let mut sensor = SPL06::new(bus, ADDRESS);

        let error = sensor
            .init(&mut NoDelay)
            .expect_err("init must not accept a part whose coefficients never become valid");

        assert!(matches!(error, Error::Timeout));
    }

    /// `SENSOR_RDY` (bit 6) is not a precondition for reading the coefficient
    /// block - `COEF_RDY` (bit 7) is - so a part that reports its coefficients
    /// without reporting sensor initialisation must still initialise.
    #[test]
    fn init_does_not_require_the_sensor_ready_flag() {
        let bus = FakeSpl06 {
            status: regs::COEF_RDY | regs::TMP_RDY | regs::PRS_RDY,
            ..FakeSpl06::default()
        };
        let mut sensor = SPL06::new(bus, ADDRESS);

        sensor
            .init(&mut NoDelay)
            .expect("COEF_RDY alone must be enough to read the coefficients");

        assert_eq!(sensor.calib.c00, 81306);
    }

    #[test]
    fn measure_compensates_the_raw_registers() {
        let mut sensor = SPL06::new(FakeSpl06::default(), ADDRESS);
        sensor.init(&mut NoDelay).expect("init should succeed against the fake");

        let measurements = sensor.measure(&mut NoDelay).expect("measure should succeed");

        assert_eq!(measurements.raw_temperature(), 0x100000);
        assert_eq!(measurements.raw_pressure(), 0x300000);
        assert_eq!(measurements.temperature, 6683);
        assert_eq!(measurements.pressure, 57551);
        assert_eq!(measurements.temperature_celsius(), 66.83);
        assert_eq!(measurements.pressure_hpa(), 575.51);
    }

    #[test]
    fn measure_before_init_is_rejected() {
        let mut sensor = SPL06::new(FakeSpl06::default(), ADDRESS);
        let error = sensor
            .measure(&mut NoDelay)
            .expect_err("measure without calibration must fail");

        assert!(matches!(error, Error::NotCalibrated));
    }

    #[test]
    fn measure_times_out_when_the_sensor_never_reports_data() {
        let bus = FakeSpl06 {
            status: regs::COEF_RDY | regs::SENSOR_RDY,
            ..FakeSpl06::default()
        };
        let mut sensor = SPL06::new(bus, ADDRESS);
        sensor.init(&mut NoDelay).expect("init should succeed against the fake");

        let error = sensor
            .measure(&mut NoDelay)
            .expect_err("measure must time out when no data-ready flag appears");

        assert!(matches!(error, Error::Timeout));
    }

    #[test]
    fn reset_marks_the_driver_uncalibrated() {
        let mut sensor = SPL06::new(FakeSpl06::default(), ADDRESS);
        sensor.init(&mut NoDelay).expect("init should succeed against the fake");
        sensor.reset(&mut NoDelay).expect("reset should succeed");

        let writes = sensor.i2c.writes.borrow().clone();
        // SOFT_RST[3:0] = 0b1001 in RESET (0x0C).
        assert!(
            writes.contains(&(regs::RESET, 0x09)),
            "reset must write the soft reset command, got {writes:02X?}"
        );

        let error = sensor.measure(&mut NoDelay).expect_err("measure after reset must fail");
        assert!(matches!(error, Error::NotCalibrated));
    }
}
