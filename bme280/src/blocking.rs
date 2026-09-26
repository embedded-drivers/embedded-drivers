//! Blocking API

use embedded_hal::delay::DelayNs;

use crate::{ADDRESS, CHIP_ID_BME280, CHIP_ID_BMP280, CalibrationData, Error, Measurements, regs};

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

        // `CONTROL_WRITES` carries the required order and the reason for it.
        for (reg, value) in crate::CONTROL_WRITES {
            self.write_reg(reg, value)?;
        }

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

    /// Trigger a measurement, wait for the device to finish it, then read the
    /// compensated result.
    ///
    /// See [`crate::BME280::read_measurement`] for why the wait is not optional:
    /// until a conversion completes the data registers hold `0x80000`, the
    /// datasheet's "measurement skipped" marker, and that placeholder decodes to
    /// *plausible* values - about 2.4 degC low, with a pressure about a third low.
    ///
    /// `delay` paces the wait for the conversion.
    pub fn read_measurement(&mut self, mut delay: impl DelayNs) -> Result<Measurements, Error<I2C::Error>> {
        // mode = forced, keeping the x1 oversampling programmed by `init`.
        self.write_reg(regs::CTRL_MEAS, crate::CTRL_MEAS_FORCED)?;

        // A fixed wait first: right after the write the device may not have set
        // `measuring` yet, so polling straight away can read the *previous*
        // result. Then confirm it really finished.
        delay.delay_ms(crate::MEASUREMENT_TIME_MS);

        let mut extra = 0;
        while self.read_reg(regs::STATUS)? & regs::STATUS_MEASURING != 0 {
            if extra >= crate::MEASUREMENT_POLL_LIMIT_MS {
                return Err(Error::Timeout);
            }
            delay.delay_ms(1);
            extra += 1;
        }

        let mut raw = [0u8; regs::DATA_LEN];
        self.read_regs(regs::DATA_MSB, &mut raw)?;

        // Bosch's `parse_sensor_data` field order: pressure, temperature, humidity.
        let adc_p = ((raw[0] as i32) << 12) | ((raw[1] as i32) << 4) | ((raw[2] as i32) >> 4);
        let adc_t = ((raw[3] as i32) << 12) | ((raw[4] as i32) << 4) | ((raw[5] as i32) >> 4);
        let adc_h = ((raw[6] as i32) << 8) | (raw[7] as i32);

        // Returns temperature in DegC, resolution is 0.01 DegC.
        // t_fine carries fine temperature as global value
        let (t_fine, t) = crate::convert_temperature(adc_t, &self.calib);

        // Pressure in Q24.8 Pa; converted to centi-pascal below.
        let p = crate::convert_pressure(adc_p, t_fine, &self.calib);
        let p = p * 100 / 256;

        // BME280 only
        let mut h = 0;
        if self.is_bme280 {
            let h0 = super::convert_humidity(adc_h, t_fine, &self.calib);

            // convert Q22.10 to centi
            h = h0 * 100 / 1024;
        }

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

#[cfg(test)]
mod tests {
    use core::cell::RefCell;
    use std::vec::Vec;

    use embedded_hal::i2c::ErrorKind;

    use super::*;
    use crate::CONTROL_WRITES;

    #[derive(Debug)]
    struct MockError;

    impl embedded_hal::i2c::Error for MockError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    /// An I2C bus that records every register write and answers reads with a
    /// BME280 chip ID, so `init` can run without hardware.
    #[derive(Default)]
    struct RecordingI2c {
        writes: RefCell<Vec<(u8, u8)>>,
        reads: RefCell<Vec<u8>>,
    }

    impl embedded_hal::i2c::ErrorType for RecordingI2c {
        type Error = MockError;
    }

    impl embedded_hal::i2c::I2c for RecordingI2c {
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
            if reg == regs::CHIP_ID {
                read[0] = CHIP_ID_BME280;
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

    /// The humidity oversampling register must be written before the
    /// measurement control register.
    ///
    /// Bosch's BME280 API is explicit that "humidity related changes will be only
    /// effective after a write operation to ctrl_meas register". This driver used
    /// to write them the other way round, which meant `osrs_h` never left its
    /// reset default (skipped) and humidity was never measured. This test fails
    /// on that ordering.
    #[test]
    fn humidity_is_programmed_before_measurement() {
        let mut sensor = BME280::new(RecordingI2c::default(), ADDRESS);
        sensor.init().expect("init should succeed against the mock");

        let writes = sensor.i2c.writes.borrow().clone();

        let position = |reg: u8| {
            writes
                .iter()
                .position(|(r, _)| *r == reg)
                .unwrap_or_else(|| panic!("reg 0x{reg:02X} was never written; got {writes:02X?}"))
        };

        assert!(
            position(regs::CTRL_HUM) < position(regs::CTRL_MEAS),
            "CTRL_HUM (0xF2) must be written before CTRL_MEAS (0xF4), got {writes:02X?}"
        );

        // The whole documented sequence has to be emitted, not just the two
        // registers above, and in the published order.
        assert_eq!(writes, CONTROL_WRITES.to_vec(), "init wrote an unexpected sequence");
    }
    /// A BME280 that models the part around a forced conversion.
    ///
    /// After a reset - and until a conversion has actually **completed** - the
    /// data registers hold the datasheet's "measurement skipped" marker:
    /// `0x80000` in each 20-bit field and `0x8000` in humidity. Reading them at
    /// that point is the bug this test exists to catch, because those bytes
    /// compensate to values that look like real weather.
    #[derive(Default)]
    struct DeviceModel {
        /// A conversion is running.
        measuring: RefCell<bool>,
        /// A conversion has finished, so the data registers are meaningful.
        converted: RefCell<bool>,
        /// How many times the driver asked for `status`.
        status_reads: RefCell<u32>,
    }

    /// What a real conversion produces here: 0x81600 / 0x52700, i.e. about
    /// 25.88 degC and 101779 Pa.
    const REAL_DATA: [u8; 8] = [0x52, 0x70, 0x00, 0x81, 0x60, 0x00, 0x80, 0x00];

    /// The same block before any conversion: every field is its "not measured"
    /// marker. This compensates to about 24.10 degC and **69943 Pa** - a
    /// plausible-looking value roughly a third low.
    const PLACEHOLDER_DATA: [u8; 8] = [0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00];

    impl embedded_hal::i2c::ErrorType for DeviceModel {
        type Error = MockError;
    }

    impl embedded_hal::i2c::I2c for DeviceModel {
        fn read(&mut self, _address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
            read.fill(0);
            Ok(())
        }

        fn write(&mut self, _address: u8, write: &[u8]) -> Result<(), Self::Error> {
            // mode = forced (CTRL_MEAS bits 1:0 == 0b01) starts one conversion,
            // which invalidates whatever was in the data registers.
            match write {
                [reg, value] if *reg == regs::CTRL_MEAS && value & 0x03 == 0x01 => {
                    *self.measuring.borrow_mut() = true;
                    *self.converted.borrow_mut() = false;
                }
                _ => {}
            }
            Ok(())
        }

        fn write_read(&mut self, _address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
            match write[0] {
                regs::CHIP_ID => {
                    read.fill(0);
                    read[0] = CHIP_ID_BME280;
                }
                regs::STATUS => {
                    *self.status_reads.borrow_mut() += 1;
                    if *self.measuring.borrow() {
                        // The conversion finishes while the driver is waiting
                        // for it - which is precisely what the wait is for.
                        *self.measuring.borrow_mut() = false;
                        *self.converted.borrow_mut() = true;
                        read[0] = regs::STATUS_MEASURING;
                    } else {
                        read[0] = 0;
                    }
                }
                regs::DATA_MSB => {
                    let block = if *self.converted.borrow() {
                        REAL_DATA
                    } else {
                        PLACEHOLDER_DATA
                    };
                    read[..regs::DATA_LEN].copy_from_slice(&block);
                }
                _ => read.fill(0),
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

    /// The driver must not hand back the "measurement skipped" placeholder.
    ///
    /// Before 0.1.1 `read_measurement` read the data registers immediately, so
    /// right after `init` it returned the placeholder bytes compensated into
    /// about 24.1 degC and 69943 Pa instead of the real 25.9 degC and 101779 Pa.
    /// Nothing errored: the numbers were simply wrong by about a third on
    /// pressure, and looked like high-altitude weather.
    #[test]
    fn measurement_waits_for_the_conversion_to_finish() {
        let mut sensor = BME280::new(DeviceModel::default(), ADDRESS);
        sensor.is_bme280 = true;
        sensor.calib = CalibrationData {
            dig_t1: 28000,
            dig_t2: 26500,
            dig_t3: 50,
            dig_p1: 37000,
            dig_p2: -10500,
            dig_p3: 3024,
            dig_p4: 6800,
            dig_p5: -120,
            dig_p6: -7,
            dig_p7: 9900,
            dig_p8: -10230,
            dig_p9: 4285,
            dig_h1: 75,
            dig_h2: 360,
            dig_h3: 0,
            dig_h4: 310,
            dig_h5: 50,
            dig_h6: 30,
        };

        let m = sensor.read_measurement(NoopDelay).expect("measurement should succeed");

        // The decisive assertion: the placeholder yields ~69943 Pa.
        assert!(
            m.pressure_pa() > 90_000,
            "got {} Pa; the pre-conversion placeholder compensates to ~69943 Pa, \
             so the driver read the data registers before the conversion finished",
            m.pressure_pa()
        );
        // The placeholder is 24.10 degC, the real value 25.88 degC.
        assert!(
            m.temperature > 2_500,
            "got {} (0.01 degC); the placeholder is 2410",
            m.temperature
        );

        // And it must have actually asked the device whether it was done.
        assert!(
            *sensor.i2c.status_reads.borrow() > 0,
            "the driver never read `status`, so it cannot know the conversion finished"
        );
    }

    /// A delay that does nothing: the device model decides when the conversion
    /// completes, so the test does not need to wait in real time.
    struct NoopDelay;

    impl DelayNs for NoopDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }
}
