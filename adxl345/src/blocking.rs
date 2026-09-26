use embedded_hal::delay::DelayNs;

use super::{
    Config, DATA_READY_POLL_MARGIN_MS, Error, POWER_CTL_MEASURE, POWER_CTL_STANDBY, PRIMARY_ADDRESS, Range, Rate,
    SECONDARY_ADDRESS, regs,
};

/// A struct representing the ADXL345 accelerometer.
//
// This struct encapsulates the I2C interface, the device address, and the least significant bit (LSB) scale factor.
// It provides methods to initialize the accelerometer, read raw acceleration data, and read acceleration values in g-force.
pub struct ADXL345<I2C> {
    i2c: I2C,
    addr: u8,
    lsb_scale: f32,
}

impl<I2C> ADXL345<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            lsb_scale: 0.0,
        }
    }

    /// Creates a new instance of the ADXL345 accelerometer with the primary address.
    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, PRIMARY_ADDRESS)
    }

    pub fn new_secondary(i2c: I2C) -> Self {
        Self::new(i2c, SECONDARY_ADDRESS)
    }

    /// Configure the part and wait until it has produced its first sample.
    ///
    /// The data registers hold their reset value (zero) from power-up until the
    /// first conversion completes, which takes the datasheet's turn-on time
    /// after the Measure bit is set (`1/data rate + 1.1` ms; about 11.1 ms at
    /// the default 100 Hz). Returning from `init` before then makes a caller's
    /// first read return `(0, 0, 0)` with no error.
    ///
    /// The configuration registers are therefore written while the part is
    /// still in standby, and `init` only returns once `INT_SOURCE` reports
    /// `DATA_READY` (or the turn-on budget expires with [`Error::Timeout`]).
    /// Afterwards the part free-runs at the configured rate, so later reads
    /// return the most recent sample without an additional wait.
    pub fn init(&mut self, config: Config, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        let id = self.read_reg(regs::DEVID)?;
        if id != 0xE5 {
            return Err(Error::InvalidDevice);
        }

        // The datasheet recommends clearing SLEEP/AUTO_SLEEP by passing through
        // standby before re-entering measurement mode, and configuring the part
        // while it is in standby.
        self.write_reg(regs::POWER_CTL, POWER_CTL_STANDBY)?; // Standby
        self.write_reg(regs::POWER_CTL, 16)?; // AUTO_SLEEP, cleared by the transition below

        // Set data rate and range
        let mut data_format = (config.range as u8) & 0x03;
        data_format |= 0b100; // Set bit 2 to enable left justified mode
        self.write_reg(regs::DATA_FORMAT, data_format)?;

        let bw_rate = (config.rate as u8) & 0x0F; // Set rate
        self.write_reg(regs::BW_RATE, bw_rate)?;

        // Enter measurement mode last: the turn-on time is measured from this
        // write, so the output data rate must already be programmed.
        self.write_reg(regs::POWER_CTL, POWER_CTL_MEASURE)?;

        // Set scale factor based on the range
        self.lsb_scale = match config.range {
            Range::G2 => 4.0 / 65536.0,
            Range::G4 => 8.0 / 65536.0,
            Range::G8 => 16.0 / 65536.0,
            Range::G16 => 32.0 / 65536.0,
        };

        self.wait_for_data_ready(config.rate, &mut delay)?;

        Ok(())
    }

    /// Wait until `INT_SOURCE.DATA_READY` is set, or give up after the
    /// datasheet's turn-on time for the configured rate plus a small margin.
    fn wait_for_data_ready(&mut self, rate: Rate, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        let budget_ms = rate.turn_on_time_ms() + DATA_READY_POLL_MARGIN_MS;

        for _ in 0..=budget_ms {
            if self.read_reg(regs::INT_SOURCE)? & regs::INT_SOURCE_DATA_READY != 0 {
                return Ok(());
            }
            delay.delay_ms(1);
        }

        Err(Error::Timeout)
    }

    /// Reads the raw acceleration data from the sensor.
    ///
    /// This method reads the raw 16-bit acceleration values for the X, Y, and Z axes
    /// from the sensor's data registers. The values are returned as a tuple of three
    /// 16-bit integers representing the acceleration in each axis.
    ///
    /// # Returns
    ///
    /// A `Result` containing a tuple of three 16-bit integers `(x, y, z)` representing
    /// the raw acceleration values for the X, Y, and Z axes, or an `Error` if the read
    /// operation fails.
    ///
    /// # Errors
    ///
    /// Returns an `Error` if the I2C read operation fails.
    pub fn read_raw(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0; 6];

        self.i2c.write_read(self.addr, &[regs::DATAX0], &mut buf)?;

        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }

    /// Reads the acceleration values from the sensor and converts them to g-force.
    /// The scaling factor is set during initialization based on the configured range.
    ///
    /// Returns a tuple of (x, y, z) acceleration values in g-force.
    pub fn read_accel(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x_raw, y_raw, z_raw) = self.read_raw()?;

        // Convert raw values to g-force
        // The scaling factor is set during initialization based on the configured range
        let x = x_raw as f32 * self.lsb_scale;
        let y = y_raw as f32 * self.lsb_scale;
        let z = z_raw as f32 * self.lsb_scale;

        Ok((x, y, z))
    }

    pub fn read_reg(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    // Add this new method to write to registers
    pub fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[reg, value])
    }
}

#[cfg(test)]
mod tests {
    use core::cell::Cell;
    use std::rc::Rc;

    use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};

    use super::*;
    use crate::regs;

    #[derive(Debug)]
    struct MockError;

    impl embedded_hal::i2c::Error for MockError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    /// A shared fake clock in milliseconds.
    ///
    /// The ADXL345 needs real time to produce its first sample, so the device
    /// model advances with the delay the driver is given instead of the wall
    /// clock. That is what makes "the driver waited long enough" observable in
    /// a test that does not actually sleep.
    #[derive(Clone, Default)]
    struct Clock(Rc<Cell<u32>>);

    impl Clock {
        fn now(&self) -> u32 {
            self.0.get()
        }

        fn advance_ms(&self, ms: u32) {
            self.0.set(self.0.get() + ms);
        }
    }

    struct FakeDelay(Clock);

    impl DelayNs for FakeDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.0.advance_ms(ns.div_ceil(1_000_000));
        }
    }

    /// The sample the modelled part produces once it is really measuring.
    const REAL_SAMPLE: [u8; 6] = [0x00, 0x01, 0x00, 0x02, 0x00, 0x03];

    /// An ADXL345 whose data registers stay at their reset value until the
    /// datasheet turn-on time has elapsed since the Measure bit was set.
    struct DeviceModel {
        clock: Clock,
        /// When measurement mode was entered, or `None` in standby.
        measuring_since: Cell<Option<u32>>,
        /// How long after entering measurement mode the first sample appears.
        turn_on_ms: u32,
        /// How many times the driver asked for `INT_SOURCE`.
        int_source_reads: Cell<u32>,
    }

    impl DeviceModel {
        fn new(clock: Clock, turn_on_ms: u32) -> Self {
            Self {
                clock,
                measuring_since: Cell::new(None),
                turn_on_ms,
                int_source_reads: Cell::new(0),
            }
        }

        fn data_ready(&self) -> bool {
            self.measuring_since
                .get()
                .is_some_and(|since| self.clock.now().saturating_sub(since) >= self.turn_on_ms)
        }
    }

    impl ErrorType for DeviceModel {
        type Error = MockError;
    }

    impl I2c for DeviceModel {
        fn read(&mut self, _address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
            read.fill(0);
            Ok(())
        }

        fn write(&mut self, _address: u8, write: &[u8]) -> Result<(), Self::Error> {
            match write {
                [reg, value] if *reg == regs::POWER_CTL => {
                    if value & POWER_CTL_MEASURE != 0 {
                        self.measuring_since.set(Some(self.clock.now()));
                    } else {
                        self.measuring_since.set(None);
                    }
                }
                _ => {}
            }
            Ok(())
        }

        fn write_read(&mut self, _address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
            match write[0] {
                regs::DEVID => {
                    read.fill(0);
                    read[0] = 0xE5;
                }
                regs::INT_SOURCE => {
                    self.int_source_reads.set(self.int_source_reads.get() + 1);
                    read.fill(0);
                    if self.data_ready() {
                        read[0] = regs::INT_SOURCE_DATA_READY;
                    }
                }
                regs::DATAX0 => {
                    read.fill(0);
                    if self.data_ready() {
                        read.copy_from_slice(&REAL_SAMPLE);
                    }
                }
                _ => read.fill(0),
            }
            Ok(())
        }

        fn transaction(&mut self, _address: u8, _operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
            unimplemented!("the driver does not use transactions")
        }
    }

    /// `init` must not return before the part has produced its first sample.
    ///
    /// Before this fix `init` set the Measure bit and returned immediately, so a
    /// caller's first read returned the reset-value `(0, 0, 0)`: the device was
    /// measuring, but no conversion had completed yet.
    #[test]
    fn init_waits_for_the_first_sample() {
        let clock = Clock::default();
        // 11 ms is the datasheet turn-on time at the default 100 Hz ODR.
        let mut sensor = ADXL345::new(DeviceModel::new(clock.clone(), 11), PRIMARY_ADDRESS);

        sensor
            .init(Config::default(), FakeDelay(clock.clone()))
            .expect("init should succeed");

        let (x, y, z) = sensor.read_raw().expect("read should succeed");
        assert_eq!(
            (x, y, z),
            (0x0100, 0x0200, 0x0300),
            "read_raw returned the pre-conversion reset value; init did not wait for the first sample"
        );
        assert!(
            sensor.i2c.int_source_reads.get() > 0,
            "init never read INT_SOURCE, so it cannot know a sample is available"
        );
        assert!(clock.now() >= 11, "init returned before the turn-on time elapsed");
    }

    /// The turn-on budget follows the configured output data rate. At 0.1 Hz the
    /// datasheet's `1/data rate + 1.1` ms is 10 s, so a part that never reports
    /// data must time out instead of hanging.
    #[test]
    fn init_times_out_when_the_part_never_reports_data() {
        let clock = Clock::default();
        // The model never reports DATA_READY because it is never told to measure
        // at all: every write is dropped.
        struct NeverReady;
        impl ErrorType for NeverReady {
            type Error = MockError;
        }
        impl I2c for NeverReady {
            fn read(&mut self, _address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
                read.fill(0);
                Ok(())
            }
            fn write(&mut self, _address: u8, _write: &[u8]) -> Result<(), Self::Error> {
                Ok(())
            }
            fn write_read(&mut self, _address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
                read.fill(0);
                if write[0] == regs::DEVID {
                    read[0] = 0xE5;
                }
                Ok(())
            }
            fn transaction(&mut self, _address: u8, _operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
                unimplemented!()
            }
        }

        let mut sensor = ADXL345::new(NeverReady, PRIMARY_ADDRESS);
        let result = sensor.init(Config::default(), FakeDelay(clock));
        assert!(
            matches!(result, Err(Error::Timeout)),
            "a part that never reports DATA_READY must time out, got {result:?}"
        );
    }
}
