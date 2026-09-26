//! Driver implementation in blocking mode

use embedded_hal::delay::DelayNs;

use crate::{ADDRESS, ANALOG_TURN_ON_MS, Config, DATA_READY_POLL_MARGIN_MS, Error, Gain, MODE_CONTINUOUS, Rate, regs};

pub struct HMC5883L<I2C: embedded_hal::i2c::I2c> {
    i2c: I2C,
    addr: u8,
    gain: Gain,
}

impl<I2C> HMC5883L<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            gain: Gain::Gain1090,
        }
    }

    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, ADDRESS)
    }

    /// Configure the device and wait for the first continuous-mode measurement.
    ///
    /// `MODE` is set to continuous measurement, but the data registers keep
    /// their reset value until the first measurement completes. The datasheet
    /// allows 50 ms for the analog circuit to become ready and updates the
    /// output registers once per configured output period, so `init` polls
    /// `STATUS.RDY` before returning; a caller that reads immediately otherwise
    /// gets `(0, 0, 0)` with no error. Once running, the device refreshes the
    /// registers continuously, so later reads need no additional wait.
    pub fn init(&mut self, config: Config, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        let id_a = self.read_reg(regs::IDENT_A)?;
        let id_b = self.read_reg(regs::IDENT_B)?;
        let id_c = self.read_reg(regs::IDENT_C)?;

        if id_a != 0x48 || id_b != 0x34 || id_c != 0x33 {
            return Err(Error::InvalidDevice);
        }

        let mut cra = 0; // Initialize CRA with all bits set to 0
        cra |= (config.samples as u8) << 5; // Set MA (CRA6 to CRA5) based on config.samples
        cra |= (config.data_rate as u8) << 2; // Set DO (CRA4 to CRA2) based on config.data_rate
        self.write_reg(regs::CONFIG_A, cra)?;

        let crb = (config.gain as u8) << 5; // Set GN (CRB7 to CRB5) based on config.gain
        self.write_reg(regs::CONFIG_B, crb)?;

        self.write_reg(regs::MODE, MODE_CONTINUOUS)?; // Continuous-measurement mode

        self.gain = config.gain;

        self.wait_for_data_ready(config.data_rate, &mut delay)?;

        Ok(())
    }

    /// Poll `STATUS.RDY` until the first measurement lands, or give up.
    ///
    /// The budget is the datasheet's 50 ms analog turn-on plus one output
    /// period at the configured rate, so a slow rate is allowed to take as long
    /// as it really takes.
    fn wait_for_data_ready(&mut self, rate: Rate, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        let budget_ms = ANALOG_TURN_ON_MS + rate.period_ms() + DATA_READY_POLL_MARGIN_MS;

        for _ in 0..=budget_ms {
            if self.read_reg(regs::STATUS)? & regs::STATUS_RDY != 0 {
                return Ok(());
            }
            delay.delay_ms(1);
        }

        Err(Error::Timeout)
    }

    pub fn read_raw_measurement(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0u8; 6];

        // Read 6 bytes starting from DATA_X_MSB
        self.i2c.write_read(self.addr, &[regs::DATA_X_MSB], &mut buf)?;
        self.i2c.write(self.addr, &[regs::DATA_X_MSB])?;

        let x = ((buf[0] as i16) << 8) | buf[1] as i16;
        let y = ((buf[4] as i16) << 8) | buf[5] as i16;
        let z = ((buf[2] as i16) << 8) | buf[3] as i16;

        Ok((x, y, z))
    }

    pub fn read_measurement(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw_measurement()?;

        let resolution = self.gain.resolution();

        let x_norm = x as f32 * resolution;
        let y_norm = y as f32 * resolution;
        let z_norm = z as f32 * resolution;

        Ok((x_norm, y_norm, z_norm))
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
    use core::cell::{Cell, RefCell};
    use std::rc::Rc;

    use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};

    use super::*;

    #[derive(Debug)]
    struct MockError;

    impl embedded_hal::i2c::Error for MockError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    /// A shared fake clock in milliseconds: the HMC5883L needs real time to
    /// produce its first measurement, so the device model advances with the
    /// delay the driver is given instead of the wall clock.
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

    /// One complete measurement: X = 0x0102, Z = 0x0304, Y = 0x0506, in the
    /// register order X_MSB, X_LSB, Z_MSB, Z_LSB, Y_MSB, Y_LSB.
    const REAL_SAMPLE: [u8; 6] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06];

    /// An HMC5883L whose data registers stay at their reset value until the
    /// first measurement lands.
    struct DeviceModel {
        clock: Clock,
        measuring_since: Cell<Option<u32>>,
        /// Datasheet: 50 ms analog turn-on plus a 6 ms measurement.
        first_sample_ms: u32,
        status_reads: Cell<u32>,
        writes: RefCell<Vec<(u8, u8)>>,
    }

    impl DeviceModel {
        fn new(clock: Clock, first_sample_ms: u32) -> Self {
            Self {
                clock,
                measuring_since: Cell::new(None),
                first_sample_ms,
                status_reads: Cell::new(0),
                writes: RefCell::new(Vec::new()),
            }
        }

        fn data_ready(&self) -> bool {
            self.measuring_since
                .get()
                .is_some_and(|since| self.clock.now().saturating_sub(since) >= self.first_sample_ms)
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
            if let [reg, value] = write {
                self.writes.borrow_mut().push((*reg, *value));
                if *reg == regs::MODE {
                    self.measuring_since.set(Some(self.clock.now()));
                }
            }
            // A one-byte write is the datasheet's "move the address pointer"
            // sequence and must not disturb the measurement.
            Ok(())
        }

        fn write_read(&mut self, _address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
            match write[0] {
                regs::IDENT_A => {
                    read.fill(0);
                    read[0] = 0x48;
                }
                regs::IDENT_B => {
                    read.fill(0);
                    read[0] = 0x34;
                }
                regs::IDENT_C => {
                    read.fill(0);
                    read[0] = 0x33;
                }
                regs::STATUS => {
                    self.status_reads.set(self.status_reads.get() + 1);
                    read.fill(0);
                    if self.data_ready() {
                        read[0] = regs::STATUS_RDY;
                    }
                }
                regs::DATA_X_MSB => {
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

    /// `init` must not return before the first continuous-mode measurement has
    /// landed.
    ///
    /// Continuous mode refreshes the data registers, but the registers hold
    /// their reset value until the first measurement completes: the datasheet
    /// allows 50 ms for the analog circuit to be ready plus a 6 ms measurement.
    /// Before this fix `init` returned immediately and the first read returned
    /// `(0, 0, 0)` with no error.
    #[test]
    fn init_waits_for_the_first_measurement() {
        let clock = Clock::default();
        let mut sensor = HMC5883L::new(DeviceModel::new(clock.clone(), 56), ADDRESS);

        sensor
            .init(Config::default(), FakeDelay(clock.clone()))
            .expect("init should succeed");

        let (x, y, z) = sensor.read_raw_measurement().expect("read should succeed");
        assert_eq!(
            (x, y, z),
            (0x0102, 0x0506, 0x0304),
            "read_raw_measurement returned the pre-measurement reset value; init did not wait"
        );
        assert!(
            sensor.i2c.status_reads.get() > 0,
            "init never read STATUS, so it cannot know a measurement is available"
        );
        assert!(clock.now() >= 56, "init returned before the first measurement");
    }

    /// The device is put into continuous mode, not single-measurement mode.
    #[test]
    fn init_selects_continuous_measurement_mode() {
        let clock = Clock::default();
        let mut sensor = HMC5883L::new(DeviceModel::new(clock.clone(), 56), ADDRESS);
        sensor
            .init(Config::default(), FakeDelay(clock))
            .expect("init should succeed");

        let writes = sensor.i2c.writes.borrow().clone();
        let mode = writes
            .iter()
            .find(|(reg, _)| *reg == regs::MODE)
            .map(|(_, value)| *value)
            .expect("MODE must be written");
        assert_eq!(mode, MODE_CONTINUOUS);
    }
}
