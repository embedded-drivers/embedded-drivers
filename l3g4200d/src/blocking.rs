use embedded_hal::delay::DelayNs;

use super::{ADDRESS, CTRL_REG1_NORMAL_MODE, Config, DATA_READY_TIMEOUT_MS, Error, Scale, regs};

pub struct L3G4200D<I2C: embedded_hal::i2c::I2c> {
    i2c: I2C,
    addr: u8,
    /// The configured full scale. Stored rather than a precomputed float so the
    /// integer conversion ([`Self::read_gyro_mdps`]) stays exact.
    scale: Scale,
}

impl<I2C: embedded_hal::i2c::I2c> L3G4200D<I2C> {
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            // CTRL_REG4 resets to FS = 00, which is +/-250 dps, so this is what
            // the part is actually doing before `init` reprograms it.
            scale: Scale::Dps250,
        }
    }

    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, ADDRESS)
    }

    /// Configure the device and wait for the first sample.
    ///
    /// The output registers reset to zero and only hold a measurement once the
    /// gyro has produced one, so a read issued straight after `init` would
    /// return `(0, 0, 0)`. `init` therefore polls `STATUS_REG.ZYXDA` (datasheet
    /// Table 41/42) before returning. Once running, the device refreshes the
    /// registers at the configured output data rate, so later reads need no
    /// additional wait.
    pub fn init(&mut self, config: Config, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        if self.read_reg(regs::WHO_AM_I)? != 0xD3 {
            return Err(Error::InvalidDevice);
        }

        // Enable all axis and setup normal mode + Output Data Range & Bandwidth
        let mut reg1 = CTRL_REG1_NORMAL_MODE; // PD = 1 (normal mode), all axes enabled
        reg1 |= (config.data_rate as u8) << 4; // Set output data rate & bandwidth
        self.write_reg(regs::CTRL_REG1, reg1)?;

        // Disable high pass filter
        self.write_reg(regs::CTRL_REG2, 0x00)?;

        // Generate data ready interrupt on INT2
        self.write_reg(regs::CTRL_REG3, 0x08)?;

        // Set full scale selection in continuous mode
        self.write_reg(regs::CTRL_REG4, (config.scale as u8) << 4)?;

        self.scale = config.scale;

        // Boot in normal mode, disable FIFO, HPF disabled
        self.write_reg(regs::CTRL_REG5, 0x00)?;

        self.wait_for_data_ready(&mut delay)?;

        Ok(())
    }

    /// Poll `STATUS_REG.ZYXDA` until the first sample lands, or give up.
    fn wait_for_data_ready(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        for _ in 0..=DATA_READY_TIMEOUT_MS {
            if self.read_reg(regs::STATUS_REG)? & regs::STATUS_ZYXDA != 0 {
                return Ok(());
            }
            delay.delay_ms(1);
        }

        Err(Error::Timeout)
    }

    pub fn read_raw(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0u8; 6];

        // Read 6 bytes starting from OUT_X_L register (0x28 | 0x80 for auto-increment)
        self.i2c
            .write_read(self.addr, &[regs::OUT_X_L_AUTO_INCREMENT], &mut buf)?;

        // Combine high and low bytes into 16-bit integers
        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }
    /// Angular rate in **degrees per second**.
    ///
    /// This is the convenient form, and it matches `edrv-adxl345`'s `read_accel`,
    /// which also returns `f32`. On a soft-float target the arithmetic is
    /// emulated, but that is cheap next to *formatting* a float, which is what
    /// actually costs flash. Use [`Self::read_gyro_mdps`] to stay off floating
    /// point entirely.
    pub fn read_gyro(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw()?;
        let sensitivity = self.scale.dps_per_digit();

        Ok((x as f32 * sensitivity, y as f32 * sensitivity, z as f32 * sensitivity))
    }

    /// Angular rate in **milli-degrees per second**, with no floating point.
    ///
    /// The datasheet sensitivities (8.75 / 17.5 / 70 mdps per LSB) are not whole
    /// numbers, so the conversion is done in quarters and divided once:
    /// `raw * (mdps_per_digit * 4) / 4`. That single division truncates toward
    /// zero, leaving under **1 mdps** of error.
    pub fn read_gyro_mdps(&mut self) -> Result<(i32, i32, i32), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw()?;
        let factor = self.scale.mdps_per_digit_x4();

        Ok((
            (x as i32 * factor) / 4,
            (y as i32 * factor) / 4,
            (z as i32 * factor) / 4,
        ))
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
    use crate::DataRate;

    #[derive(Debug)]
    struct MockError;

    impl embedded_hal::i2c::Error for MockError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    /// A shared fake clock in milliseconds.
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

    /// A sample in the driver's little-endian X/Y/Z register order.
    const REAL_SAMPLE: [u8; 6] = [0x10, 0x01, 0x20, 0x02, 0x30, 0x03];

    /// State shared between the device model and the test, so the test can
    /// inspect what the driver did after the model has been moved into it.
    struct Shared {
        clock: Clock,
        /// When normal mode was entered, or `None` while powered down.
        measuring_since: Cell<Option<u32>>,
        /// How long the first sample takes once the part is in normal mode.
        first_sample_ms: u32,
        status_reads: Cell<u32>,
        writes: RefCell<Vec<(u8, u8)>>,
    }

    impl Shared {
        fn new(clock: Clock, first_sample_ms: u32) -> Rc<Self> {
            Rc::new(Self {
                clock,
                measuring_since: Cell::new(None),
                first_sample_ms,
                status_reads: Cell::new(0),
                writes: RefCell::new(Vec::new()),
            })
        }

        fn data_ready(&self) -> bool {
            self.measuring_since
                .get()
                .is_some_and(|since| self.clock.now().saturating_sub(since) >= self.first_sample_ms)
        }
    }

    /// An L3G4200D whose output registers stay at their reset value until the
    /// gyro has produced a sample.
    struct DeviceModel(Rc<Shared>);

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
                self.0.writes.borrow_mut().push((*reg, *value));
                if *reg == regs::CTRL_REG1 {
                    // PD is active-low: 1 = normal mode, 0 = power-down.
                    if value & 0x08 != 0 {
                        self.0.measuring_since.set(Some(self.0.clock.now()));
                    } else {
                        self.0.measuring_since.set(None);
                    }
                }
            }
            Ok(())
        }

        fn write_read(&mut self, _address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
            match write[0] {
                regs::WHO_AM_I => {
                    read.fill(0);
                    read[0] = 0xD3;
                }
                regs::STATUS_REG => {
                    self.0.status_reads.set(self.0.status_reads.get() + 1);
                    read.fill(0);
                    if self.0.data_ready() {
                        read[0] = regs::STATUS_ZYXDA;
                    }
                }
                regs::OUT_X_L_AUTO_INCREMENT => {
                    read.fill(0);
                    if self.0.data_ready() {
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

    /// `init` must not return before the gyro has produced its first sample.
    ///
    /// Before this fix `init` enabled the part and returned immediately, so a
    /// caller's first read returned the output-register reset value `(0, 0, 0)`.
    #[test]
    fn init_waits_for_the_first_sample() {
        let clock = Clock::default();
        let shared = Shared::new(clock.clone(), 5);
        let mut gyro = L3G4200D::new(DeviceModel(shared.clone()), ADDRESS);

        gyro.init(Config::default(), FakeDelay(clock.clone()))
            .expect("init should succeed");

        let (x, y, z) = gyro.read_raw().expect("read should succeed");
        assert_eq!(
            (x, y, z),
            (0x0110, 0x0220, 0x0330),
            "read_raw returned the pre-measurement reset value; init did not wait"
        );
        assert!(
            shared.status_reads.get() > 0,
            "init never read STATUS_REG, so it cannot know a sample is available"
        );
    }

    /// `CTRL_REG1.PD` is active-low.
    ///
    /// The datasheet's Table 21 reads "PD: Power down mode enable. Default
    /// value: 0 (0: power down mode, 1: normal mode or sleep mode)", and the
    /// register's reset value is `0x07`. Clearing PD therefore powers the part
    /// down and every reading becomes zero; the driver must keep it set.
    #[test]
    fn ctrl_reg1_selects_normal_mode() {
        let clock = Clock::default();
        let shared = Shared::new(clock.clone(), 5);
        let mut gyro = L3G4200D::new(DeviceModel(shared.clone()), ADDRESS);
        gyro.init(Config::default(), FakeDelay(clock))
            .expect("init should succeed");

        let writes = shared.writes.borrow().clone();
        let (_, reg1) = writes
            .iter()
            .find(|(reg, _)| *reg == regs::CTRL_REG1)
            .expect("CTRL_REG1 must be written");

        assert_eq!(
            *reg1 & 0x08,
            0x08,
            "PD (bit 3) must be 1 for normal mode; 0 powers the gyro down, got 0x{reg1:02X}"
        );
        assert_eq!(
            *reg1,
            CTRL_REG1_NORMAL_MODE | ((DataRate::Hz400Bw50 as u8) << 4),
            "unexpected CTRL_REG1 value 0x{reg1:02X}"
        );
    }
    /// The two sensitivity tables must agree, and match the datasheet.
    ///
    /// L3G4200D datasheet (Doc ID 17116 Rev 3) Table 3: 8.75 / 17.5 / 70 mdps
    /// per digit. The integer form stores those in quarters because none of them
    /// is a whole number, so the float form is derived from it rather than the
    /// other way round - that is what keeps `read_gyro_mdps` exact.
    #[test]
    fn scale_sensitivities_match_the_datasheet() {
        for (scale, x4, dps) in [
            (Scale::Dps250, 35, 0.00875f32),
            (Scale::Dps500, 70, 0.0175),
            (Scale::Dps2000, 280, 0.07),
        ] {
            assert_eq!(scale.mdps_per_digit_x4(), x4, "{scale:?} quarter-mdps");
            assert_eq!(scale.mdps_per_digit_x4() / 4, x4 / 4, "{scale:?} whole mdps");
            assert!(
                (scale.dps_per_digit() - dps).abs() < f32::EPSILON,
                "{scale:?} dps_per_digit: got {}, want {dps}",
                scale.dps_per_digit()
            );
        }
    }

    /// Sub-degree rates must survive the conversion.
    ///
    /// This is the bug the `f32` and `mdps` returns replaced. `read_gyro` used to
    /// be `(raw as f32 * dps_per_digit) as i16`, i.e. **whole degrees per
    /// second**: at the default +/-2000 dps one output step is 14 LSB, so the
    /// sample below - 272 LSB, a perfectly ordinary 19.04 dps - collapsed to 19,
    /// and anything under 0.5 dps collapsed to 0.
    #[test]
    fn gyro_conversion_keeps_sub_degree_resolution() {
        let clock = Clock::default();
        let shared = Shared::new(clock.clone(), 5);
        let mut gyro = L3G4200D::new(DeviceModel(shared.clone()), ADDRESS);
        gyro.init(Config::default(), FakeDelay(clock.clone()))
            .expect("init should succeed");

        // The model's sample, with the default +/-2000 dps scale (70 mdps/LSB).
        let (x, y, z) = gyro.read_gyro_mdps().expect("read should succeed");
        assert_eq!(
            (x, y, z),
            (19_040, 38_080, 57_120),
            "272/544/816 LSB at 70 mdps/LSB is 19.04/38.08/57.12 dps; anything \
             coarser means the conversion is quantising again"
        );

        let (fx, fy, fz) = gyro.read_gyro().expect("read should succeed");
        for (got, want) in [(fx, 19.04f32), (fy, 38.08), (fz, 57.12)] {
            assert!((got - want).abs() < 0.001, "got {got}, want {want}");
        }

        // A single LSB must not round to nothing: that is what used to happen.
        let one_lsb_mdps = Scale::Dps2000.mdps_per_digit_x4() / 4;
        assert_eq!(one_lsb_mdps, 70, "one LSB is 70 mdps at +/-2000 dps");
        assert_ne!(one_lsb_mdps, 0, "one LSB must not be quantised away");
    }
}
