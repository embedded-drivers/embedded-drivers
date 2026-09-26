use crate::{
    DEFAULT_TUNING_SETTINGS, Error, MODEL_ID_VL53L0X, PRIMARY_ADDRESS, SeqStepEnables, SeqStepTimeouts,
    VcselPeriodType, decode_timeout, decode_vcsel_period, encode_timeout, regs, timeout_mclks_to_microseconds,
    timeout_microseconds_to_mclks,
};

pub struct VL53L0X<I2C: embedded_hal::i2c::I2c> {
    i2c: I2C,
    addr: u8,
    io_mode2v8: bool,
    stop_variable: u8,
    measurement_timing_budget_microseconds: u32,
}

impl<I2C: embedded_hal::i2c::I2c> VL53L0X<I2C> {
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
    pub fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        let model_id = self.read_reg(regs::WHO_AM_I)?;
        if model_id != MODEL_ID_VL53L0X {
            return Err(Error::InvalidDevice(model_id));
        }

        self.init_hardware()
    }

    /// Change the I2C address of the sensor.
    ///
    /// The address resets when the device is powered off. Only `0x08..=0x77` is
    /// accepted; `0x00..=0x07` and `0x78..=0x7F` are reserved 7-bit addresses.
    pub fn set_address(&mut self, new_address: u8) -> Result<(), Error<I2C::Error>> {
        if !(0x08..=0x77).contains(&new_address) {
            return Err(Error::InvalidAddress(new_address));
        }
        self.write_reg(regs::I2C_SLAVE_DEVICE_ADDRESS, new_address)?;
        self.addr = new_address;

        Ok(())
    }

    /// Read the model ID register (`0xC0`).
    pub fn who_am_i(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_reg(regs::WHO_AM_I)
    }

    /// Start continuous ranging.
    ///
    /// `period_millis` of 0 selects back-to-back mode; any other value selects
    /// timed mode with that inter-measurement period.
    pub fn start_continuous(&mut self, period_millis: u32) -> Result<(), Error<I2C::Error>> {
        self.write_reg(0x80, 0x01)?;
        self.write_reg(0xFF, 0x01)?;
        self.write_reg(0x00, 0x00)?;
        let sv = self.stop_variable;
        self.write_reg(0x91, sv)?;
        self.write_reg(0x00, 0x01)?;
        self.write_reg(0xFF, 0x00)?;
        self.write_reg(0x80, 0x00)?;

        let mut period_millis = period_millis;
        if period_millis != 0 {
            // Continuous timed mode.
            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
            let osc_calibrate_value = self.read_reg16(regs::OSC_CALIBRATE_VAL)?;

            if osc_calibrate_value != 0 {
                period_millis *= osc_calibrate_value as u32;
            }

            self.write_reg32(regs::SYSTEM_INTERMEASUREMENT_PERIOD, period_millis)?;
            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end
            // VL53L0X_REG_SYSRANGE_MODE_TIMED
            self.write_reg(regs::SYSRANGE_START, 0x04)?;
        } else {
            // Continuous back-to-back mode.
            // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
            self.write_reg(regs::SYSRANGE_START, 0x02)?;
        }

        Ok(())
    }

    /// Stop continuous ranging.
    pub fn stop_continuous(&mut self) -> Result<(), Error<I2C::Error>> {
        // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
        self.write_reg(regs::SYSRANGE_START, 0x01)?;
        self.write_reg(0xFF, 0x01)?;
        self.write_reg(0x00, 0x00)?;
        self.write_reg(0x91, 0x00)?;
        self.write_reg(0x00, 0x01)?;
        self.write_reg(0xFF, 0x00)?;

        Ok(())
    }

    /// Perform a single-shot measurement and return the distance in
    /// millimetres.
    ///
    /// A return value of [`OUT_OF_RANGE_MILLIMETERS`](crate::OUT_OF_RANGE_MILLIMETERS)
    /// (8190) means no target was detected or the target is out of range; it is
    /// not a distance.
    pub fn read_range_single_millimeters(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.write_reg(0x80, 0x01)?;
        self.write_reg(0xFF, 0x01)?;
        self.write_reg(0x00, 0x00)?;
        let sv = self.stop_variable;
        self.write_reg(0x91, sv)?;
        self.write_reg(0x00, 0x01)?;
        self.write_reg(0xFF, 0x00)?;
        self.write_reg(0x80, 0x00)?;

        self.write_reg(regs::SYSRANGE_START, 0x01)?;

        // "Wait until start bit has been cleared"
        let mut c = 0;
        while (self.read_reg(regs::SYSRANGE_START)? & 0x01) != 0 {
            c += 1;
            if c == 10000 {
                return Err(Error::Timeout);
            }
        }

        self.read_range_continuous_millimeters()
    }

    /// Read the result of the current continuous measurement, in millimetres.
    ///
    /// A return value of [`OUT_OF_RANGE_MILLIMETERS`](crate::OUT_OF_RANGE_MILLIMETERS)
    /// (8190) means no target was detected or the target is out of range; it is
    /// not a distance.
    pub fn read_range_continuous_millimeters(&mut self) -> Result<u16, Error<I2C::Error>> {
        let mut c = 0;
        while (self.read_reg(regs::RESULT_INTERRUPT_STATUS)? & 0x07) == 0 {
            c += 1;
            if c == 10000 {
                return Err(Error::Timeout);
            }
        }

        let range = self.read_reg16(regs::RESULT_RANGE_STATUS_PLUS_10)?;
        // Clear the interrupt even if the result read above failed.
        self.write_reg(regs::SYSTEM_INTERRUPT_CLEAR, 0x01)?;

        Ok(range)
    }

    /// Get the measurement timing budget in microseconds.
    pub fn get_measurement_timing_budget(&mut self) -> Result<u32, Error<I2C::Error>> {
        let start_overhead: u32 = 1910;
        let end_overhead: u32 = 960;
        let msrc_overhead: u32 = 660;
        let tcc_overhead: u32 = 590;
        let dss_overhead: u32 = 690;
        let pre_range_overhead: u32 = 660;
        let final_range_overhead: u32 = 550;

        let enables = self.get_sequence_step_enables()?;
        let timeouts = self.get_sequence_step_timeouts(&enables)?;

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
    pub fn set_measurement_timing_budget(&mut self, budget_microseconds: u32) -> Result<bool, Error<I2C::Error>> {
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

        let enables = self.get_sequence_step_enables()?;
        let timeouts = self.get_sequence_step_timeouts(&enables)?;

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
        )?;

        // set_sequence_step_timeout() end
        // Store for internal reuse.
        self.measurement_timing_budget_microseconds = budget_microseconds;
        Ok(true)
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
    pub fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, &[reg, value])?;
        Ok(())
    }

    fn read_reg16(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.read_regs(reg, &mut buf)?;
        Ok(u16::from_be_bytes([buf[0], buf[1]]))
    }

    fn write_reg16(&mut self, reg: u8, word: u16) -> Result<(), Error<I2C::Error>> {
        let [hi, lo] = word.to_be_bytes();
        self.i2c.write(self.addr, &[reg, hi, lo])?;
        Ok(())
    }

    fn write_reg32(&mut self, reg: u8, word: u32) -> Result<(), Error<I2C::Error>> {
        // Big-endian, matching ST's `VL53L0X_WrDWord`, Pololu's `writeReg32Bit`
        // and every other multi-byte value in this driver. See the async module
        // for why this deliberately differs from the reference crate.
        let bytes = word.to_be_bytes();
        self.i2c
            .write(self.addr, &[reg, bytes[0], bytes[1], bytes[2], bytes[3]])?;
        Ok(())
    }

    fn set_signal_rate_limit(&mut self, limit: f32) -> Result<bool, Error<I2C::Error>> {
        if !(0.0..=511.99).contains(&limit) {
            Ok(false)
        } else {
            // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
            self.write_reg16(
                regs::FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                (limit * ((1 << 7) as f32)) as u16,
            )?;
            Ok(true)
        }
    }

    fn get_spad_info(&mut self) -> Result<(u8, u8), Error<I2C::Error>> {
        self.write_reg(0x80, 0x01)?;
        self.write_reg(0xFF, 0x01)?;
        self.write_reg(0x00, 0x00)?;

        self.write_reg(0xFF, 0x06)?;
        let mut tmp83 = self.read_reg(0x83)?;
        self.write_reg(0x83, tmp83 | 0x04)?;
        self.write_reg(0xFF, 0x07)?;
        self.write_reg(0x81, 0x01)?;

        self.write_reg(0x80, 0x01)?;

        self.write_reg(0x94, 0x6b)?;
        self.write_reg(0x83, 0x00)?;

        let mut c = 0;
        while self.read_reg(0x83)? == 0x00 {
            c += 1;
            if c == 65535 {
                return Err(Error::Timeout);
            }
        }

        self.write_reg(0x83, 0x01)?;
        let tmp = self.read_reg(0x92)?;

        let count: u8 = tmp & 0x7f;
        let type_is_aperture: u8 = (tmp >> 7) & 0x01;

        self.write_reg(0x81, 0x00)?;
        self.write_reg(0xFF, 0x06)?;
        tmp83 = self.read_reg(0x83)?;
        self.write_reg(0x83, tmp83 & !0x04)?;
        self.write_reg(0xFF, 0x01)?;
        self.write_reg(0x00, 0x01)?;

        self.write_reg(0xFF, 0x00)?;
        self.write_reg(0x80, 0x00)?;

        Ok((count, type_is_aperture))
    }

    /// `performSingleRefCalibration(uint8_t vhvInitByte)`
    fn perform_single_ref_calibration(&mut self, vhv_init_byte: u8) -> Result<(), Error<I2C::Error>> {
        // VL53L0X_REG_SYSRANGE_MODE_START_STOP
        self.write_reg(regs::SYSRANGE_START, 0x01 | vhv_init_byte)?;

        let mut c = 0;
        while (self.read_reg(regs::RESULT_INTERRUPT_STATUS)? & 0x07) == 0 {
            c += 1;
            if c == 10000 {
                return Err(Error::Timeout);
            }
        }
        self.write_reg(regs::SYSTEM_INTERRUPT_CLEAR, 0x01)?;
        self.write_reg(regs::SYSRANGE_START, 0x00)?;

        Ok(())
    }

    fn get_vcsel_pulse_period(&mut self, ty: VcselPeriodType) -> Result<u8, Error<I2C::Error>> {
        match ty {
            VcselPeriodType::PreRange => Ok(decode_vcsel_period(self.read_reg(regs::PRE_RANGE_CONFIG_VCSEL_PERIOD)?)),
            VcselPeriodType::FinalRange => Ok(decode_vcsel_period(
                self.read_reg(regs::FINAL_RANGE_CONFIG_VCSEL_PERIOD)?,
            )),
        }
    }

    /// `getSequenceStepEnables(VL53L0XSequenceStepEnables* enables)`
    fn get_sequence_step_enables(&mut self) -> Result<SeqStepEnables, Error<I2C::Error>> {
        let sequence_config = self.read_reg(regs::SYSTEM_SEQUENCE_CONFIG)?;
        Ok(SeqStepEnables {
            tcc: (sequence_config & 0x10) != 0,
            dss: (sequence_config & 0x08) != 0,
            msrc: (sequence_config & 0x04) != 0,
            pre_range: (sequence_config & 0x40) != 0,
            final_range: (sequence_config & 0x80) != 0,
        })
    }

    /// `getSequenceStepTimeouts(timeouts)`
    fn get_sequence_step_timeouts(&mut self, enables: &SeqStepEnables) -> Result<SeqStepTimeouts, Error<I2C::Error>> {
        let pre_range_mclks = decode_timeout(self.read_reg16(regs::PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI)?);
        let mut final_range_mclks = decode_timeout(self.read_reg16(regs::FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI)?);
        if enables.pre_range {
            final_range_mclks -= pre_range_mclks;
        }

        let pre_range_vcselperiod_pclks = self.get_vcsel_pulse_period(VcselPeriodType::PreRange)?;
        let msrc_dss_tcc_mclks = self.read_reg(regs::MSRC_CONFIG_TIMEOUT_MACROP)? + 1;
        let final_range_vcsel_period_pclks = self.get_vcsel_pulse_period(VcselPeriodType::FinalRange)?;

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

    fn load_tuning_settings(&mut self) -> Result<(), Error<I2C::Error>> {
        // -- VL53L0X_load_tuning_settings() begin
        for (reg, value) in DEFAULT_TUNING_SETTINGS {
            self.write_reg(*reg, *value)?;
        }

        Ok(())
        // -- VL53L0X_load_tuning_settings() end
    }

    fn power_on(&mut self) -> Result<(), Error<I2C::Error>> {
        // TODO: drive XSHUT to power the sensor on/off.
        Ok(())
    }

    fn init_hardware(&mut self) -> Result<(), Error<I2C::Error>> {
        self.power_on()?;

        // VL53L0X_DataInit() begin

        // The sensor uses 1V8 mode for I/O by default; switch to 2V8 mode.
        if self.io_mode2v8 {
            // set bit 0
            let ext_sup_hv = self.read_reg(regs::VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV)?;
            self.write_reg(regs::VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, ext_sup_hv | 0x01)?;
        }

        // "Set I2C standard mode"
        self.write_reg(0x88, 0x00)?;
        self.write_reg(0x80, 0x01)?;
        self.write_reg(0xFF, 0x01)?;
        self.write_reg(0x00, 0x00)?;
        self.stop_variable = self.read_reg(0x91)?;
        self.write_reg(0x00, 0x01)?;
        self.write_reg(0xFF, 0x00)?;
        self.write_reg(0x80, 0x00)?;

        // Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4)
        // limit checks.
        let config = self.read_reg(regs::MSRC_CONFIG_CONTROL)?;
        self.write_reg(regs::MSRC_CONFIG_CONTROL, config | 0x12)?;

        // Set the final range signal rate limit to 0.25 MCPS.
        self.set_signal_rate_limit(0.25)?;

        self.write_reg(regs::SYSTEM_SEQUENCE_CONFIG, 0xFF)?;

        // VL53L0X_DataInit() end

        // VL53L0X_StaticInit() begin

        let (spad_count, spad_type_is_aperture) = self.get_spad_info()?;

        // The SPAD map (RefGoodSpadMap) is read by
        // VL53L0X_get_info_from_device() in the API, but the same data is more
        // easily readable from GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so
        // read it from there.
        let mut ref_spad_map = [0u8; 6];
        self.read_regs(regs::GLOBAL_CONFIG_SPAD_ENABLES_REF_0, &mut ref_spad_map)?;

        // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

        self.write_reg(0xFF, 0x01)?;
        self.write_reg(regs::DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)?;
        self.write_reg(regs::DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)?;
        self.write_reg(0xFF, 0x00)?;
        self.write_reg(regs::GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)?;

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
        self.i2c.write(self.addr, &spad_write)?;

        // -- VL53L0X_set_reference_spads() end

        self.load_tuning_settings()?;

        // "Set interrupt config to new sample ready"
        // -- VL53L0X_SetGpioConfig() begin

        self.write_reg(regs::SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)?;
        // active low
        let high = self.read_reg(regs::GPIO_HV_MUX_ACTIVE_HIGH)?;
        self.write_reg(regs::GPIO_HV_MUX_ACTIVE_HIGH, high & !0x10)?;
        self.write_reg(regs::SYSTEM_INTERRUPT_CLEAR, 0x01)?;

        // -- VL53L0X_SetGpioConfig() end
        // "Disable MSRC and TCC by default"
        // MSRC = Minimum Signal Rate Check
        // TCC = Target Centre Check
        // -- VL53L0X_SetSequenceStepEnable() begin
        self.measurement_timing_budget_microseconds = self.get_measurement_timing_budget()?;
        self.write_reg(regs::SYSTEM_SEQUENCE_CONFIG, 0xE8)?;

        // -- VL53L0X_SetSequenceStepEnable() end

        // "Recalculate timing budget"
        let mtbm = self.measurement_timing_budget_microseconds;
        self.set_measurement_timing_budget(mtbm)?;

        // VL53L0X_StaticInit() end

        // VL53L0X_PerformRefCalibration() begin

        // -- VL53L0X_perform_vhv_calibration() begin
        self.write_reg(regs::SYSTEM_SEQUENCE_CONFIG, 0x01)?;
        self.perform_single_ref_calibration(0x40)?;
        // -- VL53L0X_perform_vhv_calibration() end
        // -- VL53L0X_perform_phase_calibration() begin

        self.write_reg(regs::SYSTEM_SEQUENCE_CONFIG, 0x02)?;
        self.perform_single_ref_calibration(0x00)?;

        // -- VL53L0X_perform_phase_calibration() end

        // "restore the previous Sequence Config"
        self.write_reg(regs::SYSTEM_SEQUENCE_CONFIG, 0xE8)?;

        // VL53L0X_PerformRefCalibration() end

        Ok(())
    }
}
