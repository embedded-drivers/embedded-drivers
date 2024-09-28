use embedded_hal::digital::OutputPin;

use crate::{cmds, HEIGHT, WIDTH};

/// SSD1327 driver, blocking mode
///
/// Framebuffer format:
///
/// ```
/// let mut fb = Framebuffer::<Gray4, _, LittleEndian, 128, 128, { embedded_graphics::framebuffer::buffer_size::<Gray4>(128, 128) }>::new();
/// // or
/// let mut fb = Framebuffer::<Gray4, _, LittleEndian, 128, 128, { 128 * 128 / 2 }>::new();
/// ```
pub struct SSD1327<SPI: embedded_hal::spi::SpiBus, DC: OutputPin, CS: OutputPin> {
    spi: SPI,
    dc: DC,
    cs: CS,
}

impl<SPI: embedded_hal::spi::SpiBus, DC: OutputPin, CS: OutputPin> SSD1327<SPI, DC, CS> {
    pub fn new(spi: SPI, cs: CS, dc: DC) -> Self {
        Self { spi, cs, dc }
    }

    pub fn init(&mut self) {
        self.write_command(&[cmds::SET_DISPLAY_OFF]);
        // 0x3F
        self.write_command(&[cmds::SET_COLUMN_ADDRESS, 0x00, WIDTH / 8 * 4 - 1]);
        // 0x7F
        self.write_command(&[cmds::SET_ROW_ADDRESS, 0x00, HEIGHT - 1]);
        self.write_command(&[cmds::SET_CONTRAST_CURRENT, 0x80]);

        // address remap
        self.write_command(&[cmds::SET_REMAP, 0x51]);

        self.write_command(&[cmds::SET_DISPLAY_START_LINE, 0x00]);
        self.write_command(&[cmds::SET_DISPLAY_OFFSET, 0x00]);

        self.write_command(&[cmds::SET_MULTIPLEX_RATIO, 0x7F]);
        self.write_command(&[cmds::SET_PHASE_LENGTH, 0x11]); // gray scale tune

        // gamma setting
        // 0xb8: SET_GRAY_SCALE_TABLE, [u8; 15]
        // 0xb9: SET_DEFAULT_LINEAR_GRAY_SCALE_TABLE
        //self.send_cmd_data(0xb8, &[1,2,30,40,5,6,7,8,9,10,11,12,13,14,0b11111])?;
        self.write_command(&[cmds::SELECT_DEFAULT_LINEAR_GRAY_SCALE_TABLE]);

        self.write_command(&[cmds::SET_FRONT_CLOCK_DIVIDER, 0x00]);
        self.write_command(&[cmds::FUNCTION_SELECTION_A, 0x01]);
        self.write_command(&[cmds::SET_SECOND_PRECHARGE_PERIOD, 0x08]);
        self.write_command(&[cmds::SET_VCOMH_VOLTAGE, 0x0f]);
        self.write_command(&[cmds::SET_PRECHARGE_VOLTAGE, 0x08]);
        self.write_command(&[cmds::FUNCTION_SELECTION_B, 0x62]);
        self.write_command(&[cmds::SET_COMMAND_LOCK, 0x12]);
        self.write_command(&[cmds::SET_DISPLAY_MODE]); // display mode normal
        self.write_command(&[cmds::SET_DISPLAY_ON]);

        self.write_command(&[cmds::DEACTIVATE_SCROLL]);
    }

    pub fn write_framebuffer(&mut self, fb: &[u8]) {
        //self.write_command(&[cmds::SET_COLUMN_ADDRESS, 0x00, 0x3F])
        //    ;
        //self.write_command(&[cmds::SET_ROW_ADDRESS, 0x00, 0x7F])
        //    ;
        self.write_data(fb);
    }

    fn write_command(&mut self, cmd: &[u8]) {
        self.dc.set_low().unwrap();
        self.cs.set_low().unwrap();
        self.spi.write(cmd).unwrap();
        self.cs.set_high().unwrap();
    }

    fn write_data(&mut self, data: &[u8]) {
        self.dc.set_high().unwrap();
        self.cs.set_low().unwrap();
        self.spi.write(data).unwrap();
        self.cs.set_high().unwrap();

        self.dc.set_low().unwrap();
    }
}
