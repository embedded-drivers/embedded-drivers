//! Driver for SSD1327.

#![no_std]

use embedded_hal::digital::OutputPin;

pub mod blocking;

pub mod cmds {
    // Fundamental Commands
    pub const SET_COLUMN_ADDRESS: u8 = 0x15;
    pub const SET_ROW_ADDRESS: u8 = 0x75;
    pub const SET_CONTRAST_CURRENT: u8 = 0x81;
    pub const SET_REMAP: u8 = 0xA0;
    pub const SET_DISPLAY_START_LINE: u8 = 0xA1;
    pub const SET_DISPLAY_OFFSET: u8 = 0xA2;
    pub const SET_DISPLAY_MODE: u8 = 0xA4;
    pub const SET_MULTIPLEX_RATIO: u8 = 0xA8;
    pub const FUNCTION_SELECTION_A: u8 = 0xAB;
    pub const SET_DISPLAY_ON: u8 = 0xAF;
    pub const SET_DISPLAY_OFF: u8 = 0xAE;
    pub const SET_PHASE_LENGTH: u8 = 0xB1;
    pub const SET_FRONT_CLOCK_DIVIDER: u8 = 0xB3;
    pub const SET_GPIO: u8 = 0xB5;
    pub const SET_SECOND_PRECHARGE_PERIOD: u8 = 0xB6;
    pub const SET_GRAY_SCALE_TABLE: u8 = 0xB8;
    pub const SELECT_DEFAULT_LINEAR_GRAY_SCALE_TABLE: u8 = 0xB9;
    pub const SET_PRECHARGE_VOLTAGE: u8 = 0xBC;
    pub const SET_VCOMH_VOLTAGE: u8 = 0xBE;
    pub const FUNCTION_SELECTION_B: u8 = 0xD5;
    pub const SET_COMMAND_LOCK: u8 = 0xFD;

    // Graphic Acceleration Commands
    pub const HORIZONTAL_SCROLL_SETUP: u8 = 0x26;
    pub const HORIZONTAL_SCROLL_SETUP_ALT: u8 = 0x27; // Alternative command for Horizontal Scroll Setup
    pub const DEACTIVATE_SCROLL: u8 = 0x2E;
    pub const ACTIVATE_SCROLL: u8 = 0x2F;
}

pub(crate) const WIDTH: u8 = 128;
pub(crate) const HEIGHT: u8 = 128;

/// SSD1327 driver
///
/// Framebuffer format:
///
/// ```
/// let mut fb = Framebuffer::<Gray4, _, LittleEndian, 128, 128, { embedded_graphics::framebuffer::buffer_size::<Gray4>(128, 128) }>::new();
/// // or
/// let mut fb = Framebuffer::<Gray4, _, LittleEndian, 128, 128, { 128 * 128 / 2 }>::new();
/// ```
pub struct SSD1327<SPI: embedded_hal_async::spi::SpiBus, DC: OutputPin, CS: OutputPin> {
    spi: SPI,
    dc: DC,
    cs: CS,
}

impl<SPI: embedded_hal_async::spi::SpiBus, DC: OutputPin, CS: OutputPin> SSD1327<SPI, DC, CS> {
    pub fn new(spi: SPI, cs: CS, dc: DC) -> Self {
        Self { spi, cs, dc }
    }

    pub async fn init(&mut self) {
        self.write_command(&[cmds::SET_DISPLAY_OFF]).await;
        // 0x3F
        self.write_command(&[cmds::SET_COLUMN_ADDRESS, 0x00, WIDTH / 8 * 4 - 1])
            .await;
        // 0x7F
        self.write_command(&[cmds::SET_ROW_ADDRESS, 0x00, HEIGHT - 1]).await;
        self.write_command(&[cmds::SET_CONTRAST_CURRENT, 0x80]).await;

        // address remap
        self.write_command(&[cmds::SET_REMAP, 0x51]).await;

        self.write_command(&[cmds::SET_DISPLAY_START_LINE, 0x00]).await;
        self.write_command(&[cmds::SET_DISPLAY_OFFSET, 0x00]).await;

        self.write_command(&[cmds::SET_MULTIPLEX_RATIO, 0x7F]).await;
        self.write_command(&[cmds::SET_PHASE_LENGTH, 0x11]).await; // gray scale tune

        // gamma setting
        // 0xb8: SET_GRAY_SCALE_TABLE, [u8; 15]
        // 0xb9: SET_DEFAULT_LINEAR_GRAY_SCALE_TABLE
        //self.send_cmd_data(0xb8, &[1,2,30,40,5,6,7,8,9,10,11,12,13,14,0b11111])?;
        self.write_command(&[cmds::SELECT_DEFAULT_LINEAR_GRAY_SCALE_TABLE])
            .await;

        self.write_command(&[cmds::SET_FRONT_CLOCK_DIVIDER, 0x00]).await;
        self.write_command(&[cmds::FUNCTION_SELECTION_A, 0x01]).await;
        self.write_command(&[cmds::SET_SECOND_PRECHARGE_PERIOD, 0x08]).await;
        self.write_command(&[cmds::SET_VCOMH_VOLTAGE, 0x0f]).await;
        self.write_command(&[cmds::SET_PRECHARGE_VOLTAGE, 0x08]).await;
        self.write_command(&[cmds::FUNCTION_SELECTION_B, 0x62]).await;
        self.write_command(&[cmds::SET_COMMAND_LOCK, 0x12]).await;
        self.write_command(&[cmds::SET_DISPLAY_MODE]).await; // display mode normal
        self.write_command(&[cmds::SET_DISPLAY_ON]).await;

        self.write_command(&[cmds::DEACTIVATE_SCROLL]).await;
    }

    pub async fn write_framebuffer(&mut self, fb: &[u8]) {
        //self.write_command(&[cmds::SET_COLUMN_ADDRESS, 0x00, 0x3F])
        //    .await;
        //self.write_command(&[cmds::SET_ROW_ADDRESS, 0x00, 0x7F])
        //    .await;
        self.write_data(fb).await;
    }

    async fn write_command(&mut self, cmd: &[u8]) {
        self.dc.set_low().unwrap();
        self.cs.set_low().unwrap();
        self.spi.write(cmd).await.unwrap();
        self.cs.set_high().unwrap();
    }

    async fn write_data(&mut self, data: &[u8]) {
        self.dc.set_high().unwrap();
        self.cs.set_low().unwrap();
        self.spi.write(data).await.unwrap();
        self.cs.set_high().unwrap();

        self.dc.set_low().unwrap();
    }
}
