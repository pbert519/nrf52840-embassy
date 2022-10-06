// lsm6ds22 int P1.11 -> GPIO Input
// apds int P1.00 -> GPIO Input
// radio

const _ADDRESS_APDS9960: u8 = 0x39; // 57
const _ADDRESS_LSM6DS33: u8 = 0x6A; // 106
const _ADDRESS_LIS3MDL: u8 = 0x1C; // 28

use embassy_nrf::{
    gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin, Pull},
    gpiote::{InputChannel, InputChannelPolarity},
    interrupt,
    pdm::Pdm,
    peripherals::{GPIOTE_CH0, QSPI, SPI3, TWISPI0, TWISPI1},
    qspi::*,
    saadc::Saadc,
    spim::Spim,
    twim::Twim,
};

pub const EXTERNAL_FLASH_SIZE: usize = 2097152; // 2048kByte
pub const EXTERNAL_FLASH_PAGE_SIZE: usize = 256; // 256Byte per programmable flash page
pub const _EXTERNAL_FLASH_BLOCK_SIZE: usize = 4096; // 4096Byte sector are the smallest unit to erase

pub struct Board {
    /// onboard red led
    pub led_d13: Output<'static, AnyPin>,
    /// onboard blue led
    pub led2: Output<'static, AnyPin>,
    /// onboard switch, triggers on High to Low (Pressed)
    pub switch: InputChannel<'static, GPIOTE_CH0, AnyPin>,
    /// Adc is configured with one channel to measure battery voltage
    pub adc: Saadc<'static, 1>,
    /// onbard pdm microphon, default settings
    pub pdm: Pdm<'static>,
    /// transmit only spi, conected to neopixel led onboard
    pub spi: Spim<'static, SPI3>,
    /// twi interface connects to onbard i2cs, see documentation
    pub twim: Twim<'static, TWISPI0>,
    /// additonal i2c interface for external oled screen
    pub twim_disp: Twim<'static, TWISPI1>,
    /// qspi interface for onbard flash
    pub qspi: Qspi<'static, QSPI, EXTERNAL_FLASH_SIZE>,
}

impl Board {
    pub fn configure(mut p: embassy_nrf::Peripherals) -> Board {
        // configure gpio
        let led_d13 = Output::new(p.P1_09.degrade(), Level::Low, OutputDrive::Standard);
        let led2 = Output::new(p.P1_10.degrade(), Level::Low, OutputDrive::Standard);
        let switch = InputChannel::new(
            p.GPIOTE_CH0,
            Input::new(p.P1_02.degrade(), Pull::Up),
            InputChannelPolarity::HiToLo,
        );

        // configure adc
        let adc_config = embassy_nrf::saadc::Config::default();
        let channel_config = embassy_nrf::saadc::ChannelConfig::single_ended(&mut p.P0_29);
        let adc = embassy_nrf::saadc::Saadc::new(
            p.SAADC,
            interrupt::take!(SAADC),
            adc_config,
            [channel_config],
        );

        // configure twi
        let twim_config = embassy_nrf::twim::Config::default();
        let twim_irq = interrupt::take!(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0);
        let twim = Twim::new(p.TWISPI0, twim_irq, p.P0_12, p.P0_11, twim_config);

        // configure twi
        let twim_config_disp = embassy_nrf::twim::Config::default();
        let twim_irq_disp = interrupt::take!(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1);
        let twim_disp = Twim::new(p.TWISPI1, twim_irq_disp, p.P1_08, p.P0_07, twim_config_disp);

        // the mic samples with 16 kHz
        let pdm_config = embassy_nrf::pdm::Config::default();
        let pdm_irq = interrupt::take!(PDM);
        let pdm = Pdm::new(p.PDM, pdm_irq, p.P0_01, p.P0_00, pdm_config);

        // qspi flash interface
        let mut qspi_config = embassy_nrf::qspi::Config::default();
        qspi_config.read_opcode = ReadOpcode::READ4IO;
        qspi_config.write_opcode = WriteOpcode::PP4O;
        qspi_config.write_page_size = WritePageSize::_256BYTES;
        let qspi_irq = interrupt::take!(QSPI);
        let mut qspi: Qspi<'_, _, EXTERNAL_FLASH_SIZE> = Qspi::new(
            p.QSPI,
            qspi_irq,
            p.P0_19,
            p.P0_20,
            p.P0_17,
            p.P0_22,
            p.P0_23,
            p.P0_21,
            qspi_config,
        );
        let mut status = [0; 2];
        // Read status register
        qspi.blocking_custom_instruction(0x05, &[], &mut status[..1])
            .unwrap();
        qspi.blocking_custom_instruction(0x35, &[], &mut status[1..2])
            .unwrap();
        // bit 9 is quad enable
        if status[1] & 0x02 == 0 {
            status[1] = 0x02;
            // Write status register to enable quad spi
            qspi.blocking_custom_instruction(0x01, &status, &mut [])
                .unwrap();
        }

        // spi
        let mut spi_config = embassy_nrf::spim::Config::default();
        spi_config.frequency = embassy_nrf::spim::Frequency::M4;
        let spi_irq = interrupt::take!(SPIM3);
        let spi = Spim::new_txonly(p.SPI3, spi_irq, p.P0_15, p.P0_16, spi_config);

        // usb
        let usb_irq = interrupt::take!(USBD);
        let usb_power_irq = interrupt::take!(POWER_CLOCK);
        let usb_power = embassy_nrf::usb::PowerUsb::new(usb_power_irq);
        let _usb_driver = embassy_nrf::usb::Driver::new(p.USBD, usb_irq, usb_power);

        Board {
            led_d13,
            led2,
            switch,
            adc,
            pdm,
            spi,
            twim,
            qspi,
            twim_disp,
        }
    }
}
