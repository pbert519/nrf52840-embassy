// lsm6ds22 int P1.11 -> GPIO Input
// apds int P1.00 -> GPIO Input
// radio

const _ADDRESS_APDS9960: u8 = 0x39; // 57
const _ADDRESS_LSM6DS33: u8 = 0x6A; // 106
const _ADDRESS_LIS3MDL: u8 = 0x1C; // 28

use embassy_nrf::{
    bind_interrupts,
    gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin, Pull},
    gpiote::{InputChannel, InputChannelPolarity},
    interrupt::{self, InterruptExt, Priority},
    pdm,
    peripherals::{self, GPIOTE_CH0, QSPI, SPI3, TWISPI0, TWISPI1},
    qspi, saadc, spim, twim,
};

bind_interrupts!(struct Irqs {
    PDM => pdm::InterruptHandler<peripherals::PDM>;
    QSPI => qspi::InterruptHandler<peripherals::QSPI>;
    SAADC => saadc::InterruptHandler;
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => twim::InterruptHandler<peripherals::TWISPI0>;
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1 => twim::InterruptHandler<peripherals::TWISPI1>;
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

pub const EXTERNAL_FLASH_SIZE: u32 = 2097152; // 2048kByte
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
    pub adc: saadc::Saadc<'static, 1>,
    /// onbard pdm microphon, default settings
    pub pdm: pdm::Pdm<'static, peripherals::PDM>,
    /// transmit only spi, conected to neopixel led onboard
    pub spi: spim::Spim<'static, SPI3>,
    /// twi interface connects to onbard i2cs, see documentation
    pub twim: twim::Twim<'static, TWISPI0>,
    /// additonal i2c interface for external oled screen
    pub twim_disp: twim::Twim<'static, TWISPI1>,
    /// qspi interface for onbard flash
    pub qspi: qspi::Qspi<'static, QSPI>,
}

impl Board {
    pub fn init() -> Board {
        let mut embassy_config = embassy_nrf::config::Config::default();
        embassy_config.time_interrupt_priority = interrupt::Priority::P2;
        embassy_config.gpiote_interrupt_priority = interrupt::Priority::P7;
        let mut p = embassy_nrf::init(embassy_config);

        interrupt::PDM.set_priority(Priority::P3);
        interrupt::QSPI.set_priority(Priority::P3);
        interrupt::SAADC.set_priority(Priority::P3);
        interrupt::SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0.set_priority(Priority::P3);
        interrupt::SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1.set_priority(Priority::P3);
        interrupt::SPIM3.set_priority(Priority::P3);

        // configure gpio
        let led_d13 = Output::new(p.P1_09.degrade(), Level::Low, OutputDrive::Standard);
        let led2 = Output::new(p.P1_10.degrade(), Level::Low, OutputDrive::Standard);
        let switch = InputChannel::new(
            p.GPIOTE_CH0,
            Input::new(p.P1_02.degrade(), Pull::Up),
            InputChannelPolarity::HiToLo,
        );

        // configure adc
        let adc_config = saadc::Config::default();
        let channel_config = saadc::ChannelConfig::single_ended(&mut p.P0_29);
        let adc = saadc::Saadc::new(p.SAADC, Irqs, adc_config, [channel_config]);

        // configure twi
        let twim_config = twim::Config::default();
        let twim = twim::Twim::new(p.TWISPI0, Irqs, p.P0_12, p.P0_11, twim_config);

        // configure twi
        let twim_config_disp = twim::Config::default();
        let twim_disp = twim::Twim::new(p.TWISPI1, Irqs, p.P1_08, p.P0_07, twim_config_disp);

        // the mic samples with 16 kHz
        let pdm_config = pdm::Config::default();
        let pdm = pdm::Pdm::new(p.PDM, Irqs, p.P0_01, p.P0_00, pdm_config);

        // qspi flash interface
        let mut qspi_config = qspi::Config::default();
        qspi_config.capacity = EXTERNAL_FLASH_SIZE;
        qspi_config.read_opcode = qspi::ReadOpcode::READ4IO;
        qspi_config.write_opcode = qspi::WriteOpcode::PP4O;
        qspi_config.write_page_size = qspi::WritePageSize::_256BYTES;
        let mut qspi = qspi::Qspi::new(
            p.QSPI,
            Irqs,
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
        let spi = spim::Spim::new_txonly(p.SPI3, Irqs, p.P0_15, p.P0_16, spi_config);

        // usb
        // let usb_irq = interrupt::take!(USBD);
        // usb_irq.set_priority(Priority::P3);
        // let usb_power_irq = interrupt::take!(POWER_CLOCK);
        // usb_power_irq.set_priority(Priority::P3);
        // let usb_power = embassy_nrf::usb::PowerUsb::new(usb_power_irq);
        // let _usb_driver = embassy_nrf::usb::Driver::new(p.USBD, usb_irq, usb_power);

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
