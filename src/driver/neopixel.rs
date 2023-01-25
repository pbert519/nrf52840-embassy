// every bit is 1.25us
// bit 0 -> 0.35us High, 0.9us  Low
// bit 1 -> 0.9us  High, 0.35us Low
// every led has 24bits -> G8:R8:B8
// low for > 5us is reset -> pixel is shown
// using spi to generate timings:
// only mosi is necessary
// 4bits (spi) per bit (led)
// 3.2 Mhz Spi clk -> each Spi bit is 0.3125us -> each led bit is 1.25us
// low  -> 0x08 = 0b0001000 -> 0.3125us high, 0.9375 low
// high -> 0x0E = 0b0001110 -> 0.9375us high, 0.3125 low

pub struct Neopixel<SPI: embedded_hal_async::spi::SpiBusWrite> {
    driver: SPI,
}

impl<SPI: embedded_hal_async::spi::SpiBusWrite> Neopixel<SPI> {
    pub fn new(driver: SPI) -> Self {
        Self { driver }
    }

    pub async fn set_pixel(&mut self, red: u8, green: u8, blue: u8) {
        let mut buf: [u8; 12] = [0x88; 12];
        Self::u8_to_wscolor(green, &mut buf[0..4]);
        Self::u8_to_wscolor(red, &mut buf[4..8]);
        Self::u8_to_wscolor(blue, &mut buf[8..12]);

        self.driver.write(&buf).await.unwrap();
    }

    fn u8_to_wscolor(color: u8, bytes: &mut [u8]) {
        for i in 0..8 {
            let mut bit: u8 = 0x08;
            if (color & (1 << (7 - i))) > 0 {
                bit = 0x0E;
            }
            let index: usize = i / 2;
            if i % 2 > 0 {
                bytes[index] |= bit;
            } else {
                bytes[index] |= bit << 4;
            }
        }
    }
}
