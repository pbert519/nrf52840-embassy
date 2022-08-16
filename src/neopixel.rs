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
