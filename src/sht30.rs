use embassy_executor::time::{Duration, Timer};

const ADDRESS_SHT30: u8 = 0x44; // 68

pub struct SHT30<I2C: embedded_hal_async::i2c::I2c> {
    driver: I2C,
}

impl<I2C: embedded_hal_async::i2c::I2c> SHT30<I2C> {
    pub fn new(driver: I2C) -> Self {
        Self { driver }
    }

    pub async fn measure(&mut self) {
        let buf: [u8; 2] = [0x24, 0x00];
        if let Err(_res) = self.driver.write(ADDRESS_SHT30, &buf).await {
            defmt::info!("Error write SHT I2C Command, retry in 100ms");
            Timer::after(Duration::from_millis(100)).await;
            return;
        }
        // measurement duration is maximal 15ms
        Timer::after(Duration::from_millis(20)).await;
        let mut ibuf: [u8; 6] = [0; 6];
        // use trait over method
        if let Ok(_res) = self.driver.read(ADDRESS_SHT30, &mut ibuf).await {
            //defmt::info!("Got buffer {}", ibuf);
            let raw_temp = u16::from_le_bytes([ibuf[1], ibuf[0]]) as f32;
            let temp: f32 = -45.0 + 175.0 * raw_temp / 65535.0;
            let raw_humidity = u16::from_le_bytes([ibuf[4], ibuf[3]]) as f32;
            let humidity: f32 = 100.0 * raw_humidity / 65535.0;
            defmt::info!("Temperature: {} and Humidity: {}", temp, humidity);
        } else {
            defmt::info!("Error while reading SHT30");
        }
    }
}
