use embassy_time::{Duration, Timer};

const ADDRESS_BMP280: u8 = 0x77; //119

struct BMP280Config {
    dig_t1: f32,
    dig_t2: f32,
    dig_t3: f32,
    dig_p1: f32,
    dig_p2: f32,
    dig_p3: f32,
    dig_p4: f32,
    dig_p5: f32,
    dig_p6: f32,
    dig_p7: f32,
    dig_p8: f32,
    dig_p9: f32,
}

impl BMP280Config {
    fn from_buffer(calibration: &[u8; 26]) -> BMP280Config {
        BMP280Config {
            dig_t1: u16::from_le_bytes([calibration[0], calibration[1]]) as f32,
            dig_t2: i16::from_le_bytes([calibration[2], calibration[3]]) as f32,
            dig_t3: i16::from_le_bytes([calibration[4], calibration[5]]) as f32,
            dig_p1: u16::from_le_bytes([calibration[6], calibration[7]]) as f32,
            dig_p2: i16::from_le_bytes([calibration[8], calibration[9]]) as f32,
            dig_p3: i16::from_le_bytes([calibration[10], calibration[11]]) as f32,
            dig_p4: i16::from_le_bytes([calibration[12], calibration[13]]) as f32,
            dig_p5: i16::from_le_bytes([calibration[14], calibration[15]]) as f32,
            dig_p6: i16::from_le_bytes([calibration[16], calibration[17]]) as f32,
            dig_p7: i16::from_le_bytes([calibration[18], calibration[19]]) as f32,
            dig_p8: i16::from_le_bytes([calibration[20], calibration[21]]) as f32,
            dig_p9: i16::from_le_bytes([calibration[22], calibration[23]]) as f32,
        }
    }
}

pub struct Measurment {
    pub temp: f32,
    pub pressure: f32,
}

pub struct BMP280<I2C: embedded_hal_async::i2c::I2c> {
    driver: I2C,
    config: BMP280Config,
}

impl<I2C: embedded_hal_async::i2c::I2c> BMP280<I2C> {
    pub async fn new(mut driver: I2C) -> Result<Self, ()> {
        // read BMP280 calibration
        let mut calibration: [u8; 26] = [0; 26];
        let address: [u8; 1] = [0x88];
        if let Err(_res) = driver
            .write_read(ADDRESS_BMP280, &address, &mut calibration)
            .await
        {
            return Err(());
        }
        let config = BMP280Config::from_buffer(&calibration);

        Ok(Self { driver, config })
    }

    pub async fn measure(&mut self) -> Result<Measurment, ()> {
        // start measurement
        let start_buf: [u8; 2] = [0xF4, (0x01 << 5) | (0x01 << 2) | 0x01];
        if let Err(_res) = self.driver.write(ADDRESS_BMP280, &start_buf).await {
            return Err(());
        }

        // wait till measurement is finished
        Timer::after(Duration::from_millis(20)).await;
        // read measurements from sensor
        let mut ibuf: [u8; 6] = [0; 6];
        let address: [u8; 1] = [0xF7];
        if let Ok(_res) = self
            .driver
            .write_read(ADDRESS_BMP280, &address, &mut ibuf)
            .await
        {
            let pressure_raw: i32 =
                ((ibuf[0] as i32) << (12)) | ((ibuf[1] as i32) << 4) | ((ibuf[2] >> 4) as i32);
            let temperature_raw: i32 =
                ((ibuf[3] as i32) << (12)) | ((ibuf[4] as i32) << 4) | ((ibuf[5] >> 4) as i32);
            let (temp, t_fine) = self.compensate_temp(temperature_raw);
            let pressure = self.compensate_pressure(pressure_raw, t_fine);
            Ok(Measurment { temp, pressure })
        } else {
            Err(())
        }
    }

    fn compensate_temp(&self, raw: i32) -> (f32, f32) {
        let var1: f32 =
            (((raw as f32) / 16384.0) - (self.config.dig_t1 / 1024.0)) * self.config.dig_t2;
        let var2: f32 = (((raw as f32) / 131072.0 - (self.config.dig_t1) / 8192.0)
            * ((raw as f32) / 131072.0 - (self.config.dig_t1) / 8192.0))
            * (self.config.dig_t3);
        let t_fine = var1 + var2;
        ((var1 + var2) / 5120.0, t_fine)
    }

    fn compensate_pressure(&self, raw: i32, t_fine: f32) -> f32 {
        let var1 = (t_fine / 2.0) - 64000.0;
        let var2 = var1 * var1 * (self.config.dig_p6) / 32768.0;
        let var2 = var2 + var1 * (self.config.dig_p5) * 2.0;
        let var2 = (var2 / 4.0) + ((self.config.dig_p4) * 65536.0);
        let var1 = ((self.config.dig_p3) * var1 * var1 / 524288.0 + (self.config.dig_p2) * var1)
            / 524288.0;
        let var1 = (1.0 + var1 / 32768.0) * (self.config.dig_p1);
        if var1 == 0.0 {
            // avoid exception caused by division by zero
            return 0.0;
        }
        let p = 1048576.0 - (raw as f32);
        let p = (p - (var2 / 4096.0)) * 6250.0 / var1;
        let var1 = (self.config.dig_p9) * p * p / 2147483648.0;
        let var2 = p * (self.config.dig_p8) / 32768.0;
        p + (var1 + var2 + (self.config.dig_p7)) / 16.0
    }
}
