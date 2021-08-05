pub struct BMP280Config {
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
    pub fn from_buffer(calibration: &[u8; 26]) -> BMP280Config {
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

    pub fn example() -> BMP280Config {
        BMP280Config {
            dig_t1: 27504.0,
            dig_t2: 26435.0,
            dig_t3: -1000.0,
            dig_p1: 36477.0,
            dig_p2: -10685.0,
            dig_p3: 3024.0,
            dig_p4: 2855.0,
            dig_p5: 140.0,
            dig_p6: -7.0,
            dig_p7: 15500.0,
            dig_p8: -14600.0,
            dig_p9: 6000.0,
        }
    }
}

pub fn bmp280_compensate_temp(raw: i32, config: &BMP280Config) -> (f32, f32) {
    let var1: f32 = (((raw as f32) / 16384.0) - (config.dig_t1 / 1024.0)) * config.dig_t2;
    let var2: f32 = (((raw as f32) / 131072.0 - (config.dig_t1) / 8192.0)
        * ((raw as f32) / 131072.0 - (config.dig_t1) / 8192.0))
        * (config.dig_t3);
    let t_fine = (var1 + var2) as f32;
    ((var1 + var2) / 5120.0, t_fine)
}

pub fn bmp280_compensate_pressure(raw: i32, t_fine: f32, config: &BMP280Config) -> f32 {
    let var1 = (t_fine / 2.0) - 64000.0;
    let var2 = var1 * var1 * (config.dig_p6) / 32768.0;
    let var2 = var2 + var1 * (config.dig_p5) * 2.0;
    let var2 = (var2 / 4.0) + ((config.dig_p4) * 65536.0);
    let var1 = ((config.dig_p3) * var1 * var1 / 524288.0 + (config.dig_p2) * var1) / 524288.0;
    let var1 = (1.0 + var1 / 32768.0) * (config.dig_p1);
    if var1 == 0.0 {
        // avoid exception caused by division by zero
        return 0.0;
    }
    let p = 1048576.0 - (raw as f32);
    let p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    let var1 = (config.dig_p9) * p * p / 2147483648.0;
    let var2 = p * (config.dig_p8) / 32768.0;
    p + (var1 + var2 + (config.dig_p7)) / 16.0
}
