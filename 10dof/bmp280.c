#include <math.h>
#include <stdio.h>
#include "bmp280.h"

static bool bmp280_read_calibration_data(bmp280_t *dev);
static int32_t bmp280_compensate_T(bmp280_t *dev, int32_t adc_T);
static uint32_t bmp280_compensate_P(bmp280_t *dev, int32_t adc_P);

bool bmp280_init(bmp280_t *dev, i2c_inst_t *i2c, uint8_t addr) {
    dev->i2c = i2c;
    dev->addr = addr;
    dev->t_fine = 0;
    
    // Check chip ID
    uint8_t chip_id;
    uint8_t reg_chip_id = BMP280_REG_CHIP_ID;
    i2c_write_blocking(dev->i2c, dev->addr, &reg_chip_id, 1, true);
    i2c_read_blocking(dev->i2c, dev->addr, &chip_id, 1, false);
    
    if (chip_id != BMP280_CHIP_ID) {
        return false;
    }
    
    // Read calibration data
    if (!bmp280_read_calibration_data(dev)) {
        return false;
    }
    
    // Configure: normal mode, 16x pressure oversampling, 2x temperature oversampling
    uint8_t config_data[2];
    uint8_t ctrl_meas_data[2];
    
    // Configure register
    config_data[0] = BMP280_REG_CONFIG;
    config_data[1] = (0x04 << 5) | (0x05 << 2); // t_sb=0x04 (1000ms), filter=0x05 (16)
    i2c_write_blocking(dev->i2c, dev->addr, config_data, 2, false);
    
    // Control measurement register
    ctrl_meas_data[0] = BMP280_REG_CTRL_MEAS;
    ctrl_meas_data[1] = (0x03 << 5) | (0x02 << 2) | 0x03; // osrs_p=0x03, osrs_t=0x02, mode=0x03 (normal)
    i2c_write_blocking(dev->i2c, dev->addr, ctrl_meas_data, 2, false);
    
    return true;
}

bool bmp280_read(bmp280_t *dev, float *temperature, float *pressure) {
    uint8_t reg_press_msb = BMP280_REG_PRESS_MSB;
    uint8_t data[6];
    
    // Read pressure and temperature data
    i2c_write_blocking(dev->i2c, dev->addr, &reg_press_msb, 1, true);
    i2c_read_blocking(dev->i2c, dev->addr, data, 6, false);
    
    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    
    // Compensate temperature
    int32_t comp_temp = bmp280_compensate_T(dev, adc_T);
    *temperature = comp_temp / 100.0f;
    
    // Compensate pressure
    uint32_t comp_pressure = bmp280_compensate_P(dev, adc_P);
    *pressure = comp_pressure / 256.0f / 100.0f; // Convert to hPa
    
    return true;
}

float bmp280_calculate_altitude(float pressure, float sea_level_pressure) {
    if (pressure <= 0) return 0.0f;
    // Barometric formula: h = 44330 * (1 - (P/P0)^(1/5.255))
    return 44330.0f * (1.0f - powf(pressure / sea_level_pressure, 0.1903f));
}

static bool bmp280_read_calibration_data(bmp280_t *dev) {
    uint8_t reg_calib = BMP280_REG_CALIB;
    uint8_t calib_data[24];
    
    i2c_write_blocking(dev->i2c, dev->addr, &reg_calib, 1, true);
    i2c_read_blocking(dev->i2c, dev->addr, calib_data, 24, false);
    
    dev->dig_T1 = (calib_data[1] << 8) | calib_data[0];
    dev->dig_T2 = (calib_data[3] << 8) | calib_data[2];
    dev->dig_T3 = (calib_data[5] << 8) | calib_data[4];
    
    dev->dig_P1 = (calib_data[7] << 8) | calib_data[6];
    dev->dig_P2 = (calib_data[9] << 8) | calib_data[8];
    dev->dig_P3 = (calib_data[11] << 8) | calib_data[10];
    dev->dig_P4 = (calib_data[13] << 8) | calib_data[12];
    dev->dig_P5 = (calib_data[15] << 8) | calib_data[14];
    dev->dig_P6 = (calib_data[17] << 8) | calib_data[16];
    dev->dig_P7 = (calib_data[19] << 8) | calib_data[18];
    dev->dig_P8 = (calib_data[21] << 8) | calib_data[20];
    dev->dig_P9 = (calib_data[23] << 8) | calib_data[22];
    
    return true;
}

static int32_t bmp280_compensate_T(bmp280_t *dev, int32_t adc_T) {
    int32_t var1, var2, T;
    
    var1 = ((((adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) * ((int32_t)dev->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dev->dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->dig_T1))) >> 12) * ((int32_t)dev->dig_T3)) >> 14;
    
    dev->t_fine = var1 + var2;
    T = (dev->t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t bmp280_compensate_P(bmp280_t *dev, int32_t adc_P) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int32_t)dev->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->dig_P1) >> 33;
    
    if (var1 == 0) return 0;
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->dig_P8) * p) >> 19;
    
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->dig_P7) << 4);
    return (uint32_t)p;
}

float bmp280_calculate_sea_level_pressure(float measured_pressure, float actual_altitude) {
    // Inverse barometric formula: P0 = P / (1 - h/44330)^5.255
    return measured_pressure / powf(1.0f - (actual_altitude / 44330.0f), 5.255f);
}

// Calibrate sensor with known altitude
float bmp280_calibrate_altitude(bmp280_t *dev, float actual_altitude) {
    float temperature, pressure;
    if (!bmp280_read(dev, &temperature, &pressure)) {
        return 0.0f;
    }
    
    // Calculate what sea level pressure would give correct altitude
    float sea_level_pressure = bmp280_calculate_sea_level_pressure(pressure, actual_altitude);
    printf("Calibrated sea level pressure: %.2f hPa\n", sea_level_pressure);
    
    return sea_level_pressure;
}
