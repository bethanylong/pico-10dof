#ifndef BMP280_H
#define BMP280_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define BMP280_I2C_ADDR 0x76
#define BMP280_CHIP_ID 0x58

// BMP280 registers
#define BMP280_REG_TEMP_XLSB   0xFC
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_PRESS_XLSB  0xF9
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_CHIP_ID     0xD0
#define BMP280_REG_CALIB       0x88

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    
    // Calibration data
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    
    int32_t t_fine;
} bmp280_t;

bool bmp280_init(bmp280_t *dev, i2c_inst_t *i2c, uint8_t addr);
bool bmp280_read(bmp280_t *dev, float *temperature, float *pressure);
float bmp280_calculate_altitude(float pressure, float sea_level_pressure);
float bmp280_calibrate_altitude(bmp280_t *dev, float actual_altitude);

#endif
