#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

bool mpu6050_reset() {
    uint8_t buf[] = {MPU6050_REG_PWR_MGMT_1, MPU6050_VAL_DEVICE_RESET};
    i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 2, BUS_RELEASE);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
    i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 2, BUS_RELEASE);
    sleep_ms(10); // Allow stabilization after waking up

    uint8_t identity;
    buf[0] = MPU6050_REG_WHO_AM_I;
    i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 1, BUS_CONTROL);
    i2c_read_blocking(i2c_default, MPU6050_ADDR, &identity, 1, BUS_RELEASE);
    return identity == 0x68;
}

void mpu6050_read_raw(int16_t accel[DIM], int16_t gyro[DIM]) {
    uint8_t buffer[6];
    uint8_t val = MPU6050_REG_ACCEL_XOUT_H;
    i2c_write_blocking(i2c_default, MPU6050_ADDR, &val, 1, BUS_CONTROL);
    i2c_read_blocking(i2c_default, MPU6050_ADDR, buffer, 6, BUS_RELEASE);
    for (int i = 0; i < DIM; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = MPU6050_REG_GYRO_XOUT_H;
    i2c_write_blocking(i2c_default, MPU6050_ADDR, &val, 1, BUS_CONTROL);
    i2c_read_blocking(i2c_default, MPU6050_ADDR, buffer, 6, BUS_RELEASE);
    for (int i = 0; i < DIM; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

void mpu6050_scale(int16_t *raw, float *scaled, float scale_factor, int count) {
    for (int i = 0; i < count; i++) {
        scaled[i] = raw[i] / scale_factor;
    }
}

float complement(float accel1, float accel2, float gyro, float prev_val, float elapsed) {
    float accel_tilt = atan2f(accel1, accel2);
    if (elapsed < 0) {
        return accel_tilt;
    }
    float gyro_tilt = (prev_val + gyro * elapsed);
    return CMPL_GYRO * gyro_tilt + CMPL_ACCEL * accel_tilt;
}
