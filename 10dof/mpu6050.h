#ifndef MPU6050_H
#define MPU6050_H

#define DIM 3
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

#define DEG_PER_RAD (180.0f / 3.14159f)
#define US_PER_SEC 1000000.0f
#define CMPL_GYRO 0.98f
#define CMPL_ACCEL 0.02f

#define MPU6050_ADDR 0x68

// MPU6050 Register Map
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75

#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

#define MPU6050_VAL_DEVICE_RESET    BIT7

// When calling i2c_write_blocking or i2c_read_blocking, the 5th argument
// being `true` means that we keep master control of the bus. When it's `false`,
// we release control.
#define BUS_CONTROL                 true
#define BUS_RELEASE                 false

// Scale factors
#define ACCEL_SCALE_2G              16384.0f    // +/- 2g setting. Units per `g` (1g == Earth's gravity).
#define GYRO_SCALE_DEG_250          131.0f      // +/- 250°/sec setting. Units per `°/sec`.
#define GYRO_SCALE_RAD_250 (GYRO_SCALE_DEG_250 * DEG_PER_RAD)

int mpu6050_loop();
bool mpu6050_reset();
void mpu6050_read_raw(int16_t accel[DIM], int16_t gyro[DIM]);
void mpu6050_scale(int16_t *raw, float *scaled, float scale_factor, int count);
float complement(float accel1, float accel2, float gyro, float prev_val, float elapsed);

#endif
