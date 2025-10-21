#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include "qmc5883l.h"
#include "bmp280.h"

#define FRESH_HDG -1000.0f
#define OFFICE_ELEV_METERS 128.0f

int hdg_deg(float hdg_rad, float decl) {
    float uncorrected_deg = hdg_rad / M_PI * 180.0f;
    float corrected_deg = uncorrected_deg + decl;
    float normalized_deg = round(corrected_deg);
    if (normalized_deg <= 0) {
        normalized_deg += 360;
    } else if (normalized_deg > 360) {
        normalized_deg -= 360;
    }
    return normalized_deg;
}

int main() {
    stdio_init_all();
    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);  // 400 kHz
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    while (!mpu6050_reset()) {
        printf("Unable to initialize MPU6050 :(\n");
        sleep_ms(5000);
    }

    qmc5883l_init();

    // Initialize BMP280
    bmp280_t bmp;
    float temperature, pressure;
    if (!bmp280_init(&bmp, i2c0, BMP280_I2C_ADDR)) {
        printf("BMP280 initialization failed!\n");
    } else {
        printf("BMP280 initialized successfully\n");
    }
    float sea_level_pressure = bmp280_calibrate_altitude(&bmp, OFFICE_ELEV_METERS);
    while (sea_level_pressure < 900) {
        printf("Retrying barometer calibration\n");
        sleep_ms(100);
        sea_level_pressure = bmp280_calibrate_altitude(&bmp, OFFICE_ELEV_METERS);
    }

    int16_t accel_raw[DIM], gyro_raw[DIM];
    float accel_g[DIM], gyro_rad[DIM];

    uint32_t now = time_us_32();
    uint32_t m6_last_time = 0;
    float m6_roll = 0;
    float m6_pitch = 0;
    float m6_hdg = FRESH_HDG;
    float elapsed = -1;
    float q5_scaled[3];

    mag_data_t q5_mag_data = {0};

    q5_scalar_t q5_scalars;
    //calibrate_magnetometer(&q5_scalars);
    //sleep_ms(1000);
    q5_scalars.xo = 3875.0f;
    q5_scalars.yo = -4882.5f;
    q5_scalars.zo = 197.5f;
    q5_scalars.xs = 6080.0f;
    q5_scalars.ys = 5859.5f;
    q5_scalars.zs = 3487.5f;
    float local_declination_deg = 14.5f;

    while (true) {
        mpu6050_read_raw(accel_raw, gyro_raw);
        mpu6050_scale(accel_raw, accel_g, ACCEL_SCALE_2G, DIM);
        mpu6050_scale(gyro_raw, gyro_rad, GYRO_SCALE_RAD_250, DIM);
        m6_roll = complement(-accel_g[AXIS_X], accel_g[AXIS_Z], gyro_rad[AXIS_Y], m6_roll, elapsed);
        m6_pitch = complement(accel_g[AXIS_Y], accel_g[AXIS_Z], gyro_rad[AXIS_X], m6_pitch, elapsed);

        float cos_roll = cosf(-m6_roll);
        float sin_roll = sinf(-m6_roll);
        float cos_pitch = cosf(-m6_pitch);
        float sin_pitch = sinf(-m6_pitch);

        qmc5883l_read_raw(&q5_mag_data);
        qmc5883l_scale(&q5_mag_data, q5_scaled, &q5_scalars);

        float mx = q5_scaled[0];
        float my = q5_scaled[1];
        float mz = q5_scaled[2];
        float mx_comp = mx * cos_pitch + my * sin_pitch * sin_roll + mz * sin_pitch * cos_roll;
        float my_comp = my * cos_roll - mz * sin_roll;
        float q5_hdg = -atan2(my_comp, mx_comp);

        if (m6_hdg == FRESH_HDG) {
            m6_hdg = q5_hdg;
        } else {
            float delta_hdg_gyro = -gyro_rad[AXIS_Z] * elapsed;
            m6_hdg += delta_hdg_gyro;
            float hdg_error = q5_hdg - m6_hdg;  // NOT DECLINATION COMPENSATED
            if (hdg_error > M_PI) {
                hdg_error -= 2 * M_PI;
            } else if (hdg_error < -M_PI) {
                hdg_error += 2 * M_PI;
            }
            float hdg_fused = m6_hdg + 0.02f * hdg_error;
            if (hdg_fused > M_PI) {
                hdg_fused -= 2 * M_PI;
            } else if (hdg_fused < -M_PI) {
                hdg_fused += 2 * M_PI;
            }
            m6_hdg = hdg_fused;
        }

        float elevation = 0.0f;
        if (bmp280_read(&bmp, &temperature, &pressure)) {
            elevation = bmp280_calculate_altitude(pressure, sea_level_pressure);
        }

        // Finally
        m6_last_time = now;
        now = time_us_32();
        elapsed = (now - m6_last_time) / US_PER_SEC;
        printf("Roll: %.1f°\t", -m6_roll * DEG_PER_RAD);
        printf("Pitch: %.1f°\t", -m6_pitch * DEG_PER_RAD);
        printf("Heading: %d°\t", hdg_deg(m6_hdg, local_declination_deg));
        printf("Elevation: %.1fm\n", elevation);
        sleep_ms(10);
    }
}
