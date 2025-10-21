#include <math.h>
#include <stdio.h>
#include "hardware/i2c.h"
#include "qmc5883l.h"

void qmc5883l_init() {
    uint8_t config[2] = {
        0x09, // Control register 1
        /*
         * CR1:
         * OSR (oversample ratio): 0b00 -> 512
         * //RNG (range): 0b01 -> 8 Gauss
         * RNG (range): 0b00 -> 2 Gauss
         * ODR (output data rate): 0b11 -> 200 Hz
         * MODE: 0b01 -> Continuous
         */
        //0b00011101,
        0b00001101,
    };

    i2c_write_blocking(i2c_default, QMC5883L_ADDR, config, 2, false);
}

bool qmc5883l_read_raw(mag_data_t *data) {
    uint8_t reg = 0x00; // Start from data register 00
    uint8_t buffer[6];

    // Read 6 bytes of magnetometer data
    i2c_write_blocking(i2c_default, QMC5883L_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, QMC5883L_ADDR, buffer, 6, false);

    data->x = (int16_t)(buffer[0] | (buffer[1] << 8));
    data->y = (int16_t)(buffer[2] | (buffer[3] << 8));
    data->z = (int16_t)(buffer[4] | (buffer[5] << 8));

    return true;
}

void qmc5883l_scale(mag_data_t *raw, float *scaled, q5_scalar_t *sf) {
    // Eww
    scaled[0] = ((float)(raw->x) - sf->xo) / sf->xs;
    scaled[1] = ((float)(raw->y) - sf->yo) / sf->ys;
    scaled[2] = ((float)(raw->z) - sf->zo) / sf->zs;

    float magnitude = sqrtf(scaled[0] * scaled[0]
                            + scaled[1] * scaled[1]
                            + scaled[2] * scaled[2]);
    scaled[0] /= magnitude;
    scaled[1] /= magnitude;
    scaled[2] /= magnitude;

    // XXX Opposite Y and Z, I think?
    scaled[1] = 0 - scaled[1];
    scaled[2] = 0 - scaled[2];
}

float qmc5883l_naive_heading(float *scaled) {
    return atan2(scaled[1], scaled[0]);  // Y, X
}

void calibrate_magnetometer(q5_scalar_t *scale_factors) {
    mag_data_t min = {32767, 32767, 32767};
    mag_data_t max = {-32768, -32768, -32768};
    mag_data_t sample;

    printf("Rotate device in figure-8 pattern for 10 seconds...\n");

    absolute_time_t start = get_absolute_time();
    while (absolute_time_diff_us(start, get_absolute_time()) < 10000000) {
        qmc5883l_read_raw(&sample);
        printf("For calibration: X %i Y %i Z %i\n", sample.x, sample.y, sample.z);

        // Update min/max
        if (sample.x < min.x) min.x = sample.x;
        if (sample.y < min.y) min.y = sample.y;
        if (sample.z < min.z) min.z = sample.z;
        if (sample.x > max.x) max.x = sample.x;
        if (sample.y > max.y) max.y = sample.y;
        if (sample.z > max.z) max.z = sample.z;

        sleep_ms(20);
    }

    printf("Calibration complete:\n");
    scale_factors->xo = (max.x + min.x)/2.0;
    scale_factors->yo = (max.y + min.y)/2.0;
    scale_factors->zo = (max.z + min.z)/2.0;
    scale_factors->xs = (max.x - min.x)/2.0;
    scale_factors->ys = (max.y - min.y)/2.0;
    scale_factors->zs = (max.z - min.z)/2.0;
    printf("Offset X: %f, Y: %f, Z: %f\n", scale_factors->xo, scale_factors->yo, scale_factors->zo);
    printf("Scale X: %f, Y: %f, Z: %f\n", scale_factors->xs, scale_factors->ys, scale_factors->zs);
}
