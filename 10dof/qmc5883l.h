#ifndef QMC5883L_H
#define QMC5883L_H

#define QMC5883L_ADDR 0x0D

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mag_data_t;

typedef struct {
    float xo;  // X offset
    float xs;  // X scale
    float yo;
    float ys;
    float zo;
    float zs;
} q5_scalar_t;

void qmc5883l_init();
bool qmc5883l_read_raw(mag_data_t *data);
void qmc5883l_scale(mag_data_t *raw, float *scaled, q5_scalar_t *sf);
float qmc5883l_naive_heading(float *scaled);
void calibrate_magnetometer(q5_scalar_t *scale_factors);

#endif
