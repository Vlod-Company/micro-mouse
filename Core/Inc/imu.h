#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

bool imu_init(void);

void imu_reset_yaw(void);

float imu_get_yaw(void);

float imu_get_gyro_z_dps(void);

void imu_get_raw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
void imu_get_scaled(float *ax_g, float *ay_g, float *az_g, float *gx_dps, float *gy_dps, float *gz_dps);


#endif
