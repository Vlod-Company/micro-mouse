/*
 * imu.h
 *
 *  Created on: Jun 9, 2026
 *      Author: vlad_
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

// Инициализация MPU6500 (проверка связи, настройка, калибровка)
// Возвращает true при успехе
bool imu_init(void);

// Сбросить накопленный угол (перед началом поворота)
void imu_reset_yaw(void);

// Получить текущий угол поворота (градусы, по часовой стрелке +)
float imu_get_yaw(void);

// Получить скорость вращения (градусы/сек)
float imu_get_gyro_z_dps(void);

// Получить сырые значения (для отладки)
void imu_get_raw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
void imu_get_scaled(float *ax_g, float *ay_g, float *az_g, float *gx_dps, float *gy_dps, float *gz_dps);


#endif
