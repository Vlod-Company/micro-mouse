#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include "map.h"

// Проехать расстояние (мм). Положительное – вперёд, отрицательное – назад.
// Использует энкодеры и PID для удержания прямой.
void move_forward(int32_t distance_mm);

// Повернуть на угол (градусы). Положительный – по часовой стрелке.
// Использует гироскоп (imu_get_yaw).
void turn_degrees(int16_t angle_deg);

// Повернуть с текущего направления на целевое (минимальный поворот)
// current – текущее направление робота (DIR_NORTH и т.д.)
// target – желаемое направление
void turn_to_direction(direction_t target, direction_t current);

// Получить текущие тики энкодеров (для отладки)
int32_t get_left_encoder_ticks(void);
int32_t get_right_encoder_ticks(void);

#endif
