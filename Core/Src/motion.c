#include "motion.h"
#include "motor.h"
#include "imu.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>

// ========== Конфигурация (подберите под свой робот) ==========
// Коэффициент пересчёта тиков энкодера в миллиметры
// Если при 1333 тиках (тестовый код) проезжали 180 мм, то:
// TICKS_PER_MM = 1333 / 180 = 7.405555...
#define TICKS_PER_MM        1.4056f

// Базовая скорость движения (0..MAX_PWM, где MAX_PWM=1000)
#define BASE_SPEED_FORWARD  750
#define BASE_SPEED_BACKWARD -750

// ПИД для удержания прямой (по разнице энкодеров)
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} PID;

static PID pid_straight = {1.2f, 0.02f, 0.5f, 0.0f, 0.0f};

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

static inline int32_t get_left_encoder(void) {
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
}
static inline int32_t get_right_encoder(void) {
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
}
static inline void reset_encoders(void) {
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
}

// ========== ПИД-регулятор ==========
static float pid_update(PID *pid, float error, float dt) {
    pid->integral += error * dt;
    // Анти-виндап
    if (pid->integral > 500.0f) pid->integral = 500.0f;
    if (pid->integral < -500.0f) pid->integral = -500.0f;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

// ========== Движение вперёд/назад ==========
void move_forward(int32_t distance_mm) {
    if (distance_mm == 0) return;
    bool forward = (distance_mm > 0);
    int32_t target_ticks = (int32_t)((float)abs(distance_mm) * TICKS_PER_MM);
    reset_encoders();

    int16_t base_speed = forward ? BASE_SPEED_FORWARD : BASE_SPEED_BACKWARD;
    float dt = 0.02f;   // 20 мс цикл

    while (1) {
        int32_t left = get_left_encoder();
        int32_t right = get_right_encoder();
        int32_t avg = (abs(left) + abs(right)) / 2;

        // Проверка достижения цели
        if (abs(avg) >= target_ticks) break;

        // Ошибка прямолинейности: если left > right, то робот поворачивает направо
        int32_t diff = left - right;
        float correction = pid_update(&pid_straight, (float)diff, dt);

        int16_t left_speed  = (int16_t)(base_speed - correction);
        int16_t right_speed = (int16_t)(base_speed + correction);

        motor_set_speed(left_speed, right_speed);
        HAL_Delay((uint32_t)(dt * 1000));
    }
    motor_brake();
    HAL_Delay(20);   // небольшая пауза для стабилизации
}

// ========== Поворот на угол (гироскоп) ==========
void turn_degrees(int16_t angle_deg) {
    if (angle_deg == 0) return;
    imu_reset_yaw();
    float target = (float)angle_deg;
    float kp = 10.0f;       // подобрать экспериментально (0.5..3.0)
    float dt = 0.01f;      // 10 мс
    float error;

    while (1) {
        float current = imu_get_yaw();
        error = target - current;
        if (fabsf(error) < 0.8f) break;  // точность 0.8 градуса

        float output = kp * error;
        // Ограничиваем скорость поворота
        if (output > 0) {
        	if (output > 999.0f) output = 999.0f;
        	if (output < 750.0f) output = 750.0f;
        }
        else {
        	if (output < -999.0f) output = -999.0f;
        	if (output > -750.0f) output = -750.0f;
        }

        // Дифференциальный поворот: левый едет назад, правый вперёд
        motor_set_speed((int16_t)(-output), (int16_t)(output));
        HAL_Delay((uint32_t)(dt * 1000));
    }
    motor_brake();
    HAL_Delay(20);
}

// ========== Поворот по направлениям (для flood fill) ==========
void turn_to_direction(direction_t target, direction_t current) {
    int8_t delta = (int8_t)(target - current);
    // Нормализуем в диапазон [-2, 2]
    if (delta > 2) delta -= 4;
    if (delta < -2) delta += 4;
    int16_t angle = delta * 90;   // 90° на одно направление
    turn_degrees(angle);
}

int32_t get_left_encoder_ticks(void) { return get_left_encoder(); }
int32_t get_right_encoder_ticks(void) { return get_right_encoder(); }
