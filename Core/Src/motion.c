#include "motion.h"
#include "motor.h"
#include "imu.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>

// ========== Физические параметры робота ==========
// Замените на реальные характеристики своего мотора и колёс

#define WHEEL_DIAMETER_MM   34.0f   // диаметр колеса в мм
#define ENCODER_PPR         7       // импульсов на оборот вала МОТОРА (до редуктора)
#define GEAR_RATIO          97.0f   // передаточное число редуктора

// Вычисляемые константы (не трогать)
#define TICKS_PER_REV       ((float)ENCODER_PPR * GEAR_RATIO)
#define WHEEL_CIRCUMFERENCE (M_PI * WHEEL_DIAMETER_MM)
#define TICKS_PER_MM        (TICKS_PER_REV / WHEEL_CIRCUMFERENCE)

// ========== Скорости ==========
#define BASE_SPEED_FORWARD   750
#define BASE_SPEED_BACKWARD -750

// ========== ПИД-регулятор ==========
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} PID;

// Коэффициенты подобраны экспериментально — при необходимости скорректируйте
static PID pid_straight = {
    .kp         = 1.2f,
    .ki         = 0.02f,
    .kd         = 0.5f,
    .integral   = 0.0f,
    .prev_error = 0.0f,
};

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

// ========== Энкодеры ==========
static inline int32_t get_left_encoder(void) {
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
}

static inline int32_t get_right_encoder(void) {
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

// Сброс состояния ПИД перед новым движением
static void pid_reset(PID *pid) {
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

// Шаг ПИД-регулятора
static float pid_update(PID *pid, float error, float dt) {
    pid->integral += error * dt;

    // Анти-виндап
    if (pid->integral >  500.0f) pid->integral =  500.0f;
    if (pid->integral < -500.0f) pid->integral = -500.0f;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

// ========== Движение вперёд / назад ==========
void move_forward(int32_t distance_mm) {
    if (distance_mm == 0) return;

    // Сбрасываем ПИД, чтобы накопленный интеграл предыдущего движения
    // не влиял на текущее
    pid_reset(&pid_straight);

    // Целевое количество тиков (всегда положительное)
    int32_t target_ticks = (int32_t)((float)abs(distance_mm) * TICKS_PER_MM);

    // Запоминаем стартовые позиции энкодеров.
    // Это позволяет корректно работать и при движении назад
    // (счётчик убывает), и при любом начальном значении счётчика.
    int32_t left_start  = get_left_encoder();
    int32_t right_start = get_right_encoder();

    int16_t base_speed = (distance_mm > 0) ? BASE_SPEED_FORWARD : BASE_SPEED_BACKWARD;
    const float dt = 0.02f;   // период цикла управления — 20 мс

    while (1) {
        // Пройденное расстояние в тиках (со знаком) относительно старта
        int32_t left_delta  = get_left_encoder()  - left_start;
        int32_t right_delta = get_right_encoder() - right_start;

        // Среднее пройденное (модуль — нам важна дистанция, а не направление)
        int32_t avg = (abs(left_delta) + abs(right_delta)) / 2;

        if (avg >= target_ticks) break;

        // Ошибка прямолинейности: левый опережает правый → надо притормозить левый
        float error = (float)(left_delta - right_delta);
        float correction = pid_update(&pid_straight, error, dt);

        int16_t left_speed  = (int16_t)(base_speed - correction);
        int16_t right_speed = (int16_t)(base_speed + correction);

        motor_set_speed(left_speed, right_speed);
        HAL_Delay((uint32_t)(dt * 1000.0f));
    }

    motor_brake();
    HAL_Delay(20);
}

// ========== Поворот на угол (гироскоп) ==========
void turn_degrees(int16_t angle_deg) {
    if (angle_deg == 0) return;

    imu_reset_yaw();

    const float target = (float)angle_deg;
    const float kp     = 10.0f;    // пропорциональный коэффициент; подберите под свой робот
    const float dt     = 0.01f;    // период цикла — 10 мс

    // Минимальная и максимальная скорость поворота (по модулю)
    const float MIN_TURN_SPEED = 750.0f;
    const float MAX_TURN_SPEED = 999.0f;

    while (1) {
        float current = imu_get_yaw();
        float error   = target - current;

        if (fabsf(error) < 0.8f) break;   // точность ±0.8°

        float output = kp * error;

        // Насыщение: ограничиваем диапазон [MIN..MAX] с сохранением знака
        if (output > 0.0f) {
            if (output > MAX_TURN_SPEED) output = MAX_TURN_SPEED;
            if (output < MIN_TURN_SPEED) output = MIN_TURN_SPEED;
        } else {
            if (output < -MAX_TURN_SPEED) output = -MAX_TURN_SPEED;
            if (output > -MIN_TURN_SPEED) output = -MIN_TURN_SPEED;
        }

        // Дифференциальный поворот: один мотор вперёд, другой назад
        motor_set_speed((int16_t)(-output), (int16_t)(output));
        HAL_Delay((uint32_t)(dt * 1000.0f));
    }

    motor_brake();
    HAL_Delay(20);
}

// ========== Поворот по направлениям (для flood fill) ==========
void turn_to_direction(direction_t target, direction_t current) {
    int8_t delta = (int8_t)(target - current);

    // Нормализуем в диапазон [-2, 2]:
    // 3 шага вправо == 1 шаг влево, и т.д.
    if (delta >  2) delta -= 4;
    if (delta < -2) delta += 4;

    turn_degrees((int16_t)(delta * 90));
}

// ========== Публичные геттеры энкодеров ==========
int32_t get_left_encoder_ticks(void)  { return get_left_encoder();  }
int32_t get_right_encoder_ticks(void) { return get_right_encoder(); }
