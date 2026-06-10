// motor.c
#include "motor.h"
#include "main.h"

// Левый мотор: TIM2, канал 1 = IN1, канал 2 = IN2
#define LEFT_TIM      &htim2
#define LEFT_IN1_CH   TIM_CHANNEL_1
#define LEFT_IN2_CH   TIM_CHANNEL_2

// Правый мотор: TIM3, канал 1 = IN3, канал 2 = IN4
#define RIGHT_TIM     &htim3
#define RIGHT_IN1_CH  TIM_CHANNEL_1
#define RIGHT_IN2_CH  TIM_CHANNEL_2

#define MAX_PWM 1000   // должно совпадать с периодом таймера (ARR) в CubeMX

static void left_set(int16_t speed) {
    if (speed > MAX_PWM) speed = MAX_PWM;
    if (speed < -MAX_PWM) speed = -MAX_PWM;

    if (speed == 0) {
        __HAL_TIM_SET_COMPARE(LEFT_TIM, LEFT_IN1_CH, 0);
        __HAL_TIM_SET_COMPARE(LEFT_TIM, LEFT_IN2_CH, 0);
    } else if (speed > 0) {
        // Вперёд: IN1 = PWM, IN2 = 0
        __HAL_TIM_SET_COMPARE(LEFT_TIM, LEFT_IN1_CH, speed);
        __HAL_TIM_SET_COMPARE(LEFT_TIM, LEFT_IN2_CH, 0);
    } else {
        // Назад: IN1 = 0, IN2 = PWM
        __HAL_TIM_SET_COMPARE(LEFT_TIM, LEFT_IN1_CH, 0);
        __HAL_TIM_SET_COMPARE(LEFT_TIM, LEFT_IN2_CH, -speed);
    }
}

static void right_set(int16_t speed) {
    if (speed > MAX_PWM) speed = MAX_PWM;
    if (speed < -MAX_PWM) speed = -MAX_PWM;

    if (speed == 0) {
        __HAL_TIM_SET_COMPARE(RIGHT_TIM, RIGHT_IN1_CH, 0);
        __HAL_TIM_SET_COMPARE(RIGHT_TIM, RIGHT_IN2_CH, 0);
    } else if (speed > 0) {
        __HAL_TIM_SET_COMPARE(RIGHT_TIM, RIGHT_IN1_CH, speed);
        __HAL_TIM_SET_COMPARE(RIGHT_TIM, RIGHT_IN2_CH, 0);
    } else {
        __HAL_TIM_SET_COMPARE(RIGHT_TIM, RIGHT_IN1_CH, 0);
        __HAL_TIM_SET_COMPARE(RIGHT_TIM, RIGHT_IN2_CH, -speed);
    }
}

void motor_set_speed(int16_t left, int16_t right) {
    left_set(left);
    right_set(right);
}

void motor_brake(void) {
    left_set(0);
    right_set(0);
}
