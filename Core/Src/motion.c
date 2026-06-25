#include "motion.h"
#include "motor.h"
#include "imu.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>


#define WHEEL_DIAMETER_MM   34.0f
#define ENCODER_PPR         7
#define GEAR_RATIO          97.0f

#define TICKS_PER_REV       ((float)ENCODER_PPR * GEAR_RATIO)
#define WHEEL_CIRCUMFERENCE (M_PI * WHEEL_DIAMETER_MM)
#define TICKS_PER_MM        (TICKS_PER_REV / WHEEL_CIRCUMFERENCE)

#define BASE_SPEED_FORWARD   700
#define BASE_SPEED_BACKWARD  -800

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} PID;

static PID pid_straight = {
    .kp         = 1.2f,
    .ki         = 0.02f,
    .kd         = 0.5f,
    .integral   = 0.0f,
    .prev_error = 0.0f,
};

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

static inline int32_t get_left_encoder(void) {
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
}

static inline int32_t get_right_encoder(void) {
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

static void pid_reset(PID *pid) {
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

static float pid_update(PID *pid, float error, float dt) {
    pid->integral += error * dt;

    if (pid->integral >  500.0f) pid->integral =  500.0f;
    if (pid->integral < -500.0f) pid->integral = -500.0f;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

void move_forward(int32_t distance_mm) {
    if (distance_mm == 0) return;

    pid_reset(&pid_straight);

    int32_t target_ticks = (int32_t)((float)abs(distance_mm) * TICKS_PER_MM);

    int32_t left_start  = get_left_encoder();
    int32_t right_start = get_right_encoder();

    int16_t base_speed = (distance_mm > 0) ? BASE_SPEED_FORWARD : BASE_SPEED_BACKWARD;
    const float dt = 0.02f;

    while (1) {
        int32_t left_delta  = get_left_encoder()  - left_start;
        int32_t right_delta = get_right_encoder() - right_start;
		
        int32_t avg = (abs(left_delta) + abs(right_delta)) / 2;

        if (avg >= target_ticks) break;

        float error = (float)(left_delta - right_delta);
        float correction = pid_update(&pid_straight, error, dt);

        int16_t left_speed  = (int16_t)(base_speed - correction);
        int16_t right_speed = (int16_t)(base_speed + correction + 100);

        motor_set_speed(left_speed, right_speed);
        HAL_Delay((uint32_t)(dt * 1000.0f));
    }

    motor_brake();
    HAL_Delay(20);
}

void turn_degrees(int16_t angle_deg) {
    if (angle_deg == 0) return;

    imu_reset_yaw();

    const float target = (float)angle_deg;
    const float kp     = 10.0f;
    const float dt     = 0.01f;

    const float MIN_TURN_SPEED = 650.0f;
    const float MAX_TURN_SPEED = 650.0f;

    while (1) {
        float current = imu_get_yaw();
        float error   = target - current;

        if (fabsf(error) < 0.8f) break;

        float output = kp * error;

        if (output > 0.0f) {
            if (output > MAX_TURN_SPEED) output = MAX_TURN_SPEED;
            if (output < MIN_TURN_SPEED) output = MIN_TURN_SPEED;
        } else {
            if (output < -MAX_TURN_SPEED) output = -MAX_TURN_SPEED;
            if (output > -MIN_TURN_SPEED) output = -MIN_TURN_SPEED;
        }

        float output_rigth = output;

        if (output_rigth > 0.0f) output_rigth -= 50;
        else if (output_rigth == 0.0f) output_rigth = output_rigth;
        else output_rigth -= 50;

        motor_set_speed((int16_t)(output), (int16_t)(-output_rigth));
        HAL_Delay((uint32_t)(dt * 1000.0f));
    }

    motor_brake();
    HAL_Delay(20);
}

void turn_to_direction(direction_t target, direction_t current) {
    int8_t delta = (int8_t)(target - current);

    if (delta >  2) delta -= 4;
    if (delta < -2) delta += 4;

    turn_degrees((int16_t)-(delta * 90));
}

int32_t get_left_encoder_ticks(void)  { return get_left_encoder();  }
int32_t get_right_encoder_ticks(void) { return get_right_encoder(); }
