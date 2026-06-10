#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_set_speed(int16_t left, int16_t right);
void motor_brake(void);

#endif
