#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include "map.h"

void move_forward(int32_t distance_mm);

void turn_degrees(int16_t angle_deg);

void turn_to_direction(direction_t target, direction_t current);

int32_t get_left_encoder_ticks(void);
int32_t get_right_encoder_ticks(void);

#endif
