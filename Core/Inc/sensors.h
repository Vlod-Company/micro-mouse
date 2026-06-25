#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

void sensors_init(void);

uint16_t sensor_get_distance(uint8_t index);

uint16_t sensor_get_distance_front(void);
uint16_t sensor_get_distance_left(void);
uint16_t sensor_get_distance_right(void);

#endif
