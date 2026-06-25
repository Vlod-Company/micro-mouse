#ifndef MAP_H
#define MAP_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
} direction_t;

typedef struct {
    uint16_t width;
    uint16_t height;
    uint16_t goal_x;
    uint16_t goal_y;
    uint8_t*  walls;
    uint16_t* weights;
    uint8_t*  visited;
} maze_t;


void map_init(maze_t* maze, uint16_t width, uint16_t height,
              uint16_t goal_x, uint16_t goal_y,
              uint8_t* walls_buf, uint16_t* weights_buf, uint8_t* visited_buf);


void map_set_wall(maze_t* maze, uint16_t x, uint16_t y, direction_t dir, bool present);

bool map_is_wall(maze_t* maze, uint16_t x, uint16_t y, direction_t dir);

void map_set_visited(maze_t* maze, uint16_t x, uint16_t y);

bool map_is_visited(maze_t* maze, uint16_t x, uint16_t y);

void map_compute_weights(maze_t* maze);

uint16_t map_get_weight(maze_t* maze, uint16_t x, uint16_t y);

direction_t map_get_best_neighbor(maze_t* maze, uint16_t x, uint16_t y);

#endif
