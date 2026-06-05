#include "map.h"

#define INF 0xFFFF

// Максимальное количество клеток для BFS (можно менять под свой лабиринт)
#ifndef MAP_MAX_CELLS
#define MAP_MAX_CELLS 1024   // достаточно для 32x32
#endif

// Вспомогательная функция: получить индекс в одномерном массиве
static inline uint16_t idx(maze_t* maze, uint16_t x, uint16_t y) {
    return y * maze->width + x;
}

void map_init(maze_t* maze, uint16_t width, uint16_t height,
              uint16_t goal_x, uint16_t goal_y,
              uint8_t* walls_buf, uint16_t* weights_buf, uint8_t* visited_buf) {
    maze->width = width;
    maze->height = height;
    maze->goal_x = goal_x;
    maze->goal_y = goal_y;
    maze->walls   = walls_buf;
    maze->weights = weights_buf;
    maze->visited = visited_buf;

    // Инициализация: все стены присутствуют (биты 0-3 = 1)
    uint16_t total = width * height;
    for (uint16_t i = 0; i < total; i++) {
        walls_buf[i]   = 0x0F;   // биты N,E,S,W = 1
        weights_buf[i] = INF;
        visited_buf[i] = 0;
    }
}

void map_set_wall(maze_t* maze, uint16_t x, uint16_t y, direction_t dir, bool present) {
    if (x >= maze->width || y >= maze->height) return;
    uint16_t i = idx(maze, x, y);
    uint8_t mask = 1 << dir;
    if (present)
        maze->walls[i] |= mask;
    else
        maze->walls[i] &= ~mask;
}

bool map_is_wall(maze_t* maze, uint16_t x, uint16_t y, direction_t dir) {
    if (x >= maze->width || y >= maze->height) return true;
    uint16_t i = idx(maze, x, y);
    return (maze->walls[i] >> dir) & 1;
}

void map_set_visited(maze_t* maze, uint16_t x, uint16_t y) {
    if (x < maze->width && y < maze->height) {
        maze->visited[idx(maze, x, y)] = 1;
    }
}

bool map_is_visited(maze_t* maze, uint16_t x, uint16_t y) {
    if (x >= maze->width || y >= maze->height) return false;
    return maze->visited[idx(maze, x, y)] != 0;
}

void map_compute_weights(maze_t* maze) {
    uint16_t total = maze->width * maze->height;
    // Сброс весов в бесконечность
    for (uint16_t i = 0; i < total; i++) {
        maze->weights[i] = INF;
    }

    // Статическая очередь (максимальный размер)
    uint16_t queue_x[MAP_MAX_CELLS];
    uint16_t queue_y[MAP_MAX_CELLS];
    uint16_t head = 0, tail = 0;

    // Старт с целевой клетки
    uint16_t goal_idx = idx(maze, maze->goal_x, maze->goal_y);
    maze->weights[goal_idx] = 0;
    queue_x[tail] = maze->goal_x;
    queue_y[tail] = maze->goal_y;
    tail++;

    // Смещения для направлений (N, E, S, W)
    const int8_t dx[4] = {0, 1, 0, -1};
    const int8_t dy[4] = {-1, 0, 1, 0};

    while (head != tail) {
        uint16_t x = queue_x[head];
        uint16_t y = queue_y[head];
        head++;
        uint16_t cur_w = maze->weights[idx(maze, x, y)];

        for (direction_t d = 0; d < 4; d++) {
            if (!map_is_wall(maze, x, y, d)) {
                int16_t nx = (int16_t)x + dx[d];
                int16_t ny = (int16_t)y + dy[d];
                if (nx >= 0 && nx < maze->width && ny >= 0 && ny < maze->height) {
                    uint16_t n_idx = idx(maze, (uint16_t)nx, (uint16_t)ny);
                    if (maze->weights[n_idx] > cur_w + 1) {
                        maze->weights[n_idx] = cur_w + 1;
                        if (tail < MAP_MAX_CELLS) {
                            queue_x[tail] = (uint16_t)nx;
                            queue_y[tail] = (uint16_t)ny;
                            tail++;
                        } // иначе очередь переполнена – увеличить MAP_MAX_CELLS
                    }
                }
            }
        }
    }
}

uint16_t map_get_weight(maze_t* maze, uint16_t x, uint16_t y) {
    if (x >= maze->width || y >= maze->height) return INF;
    return maze->weights[idx(maze, x, y)];
}

direction_t map_get_best_neighbor(maze_t* maze, uint16_t x, uint16_t y) {
    const int8_t dx[4] = {0, 1, 0, -1};
    const int8_t dy[4] = {-1, 0, 1, 0};
    uint16_t best_weight = INF;
    direction_t best_dir = 0xFF; // недопустимое направление

    for (direction_t d = 0; d < 4; d++) {
        if (!map_is_wall(maze, x, y, d)) {
            int16_t nx = (int16_t)x + dx[d];
            int16_t ny = (int16_t)y + dy[d];
            if (nx >= 0 && nx < maze->width && ny >= 0 && ny < maze->height) {
                uint16_t w = maze->weights[idx(maze, (uint16_t)nx, (uint16_t)ny)];
                if (w < best_weight) {
                    best_weight = w;
                    best_dir = d;
                } else if (w == best_weight && best_dir != 0xFF) {
                    // При равных весах предпочитаем непосещённую клетку
                    bool curr_visited = map_is_visited(maze, (uint16_t)nx, (uint16_t)ny);
                    int16_t best_nx = (int16_t)x + dx[best_dir];
                    int16_t best_ny = (int16_t)y + dy[best_dir];
                    bool best_visited = map_is_visited(maze, (uint16_t)best_nx, (uint16_t)best_ny);
                    if (!curr_visited && best_visited) {
                        best_dir = d;
                    }
                }
            }
        }
    }
    return best_dir;
}
