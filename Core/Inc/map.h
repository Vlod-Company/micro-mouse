#ifndef MAP_H
#define MAP_H

#include <stdint.h>
#include <stdbool.h>

// Направления: N=0, E=1, S=2, W=3
typedef enum {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
} direction_t;

// Структура лабиринта – хранит только размеры, цель и указатели на буферы
typedef struct {
    uint16_t width;          // ширина (клеток)
    uint16_t height;         // высота (клеток)
    uint16_t goal_x;         // цель X
    uint16_t goal_y;         // цель Y
    uint8_t*  walls;         // буфер размером width*height, 4 бита на клетку (биты 0..3 = стены)
    uint16_t* weights;       // буфер width*height, расстояние до цели (0xFFFF = бесконечность)
    uint8_t*  visited;       // буфер width*height, 1 = посещена, 0 = нет
} maze_t;

// Инициализация: задаёт размеры, цель и привязывает внешние буферы.
// Все стены изначально считаются присутствующими (биты = 1).
void map_init(maze_t* maze, uint16_t width, uint16_t height,
              uint16_t goal_x, uint16_t goal_y,
              uint8_t* walls_buf, uint16_t* weights_buf, uint8_t* visited_buf);

// Установить стену в клетке (x,y) по направлению dir.
// present = true  – стена есть,
// present = false – стены нет (проход открыт).
void map_set_wall(maze_t* maze, uint16_t x, uint16_t y, direction_t dir, bool present);

// Проверить наличие стены
bool map_is_wall(maze_t* maze, uint16_t x, uint16_t y, direction_t dir);

// Отметить клетку как посещённую
void map_set_visited(maze_t* maze, uint16_t x, uint16_t y);

// Проверить, посещена ли клетка
bool map_is_visited(maze_t* maze, uint16_t x, uint16_t y);

// Пересчитать веса клеток (BFS от цели). Использует очередь фиксированного размера.
// Максимальный размер лабиринта задаётся макросом MAP_MAX_CELLS (по умолчанию 1024).
void map_compute_weights(maze_t* maze);

// Получить вес клетки (расстояние до цели, 0xFFFF если недостижима)
uint16_t map_get_weight(maze_t* maze, uint16_t x, uint16_t y);

// Выбрать лучшее направление из клетки (x,y) – с минимальным весом.
// Если нет доступных соседей, возвращает 0xFF (надо проверять).
direction_t map_get_best_neighbor(maze_t* maze, uint16_t x, uint16_t y);

#endif // MAP_H
