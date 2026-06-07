/*
 * sensors.c
 *
 *  Created on: Jun 5, 2026
 *      Author: vlad_
 */


#include "sensors.h"
#include "main.h"

static UART_HandleTypeDef* uart_handles[SENSOR_COUNT] = {
    &huart1,
    &huart2,
    &huart3
};

#define RESPONSE_TIMEOUT_MS 100
#define CMD_DISTANCE 0x55
#define RESP_LEN 4

#define RESP_HEADER1 0xAA
#define RESP_HEADER2 0xAA

void sensors_init() {

}

uint16_t sensor_get_distance(uint8_t index) {
    if (index >= SENSOR_COUNT) return 0xFFFF;

    uint8_t cmd = CMD_DISTANCE;
    uint8_t resp[RESP_LEN];
    HAL_StatusTypeDef status;

    // Очищаем буфер UART от мусора (опционально: сброс ошибок)
    // Можно сделать __HAL_UART_FLUSH_DRREGISTER, но лучше просто перед отправкой убедиться, что UART готов
    while (HAL_UART_GetState(uart_handles[index]) != HAL_UART_STATE_READY) {
        HAL_Delay(1);
    }

    // Отправляем команду
    status = HAL_UART_Transmit(uart_handles[index], &cmd, 1, RESPONSE_TIMEOUT_MS);
    if (status != HAL_OK) return 0xFFFF;

    // Принимаем ответ (4 байта)
    status = HAL_UART_Receive(uart_handles[index], resp, RESP_LEN, RESPONSE_TIMEOUT_MS);
    if (status != HAL_OK) return 0xFFFF;

    // Проверяем заголовок
    if (resp[0] != RESP_HEADER1 || resp[1] != RESP_HEADER2) return 0xFFFF;

    // Если оба байта расстояния равны 0xFF – ошибка измерения
    if (resp[2] == 0xFF && resp[3] == 0xFF) return 0xFFFF;

    // Собираем расстояние (большой эндиан: старший байт resp[2], младший resp[3])
    uint16_t distance = ((uint16_t)resp[2] << 8) | resp[3];
    return distance;
}

uint16_t sensor_get_distance_front(void) { return sensor_get_distance(0); }
uint16_t sensor_get_distance_left(void)  { return sensor_get_distance(1); }
uint16_t sensor_get_distance_right(void) { return sensor_get_distance(2); }
