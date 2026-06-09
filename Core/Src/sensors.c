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

#define PACKET_SIZE 8
#define TIMEOUT_MS  100

#define RESP_HEADER1 0xAA
#define RESP_HEADER2 0xAA

void sensors_init() {

}

static uint16_t read_sensor_packet(UART_HandleTypeDef *huart) {
    uint8_t packet[PACKET_SIZE];
    uint8_t b;
    HAL_StatusTypeDef status;

    // 1. Ищем первый 0x5A
    while (1) {
        status = HAL_UART_Receive(huart, &b, 1, TIMEOUT_MS);
        if (status != HAL_OK) return 0xFFFF;
        if (b == 0x5A) break;
    }

    // 2. Ищем второй 0x5A (следующий байт)
    status = HAL_UART_Receive(huart, &b, 1, TIMEOUT_MS);
    if (status != HAL_OK || b != 0x5A) return 0xFFFF;

    packet[0] = 0x5A;
    packet[1] = 0x5A;

    // 3. Читаем оставшиеся 6 байт
    status = HAL_UART_Receive(huart, &packet[2], 6, TIMEOUT_MS);
    if (status != HAL_OK) return 0xFFFF;

    // 4. Проверяем тип кадра (ваши значения)
    if (packet[2] != 0x15) return 0xFFFF;
    if (packet[3] != 0x03) return 0xFFFF;

    // 5. Проверяем контрольную сумму (сумма первых 7 байт)
    uint8_t sum = 0;
    for (int i = 0; i < 7; i++) {
        sum += packet[i];
    }
    if (sum != packet[7]) return 0xFFFF;

    // 6. Извлекаем расстояние
    uint16_t distance = ((uint16_t)packet[4] << 8) | packet[5];
    return distance;
}

uint16_t sensor_get_distance_front(void) { return sensor_get_distance(0); }
uint16_t sensor_get_distance_left(void)  { return sensor_get_distance(1); }
uint16_t sensor_get_distance_right(void) { return sensor_get_distance(2); }
