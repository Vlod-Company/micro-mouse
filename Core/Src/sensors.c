#include "sensors.h"
#include "main.h"

static UART_HandleTypeDef* uart_handles[3] = {
    &huart1,
    &huart2,
    &huart3
};

#define PACKET_SIZE 8
#define BYTE_TIMEOUT_MS 10
#define MAX_ATTEMPTS 200

void sensors_init() {

}

static uint16_t read_sensor_packet(UART_HandleTypeDef *huart) {
	uint8_t packet[PACKET_SIZE];
	uint8_t b;
	HAL_StatusTypeDef status;

	for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++)
	{
		status = HAL_UART_Receive(huart, &b, 1, BYTE_TIMEOUT_MS);
		if (status != HAL_OK)
			continue;

		if (b != 0x5A)
			continue;

		packet[0] = 0x5A;

		status = HAL_UART_Receive(huart, &packet[1], 1, BYTE_TIMEOUT_MS);
		if (status != HAL_OK)
			continue;

		if (packet[1] != 0x5A)
			continue;

		status = HAL_UART_Receive(huart, &packet[2], 1, BYTE_TIMEOUT_MS);
		if (status != HAL_OK)
			continue;

		if (packet[2] != 0x15)
			continue;

		status = HAL_UART_Receive(huart, &packet[3], 1, BYTE_TIMEOUT_MS);
		if (status != HAL_OK)
			continue;

		if (packet[3] != 0x03)
			continue;

		status = HAL_UART_Receive(huart, &packet[4], 4, BYTE_TIMEOUT_MS);
		if (status != HAL_OK)
			continue;

		uint8_t sum = 0;
		for (int i = 0; i < 7; i++)
			sum += packet[i];

		if (sum != packet[7])
			continue;

		return ((uint16_t)packet[4] << 8) | packet[5];
	}

	return 0xFFFF;
}

uint16_t sensor_get_distance_front(void) { return read_sensor_packet(uart_handles[0]); }
uint16_t sensor_get_distance_left(void)  { return read_sensor_packet(uart_handles[1]); }
uint16_t sensor_get_distance_right(void) { return read_sensor_packet(uart_handles[2]); }
