#ifndef LIDAR_H
#define LIDAR_H

#include "stm32f4xx_hal.h"

void lidar_init(UART_HandleTypeDef *huart);
void lidar_on_rx_cplt(UART_HandleTypeDef *huart);
void lidar_on_error(UART_HandleTypeDef *huart);
uint16_t lidar_get_distance_mm(void);

#endif /* LIDAR_H */