#ifndef LIDAR_H
#define LIDAR_H

#include "stm32f4xx_hal.h"   /* 프로젝트에서 쓰는 실제 HAL 헤더로 맞춰주세요 */

void lidar_init(UART_HandleTypeDef *huart);
void lidar_on_rx_cplt(UART_HandleTypeDef *huart);
void lidar_on_error(UART_HandleTypeDef *huart);
uint16_t lidar_get_distance_mm(void);

#endif /* LIDAR_H */