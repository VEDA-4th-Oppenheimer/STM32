#ifndef LIDAR_H
#define LIDAR_H

#include "stm32f4xx_hal.h"
#include "lidar_parser.h"

void lidar_init(UART_HandleTypeDef *huart);
void lidar_on_rx_cplt(UART_HandleTypeDef *huart);
void lidar_on_error(UART_HandleTypeDef *huart);
uint16_t lidar_get_distance_mm(void);
uint8_t  lidar_get_dis_status(void);
uint16_t lidar_get_intensity(void);
uint16_t lidar_get_system_time_ms(void);
lidar_confidence_t lidar_get_confidence(void);

#endif /* LIDAR_H */