#ifndef APP_LIDAR_LIDAR_H_
#define APP_LIDAR_LIDAR_H_

#include "main.h"

/* 라이다 상태머신 초기화 및 인터럽트 시작 */
void lidar_init(UART_HandleTypeDef *huart);

/* 수신 완료된 최신 거리 값 반환 (mm 단위) */
uint16_t lidar_get_distance_mm(void);

/* main.c의 HAL_UART_RxCpltCallback에서 호출해줄 라이다 전용 ISR 핸들러 */
void lidar_on_rx_cplt(UART_HandleTypeDef *huart);

#endif /* APP_LIDAR_LIDAR_H_ */