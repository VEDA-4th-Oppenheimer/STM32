/* ============================================================================
 *  uart_rpi.h  --  RPi <-> STM32 UART 포트 제어 및 프로토콜 디스패처
 * ----------------------------------------------------------------------------
 *  담당: 이현우
 *  역할: USART1(RPi 링크)로 들어오는 바이트를 링버퍼에 적재(ISR)하고,
 *        메인루프에서 protocol.h 프레임으로 파싱 → CMD 디스패치.
 *        상행 프레임(PONG/HOMED/DISTANCE 등) 조립·송신도 담당.
 *
 *  main.c 는 이 모듈의 훅만 호출한다 (HAL 콜백/메인루프 위임).
 * ==========================================================================*/
#ifndef UART_RPI_H
#define UART_RPI_H

#include "main.h"        /* UART_HandleTypeDef (HAL 타입) */
#include "protocol.h"    /* PROTO_*, enum proto_cmd, struct proto_* */

/* USART1(RPi 링크) 초기화 + 수신(IT) 시작. main() USER CODE 2 에서 호출. */
void uart_rpi_init(UART_HandleTypeDef *huart);

/* HAL_UART_RxCpltCallback 에서 위임: 수신 바이트를 링버퍼에 적재 후 재등록. */
void uart_rpi_on_rx_cplt(UART_HandleTypeDef *huart);

/* HAL_UART_ErrorCallback 에서 위임: 오버런 플래그 클리어 + 수신 재개. */
void uart_rpi_on_error(UART_HandleTypeDef *huart);

/* 메인루프에서 호출: 링버퍼를 비우며 프레임 파싱/디스패치. */
void uart_rpi_process(void);

/* protocol.h 프레임 조립 후 USART1 로 상행 송신 (PONG/HOMED/DISTANCE ...). */
void uart_rpi_send_frame(uint8_t cmd, const void *payload, uint8_t payload_len);

#endif /* UART_RPI_H */
