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

#include <stdbool.h>
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
/* 반환 true = 프레임 전체 송신 성공. 실패를 무시하면 스캔 점 카운터가
 * 실제 수신 수보다 커진다(구현부 주석). */
bool uart_rpi_send_frame(uint8_t cmd, const void *payload, uint8_t payload_len);

/* 스캔 점 카운터 리셋 — scan_start 가 요청을 승인한 뒤에만 부른다. */
void uart_rpi_reset_scan_count(void);

/* 송신 실패 누적 (진단). */
uint32_t uart_rpi_tx_fail_count(void);

/* 수신 링버퍼 오버플로 누적 (진단). 0 이 아니면 메인루프가 오래 막혔다는 뜻. */
uint32_t uart_rpi_rx_overflow_count(void);


/* 스캔 점 1개 상행 (CMD_SCAN_DATA, protocol v5):
 *   pan/tilt = 기구각(0.1°, 틸트는 부호), d_mm = 거리.
 *   나머지는 TOFSense-F2P 프레임 원본 그대로 — 정규화도 유효성 판정도 하지
 *   않는다(판정 기준이 바뀌어도 재해석할 수 있어야 하므로). stm_ts_ms 는
 *   내부에서 HAL_GetTick() 으로 채운다.
 *   내부 point 카운터(s_scan_count)를 1 증가시킨다. */
void uart_rpi_send_scan_point(int16_t pan_ddeg, int16_t tilt_ddeg,
                              uint16_t d_mm, uint16_t signal_strength,
                              uint32_t device_time_ms,
                              uint8_t dis_status, uint8_t range_precision);

/* 스캔 완료 통지 (CMD_SCAN_DONE):
 *   이번 스캔에서 상행한 총 point 수(내부 카운터)를 담아 전송.
 *   카운터는 CMD_SCAN_START 수신 시 0 으로 리셋된다. */
void uart_rpi_send_scan_done(void);
#endif /* UART_RPI_H */
