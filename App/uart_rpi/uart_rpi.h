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

uint32_t uart_rpi_get_last_hb_tick(void);

/* 브링업 진단 카운터 (RPi 링크 문제 해결 후 제거 가능).
 *   rx=0                   → USART1 에 바이트가 안 옴 (배선/RPi 미송신)
 *   rx>0, frames=0, crc=0  → 프레임 경계를 못 잡음 (SOF 불일치)
 *   crc_err>0              → 바이트는 오는데 깨짐 (baud/GND/노이즈)
 *   frames>0, tx=0         → 파싱은 되는데 응답 안 함 (디스패치)
 *   frames>0, tx>0         → STM 은 정상 → RPi 수신측 문제 */
void uart_rpi_get_diag(uint32_t *rx, uint32_t *frames, uint32_t *crc_err,
                       uint32_t *tx_frames);


/* 스캔 점 1개 상행 (CMD_SCAN_DATA).
 *   v5 부터 원시 품질 필드까지 8개라 개별 인자 대신 구조체를 그대로 받는다.
 *   내부 point 카운터(s_scan_count)를 1 증가시킨다. */
void uart_rpi_send_scan_point(const struct proto_scan_point *pt);

/* 스캔 완료 통지 (CMD_SCAN_DONE):
 *   이번 스캔에서 상행한 총 point 수(내부 카운터)를 담아 전송.
 *   카운터는 CMD_SCAN_START 수신 시 0 으로 리셋된다. */
void uart_rpi_send_scan_done(void);
#endif /* UART_RPI_H */
