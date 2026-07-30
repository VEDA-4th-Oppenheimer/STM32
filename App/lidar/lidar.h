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

/* ---------------------------------------------------------------------------
 *  샘플 도착 콜백 (스캔용) — 이현우 추가
 *
 *  유효 패킷을 파싱한 "그 순간" ISR 문맥에서 호출된다. 스캔은 이 시점에
 *  모터 각도를 래치해야 각도-거리 동기가 맞는다(도착순간 래치 방식).
 *
 *  ⚠️ 콜백은 ISR 에서 불리므로 반드시 짧아야 한다. UART 송신 등 블로킹
 *    작업을 하면 라이다 바이트를 놓친다(수신은 바이트 단위 IT).
 *
 *  등록하지 않으면(기본 NULL) 아무 일도 하지 않으므로 기존 동작과 동일하다.
 * ------------------------------------------------------------------------- */
/* v5: 거리뿐 아니라 원시 품질 필드 전체를 넘긴다(포인터는 콜백 반환 시 무효). */
typedef void (*lidar_sample_cb_t)(const lidar_sample_t *s);
void lidar_set_sample_callback(lidar_sample_cb_t cb);

/* 브링업 진단 카운터 (문제 해결 후 제거 가능).
 *   rx=0       → 바이트가 아예 안 옴 (배선/전원/USART6 설정)
 *   rx>0, ok=0 → baud 불일치 또는 프레임 포맷 다름
 *   ok>0       → 정상 수신 */
void lidar_get_diag(uint32_t *rx_bytes, uint32_t *valid_pkts, uint32_t *bad_pkts);

/* UART 수신 상태 진단.
 *   rearm_rc : 마지막 HAL_UART_Receive_IT 반환 (0=OK, 2=BUSY → 수신 미기동)
 *   rx_state : HAL RxState (0x62=BUSY_RX 가 정상 수신대기, 0x22=READY 면 미대기)
 *   err_code : ErrorCode (0=정상, 8=ORE 오버런 등) */
void lidar_get_uart_diag(uint8_t *rearm_rc, uint32_t *rx_state, uint32_t *err_code);

#endif /* LIDAR_H */