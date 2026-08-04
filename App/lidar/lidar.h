/* ============================================================================
 *  lidar.h  --  TOFSense-F2P UART 수신 드라이버 (USART6)
 * ----------------------------------------------------------------------------
 *  담당: 송영빈 (원 구현) / 이현우 (각도 래치 + 샘플 큐 확장)
 *
 *  ★ 각도-거리 짝짓기가 이 파일의 핵심이다.
 *
 *    프레임 마지막 바이트를 받은 **그 순간** 모터 각도를 함께 잡아 둔다.
 *    폴링(예전 lidar_get_distance_mm)으로는 안 된다 — 메인루프가 언제 읽느냐에
 *    따라 최대 한 프레임 주기(10ms)만큼 어긋나는데, 틸트가 90도/s 로 도는
 *    동안 그 사이 0.9도를 움직인다. 격자가 1도인데 스미어가 0.9도면 못 쓴다.
 *
 *    래치는 ISR 에서 하고(모터 카운터 읽기뿐이라 안전), 실제 상행은 메인루프가
 *    한다. 그 사이를 잇는 게 아래 SPSC 링버퍼다.
 *
 *  데이터 흐름:
 *    USART6 RX ISR ─ 1바이트씩 상태머신 ─ 16바이트 완성
 *                          │
 *                          ├─ scan_latch_angles()  (pan/tilt 스냅샷)
 *                          └─ 링버퍼에 push
 *                                    │
 *    메인루프 lidar_process() ────────┴─ pop ─ scan_submit_sample() ─ RPi 상행
 * ==========================================================================*/
#ifndef LIDAR_H
#define LIDAR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 샘플 링버퍼 깊이.
 * 라이다가 100Hz(10ms/프레임)이고 메인루프는 프레임 상행(115200 에서 23바이트
 * = 약 2ms)이 가장 느린 구간이라 보통 한 칸을 넘지 않는다. 8칸이면 메인루프가
 * 80ms 밀려도 샘플을 잃지 않는다. 2의 거듭제곱이라 인덱스 마스킹이 싸다. */
#define LIDAR_SAMPLE_QUEUE_LEN   8U

void lidar_init(UART_HandleTypeDef *huart);

/* HAL_UART_RxCpltCallback / ErrorCallback 에서 위임 (USART6) */
void lidar_on_rx_cplt(UART_HandleTypeDef *huart);
void lidar_on_error(UART_HandleTypeDef *huart);

/* 메인루프에서 호출: 큐에 쌓인 샘플을 꺼내 scan 으로 넘긴다. */
void lidar_process(void);

/* 최근 거리(mm). 스캔과 무관한 단순 조회/디버그용. */
uint16_t lidar_get_distance_mm(void);

/* --- 진단 카운터 -----------------------------------------------------------
 *  1축 브링업에서 "라이다가 얼어붙었다" 를 원인별로 가르는 데 하루가 걸렸다.
 *  (배선 / 체크섬 / 큐 넘침 / 메인루프 지연) 이 값들을 보면 1초면 갈린다. */
uint32_t lidar_get_frame_count(void);   /* 체크섬 통과 프레임 수      */
uint32_t lidar_get_csum_errors(void);   /* 체크섬 불일치 수           */
uint32_t lidar_get_queue_drops(void);   /* 큐가 차서 버린 샘플 수     */

/* rx=0 이면 위 카운터 이전 단계(배선/전원/USART6 설정)에서 바이트가 아예
 * 안 들어온다는 뜻 — frame_count/csum_errors 만으로는 구분이 안 된다. */
uint32_t lidar_get_rx_bytes(void);      /* USART6 로 들어온 총 바이트 */

/* UART 수신 재무장(HAL_UART_Receive_IT) 상태 진단.
 *   rearm_rc : 마지막 재무장 반환값 (0=OK, 2=BUSY → 수신 미기동)
 *   rx_state : HAL RxState (0x62=BUSY_RX 가 정상 수신대기, 0x22=READY 면 미대기)
 *   err_code : ErrorCode (0=정상, 8=ORE 오버런 등) */
void lidar_get_uart_diag(uint8_t *rearm_rc, uint32_t *rx_state, uint32_t *err_code);

#endif /* LIDAR_H */
