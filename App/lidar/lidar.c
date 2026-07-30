#include "lidar.h"
#include "lidar_parser.h"
#include <stdio.h>
#include <stddef.h>

/* 패킷 수신 진행 상태 */
typedef enum {
    STATE_WAIT_HEADER = 0,
    STATE_WAIT_FUNC_MARK,
    STATE_COLLECT_BODY
} lidar_rx_state_t;

/* ===== 내부 정적 변수 ===== */
static UART_HandleTypeDef *g_huart = NULL;

// cppcheck-suppress misra-c2012-8.9
static uint8_t              g_rx_byte = 0U;
static uint8_t              g_idx = 0U;
static volatile uint16_t    g_raw_dist_mm = 0U;

/* 샘플 도착 콜백 (스캔용, 이현우 추가). NULL 이면 미등록 = 기존 동작. */
static lidar_sample_cb_t    g_sample_cb = NULL;

/* ⚠️ 브링업 진단용 카운터 (이현우 추가, 문제 해결 후 제거 가능)
 *   rx_bytes   : USART6 로 들어온 총 바이트 (0 이면 배선/전원/baud 문제)
 *   valid_pkts : 체크섬·상태 검증 통과한 패킷 (0 인데 rx_bytes>0 이면 포맷/baud 불일치)
 *   bad_pkts   : 16바이트 모았는데 검증 실패 */
static volatile uint32_t    g_rx_bytes   = 0U;
static volatile uint32_t    g_valid_pkts = 0U;
static volatile uint32_t    g_bad_pkts   = 0U;
/* 수신 재무장(HAL_UART_Receive_IT) 결과. rc!=0 이면 수신이 아예 안 걸린 상태. */
static volatile uint8_t     g_last_rearm_rc = 0xFFU;   /* 0xFF = 아직 호출 전 */

/* ===== 내부 함수 선언 (UART/인터럽트 제어 전용) ===== */
static void lidar_reset_rx(void);
static void lidar_rearm_it(void);

/**
 * @brief 라이다 드라이버 초기화 및 인터럽트 수신 기동
 */
void lidar_init(UART_HandleTypeDef *huart)
{
    if (huart != NULL)
    {
        g_huart = huart;
        lidar_reset_rx();

        /* TODO: calib 모듈 재활성화 시 아래 블록 복원.
         * 현재는 raw 거리값을 그대로 쓰기 위해 의도적으로 비활성화 상태
         * (cppcheck misra-c2012-8.7 경고 3건은 이 블록이 켜지면 자동 해소됨) */
#if 0
        calib_reset();
#endif

        lidar_rearm_it();
    }
}

/**
 * @brief 최신 보정/필터링된 거리 값 반환 (mm 단위)
 */
uint16_t lidar_get_distance_mm(void)
{
    return g_raw_dist_mm;
}

/**
 * @brief 샘플 도착 콜백 등록 (스캔용, 이현우 추가)
 * @note  ISR 문맥에서 호출되므로 콜백은 짧아야 한다. NULL 이면 해제.
 */
void lidar_set_sample_callback(lidar_sample_cb_t cb)
{
    g_sample_cb = cb;
}

/**
 * @brief 브링업 진단 카운터 조회 (이현우 추가)
 * @note  rx=0        → 배선/전원/USART6 설정 문제 (바이트가 아예 안 옴)
 *        rx>0, ok=0  → baud 불일치 또는 프레임 포맷 다름
 *        ok>0        → 정상 수신 중
 */
void lidar_get_diag(uint32_t *rx_bytes, uint32_t *valid_pkts, uint32_t *bad_pkts)
{
    if (rx_bytes != NULL)   { *rx_bytes   = g_rx_bytes; }
    if (valid_pkts != NULL) { *valid_pkts = g_valid_pkts; }
    if (bad_pkts != NULL)   { *bad_pkts   = g_bad_pkts; }
}

/**
 * @brief UART 수신 상태 진단 (이현우 추가)
 * @param rearm_rc  마지막 HAL_UART_Receive_IT 반환값 (0=HAL_OK, 2=HAL_BUSY 등)
 * @param rx_state  HAL 의 RxState (0x22 = READY, 0x62 = BUSY_RX 가 정상 대기)
 * @param err_code  huart->ErrorCode (0=정상, bit0=PE bit1=NE bit2=FE bit3=ORE)
 */
void lidar_get_uart_diag(uint8_t *rearm_rc, uint32_t *rx_state, uint32_t *err_code)
{
    if (rearm_rc != NULL) { *rearm_rc = g_last_rearm_rc; }
    if (rx_state != NULL) {
        *rx_state = (g_huart != NULL) ? (uint32_t)g_huart->RxState : 0xFFFFFFFFU;
    }
    if (err_code != NULL) {
        *err_code = (g_huart != NULL) ? g_huart->ErrorCode : 0xFFFFFFFFU;
    }
}

/**
 * @brief UART 수신 완료 콜백 (1바이트씩 호출됨)
 */
// cppcheck-suppress constParameterPointer
void lidar_on_rx_cplt(UART_HandleTypeDef *huart)
{
    if ((g_huart != NULL) && (huart != NULL))
    {
        if (huart->Instance == g_huart->Instance)
        {
            static uint8_t g_buf[LIDAR_PACKET_SIZE];
            g_rx_bytes++;                       /* 진단: 수신 바이트 카운트 */
            const lidar_rx_state_t state = (g_idx == 0U) ? STATE_WAIT_HEADER
                                          : ((g_idx == 1U) ? STATE_WAIT_FUNC_MARK
                                          : STATE_COLLECT_BODY);

            switch (state)
            {
                case STATE_WAIT_HEADER:
                    if (lidar_parser_is_header(g_rx_byte) == true)
                    {
                        g_buf[g_idx] = g_rx_byte;
                        g_idx++;
                    }
                    break;

                case STATE_WAIT_FUNC_MARK:
                    if (lidar_parser_is_func_mark(g_rx_byte) == true)
                    {
                        g_buf[g_idx] = g_rx_byte;
                        g_idx++;
                    }
                    else
                    {
                        lidar_reset_rx();   /* 헤더 오탐 → 재동기화 */
                    }
                    break;

                case STATE_COLLECT_BODY:
                    g_buf[g_idx] = g_rx_byte;
                    g_idx++;

                    if ((uint32_t)g_idx >= (uint32_t)LIDAR_PACKET_SIZE)
                    {
                        lidar_sample_t smp = {0};
                        const bool ok = lidar_parser_validate(g_buf, &smp);
                        if (ok == false)
                        {
                            g_bad_pkts++;       /* 진단: 검증 실패 */
                        }
                        if (ok != false)
                        {
                            g_valid_pkts++;     /* 진단: 유효 패킷 */
                            /* TODO: calib 모듈 재활성화 시 아래로 교체
                             *   calib_process_distance(raw_mm);
                             *   g_raw_dist_mm = calib_get_distance_mm();
                             * 현재는 오프셋 보정/EMA 필터 없이 raw 값을 그대로 사용 중 */
#if 0
                            g_raw_dist_mm = calib_process_distance(smp.raw_mm);
#else
                            g_raw_dist_mm = (uint16_t)smp.raw_mm;  /* Raw 거리값 직접 저장 */
#endif
                            /* 스캔용 도착 통지 (이현우 추가).
                             * 이 시점이 "라이다 프레임 도착 순간" = 각도 래치 기준. */
                            if (g_sample_cb != NULL) {
                                g_sample_cb(&smp);
                            }
                        }
                        g_idx = 0U;
                    }
                    break;

                default:
                    lidar_reset_rx();
                    break;
            }

            /* 오버플로우 가드 */
            if ((size_t)g_idx >= sizeof(g_buf))
            {
                g_idx = 0U;
            }

            lidar_rearm_it();
        }
    }
}

/**
 * @brief UART 에러 콜백 (프레이밍/오버런/패리티 에러 시 호출)
 * @note  main.c의 HAL_UART_ErrorCallback()에서 반드시 호출 필요.
 */
void lidar_on_error(UART_HandleTypeDef *huart)
{
    if ((g_huart != NULL) && (huart != NULL))
    {
        if (huart->Instance == g_huart->Instance)
        {
            // cppcheck-suppress misra-c2012-14.4 ; ST HAL 매크로(do{...}while(0U)) 관용구, 벤더 코드 deviation
            __HAL_UART_CLEAR_PEFLAG(huart);
            lidar_reset_rx();
            lidar_rearm_it();
        }
    }
}


/* ===================== 내부 함수 구현 ===================== */

static void lidar_reset_rx(void)
{
    g_idx = 0U;
}

static void lidar_rearm_it(void)
{
    if (g_huart != NULL)
    {
        /* 진단: 수신 재무장 결과를 기록한다.
         * HAL_BUSY/HAL_ERROR 가 한 번 나면 이후 수신이 영영 안 되므로
         * rx=0 의 유력한 원인 중 하나다. lidar_get_uart_diag() 로 확인. */
        g_last_rearm_rc = (uint8_t)HAL_UART_Receive_IT(g_huart, &g_rx_byte, 1U);
    }
}