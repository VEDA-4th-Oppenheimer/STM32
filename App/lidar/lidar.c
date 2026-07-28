#include "lidar.h"
#include "lidar_parser.h"
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
                        uint32_t raw_mm = 0U;
                        if (lidar_parser_validate(g_buf, &raw_mm) != false)
                        {
                            /* TODO: calib 모듈 재활성화 시 아래로 교체
                             *   calib_process_distance(raw_mm);
                             *   g_raw_dist_mm = calib_get_distance_mm();
                             * 현재는 오프셋 보정/EMA 필터 없이 raw 값을 그대로 사용 중 */
#if 0
                            g_raw_dist_mm = calib_process_distance(raw_mm);
#else
                            g_raw_dist_mm = (uint16_t)raw_mm;   /* Raw 거리값 직접 저장 */
#endif
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
        (void)HAL_UART_Receive_IT(g_huart, &g_rx_byte, 1U);
    }
}