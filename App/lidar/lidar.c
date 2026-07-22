#include "lidar.h"
#include "lidar_parser.h"
#include "calib.h"
#include <stdio.h>

/* 패킷 수신 진행 상태 */
typedef enum {
    STATE_WAIT_HEADER = 0,
    STATE_WAIT_FUNC_MARK,
    STATE_COLLECT_BODY,
} lidar_rx_state_t;

/* ===== 내부 정적 변수 ===== */
static UART_HandleTypeDef *g_huart = NULL;
static uint8_t              g_rx_byte = 0;
static uint8_t              g_buf[LIDAR_PACKET_SIZE];
static uint8_t              g_idx = 0;

/* ===== 내부 함수 선언 (UART/인터럽트 제어 전용) ===== */
static void lidar_reset_rx(void);
static void lidar_rearm_it(void);


/**
 * @brief 라이다 드라이버 초기화 및 인터럽트 수신 기동
 */
void lidar_init(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    g_huart = huart;
    lidar_reset_rx();
    calib_reset();

    lidar_rearm_it();
}

/**
 * @brief 최신 보정/필터링된 거리 값 반환 (mm 단위)
 */
uint16_t lidar_get_distance_mm(void)
{
    return calib_get_distance_mm();
}

/**
 * @brief UART 수신 완료 콜백 (1바이트씩 호출됨)
 */
void lidar_on_rx_cplt(UART_HandleTypeDef *huart)
{
    if (g_huart == NULL || huart->Instance != g_huart->Instance)
    {
        return;
    }

    const lidar_rx_state_t state = (g_idx == 0) ? STATE_WAIT_HEADER
                                  : (g_idx == 1) ? STATE_WAIT_FUNC_MARK
                                  : STATE_COLLECT_BODY;

    switch (state)
    {
        case STATE_WAIT_HEADER:
            if (lidar_parser_is_header(g_rx_byte))
            {
                g_buf[g_idx++] = g_rx_byte;
            }
            break;

        case STATE_WAIT_FUNC_MARK:
            if (lidar_parser_is_func_mark(g_rx_byte))
            {
                g_buf[g_idx++] = g_rx_byte;
            }
            else
            {
                lidar_reset_rx();   /* 헤더 오탐 → 재동기화 */
            }
            break;

        case STATE_COLLECT_BODY:
            g_buf[g_idx++] = g_rx_byte;
            if (g_idx >= LIDAR_PACKET_SIZE)
            {
                uint32_t raw_mm;
                if (lidar_parser_validate(g_buf, &raw_mm))
                {
                    calib_process_distance(raw_mm);
                }
                g_idx = 0;
            }
            break;
    }

    /* 오버플로우 가드 */
    if (g_idx >= sizeof(g_buf))
    {
        g_idx = 0;
    }

    lidar_rearm_it();
}

/**
 * @brief UART 에러 콜백 (프레이밍/오버런/패리티 에러 시 호출)
 * @note  main.c의 HAL_UART_ErrorCallback()에서 반드시 호출 필요.
 */
void lidar_on_error(UART_HandleTypeDef *huart)
{
    if (g_huart == NULL || huart->Instance != g_huart->Instance)
    {
        return;
    }

    __HAL_UART_CLEAR_PEFLAG(huart);
    lidar_reset_rx();
    lidar_rearm_it();
}


/* ===================== 내부 함수 구현 ===================== */

static void lidar_reset_rx(void)
{
    g_idx = 0;
}

static void lidar_rearm_it(void)
{
    HAL_UART_Receive_IT(g_huart, &g_rx_byte, 1);
}