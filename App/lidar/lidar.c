#include "lidar.h"
#include <stdio.h>

/* 내부 정적 변수 선언 */
static UART_HandleTypeDef *g_hlidar_uart = NULL;
static volatile uint16_t g_lidar_dist_mm = 0;
static uint8_t g_lidar_rx_byte = 0;
static uint8_t g_lidar_buf[32];  /* 패킷 안전 수신 버퍼 */
static uint8_t g_lidar_idx = 0;

/**
 * @brief 라이다 드라이버 초기화 및 인터럽트 수신 기동
 */
void lidar_init(UART_HandleTypeDef *huart)
{
  if (huart != NULL)
  {
    g_hlidar_uart = huart;
    g_lidar_idx = 0;
    g_lidar_dist_mm = 0;

    /* 1바이트 수신 인터럽트 시작 */
    HAL_UART_Receive_IT(g_hlidar_uart, &g_lidar_rx_byte, 1);
  }
}

/**
 * @brief 수신된 최신 거리 값 반환 (mm 단위)
 */
uint16_t lidar_get_distance_mm(void)
{
  return g_lidar_dist_mm;
}

/**
 * @brief UART 수신 완료 콜백 핸들러
 */
void lidar_on_rx_cplt(UART_HandleTypeDef *huart)
{
  if (g_hlidar_uart == NULL)
  {
    return;
  }

  if (huart->Instance == g_hlidar_uart->Instance)
  {
    /* 1. 첫 번째 헤더 검증 (0x57) */
    if (g_lidar_idx == 0)
    {
      if (g_lidar_rx_byte == 0x57)
      {
        g_lidar_buf[g_lidar_idx++] = g_lidar_rx_byte;
      }
    }
    /* 2. 두 번째 바이트 수신 */
    else if (g_lidar_idx == 1)
    {
      g_lidar_buf[g_lidar_idx++] = g_lidar_rx_byte;
    }
    /* 3. 세 번째 바이트 이후 적재 */
    else
    {
      g_lidar_buf[g_lidar_idx++] = g_lidar_rx_byte;

      /* 16바이트 패킷이 완성되었을 때 */
      if (g_lidar_idx >= 16)
      {
        if (g_lidar_buf[0] == 0x57)
        {
          /* 🔍 영빈 님이 발견한 데이터 위치 매핑 (buf[8], [9], [10]) */
          uint32_t dist_raw = ((uint32_t)g_lidar_buf[8]) |
                              ((uint32_t)g_lidar_buf[9] << 8) |
                              ((uint32_t)g_lidar_buf[10] << 16);

          /* 유효 범위 검사 (오류 상수 제외) */
          if (dist_raw > 0 && dist_raw < 10000)
          {
            /* 🛠️ [고정 오차 보정] +10cm 튀던 날것 데이터에서 100mm를 강제 감산 */
            int32_t cal_dist = (int32_t)dist_raw - 100;

            // 센서 바로 앞을 막아 음수가 되거나 오류 코드가 찍히면 0mm 처리
            if (cal_dist < 0 || dist_raw == 12800 || dist_raw == 128000)
            {
              cal_dist = 0;
            }

            /* 🛠️ [소프트웨어 필터] 자자하게 흔들리는 수치 안정화 (EMA 필터) */
            float alpha = 0.2f;
            if (g_lidar_dist_mm == 0) {
              g_lidar_dist_mm = (uint16_t)cal_dist;
            } else {
              g_lidar_dist_mm = (uint16_t)((1.0f - alpha) * (float)g_lidar_dist_mm + alpha * (float)cal_dist);
            }
          }
        }

        // 패킷 처리가 끝나면 인덱스 리셋
        g_lidar_idx = 0;
      }
    }

    /* 오버플로우 가드 */
    if (g_lidar_idx >= sizeof(g_lidar_buf))
    {
      g_lidar_idx = 0;
    }

    /* 다음 1바이트 수신을 위해 인터럽트 재등록 */
    HAL_UART_Receive_IT(g_hlidar_uart, &g_lidar_rx_byte, 1);
  }
}