#include "calib.h"

/* ===== 캘리브레이션 파라미터 ===== */
#define CALIB_OFFSET_MM   100    /* 고정 오차 보정값 (+10cm 튀던 raw 데이터 보정) */
#define CALIB_EMA_ALPHA   0.2f   /* EMA 필터 계수 (0~1, 클수록 최신값에 민감) */

static volatile uint16_t g_dist_mm = 0;

void calib_reset(void)
{
    g_dist_mm = 0;
}

uint16_t calib_process_distance(uint32_t raw_mm)
{
    int32_t corrected = (int32_t)raw_mm - CALIB_OFFSET_MM;
    if (corrected < 0)
    {
        corrected = 0;
    }

    if (g_dist_mm == 0)
    {
        /* 첫 유효값은 필터 없이 그대로 반영 (초기 수렴 지연 방지) */
        g_dist_mm = (uint16_t)corrected;
    }
    else
    {
        g_dist_mm = (uint16_t)((1.0f - CALIB_EMA_ALPHA) * (float)g_dist_mm
                              + CALIB_EMA_ALPHA * (float)corrected);
    }

    return g_dist_mm;
}

uint16_t calib_get_distance_mm(void)
{
    return g_dist_mm;
}