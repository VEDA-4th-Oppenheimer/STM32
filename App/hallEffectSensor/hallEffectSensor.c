/* ============================================================================
 *  hallEffectSensor.c  --  MT6701 자기 각도 엔코더 I2C 판독 구현
 *  최초 작성: 2026-07-23
 *  담당: 강유근 (원 구현) / 이현우 (NULL 가드 교정)
 *  계약과 배선은 hallEffectSensor.h 상단 참조.
 * ==========================================================================*/

#include "hallEffectSensor.h"
#include <stddef.h>

/* MT6701 각도 레지스터 비트 배치 (데이터시트):
 *   0x03 : Angle[13:6]           -> rx_buf[0]
 *   0x04 : Angle[5:0] << 2       -> rx_buf[1] 의 상위 6비트
 * 따라서 raw = (rx[0] << 6) | (rx[1] >> 2) 로 14비트를 복원한다.
 * 결과는 항상 0 ~ 16383 안에 들어가므로 별도 범위 검사가 필요 없다. */
#define MT6701_ANGLE_HI_SHIFT   6U
#define MT6701_ANGLE_LO_SHIFT   2U
#define MT6701_COUNTS           16384.0f   /* 2^14 = 한 바퀴 */

HAL_StatusTypeDef Encoder_Read(I2C_HandleTypeDef *hi2c, Encoder_t *encoder_data)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    /* ⚠️ 여기는 원래 `||` 였다. 그러면 핸들만 유효하고 결과 포인터가 NULL 인
     *   경우에도 통과해 encoder_data->raw_angle 에서 NULL 역참조로 죽는다.
     *   가드가 정반대로 동작하고 있었다. 두 인자가 **모두** 유효해야 한다. */
    if ((hi2c != NULL) && (encoder_data != NULL)) {
        uint8_t rx_buf[2] = {0};

        /* 0x03 레지스터부터 2바이트.
         * 핸들은 축에 따라 다르다 — Pan=I2C3(PA8/PC9), Tilt=I2C1(PB8/PB9).
         * (원 주석은 "I2C1: PA8/PC9" 로 둘을 뒤섞어 적고 있었다) */
        status = HAL_I2C_Mem_Read(hi2c, MT6701_ADDR, REG_ANGLE_14B,
                                  I2C_MEMADD_SIZE_8BIT, rx_buf, 2u, I2C_TIMEOUT);

        if (status == HAL_OK) {
            const uint16_t raw =
                (uint16_t)(((uint16_t)rx_buf[0] << MT6701_ANGLE_HI_SHIFT) |
                           ((uint16_t)rx_buf[1] >> MT6701_ANGLE_LO_SHIFT));

            encoder_data->raw_angle = raw;
            encoder_data->degree    = (float)raw * (360.0f / MT6701_COUNTS);
        }
        /* 실패 시 encoder_data 는 건드리지 않는다 — 호출자가 status 를 보고
         * 판단해야 한다(헤더의 경고 참조). */
    }

    return status;
}
