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
        uint8_t rx_buf[2]  = {0};
        uint8_t reg_addr   = (uint8_t)REG_ANGLE_14B;

        /* 0x03 레지스터부터 2바이트.
         * 핸들은 축에 따라 다르다 — Pan=I2C3(PA8/PC9), Tilt=I2C1(PB8/PB9).
         * (원 주석은 "I2C1: PA8/PC9" 로 둘을 뒤섞어 적고 있었다) */
        /* ⚠️ 원래는 HAL_I2C_Mem_Read 였다. 그건 레지스터 주소를 쓴 뒤
         *   **repeated start** 로 읽기로 전환하는데, 실기에서 그 repeated
         *   start 가 400kHz 마진을 못 버티고 NACK(err=0x04) 났다. 벤치의
         *   방식 탐색 결과가 근거다(2026-08-05):
         *
         *     A Mem_Read(400k)      실패 NACK
         *     B Transmit+Receive    OK  raw=3531
         *     C Receive만           "OK" 지만 raw=0  <- 포인터 없이 읽어 무의미
         *     D Mem_Read(100k)      OK  raw=3531
         *
         *   B 와 D 가 같은 값을 냈다 = 3531 이 진짜 각도. 원인은 방식이
         *   아니라 400kHz 에서의 repeated start 타이밍 마진 부족이고,
         *   B(중간에 STOP) 와 D(속도 하향) 둘 다 회피책이다. 둘 다 적용해
         *   마진을 최대로 둔다 — 엔코더는 홈 확립에 쓰이고 홈이 틀리면
         *   좌표계 전체가 틀어지므로 "겨우 되는" 상태로 두면 안 된다.
         *
         *   ⚠️ 순서 의존: Transmit 이 성공해야만 Receive 가 의미 있다. */
        status = HAL_I2C_Master_Transmit(hi2c, MT6701_ADDR, &reg_addr, 1u,
                                         I2C_TIMEOUT);
        if (status == HAL_OK) {
            status = HAL_I2C_Master_Receive(hi2c, MT6701_ADDR, rx_buf, 2u,
                                            I2C_TIMEOUT);
        }

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

/* 헤더의 설명 참조. DeInit 이 페리페럴을 끄고 클럭·상태를 초기화하며,
 * Init 이 CubeMX 가 넣어둔 설정(100kHz 등)으로 다시 세운다. hi2c->Init 은
 * 구조체에 남아 있으므로 재설정 인자를 따로 들고 있을 필요가 없다.
 *
 * ⚠️ DeInit 실패는 무시하고 Init 을 시도한다. 이미 망가진 상태를 되살리려는
 *   것이므로 "정상적으로 끄는 데 실패했다" 는 진행을 막을 이유가 못 된다.
 *   최종 판단은 Init 결과로 한다. */
HAL_StatusTypeDef Encoder_BusRecover(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    if (hi2c != NULL) {
        (void)HAL_I2C_DeInit(hi2c);
        status = HAL_I2C_Init(hi2c);
    }
    return status;
}
