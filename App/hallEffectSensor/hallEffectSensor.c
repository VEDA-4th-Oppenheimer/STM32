
/* ============================================================================
 * Create by 2026.07.23
 *  hallEffectSensor.c  --  홀 효과 센서 제어
 *  담당: 강유근
 * ==========================================================================*/

#include "hallEffectSensor.h"
#include <stddef.h>

HAL_StatusTypeDef Encoder_Read(I2C_HandleTypeDef *hi2c, Encoder_t *encoder_data)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    if ((hi2c != NULL) || (encoder_data != NULL)) {
        uint8_t rx_buf[2] = {0};


        // 0x03 레지스터부터 2바이트 읽기 (I2C1: PA8/PC9)
        status = HAL_I2C_Mem_Read(hi2c, MT6701_ADDR, REG_ANGLE_14B,
                                  I2C_MEMADD_SIZE_8BIT, rx_buf, 2u, I2C_TIMEOUT);

        if (status == HAL_OK) {
            // rx_buf[0]: 상위 비트[13:6], rx_buf[1]: 하위 비트[7:2]
            uint16_t raw = ((uint16_t)rx_buf[0] << 6) | (rx_buf[1] >> 2);

            encoder_data->raw_angle = raw;
            encoder_data->degree = (float)raw * (360.0f / 16384.0f);
        }
    }




    return status;
}