#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

/* STM32F4 HAL 라이브러리 헤더 */
#include "stm32f4xx_hal.h"

/* -------------------------------------------------------------------------- */
/* MT6701 I2C 매크로 설정                                                     */
/* -------------------------------------------------------------------------- */
#define MT6701_ADDR        (0x06 << 1) // 7-bit 주소(0x06) -> HAL용 8-bit 주소(0x0C)
#define REG_ANGLE_14B      0x03        // 14비트 각도 레지스터 시작 주소
#define I2C_TIMEOUT        100         // 통신 타임아웃 (ms)

/* -------------------------------------------------------------------------- */
/* 데이터 구조체 정의                                                         */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint16_t raw_angle;  // 14-bit Raw 데이터 (0 ~ 16383)
    float degree;        // 실제 계산된 각도 (0.0° ~ 359.921°)
} Encoder_t;

/* -------------------------------------------------------------------------- */
/* 함수 원형 선언                                                             */
/* -------------------------------------------------------------------------- */
/**
 * @brief MT6701 엔코더로부터 각도를 읽어옵니다.
 * @param hi2c 연결된 I2C 핸들 포인터 (&hi2c1)
 * @param encoder_data 결과값을 저장할 구조체 포인터
 * @return HAL_StatusTypeDef 통신 성공 여부 (HAL_OK)
 */
HAL_StatusTypeDef Encoder_Read(I2C_HandleTypeDef *hi2c, Encoder_t *encoder_data);

#ifdef __cplusplus
}
#endif

#endif /*ENCODER_H*/