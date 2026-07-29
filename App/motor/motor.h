/* ============================================================================
 *  motor.h  --  스텝모터 2축 제어 (DRV8825/ELB050411)
 * ----------------------------------------------------------------------------
 *  담당: 강유근
 *  STEP = TIM PWM (PB14: TIM1_CH2N 방위, PA6 : TIM3_CH1 고각)
 *  DIR/EN = GPIO (PAN_DIR/PAN_EN, TILT_DIR/TILT_EN)  ※ EN active-low
 * ==========================================================================*/
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"              /* HAL 타입 및 핀 라벨 */
#include "hallEffectSensor.h"   /* MT6701 엔코더 헤더 */

/* 타이머 채널 정의 */
#define PAN_TIM_CHANNEL   TIM_CHANNEL_2   /* PB14: TIM1_CH2N */
#define TILT_TIM_CHANNEL  TIM_CHANNEL_1   /* PA6 : TIM3_CH1  */

/* -------------------------------------------------------------------------- */
/* 제어기 컨텍스트 구조체                                                      */
/* -------------------------------------------------------------------------- */
typedef struct {
    TIM_HandleTypeDef *pan_step;   /* Pan 타이머 핸들 포인터 */
    TIM_HandleTypeDef *tilt_step;  /* Tilt 타이머 핸들 포인터 */
    I2C_HandleTypeDef *pan_i2c;    /* Pan 엔코더 I2C 핸들 포인터 */
    I2C_HandleTypeDef *tilt_i2c;   /* Tilt 엔코더 I2C 핸들 포인터 */

    Encoder_t pan_encoder;         /* Pan 엔코더 최신 데이터 */
    Encoder_t tilt_encoder;        /* Tilt 엔코더 최신 데이터 */
} MotorController_t;

/* -------------------------------------------------------------------------- */
/* 함수 원형 선언                                                             */
/* -------------------------------------------------------------------------- */

/* 모터 및 엔코더 바인딩 초기화 */
void motor_init(MotorController_t *ctx,
                TIM_HandleTypeDef *pan_step, TIM_HandleTypeDef *tilt_step,
                I2C_HandleTypeDef *pan_i2c, I2C_HandleTypeDef *tilt_i2c);

/* 목표 각도로 제어 업데이트 (메인 루프에서 주기적 호출) */
void motor_update_position(MotorController_t *ctx, float pan_target_deg, float tilt_target_deg);

/* 기존 motor_set_target 호환용 */
void motor_set_target(int16_t pan_ddeg, int16_t tilt_ddeg);

/* 양축 정지 초기화 */
void motor_init_dual_axis(void);

/* 안전 정지 (uart_rpi.c 호환성을 위해 인자 없는 형태 유지) */
void motor_disarm(void);

#endif /* MOTOR_H */
