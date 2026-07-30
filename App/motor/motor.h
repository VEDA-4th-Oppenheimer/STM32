/* ============================================================================
 *  motor.h  --  스텝모터 2축 제어 (DRV8825/ELB050411)
 * ----------------------------------------------------------------------------
 *  담당: 강유근
 *  STEP = TIM PWM (PB14: TIM1_CH2N 방위, PA6 : TIM3_CH1 고각)
 *  DIR/EN = GPIO (PAN_DIR/PAN_EN, TILT_DIR/TILT_EN)  ※ EN active-low
 * ==========================================================================*/

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "hallEffectSensor.h"

#define PAN_TIM_CHANNEL   TIM_CHANNEL_2
#define TILT_TIM_CHANNEL  TIM_CHANNEL_1

typedef struct {
    TIM_HandleTypeDef *pan_step;
    TIM_HandleTypeDef *tilt_step;
    I2C_HandleTypeDef *pan_i2c;
    I2C_HandleTypeDef *tilt_i2c;

    Encoder_t pan_encoder;
    Encoder_t tilt_encoder;

    /* 추가: 전역 변수를 대체하여 모터 모듈 내부에서 관리할 타겟 각도 */
    float pan_target_deg;
    float tilt_target_deg;
} MotorController_t;

void motor_init(MotorController_t *ctx,
                TIM_HandleTypeDef *pan_step, TIM_HandleTypeDef *tilt_step,
                I2C_HandleTypeDef *pan_i2c, I2C_HandleTypeDef *tilt_i2c);

/* 메인 루프에서 특수 인자 없이 호출하도록 변경 */
void motor_update_position(void);

/* UART 등에서 특수 인자 없이 목표 각도만 던져주면 동작 */
void motor_set_target(int16_t pan_ddeg, int16_t tilt_ddeg);

void motor_init_dual_axis(void);
void motor_disarm(void);

#endif /* MOTOR_H */