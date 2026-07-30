/* ============================================================================
 *  motor.c  --  스텝모터 2축 제어 (DRV8825/ELB050411)
 *  담당: 강유근
 * ==========================================================================*/
#include "motor.h"
#include <stddef.h>

#define POSITION_DEADBAND_DEG   0.3f

/* 외부 전역변수 완전 삭제 */
// extern float g_pan_target_deg;
// extern float g_tilt_target_deg;

/* 모듈 내부 상태를 기억할 정적 포인터 */
static MotorController_t *s_ctx_ptr = NULL;

static void axis_run(TIM_HandleTypeDef *tim, uint32_t ch,
                     GPIO_TypeDef *dir_port, uint16_t dir_pin,
                     GPIO_TypeDef *en_port,  uint16_t en_pin,
                     int16_t degg, uint8_t connect)
{
    if ((tim == NULL) || (dir_port == NULL) || (en_port == NULL)) {
        return;
    }

    /* 1. 목표각이 0이면 정지 및 드라이버 비활성화 */
    if (degg == 0) {
        if (connect != 0u) {
            HAL_TIMEx_PWMN_Stop(tim, ch);
        } else {
            HAL_TIM_PWM_Stop(tim, ch);
        }
        HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_SET); /* EN HIGH (비활성) */
        return;
    }

    /* 2. 방향 설정 (+: CW / -: CCW) */
    HAL_GPIO_WritePin(dir_port, dir_pin, (degg > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    /* 3. 드라이버 활성화 (EN active-low -> LOW) */
    HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET);

    /* 4. STEP 펄스 듀티 50% 보장 */
    __HAL_TIM_SET_COMPARE(tim, ch, __HAL_TIM_GET_AUTORELOAD(tim) / 2u);

    /* 5. PWM 출력 시작 */
    if (connect != 0u) {
        HAL_TIMEx_PWMN_Start(tim, ch);
    } else {
        HAL_TIM_PWM_Start(tim, ch);
    }
}

void motor_init(MotorController_t *ctx, TIM_HandleTypeDef *pan_step, TIM_HandleTypeDef *tilt_step, I2C_HandleTypeDef *pan_i2c, I2C_HandleTypeDef *tilt_i2c)
{
    if (ctx != NULL) {
        ctx->pan_step  = pan_step;
        ctx->tilt_step = tilt_step;
        ctx->pan_i2c   = pan_i2c;
        ctx->tilt_i2c  = tilt_i2c;

        ctx->pan_target_deg = 0.0f;
        ctx->tilt_target_deg = 0.0f;

        s_ctx_ptr = ctx; // 초기화 시 포인터 저장

        HAL_GPIO_WritePin(PAN_EN_GPIO_Port, PAN_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TILT_EN_GPIO_Port, TILT_EN_Pin, GPIO_PIN_SET);
        HAL_TIMEx_PWMN_Stop(pan_step, PAN_TIM_CHANNEL);
        HAL_TIM_PWM_Stop(tilt_step, TILT_TIM_CHANNEL);
    }
}

/* 특수 인자 없이 내부 s_ctx_ptr을 사용해 업데이트 */
void motor_update_position(void)
{
    if (s_ctx_ptr == NULL) return;

    /* 1. Pan 축 위치 제어 */
    if ((s_ctx_ptr->pan_i2c != NULL) && (Encoder_Read(s_ctx_ptr->pan_i2c, &s_ctx_ptr->pan_encoder) == HAL_OK)) {
        float err_pan = s_ctx_ptr->pan_target_deg - s_ctx_ptr->pan_encoder.degree;
        if (err_pan > POSITION_DEADBAND_DEG) {
            axis_run(s_ctx_ptr->pan_step, PAN_TIM_CHANNEL, PAN_DIR_GPIO_Port, PAN_DIR_Pin, PAN_EN_GPIO_Port, PAN_EN_Pin, 10, 0u);
        } else if (err_pan < -POSITION_DEADBAND_DEG) {
            axis_run(s_ctx_ptr->pan_step, PAN_TIM_CHANNEL, PAN_DIR_GPIO_Port, PAN_DIR_Pin, PAN_EN_GPIO_Port, PAN_EN_Pin, -10, 0u);
        } else {
            axis_run(s_ctx_ptr->pan_step, PAN_TIM_CHANNEL, PAN_DIR_GPIO_Port, PAN_DIR_Pin, PAN_EN_GPIO_Port, PAN_EN_Pin, 0, 0u);
        }
    }

    /* 2. Tilt 축 위치 제어 (Pan과 동일한 방식으로 s_ctx_ptr->tilt_target_deg 사용) */
    if ((s_ctx_ptr->tilt_i2c != NULL) && (Encoder_Read(s_ctx_ptr->tilt_i2c, &s_ctx_ptr->tilt_encoder) == HAL_OK)) {
        float err_tilt = s_ctx_ptr->tilt_target_deg - s_ctx_ptr->tilt_encoder.degree;
        if (err_tilt > POSITION_DEADBAND_DEG) {
            axis_run(s_ctx_ptr->tilt_step, TILT_TIM_CHANNEL, TILT_DIR_GPIO_Port, TILT_DIR_Pin, TILT_EN_GPIO_Port, TILT_EN_Pin, 10, 1u);
        } else if (err_tilt < -POSITION_DEADBAND_DEG) {
            axis_run(s_ctx_ptr->tilt_step, TILT_TIM_CHANNEL, TILT_DIR_GPIO_Port, TILT_DIR_Pin, TILT_EN_GPIO_Port, TILT_EN_Pin, -10, 1u);
        } else {
            axis_run(s_ctx_ptr->tilt_step, TILT_TIM_CHANNEL, TILT_DIR_GPIO_Port, TILT_DIR_Pin, TILT_EN_GPIO_Port, TILT_EN_Pin, 0, 1u);
        }
    }
}

/* 외부에서 호출 시 내부 상태값 갱신 */
void motor_set_target(int16_t pan_ddeg, int16_t tilt_ddeg)
{
    if (s_ctx_ptr != NULL) {
        s_ctx_ptr->pan_target_deg  = (float)pan_ddeg / 10.0f;
        s_ctx_ptr->tilt_target_deg = (float)tilt_ddeg / 10.0f;
    }
}

void motor_init_dual_axis(void)
{
    motor_set_target(0, 0);
    motor_disarm();
}