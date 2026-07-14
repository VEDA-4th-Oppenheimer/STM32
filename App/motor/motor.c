/* ============================================================================
 *  motor.c  --  스텝모터 2축 제어 (경로 테스트용 최소 스텁)
 *  담당: 강유근 (현재 이현우 테스트 스텁 — 폐루프/엔코더 미구현)
 * ==========================================================================*/
#include "motor.h"

static TIM_HandleTypeDef *s_pan;   /* 방위(Pan) STEP 타이머  (TIM2) */
static TIM_HandleTypeDef *s_tilt;  /* 고각(Tilt) STEP 타이머 (TIM3) */

/* 한 축을 방향 정하고 회전 시작 (연속 PWM = 경로 검증용) */
static void axis_run(TIM_HandleTypeDef *tim, uint32_t ch,
                     GPIO_TypeDef *dir_port, uint16_t dir_pin,
                     GPIO_TypeDef *en_port,  uint16_t en_pin,
                     int16_t target)
{
    /* 방향: 목표각 부호 (+CW / -CCW) */
    HAL_GPIO_WritePin(dir_port, dir_pin,
                      (target >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 드라이버 활성 (EN active-low → LOW) */
    HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET);

    /* STEP 펄스 듀티 50% 보장 (CubeMX Pulse 값이 0이면 펄스가 안 나옴) */
    __HAL_TIM_SET_COMPARE(tim, ch, __HAL_TIM_GET_AUTORELOAD(tim) / 2u);

    (void)HAL_TIM_PWM_Start(tim, ch);   /* STEP 펄스 시작 → 회전 */
}

void motor_init(TIM_HandleTypeDef *pan_step, TIM_HandleTypeDef *tilt_step)
{
    s_pan  = pan_step;
    s_tilt = tilt_step;
    motor_disarm();                     /* 부팅 시 안전(비활성) */
}

void motor_set_target(int16_t theta_ddeg, int16_t phi_ddeg)
{
    /* [테스트] 부호로 방향만 정해 회전. 정밀 위치는 강유근 폐루프. */
    axis_run(s_pan,  TIM_CHANNEL_1,
             PAN_DIR_GPIO_Port,  PAN_DIR_Pin,
             PAN_EN_GPIO_Port,   PAN_EN_Pin,   theta_ddeg);
    axis_run(s_tilt, TIM_CHANNEL_1,
             TILT_DIR_GPIO_Port, TILT_DIR_Pin,
             TILT_EN_GPIO_Port,  TILT_EN_Pin,  phi_ddeg);
}

void motor_disarm(void)
{
    (void)HAL_TIM_PWM_Stop(s_pan,  TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(s_tilt, TIM_CHANNEL_1);
    /* EN active-low → HIGH = 비활성 */
    HAL_GPIO_WritePin(PAN_EN_GPIO_Port,  PAN_EN_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(TILT_EN_GPIO_Port, TILT_EN_Pin, GPIO_PIN_SET);
}
