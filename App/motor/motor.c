/* ============================================================================
 *  motor.c  --  스텝모터 2축 제어 (경로 테스트용 최소 스텁)
 *  담당: 강유근 (현재 이현우 테스트 스텁 — 폐루프/엔코더 미구현)
 * ==========================================================================*/
/* ============================================================================
 *  motor.c  --  스텝모터 2축 제어 (경로 테스트용 최소 스텁)
 * ==========================================================================*/
#include "motor.h"

static TIM_HandleTypeDef *s_pan  = NULL;   /* 방위(Pan) STEP 타이머  (TIM1) */
static TIM_HandleTypeDef *s_tilt = NULL;  /* 고각(Tilt) STEP 타이머 (TIM3) */

/* 축 구동 내부 함수 */
static void axis_run(TIM_HandleTypeDef *tim, uint32_t ch,
                     GPIO_TypeDef *dir_port, uint16_t dir_pin,
                     GPIO_TypeDef *en_port,  uint16_t en_pin,
                     int16_t target, uint8_t is_complementary)
{
    if (tim == NULL) return;

    /* 1. 목표각이 0이면 정지 및 드라이버 비활성화 */
    if (target == 0) {
        if (is_complementary) {
            HAL_TIMEx_PWMN_Stop(tim, ch);
        } else {
            HAL_TIM_PWM_Stop(tim, ch);
        }
        HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_SET); /* EN active-low -> HIGH(비활성) */
        return;
    }

    /* 2. 방향 설정 (+: CW / -: CCW) */
    HAL_GPIO_WritePin(dir_port, dir_pin, (target > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 3. 드라이버 활성화 (EN active-low -> LOW) */
    HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET);

    /* 4. STEP 펄스 듀티 50% 보장 */
    __HAL_TIM_SET_COMPARE(tim, ch, __HAL_TIM_GET_AUTORELOAD(tim) / 2u);

    /* 5. PWM 출력 시작 (보상 채널인 경우 EX 전용 함수 사용) */
    if (is_complementary) {
        HAL_TIMEx_PWMN_Start(tim, ch);  /* PB14 (TIM1_CH2N) 용 */
    } else {
        HAL_TIM_PWM_Start(tim, ch);     /* PA6 (TIM3_CH1) 용 */
    }
}

void motor_init(TIM_HandleTypeDef *pan_step, TIM_HandleTypeDef *tilt_step)
{
    s_pan  = pan_step;
    s_tilt = tilt_step;
    motor_disarm();                     /* 부팅 시 안전(비활성) */
}

void motor_set_target(int16_t pan_ddeg, int16_t tilt_ddeg)
{
    /* Pan 축: PB14(TIM1_CH2N) 사용 시 보상채널 플래그(1) 전달 */
    axis_run(s_pan,  PAN_TIM_CHANNEL,
             PAN_DIR_GPIO_Port,  PAN_DIR_Pin,
             PAN_EN_GPIO_Port,   PAN_EN_Pin,   pan_ddeg, 1);

    /* Tilt 축: PA6(TIM3_CH1) 일반 채널 사용 (플래그 0) */
    axis_run(s_tilt, TILT_TIM_CHANNEL,
             TILT_DIR_GPIO_Port, TILT_DIR_Pin,
             TILT_EN_GPIO_Port,  TILT_EN_Pin,  tilt_ddeg, 0);
}

void motor_disarm(void)
{
    if (s_pan) {
        HAL_TIMEx_PWMN_Stop(s_pan, PAN_TIM_CHANNEL);
        HAL_GPIO_WritePin(PAN_EN_GPIO_Port, PAN_EN_Pin, GPIO_PIN_SET);
    }
    if (s_tilt) {
        HAL_TIM_PWM_Stop(s_tilt, TILT_TIM_CHANNEL);
        HAL_GPIO_WritePin(TILT_EN_GPIO_Port, TILT_EN_Pin, GPIO_PIN_SET);
    }
}