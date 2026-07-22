/* ============================================================================
 *  motor.h  --  스텝모터 2축 제어 (DRV8825/ELB050411)
 * ----------------------------------------------------------------------------
 *  담당: 강유근  (⚠️ 현재는 이현우가 만든 "경로 테스트용 최소 스텁"
 *                 — 정밀 위치제어/폐루프/MT6701 엔코더는 강유근이 이어받아 구현)
 *
 *  STEP = TIM PWM (PB14: TIM1_CH2N 방위, PA6 : TIM3_CH1 고각)
 *  DIR/EN = GPIO (PAN_DIR/PAN_EN, TILT_DIR/TILT_EN)  ※ EN active-low
 * ==========================================================================*/
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"          /* HAL 타입 + 핀 라벨(PAN_DIR_Pin 등) */
/*타이머 채널 정의 */
#define PAN_TIM_CHANNEL   TIM_CHANNEL_2   /* PB14: TIM1_CH2N */
#define TILT_TIM_CHANNEL  TIM_CHANNEL_1   /* PA6 : TIM3_CH1  */

/* STEP PWM 타이머 핸들 저장 + 부팅 시 안전(비활성) 상태로. */
void motor_init(TIM_HandleTypeDef *pan_step, TIM_HandleTypeDef *tilt_step);

/* [테스트] 목표각 수신 → 부호로 방향만 정해 회전 시작.
 * ※ 정밀 위치제어(각도만큼 정확히 이동)는 강유근 폐루프+엔코더 몫. */
void motor_set_target(int16_t pan_ddeg, int16_t tilt_ddeg);

/* 안전정지: 양축 PWM 정지 + EN 비활성(active-low → HIGH). */
void motor_disarm(void);

#endif /* MOTOR_H */
