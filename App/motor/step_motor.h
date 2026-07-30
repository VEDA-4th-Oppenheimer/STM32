/* ============================================================================
 *  step_motor.h
 *  Pan/Tilt 2축 스텝 모터 제어 헤더 (v4 스캐너 대응)
 *  - 1.8도 스텝, 1/16 마이크로스텝 (0.1125도/펄스)
 *  - Pan: 400Hz 정속 제어 (타이머 인터럽트)
 *  - Tilt: 800Hz 정속 제어 (타이머 인터럽트)
 * ==========================================================================*/
#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "main.h"
#include "protocol.h"
#include "hallEffectSensor.h"

/* --- 유지보수성 (매크로) --- */
#define MOTOR_STEP_DEGREE        1.8f   // 모터 기본 스텝각 (1.8도)
#define MICROSTEP_DIVIDER        16     // 1/16 마이크로스텝 분주비

/* 1펄스당 움직이는 각도 계산 (1.8 / 16 = 0.1125도/펄스) */
#define DEG_PER_PULSE            (MOTOR_STEP_DEGREE / (float)MICROSTEP_DIVIDER)

#define PAN_CRUISE_FREQ_HZ       400
#define TILT_CRUISE_FREQ_HZ      800

/* 엔코더 기준 영점 조절 (천장 수직 등 기구물 보정용) */
#define PAN_ZERO_OFFSET_DEG      0.0f
#define TILT_ZERO_OFFSET_DEG     90.0f

/* Homing 시 Pan 엔코더 초기 판독 재시도 설정
 * (부팅 직후 I2C/센서가 아직 안정화되지 않아 첫 판독이 실패하는 경우 대비) */
#define HOME_ENCODER_MAX_RETRY      5
#define HOME_ENCODER_RETRY_DELAY_MS 10

/* 0.1도(ddeg) <-> 펄스 변환 매크로 */
#define DDEG_TO_PULSE(ddeg)      (((ddeg) * 8) / 9)
#define PULSE_TO_DDEG(pulse)     (((pulse) * 9) / 8)

/* 모터 구동 상태 열거형 */
typedef enum {
    MOTOR_IDLE = 0,
    MOTOR_HOMING,
    MOTOR_SCAN_MOVING_TO_START,
    MOTOR_SCAN_TILT_SWEEP,
    MOTOR_SCAN_PAN_STEP
} motor_state_t;

/* --- 전역 제어 구조체 --- */
typedef struct {
    motor_state_t state;
    uint8_t       is_homed;

    // 절대 위치 (펄스 단위, 0 = 영점)
    int32_t       current_pan_pulse;
    int32_t       current_tilt_pulse;

    // 목표 위치
    int32_t       target_pan_pulse;
    int32_t       target_tilt_pulse;

    // 스캔 시퀀스 파라미터
    int32_t       scan_pan_start_pulse;
    int32_t       scan_pan_end_pulse;
    int32_t       scan_tilt_start_pulse;
    int32_t       scan_tilt_end_pulse;
    int32_t       scan_pan_step_pulse;
} motor_ctrl_t;

/* 전역 변수 포인터 (외부 접근용) */
extern motor_ctrl_t *motor_ctrl;

/* --- 외부 인터페이스 API --- */
void step_motor_init(void);
void step_motor_home(void);
void step_motor_scan_start(const struct proto_scan_start *ss);
void step_motor_stop(void);
void motor_disarm(void); // uart_rpi.c 호출 규격 준수

/* --- 타이머 인터럽트 핸들러 (사용자가 설정한 TIM ISR에서 호출) --- */
void step_motor_pan_isr(void);  // 400Hz 주기로 호출
void step_motor_tilt_isr(void); // 800Hz 주기로 호출

#endif /* STEP_MOTOR_H */