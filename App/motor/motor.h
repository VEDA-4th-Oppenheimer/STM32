/* ============================================================================
 *  motor.h  --  Pan/Tilt 2축 스텝모터 축 드라이버 (DRV8825 / 17HS4401)
 * ----------------------------------------------------------------------------
 *  담당: 강유근 (원 구현) / 이현우 (계층 분리 리팩터링)
 *
 *  ★ 이 계층은 "축을 시키는 대로 움직이는 것"만 한다.
 *    스캔 시퀀스(줄 진행 / serpentine 반전 / 완료 판정 / 홈 절차)는
 *    App/scan 소관이다. 여기에 스캔 상태를 넣지 말 것.
 *
 *    왜 나눴나: 이전 구현은 스캔 시퀀서가 타이머 ISR 안에 있었다. 그래서
 *    시퀀서가 필요로 하는 일(HAL_Delay / printf / 블로킹 I2C)을 ISR 안에서
 *    하게 됐고, ISR 안 HAL_Delay 는 SysTick 을 기다리다 데드락이 나며
 *    블로킹 I/O 는 라이다 UART 수신을 밀어낸다. 계층을 나누면 이 부류의
 *    버그가 구조적으로 발생할 수 없다.
 *
 *  구동 방식:
 *    STEP/DIR/EN 전부 GPIO. 타이머(TIM1=Pan, TIM2=Tilt)는 Base 인터럽트만
 *    쓰고, 인터럽트 1회당 최대 1펄스를 낸다. 즉 타이머 주파수 = 최대 pps.
 *    ※ EN 은 active-low.
 *
 *  하드웨어:
 *    Pan  STEP=PB14  DIR=PB15  EN=PB1   엔코더 I2C3 (PA8/PC9)
 *    Tilt STEP=PA6   DIR=PA7   EN=PB6   엔코더 I2C1 (PB8/PB9)
 * ==========================================================================*/
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "hallEffectSensor.h"
#include <stdbool.h>
#include <stdint.h>

/* --- 축 식별 --------------------------------------------------------------*/
typedef enum {
    MOTOR_AXIS_PAN = 0,
    MOTOR_AXIS_TILT,
    MOTOR_AXIS_COUNT
} motor_axis_t;

/* --- 기구 상수 ------------------------------------------------------------*/
#define MOTOR_STEP_DEGREE     1.8f    /* 모터 기본 스텝각                    */
#define MOTOR_MICROSTEP       16      /* DRV8825 물리 스위치 설정 (1/16)     */

/* 1펄스당 각도 = 1.8 / 16 = 0.1125도 = 1.125 ddeg */
#define MOTOR_DEG_PER_PULSE   (MOTOR_STEP_DEGREE / (float)MOTOR_MICROSTEP)

/* STEP 펄스 폭. DRV8825 최소 요구는 HIGH/LOW 각 1.9us (데이터시트).
 * 84MHz 에서 volatile 루프 1회 ≈ 7사이클 이므로 50회 ≈ 4us — 2배 여유.
 * ⚠️ 이전 값은 5000(≈0.42ms)이었다. 틸트 ISR 주기가 1.25ms 인데 그 안에서
 *   0.42ms 를 스핀하면 CPU 의 3분의 1을 ISR 에서 태우고, 그동안 라이다
 *   UART(USART6) 수신 인터럽트가 밀려 프레임을 잃는다.
 * ⚠️ 실기 확인 필요 — 드라이버가 4us 펄스를 확실히 먹는지는 실측 후 확정. */
#define MOTOR_STEP_PULSE_SPIN 50u

/* DIR 셋업 시간. DRV8825 는 STEP 상승 전 650ns 를 요구한다(84MHz=55사이클).
 * GPIO 쓰기 두 번만으로는 아슬아슬해 명시적으로 벌린다. */
#define MOTOR_DIR_SETUP_SPIN  10u

/* --- 엔코더 영점 상수 -----------------------------------------------------
 *  리밋스위치를 쓰지 않으므로 양축 모두 이 상수에 의존한다.
 *  조립 후 1회 실측해 채워야 한다:
 *    ① 축을 기준 자세로 맞춘다 (팬=기준 방위 / 틸트=바닥 nadir)
 *    ② CMD_HOME 을 걸고 CMD_HOMED 가 올려주는 엔코더 raw 를 읽는다
 *       (RPi 에서 `turret_test state` 가 raw 와 도 환산을 같이 찍는다)
 *    ③ 그 값을 아래에 넣는다
 *  상수가 틀려도 CMD_HOMED 가 raw 를 함께 올리므로 이미 찍은 스캔의 각도를
 *  오프라인에서 재계산할 수 있다(재스캔 불필요). */
#define MOTOR_PAN_ZERO_OFFSET_DEG    173.61f
#define MOTOR_TILT_ZERO_OFFSET_DEG   301.01f

/* 부팅 직후 I2C/센서가 아직 안정화되지 않아 첫 판독이 NACK 나는 경우 대비 */
#define MOTOR_ENC_MAX_RETRY          5u
#define MOTOR_ENC_RETRY_DELAY_MS     10u

/* --- ddeg <-> pulse 변환 --------------------------------------------------
 *  1 pulse = 1.125 ddeg 이므로  pulse = ddeg * 8/9,  ddeg = pulse * 9/8.
 *
 *  ⚠️ 정수 연산이라 나머지가 버려진다. **절대각 변환에만 쓰고 증분에 쓰지 말 것.**
 *    예) 1도(=10 ddeg) 스텝을 미리 펄스로 굳히면 9펄스(1.0125도)로 고정돼
 *      180줄 누적 시 방위 커버리지가 어긋난다. 목표는 매번
 *        target = motor_ddeg_to_pulse(start_ddeg + line * step_ddeg)
 *      처럼 절대각에서 계산해야 절삭이 누적되지 않는다.
 *
 *  틸트는 부호가 있으므로 0 방향 절삭이 아니라 부호를 보존해 반올림한다. */
static inline int32_t motor_ddeg_to_pulse(int32_t ddeg)
{
    return (ddeg >= 0) ? (((ddeg * 8) + 4) / 9)
                       : -((((-ddeg) * 8) + 4) / 9);
}

static inline int32_t motor_pulse_to_ddeg(int32_t pulse)
{
    return (pulse >= 0) ? (((pulse * 9) + 4) / 8)
                        : -((((-pulse) * 9) + 4) / 8);
}

/* --- 수명주기 -------------------------------------------------------------*/
void motor_init(void);          /* GPIO 초기 상태. 드라이버는 비활성으로 시작 */
void motor_enable(void);        /* 양축 드라이버 전류 인가                   */
void motor_disarm(void);        /* 즉시 정지 + 전류 차단 (CMD_DISARM)        */

/* --- 위치 제어 (메인루프에서 호출) ----------------------------------------*/
void    motor_set_target(motor_axis_t ax, int32_t pulse);
int32_t motor_get_target(motor_axis_t ax);
bool    motor_is_idle(motor_axis_t ax);   /* 현재 == 목표 */

/* 현재 위치를 강제로 덮어쓴다 (홈 확립 / 탈조 재영점).
 * 목표도 함께 옮기므로 호출 직후 축은 정지 상태가 된다 — 위치만 바꾸고
 * 목표를 두면 다음 ISR 이 옛 목표를 향해 달려나간다. */
void motor_sync_pulse(motor_axis_t ax, int32_t pulse);

/* --- 관측 (ISR/메인루프 어디서든 안전) ------------------------------------
 *  32비트 정렬 워드 읽기라 Cortex-M4 에서 원자적이다. 라이다 프레임이
 *  도착한 순간 각도를 래치하는 용도. */
int32_t motor_get_pulse(motor_axis_t ax);
int16_t motor_get_ddeg(motor_axis_t ax);

/* --- 엔코더 (블로킹 — 메인루프 전용, ISR 에서 호출 금지) ------------------*/
HAL_StatusTypeDef motor_read_encoder(motor_axis_t ax, Encoder_t *out);

/* 축에 물린 I2C 핸들 (Pan=&hi2c3 / Tilt=&hi2c1).
 * 판독은 motor_read_encoder 를 쓰고, 이건 **버스 자체**를 다뤄야 할 때만
 * 쓴다(주소 스캔, SDA 물림 9클럭 복구 등 브링업 진단). 핸들 extern 이 파일
 * 곳곳에 흩어지는 걸 막으려고 여기 한 곳에서만 노출한다. */
I2C_HandleTypeDef *motor_axis_i2c(motor_axis_t ax);

/* 엔코더 실측각(도) → 펄스. 영점 상수를 적용한다. */
int32_t motor_encoder_deg_to_pulse(motor_axis_t ax, float deg);

/* 재시도 포함 판독 후 펄스로 환산. 성공 시 HAL_OK 와 *out_pulse 반환.
 * 부팅 직후용이라 HAL_Delay 를 쓴다 → 메인루프에서만 호출할 것. */
HAL_StatusTypeDef motor_read_encoder_pulse(motor_axis_t ax, int32_t *out_pulse);

/* --- 타이머 ISR 진입점 ----------------------------------------------------
 *  TIM1(Pan) / TIM2(Tilt) Base 인터럽트에서 호출. 호출 1회당 최대 1펄스.
 *  분기·printf·I2C·Delay 없음 — 순수 펄스 발생과 카운터 증감만. */
void motor_pan_isr(void);
void motor_tilt_isr(void);

#endif /* MOTOR_H */
