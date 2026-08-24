/* ============================================================================
 *  motor.h  --  Pan/Tilt 2축 스텝모터 축 드라이버 (DRV8825 / 17HS4401)
 * ----------------------------------------------------------------------------
 *  담당: 강유근 (원 구현) / 이현우 (계층 분리 리팩터링)
 *
 *  핵심: 이 계층은 "축을 시키는 대로 움직이는 것"만 한다.
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
 *    쓰고, 인터럽트 1회당 최대 1펄스를 낸다. 즉 타이머 주파수 = 그 순간의 pps.
 *    ※ EN 은 active-low.
 *
 *    핵심: 타이머 주기(ARR)는 **고정이 아니다.** 펄스마다 이 계층이 다시 쓴다
 *      (아래 "가감속 램프"). 즉 축의 속도는 상수가 아니라 이동 구간에 따라
 *      변한다 — 시작/끝은 느리고 가운데는 빠르다.
 *      주의: 프리스케일러와 ARR 은 motor_init() 이 직접 잡는다. CubeMX 가
 *        MX_TIMx_Init 에 넣어 둔 Prescaler/Period 값은 쓰이지 않는다.
 *
 *  하드웨어:
 *    Pan  STEP=PB14  DIR=PB15  EN=PB1   엔코더 I2C3 (PA8/PC9)
 *    Tilt STEP=PA6   DIR=PA7   EN=PB6   엔코더 I2C1 (PB8/PB9)
 * ==========================================================================*/
#ifndef MOTOR_H
#define MOTOR_H

#include "hallEffectSensor.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* --- 축 식별 --------------------------------------------------------------*/
typedef enum {
  MOTOR_AXIS_PAN = 0,
  MOTOR_AXIS_TILT,
  MOTOR_AXIS_COUNT
} motor_axis_t;

/* --- 기구 상수 ------------------------------------------------------------*/
#define MOTOR_STEP_DEGREE 1.8f /* 모터 기본 스텝각                    */
#define MOTOR_MICROSTEP 16     /* DRV8825 물리 스위치 설정 (1/16)     */

/* 1펄스당 각도 = 1.8 / 16 = 0.1125도 = 1.125 ddeg */
#define MOTOR_DEG_PER_PULSE (MOTOR_STEP_DEGREE / (float)MOTOR_MICROSTEP)

/* STEP 펄스 폭. DRV8825 최소 요구는 HIGH/LOW 각 1.9us (데이터시트).
 * 84MHz 에서 volatile 루프 1회 ≈ 7사이클 이므로 50회 ≈ 4us — 2배 여유.
 * 주의: 이전 값은 5000(≈0.42ms)이었다. 틸트 ISR 주기가 1.25ms 인데 그 안에서
 *   0.42ms 를 스핀하면 CPU 의 3분의 1을 ISR 에서 태우고, 그동안 라이다
 *   UART(USART6) 수신 인터럽트가 밀려 프레임을 잃는다.
 * 주의: 실기 확인 필요 — 드라이버가 4us 펄스를 확실히 먹는지는 실측 후 확정. */
#define MOTOR_STEP_PULSE_SPIN 50u

/* DIR 셋업 시간. DRV8825 는 STEP 상승 전 650ns 를 요구한다(84MHz=55사이클).
 * GPIO 쓰기 두 번만으로는 아슬아슬해 명시적으로 벌린다. */
#define MOTOR_DIR_SETUP_SPIN 10u

/* ===========================================================================
 *  가감속 램프 (사다리꼴 속도 프로파일)
 * ---------------------------------------------------------------------------
 *  왜 넣었나: 이전에는 타이머 주기가 고정이라 축이 **정지 → 순항 속도**로
 *    한 펄스 만에 튀어 올랐다가 도착하는 순간 똑같이 뚝 멈췄다. 스텝모터는
 *    그렇게 속도를 계단으로 바꾸면 로터가 고유진동으로 링잉한다(scan.h 의
 *    SCAN_*_SETTLE_MS 주석이 말하는 그 진동이다). 시작과 끝만 느리게 출발/
 *    도착시키면 그 충격이 사라진다.
 *
 *  핵심: 이동 각도는 **하나도 안 바뀐다.** 램프가 건드리는 것은 펄스 사이의
 *    간격뿐이고 펄스 개수는 그대로다. 목표 각도·좌표계·엔코더 대조 로직은
 *    영향을 받지 않는다.
 *
 *  프로파일:
 *      pps
 *       ^      ______________________ cruise
 *       |     /                      \
 *       |    /                        \
 *   start ──/                          \──
 *       +--------------------------------------> 남은 펄스
 *
 *    · 정지 상태에서는 언제나 MOTOR_START_PPS 로 출발한다.
 *    · 매 펄스 accel 만큼 속도를 올려 순항 속도에서 멈춘다.
 *    · "지금 속도에서 시작 속도까지 감속하는 데 필요한 펄스 수" 가 남은
 *      펄스 수 이상이 되는 순간부터 감속으로 돌아선다. 그래서 짧은 이동
 *      (팬 격자 1스텝 0.9도 = 8펄스)은 순항에 도달하지 못하고 삼각 프로파일이
 * 된다 — 느리게 출발해 느리게 도착하므로 짧은 이동일수록 더 조용하다.
 *
 *  주의: 램프가 못 막는 것: **목표를 현재 위치로 당겨서 하는 즉시 정지**
 *    (motor_disarm / scan_stop). 남은 펄스가 0 이 되므로 순항 속도에서 그냥
 *    끊긴다. 중단 경로라 의도한 동작이고, 종전 동작과 같은 수준의 충격이다.
 *    이동 중에 목표가 반대쪽으로 바뀌면 속도를 시작 속도로 되감아 반전한다.
 *
 *  --- 튜닝 ----------------------------------------------------------------
 *  MOTOR_START_PPS — 정지 상태에서 낼 수 있는 첫 펄스 속도(pull-in).
 *    이 값만 충분히 낮으면 출발 충격은 사라진다. 낮출수록 조용하지만 램프
 *    구간이 길어진다. 주의: 너무 낮추면 ARR 이 TIM1(16비트)을 넘는다 — 아래
 *    _Static_assert 가 잡는다(최저 약 16pps).
 *
 *  MOTOR_*_CRUISE_PPS — 램프 후 순항 속도.
 *      틸트 750pps = 84.375도/s. 라이다 100Hz 기준 0.844도/샘플이라 0.9도 격자에
 *        셀당 약 1.07샘플이 떨어져 클럭 지터와 위상 결측을 완벽히 흡수한다 (충진율 99.46%).
 *      팬 100pps = 11.25도/s. 팬은 스윕 중 정지해 있어 샘플과 무관하다.
 *
 *  MOTOR_*_ACCEL_PPS2 — 가속도(pps/s).
 *    램프가 잡아먹는 펄스 수는  n = (v_cruise^2 - v_start^2) / (2*accel).
 *      팬  (50→100, 1200) : 4펄스(0.4도) / 0.04초 (줄 바꿈 지연 최소화)
 *      틸트(50→750, 1800) : 156펄스(17.5도) / S-Curve 벨형 가속 적용 (진동 28% 감쇠)
 *    S-Curve 소프트랜딩을 통해 감속 끝단 충격을 제거하므로 스윕 후 정착 시간(SCAN_LINE_SETTLE_MS)을
 *    100ms -> 40ms 로 단축할 수 있어 총 스캔 시간을 9분 30초대로 유지한다.
 * ========================================================================= */

/* 타이머 1틱 = 1us. motor_init() 이 프리스케일러를 여기에 맞춰 직접 잡으므로
 * 펄스 간격(ARR+1)을 그대로 us 로 읽을 수 있다. */
#define MOTOR_TIM_TICK_HZ 1000000u

#define MOTOR_START_PPS 50u /* 양축 공통 출발/도착 속도 */

#define MOTOR_PAN_CRUISE_PPS 100u /* 11.25도/s */
#define MOTOR_PAN_ACCEL_PPS2 1200u

/* 주의: 800pps = 90도/s. 라이다 100Hz 이므로 샘플 간격이 **0.9도** 가 되어
 *   0.9도 격자와 정확히 같아진다. 셀당 평균 1샘플이라 여유가 없다.
 *   Phase 4: 750 PPS (84.375도/s)로 미세 조정하여 격자당 1.07개 샘플 밀도를 확보하고 결측을 제거한다.
 */
#define MOTOR_TILT_CRUISE_PPS 750u /* 84.375도/s — 셀당 1.07샘플 */
#define MOTOR_TILT_ACCEL_PPS2 1800u

/* --- Phase 3: S-Curve (저크 제한) 가감속 프로파일 ------------------------
 *  사다리꼴의 불연속 가속도 점프(Jerk=∞)를 2차 포물선 가속도 벨형 곡선으로
 * 완화하여 출발/순항진입/감속/정지 시의 기구 진동과 관성 충격을 근본적으로
 * 제거한다. MOTOR_SCURVE_FLOOR_Q8: 기동 및 착지 시 최저 가속도 비율 (64 = 25%).
 */
#define MOTOR_SCURVE_ENABLE 1u
#define MOTOR_SCURVE_FLOOR_Q8 64u

/* TIM1 의 ARR 은 16비트다. 시작 속도가 너무 느리면 한 주기를 표현할 수 없어
 * 카운터가 엉뚱하게 감기므로, 조용하게 만들겠다고 무한정 낮추지 못하게 막는다.
 */
_Static_assert(
    (MOTOR_TIM_TICK_HZ / MOTOR_START_PPS) <= 65536u,
    "MOTOR_START_PPS is too low: pulse interval overflows TIM1 16-bit ARR");
_Static_assert(
    (MOTOR_START_PPS <= MOTOR_PAN_CRUISE_PPS) &&
        (MOTOR_START_PPS <= MOTOR_TILT_CRUISE_PPS),
    "MOTOR_START_PPS must not exceed the cruise speed of either axis");

/* --- 엔코더 영점 상수 -----------------------------------------------------
 *  리밋스위치를 쓰지 않으므로 양축 모두 이 상수에 의존한다.
 *  조립 후 1회 실측해 채워야 한다:
 *    ① 축을 기준 자세로 맞춘다 (팬=기준 방위 / 틸트=바닥 nadir)
 *    ② CMD_HOME 을 걸고 CMD_HOMED 가 올려주는 엔코더 raw 를 읽는다
 *       (RPi 에서 `turret_test state` 가 raw 와 도 환산을 같이 찍는다)
 *    ③ 그 값을 아래에 넣는다
 *  상수가 틀려도 CMD_HOMED 가 raw 를 함께 올리므로 이미 찍은 스캔의 각도를
 *  오프라인에서 재계산할 수 있다(재스캔 불필요). */
#define MOTOR_PAN_ZERO_OFFSET_DEG 225.31f
#define MOTOR_TILT_ZERO_OFFSET_DEG 123.40f

/* 부팅 직후 I2C/센서가 아직 안정화되지 않아 첫 판독이 NACK 나는 경우 대비 */
#define MOTOR_ENC_MAX_RETRY 5u
#define MOTOR_ENC_RETRY_DELAY_MS 10u

/* --- ddeg <-> pulse 변환 --------------------------------------------------
 *  1 pulse = 1.125 ddeg 이므로  pulse = ddeg * 8/9,  ddeg = pulse * 9/8.
 *
 *  주의: 정수 연산이라 나머지가 버려진다. **절대각 변환에만 쓰고 증분에 쓰지 말
 * 것.** 예) 1도(=10 ddeg) 스텝을 미리 펄스로 굳히면 9펄스(1.0125도)로 고정돼
 *      180줄 누적 시 방위 커버리지가 어긋난다. 목표는 매번
 *        target = motor_ddeg_to_pulse(start_ddeg + line * step_ddeg)
 *      처럼 절대각에서 계산해야 절삭이 누적되지 않는다.
 *
 *  틸트는 부호가 있으므로 0 방향 절삭이 아니라 부호를 보존해 반올림한다. */
static inline int32_t motor_ddeg_to_pulse(int32_t ddeg) {
  return (ddeg >= 0) ? (((ddeg * 8) + 4) / 9) : -((((-ddeg) * 8) + 4) / 9);
}

static inline int32_t motor_pulse_to_ddeg(int32_t pulse) {
  return (pulse >= 0) ? (((pulse * 9) + 4) / 8) : -((((-pulse) * 9) + 4) / 8);
}

/* --- 수명주기 -------------------------------------------------------------*/
void motor_init(void);   /* GPIO 초기 상태. 드라이버는 비활성으로 시작 */
void motor_enable(void); /* 양축 드라이버 전류 인가                   */
void motor_disarm(void); /* 즉시 정지 + 전류 차단 (CMD_DISARM)        */

/* --- 위치 제어 (메인루프에서 호출) ----------------------------------------*/
void motor_set_target(motor_axis_t ax, int32_t pulse);
int32_t motor_get_target(motor_axis_t ax);
bool motor_is_idle(motor_axis_t ax); /* 현재 == 목표 */

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
/* 판독 1회. 실패하면 **버스를 되살리고 최대 MOTOR_ENC_MAX_RETRY 번** 다시
 * 시도한다(구현부 주석 참조). 실패 시 최악 대기는
 *   (재시도-1) x (I2C_TIMEOUT 10ms x 2 + MOTOR_ENC_RETRY_DELAY_MS 10ms)
 * 약 120ms 다. 홈에서 축당 한 번 부르므로 무시할 수준이다. */
HAL_StatusTypeDef motor_read_encoder(motor_axis_t ax, Encoder_t *out);

/* 지금까지 누적된 재시도 횟수(축별). 0 이 아니면 I2C 가 흔들리고 있다는 뜻.
 *
 * 주의: 재시도는 문제를 **가린다**. 배선이 서서히 나빠져도 몇 번 만에 성공하면
 *   아무도 모른 채 지나간다. 이 값을 주기적으로 보거나 산출물에 실어서,
 *   "되긴 되는데 점점 나빠지는" 상태를 알아챌 수 있게 할 것. */
uint32_t motor_encoder_retry_count(motor_axis_t ax);

/* 축에 물린 I2C 핸들 (Pan=&hi2c3 / Tilt=&hi2c1).
 * 판독은 motor_read_encoder 를 쓰고, 이건 **버스 자체**를 다뤄야 할 때만
 * 쓴다(주소 스캔, SDA 물림 9클럭 복구 등 브링업 진단). 핸들 extern 이 파일
 * 곳곳에 흩어지는 걸 막으려고 여기 한 곳에서만 노출한다. */
I2C_HandleTypeDef *motor_axis_i2c(motor_axis_t ax);

/* 엔코더 실측각(도) → 펄스. 영점 상수를 적용한다. */
int32_t motor_encoder_deg_to_pulse(motor_axis_t ax, float deg);

/* 판독 후 펄스로 환산. 성공 시 HAL_OK 와 *out_pulse 반환.
 * 재시도·버스 복구는 motor_read_encoder 가 하므로 여기서는 환산만 한다.
 * 그쪽이 HAL_Delay 를 쓰므로 메인루프에서만 호출할 것. */
HAL_StatusTypeDef motor_read_encoder_pulse(motor_axis_t ax, int32_t *out_pulse);

/* --- 타이머 ISR 진입점 ----------------------------------------------------
 *  TIM1(Pan) / TIM2(Tilt) Base 인터럽트에서 호출. 호출 1회당 최대 1펄스.
 *  분기·printf·I2C·Delay 없음 — 순수 펄스 발생과 카운터 증감만. */
void motor_pan_isr(void);
void motor_tilt_isr(void);

#endif /* MOTOR_H */
