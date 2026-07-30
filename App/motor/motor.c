/* ============================================================================
 *  motor.c  --  스텝모터 2축 제어 (경로 테스트용 최소 스텁)
 *  담당: 강유근 (현재 이현우 테스트 스텁 — 폐루프/엔코더 미구현)
 * ==========================================================================*/
#include "motor.h"

static TIM_HandleTypeDef *s_pan;   /* 방위(Pan) STEP 타이머  (TIM1, CH2N) */
static TIM_HandleTypeDef *s_tilt;  /* 고각(Tilt) STEP 타이머 (TIM3, CH1)  */

/* ---------------------------------------------------------------------------
 *  채널 배정 (adts.ioc 가 최종 진실)
 *
 *  팬  = TIM1_CH2N @ PB14  → **상보 출력**. HAL_TIMEx_PWMN_* 전용.
 *  틸트 = TIM3_CH1  @ PA6   → 일반 출력. HAL_TIM_PWM_* 사용.
 *
 *  두 축의 시작/정지 API 가 다르므로 헬퍼를 축별로 분리한다.
 * ------------------------------------------------------------------------- */
#define PAN_TIM_CHANNEL    TIM_CHANNEL_2   /* CH2N (PB14) */
#define TILT_TIM_CHANNEL   TIM_CHANNEL_1   /* CH1  (PA6)  */

/* ---------------------------------------------------------------------------
 *  스윕 축 추상화 (motor.h 의 MOTOR_SWEEP_ON_TILT 로 선택)
 *
 *  아래 헬퍼들만 축을 알고, 나머지 코드(scan.c 포함)는 전부 그대로다.
 *  TIM1(APB2)과 TIM3(APB1)은 소속 버스가 달라 클럭 산출식도 분기한다.
 * ------------------------------------------------------------------------- */
#if MOTOR_SWEEP_ON_TILT
  #define SWEEP_CHANNEL      TILT_TIM_CHANNEL
  #define SWEEP_TIM_INSTANCE TIM3
  #define SWEEP_DIR_PORT     TILT_DIR_GPIO_Port
  #define SWEEP_DIR_PIN      TILT_DIR_Pin
  #define SWEEP_EN_PORT      TILT_EN_GPIO_Port
  #define SWEEP_EN_PIN       TILT_EN_Pin
#else
  #define SWEEP_CHANNEL      PAN_TIM_CHANNEL
  #define SWEEP_TIM_INSTANCE TIM1
  #define SWEEP_DIR_PORT     PAN_DIR_GPIO_Port
  #define SWEEP_DIR_PIN      PAN_DIR_Pin
  #define SWEEP_EN_PORT      PAN_EN_GPIO_Port
  #define SWEEP_EN_PIN       PAN_EN_Pin
#endif

static TIM_HandleTypeDef *sweep_tim(void)
{
#if MOTOR_SWEEP_ON_TILT
    return s_tilt;
#else
    return s_pan;
#endif
}

/* 스윕 축 PWM 시작/정지. 팬은 상보 출력(CH2N)이라 API 가 다르다. */
static void sweep_pwm_start(void)
{
#if MOTOR_SWEEP_ON_TILT
    (void)HAL_TIM_PWM_Start(s_tilt, SWEEP_CHANNEL);
#else
    (void)HAL_TIMEx_PWMN_Start(s_pan, SWEEP_CHANNEL);
#endif
}

static void sweep_pwm_stop(void)
{
#if MOTOR_SWEEP_ON_TILT
    (void)HAL_TIM_PWM_Stop(s_tilt, SWEEP_CHANNEL);
#else
    (void)HAL_TIMEx_PWMN_Stop(s_pan, SWEEP_CHANNEL);
#endif
}

/* ---------------------------------------------------------------------------
 *  스텝 카운터 (TIM 업데이트 인터럽트로 실제 펄스 수를 센다)
 *
 *  TIM1 ARR 롤오버 1회 = STEP 펄스 1개 = 마이크로스텝 1칸.
 *  1/16 마이크로스텝 기준 3200 스텝 = 1회전(3600 ddeg).
 *
 *  ⚠️ NVIC: TIM1_UP_TIM10_IRQn, Preemption=1.
 *    USART6(라이다)=0 보다 낮아 라이다 수신을 방해하지 않는다.
 *  ⚠️ 홈(리밋스위치) 미구현 — 현재는 sweep 시작 시점을 0 으로 본다.
 *     강유근이 리밋스위치 홈을 넣으면 motor_pan_reset_steps() 를 그 시점에 호출.
 * ------------------------------------------------------------------------- */
#define MOTOR_FULL_STEPS_PER_REV   200u    /* 17HS4401: 1.8도/full step */
#define MOTOR_MICROSTEP            16u     /* DRV8825 MS1/2/3 물리설정과 일치 필수 */
#define MOTOR_STEPS_PER_REV        (MOTOR_FULL_STEPS_PER_REV * MOTOR_MICROSTEP)

/* 누적 스텝. **부호 있음** — CCW(되감기) 회전 시 감소한다.
 * 홈(=motor_pan_reset_steps 호출 시점)이 0. */
static volatile int32_t s_pan_steps = 0;
static volatile int32_t s_pan_dir   = MOTOR_DIR_CW;  /* ISR 이 더할 부호 */

/* 축 공통: 방향 지정 + 드라이버 활성 + 듀티 보장 (STEP 시작 직전까지) */
static void axis_arm(TIM_HandleTypeDef *tim, uint32_t ch,
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
}

void motor_init(TIM_HandleTypeDef *pan_step, TIM_HandleTypeDef *tilt_step)
{
    s_pan  = pan_step;
    s_tilt = tilt_step;
    motor_pan_set_pps(MOTOR_PAN_PPS_DEFAULT);   /* .ioc ARR 값을 덮어씀 */
    motor_disarm();                     /* 부팅 시 안전(비활성) */
}

void motor_set_target(int16_t theta_ddeg, int16_t phi_ddeg)
{
    /* [테스트] 부호로 방향만 정해 회전. 정밀 위치는 강유근 폐루프. */
    axis_arm(s_pan,  PAN_TIM_CHANNEL,
             PAN_DIR_GPIO_Port,  PAN_DIR_Pin,
             PAN_EN_GPIO_Port,   PAN_EN_Pin,   theta_ddeg);
    axis_arm(s_tilt, TILT_TIM_CHANNEL,
             TILT_DIR_GPIO_Port, TILT_DIR_Pin,
             TILT_EN_GPIO_Port,  TILT_EN_Pin,  phi_ddeg);

    /* 팬만 상보 출력(CH2N) — 시작 API 가 다르다. */
    (void)HAL_TIMEx_PWMN_Start(s_pan,  PAN_TIM_CHANNEL);
    (void)HAL_TIM_PWM_Start   (s_tilt, TILT_TIM_CHANNEL);
}

void motor_disarm(void)
{
    (void)HAL_TIMEx_PWMN_Stop(s_pan,  PAN_TIM_CHANNEL);
    (void)HAL_TIM_PWM_Stop   (s_tilt, TILT_TIM_CHANNEL);
    /* EN active-low → HIGH = 비활성 */
    HAL_GPIO_WritePin(PAN_EN_GPIO_Port,  PAN_EN_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(TILT_EN_GPIO_Port, TILT_EN_Pin, GPIO_PIN_SET);
}

/* ===========================================================================
 *  스캔용 API (App/scan)
 * ========================================================================= */

void motor_pan_sweep_start(void)
{
    motor_pan_sweep_start_dir(MOTOR_DIR_CW);
}

void motor_pan_sweep_start_dir(int8_t dir)
{
    TIM_HandleTypeDef *tim = sweep_tim();

    /* ISR 이 더할 부호를 먼저 정한다(펄스가 나가기 전에). */
    s_pan_dir = (dir < 0) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;

    HAL_GPIO_WritePin(SWEEP_DIR_PORT, SWEEP_DIR_PIN,
                      (s_pan_dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    /* 드라이버 활성 (EN active-low → LOW) */
    HAL_GPIO_WritePin(SWEEP_EN_PORT,  SWEEP_EN_PIN,  GPIO_PIN_RESET);
    /* 듀티 50% 보장 후 STEP 펄스 시작 */
    __HAL_TIM_SET_COMPARE(tim, SWEEP_CHANNEL,
                          __HAL_TIM_GET_AUTORELOAD(tim) / 2u);
    sweep_pwm_start();
    /* 스텝 카운트용 업데이트 인터럽트 활성 (ARR 롤오버 = STEP 1개) */
    __HAL_TIM_ENABLE_IT(tim, TIM_IT_UPDATE);

    /* TODO(강유근): 가감속 램프 — 현재는 즉시 등속이라 관성 큰 기구에서
     *   탈조 위험. 2축 스캔에서는 줄 끝 방향전환 램프가 필수. */
}

void motor_pan_stop(void)
{
    __HAL_TIM_DISABLE_IT(sweep_tim(), TIM_IT_UPDATE);
    sweep_pwm_stop();
    /* EN 은 유지: 홀딩 토크로 위치 보존. 완전 차단은 motor_disarm(). */
}

void motor_pan_reset_steps(void)
{
    s_pan_steps = 0;
}

int32_t motor_get_pan_steps(void)
{
    return s_pan_steps;
}

/* 스윕 타이머의 1틱 주파수(Hz) = TIMclk / (PSC+1).
 *
 * ⚠️ TIM1=APB2, TIM3=APB1 로 소속 버스가 달라 클럭 원천이 다르다.
 *   버스 프리스케일러가 1 이 아니면 타이머 클럭은 PCLK 의 2배가 된다
 *   (RM0368 클럭 트리). 분주 여부는 필드 최상위 비트로 판정한다. */
static uint32_t sweep_tick_hz(void)
{
    uint32_t tim_clk;

#if MOTOR_SWEEP_ON_TILT
    tim_clk = HAL_RCC_GetPCLK1Freq();                  /* TIM3 = APB1 */
    if ((RCC->CFGR & RCC_CFGR_PPRE1_2) != 0u) {
        tim_clk *= 2u;
    }
#else
    tim_clk = HAL_RCC_GetPCLK2Freq();                  /* TIM1 = APB2 */
    if ((RCC->CFGR & RCC_CFGR_PPRE2_2) != 0u) {
        tim_clk *= 2u;
    }
#endif

    const uint32_t psc = (uint32_t)sweep_tim()->Instance->PSC + 1u;
    return (psc != 0u) ? (tim_clk / psc) : 0u;
}

uint32_t motor_get_pan_pps(void)
{
    const uint32_t tick = sweep_tick_hz();
    const uint32_t arr  = (uint32_t)sweep_tim()->Instance->ARR + 1u;

    return (arr != 0u) ? (tick / arr) : 0u;
}

void motor_pan_set_pps(uint32_t pps)
{
    const uint32_t tick = sweep_tick_hz();

    if ((pps >= MOTOR_PAN_PPS_MIN) && (pps <= MOTOR_PAN_PPS_MAX) && (tick != 0u)) {
        uint32_t arr = tick / pps;
        if (arr > 0u) {
            arr -= 1u;
        }
        if (arr > 0xFFFFu) {          /* TIM1/TIM3 둘 다 16비트 카운터 */
            arr = 0xFFFFu;
        }
        __HAL_TIM_SET_AUTORELOAD(sweep_tim(), arr);
        /* ARR 을 바꾸면 듀티도 다시 잡아야 한다(CCR 은 절대값이라 안 따라옴) */
        __HAL_TIM_SET_COMPARE(sweep_tim(), SWEEP_CHANNEL, (arr + 1u) / 2u);
    }
}

int16_t motor_get_pan_ddeg(void)
{
    /* 부호 있는 누적 스텝 → 0~3599 절대각.
     * 되감기(CCW)로 음수가 될 수 있으므로 먼저 한 바퀴 안으로 접는다. */
    int32_t st = s_pan_steps % (int32_t)MOTOR_STEPS_PER_REV;
    if (st < 0) {
        st += (int32_t)MOTOR_STEPS_PER_REV;
    }
    /* 나눗셈을 마지막에 몰아 정수 오차 제거:
     *   3200 스텝 = 3600 ddeg → ddeg = steps * 3600 / 3200 */
    return (int16_t)(((uint32_t)st * 3600u) / MOTOR_STEPS_PER_REV);
}

int16_t motor_get_tilt_ddeg(void)
{
    /* TODO(강유근): MT6701 엔코더 실측값 반환.
     *   1축 임시 스캔에서는 틸트를 쓰지 않으므로 0 고정. */
    return 0;
}

/* cppcheck-suppress misra-c2012-8.7 ; 공개 API — 2축 확장 시 scan.c 가 호출 */
void motor_tilt_step(int16_t target_ddeg)
{
    (void)target_ddeg;
    /* TODO(강유근): 2축 기구 완성 후 MT6701 엔코더 폐루프로 구현.
     *   목표 절대각까지 틸트 이동 → 정착 대기.
     *   1축 임시 스캔에서는 호출되지 않는다(tilt_start == tilt_end). */
}

/* ---------------------------------------------------------------------------
 *  TIM 업데이트 인터럽트 콜백 (HAL weak 함수 오버라이드)
 *  ARR 롤오버 = STEP 펄스 1개. 짧게 유지할 것.
 * ------------------------------------------------------------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 스윕 축(MOTOR_SWEEP_ON_TILT 로 TIM1 또는 TIM3)의 롤오버만 센다.
     * 다른 축이 돌더라도 스캔 각도에는 영향을 주지 않는다. */
    if (htim->Instance == SWEEP_TIM_INSTANCE) {
        s_pan_steps += s_pan_dir;      /* CCW 되감기에서는 감소 */
    }
}
