/* ============================================================================
 *  motor.c  --  Pan/Tilt 2축 스텝모터 축 드라이버 구현
 * ----------------------------------------------------------------------------
 *  담당: 강유근 (원 구현) / 이현우 (계층 분리 리팩터링)
 *
 *  스캔 시퀀스는 여기 없다 (App/scan). 계약과 배경은 motor.h 상단 참조.
 * ==========================================================================*/
#include "motor.h"
#include <stddef.h>

/* 엔코더 I2C 핸들 (main.c 생성) */
extern I2C_HandleTypeDef hi2c3;   /* Pan  : PA8/PC9  */
extern I2C_HandleTypeDef hi2c1;   /* Tilt : PB8/PB9  */

/* ---------------------------------------------------------------------------
 *  축 배선 테이블 (컴파일타임 상수)
 *
 *  dir_forward = "현재 < 목표" 일 때 DIR 에 내보낼 레벨.
 *  주의: 실측으로 정한 값이다. 원 구현 주석 기준:
 *     홈 수행 시 소프트웨어는 0 으로 수렴했다고 판단했는데 엔코더 실측은
 *     반대로 갔다 — 즉 초기 배치가 뒤집혀 있어 극성을 반전한 결과다.
 *     배선을 바꾸지 않는 한 이 값을 임의로 뒤집지 말 것.
 * ------------------------------------------------------------------------- */
struct axis_cfg {
    GPIO_TypeDef *step_port;  uint16_t step_pin;
    GPIO_TypeDef *dir_port;   uint16_t dir_pin;
    GPIO_TypeDef *en_port;    uint16_t en_pin;
    /* 이 축의 스텝 타이머. 핸들(TIM_HandleTypeDef)이 아니라 레지스터 블록을
     * 직접 든다 — ISR 에서 매 펄스 ARR 을 갈아끼우는 것이 전부라 HAL 상태
     * 기계를 거칠 이유가 없고, main.c 의 핸들에 의존하지 않아도 된다. */
    TIM_TypeDef  *tim;
    /* 램프 파라미터 (motor.h "가감속 램프" 참조) */
    uint32_t      cruise_pps;
    uint32_t      accel_pps2;
    GPIO_PinState dir_forward;
    /* 핵심: 엔코더 방향. "펄스 카운트가 증가할 때 엔코더 각도가 증가하면 +1,
     *   감소하면 -1" 이다.
     *
     *   주의: dir_forward 와 **반드시 짝으로** 다뤄야 한다. dir_forward 를
     *     뒤집으면 "카운트 증가" 가 반대 물리 방향을 뜻하게 되는데, 엔코더는
     *     그대로이므로 이 부호도 같이 뒤집어야 한다. 하나만 바꾸면
     *     motor_encoder_deg_to_pulse() 가 반대 부호를 내놓고, 홈 폐루프가
     *     보정할수록 멀어져 ERR_STALL 로 끝난다.
     *
     *   실측법: ENCODER_BENCH_TEST=1 로 두고 축을 손으로 "카운트가 증가하는
     *     물리 방향" 으로 돌려 raw 가 늘면 +1, 줄면 -1. */
    int8_t        enc_sign;
    float         zero_offset_deg;
};

static const struct axis_cfg s_cfg[MOTOR_AXIS_COUNT] = {
    [MOTOR_AXIS_PAN] = {
        .step_port = PAN_STEP_GPIO_Port, .step_pin = PAN_STEP_Pin,
        .dir_port  = PAN_DIR_GPIO_Port,  .dir_pin  = PAN_DIR_Pin,
        .en_port   = PAN_EN_GPIO_Port,   .en_pin   = PAN_EN_Pin,
        .tim        = TIM1,
        .cruise_pps = MOTOR_PAN_CRUISE_PPS,
        .accel_pps2 = MOTOR_PAN_ACCEL_PPS2,
        /* 주의: 2026-08-10 반전. 배선 정리 후 케이블 여유가 **반대 방향**으로
         *   잡혀서, 종전 방향으로 돌리면 선이 당겨져 뽑힌다.
         *   dir_forward 를 뒤집었으므로 enc_sign 도 같이 뒤집는다(위 주석). */
        .dir_forward = GPIO_PIN_SET,
        .enc_sign    = -1,
        .zero_offset_deg = MOTOR_PAN_ZERO_OFFSET_DEG,
    },
    [MOTOR_AXIS_TILT] = {
        .step_port = TILT_STEP_GPIO_Port, .step_pin = TILT_STEP_Pin,
        .dir_port  = TILT_DIR_GPIO_Port,  .dir_pin  = TILT_DIR_Pin,
        .en_port   = TILT_EN_GPIO_Port,   .en_pin   = TILT_EN_Pin,
        .tim        = TIM2,
        .cruise_pps = MOTOR_TILT_CRUISE_PPS,
        .accel_pps2 = MOTOR_TILT_ACCEL_PPS2,
        .dir_forward = GPIO_PIN_SET,
        .enc_sign    = +1,   /* 영점 실측(MOTOR_TILT_ZERO_OFFSET_DEG)이 이 부호로 검증됨 */
        .zero_offset_deg = MOTOR_TILT_ZERO_OFFSET_DEG,
    },
};

/* ---------------------------------------------------------------------------
 *  축 런타임 상태
 *
 *  pulse  : ISR 이 쓰고 메인루프가 읽는다
 *  target : 메인루프가 쓰고 ISR 이 읽는다
 *  둘 다 32비트 정렬 워드라 Cortex-M4 에서 읽기/쓰기가 원자적이다. 서로
 *  반대 방향의 단일 생산자-단일 소비자라 별도 락이 필요 없다.
 *  (양쪽이 같은 변수를 쓰는 경우는 motor_sync_pulse 하나뿐이고, 거기서만
 *   인터럽트를 잠깐 막는다)
 * ------------------------------------------------------------------------- */
struct axis_rt {
    volatile int32_t pulse;
    volatile int32_t target;

    /* --- 램프 상태 -------------------------------------------------------
     * ISR 만 읽고 쓴다(motor_init 제외 — 타이머가 아직 안 돌 때다). 그래서
     * volatile 도 락도 필요 없다. 메인루프에 노출하지 않는 이유는 이 값들이
     * 펄스마다 바뀌어서 밖에서 본 순간 이미 낡은 값이기 때문이다. */
    uint32_t v_q8;          /* 현재 속도. pps * 256 (아래 고정소수점 주석)  */
    uint32_t c_us;          /* 현재 펄스 간격(us) = ARR + 1                 */
    bool     forward_prev;  /* 직전 펄스의 방향 — 반전 감지용               */
};

static struct axis_rt s_rt[MOTOR_AXIS_COUNT];

/* ---------------------------------------------------------------------------
 *  램프 산수 — 전부 32비트 정수다
 *
 *  ISR 안에서 도는 계산이라 부동소수점을 쓰지 않는다. 속도를 pps 정수로 들면
 *  한 펄스당 속도 증가분(가속도 x 펄스간격)이 1pps 미만일 때 통째로 절삭돼
 *  램프가 그 자리에 멈춰버리므로, 하위 8비트를 소수부로 쓰는 고정소수점
 *  (q8 = pps * 256) 으로 든다.
 *
 *    간격:  c_us   = 10^6 * 256 / v_q8        (= MOTOR_C_NUM / v_q8)
 *    증분:  dv_q8  = accel * c_us * 256 / 10^6 = accel * c_us / MOTOR_DV_DIV
 *
 *  MOTOR_C_NUM 은 2.56e8 이라 uint32 에 들어간다(4.29e9 한도).
 * ------------------------------------------------------------------------- */
#define MOTOR_V_Q8(pps)   ((uint32_t)(pps) * 256u)
#define MOTOR_C_NUM       (MOTOR_TIM_TICK_HZ * 256u)
#define MOTOR_DV_DIV      (MOTOR_TIM_TICK_HZ / 256u)
#define MOTOR_V_START_Q8  MOTOR_V_Q8(MOTOR_START_PPS)

/* 속도를 바꾸고 그 결과를 타이머에 싣는다.
 *
 * 주의: ARR 프리로드는 비활성(CubeMX 설정)이라 쓰는 즉시 이번 주기에 반영된다.
 *   이 함수는 ISR 진입 직후에만 불리므로 CNT 는 수 us 이고, 새 ARR 은 최소
 *   c_min(순항 간격 = 틸트 2500us) 이라 언제나 CNT 보다 크다. 만약 CNT 보다
 *   작은 값을 쓰면 카운터가 0xFFFF 까지 돌아 그 주기만 통째로 길어진다. */
static void axis_set_speed(motor_axis_t ax, uint32_t v_q8)
{
    struct axis_rt *rt = &s_rt[ax];

    if (v_q8 != rt->v_q8) {
        rt->v_q8 = v_q8;
        rt->c_us = MOTOR_C_NUM / v_q8;
        s_cfg[ax].tim->ARR = rt->c_us - 1u;
    }
}

/* 지금 속도에서 시작 속도까지 감속하는 데 필요한 펄스 수.
 *   n = (v^2 - v_start^2) / (2*accel)
 *
 * 올림한다 — 내림하면 감속을 한 펄스 늦게 시작해 시작 속도보다 빠른 채로
 * 목표에 닿는다. 한 펄스 일찍 시작하는 쪽이 안전하다(그만큼 순항이 짧아질
 * 뿐이다). v 는 pps 정수라 v^2 은 최대 수백만으로 uint32 안에서 논다. */
static uint32_t axis_decel_pulses(uint32_t v_pps, uint32_t accel_pps2)
{
    uint32_t n = 0u;

    if (v_pps > MOTOR_START_PPS) {
        const uint32_t dv2 = (v_pps * v_pps)
                           - (MOTOR_START_PPS * MOTOR_START_PPS);
        const uint32_t den = 2u * accel_pps2;
        n = (dv2 + (den - 1u)) / den;
    }
    return n;
}

/* 방금 펄스를 낸 뒤, 다음 펄스까지의 간격을 정한다.
 * remaining = 이 펄스를 낸 뒤에도 남은 펄스 수 (0 이면 도착). */
static void axis_ramp(motor_axis_t ax, int32_t remaining)
{
    const struct axis_cfg *cfg = &s_cfg[ax];
    const struct axis_rt *rt = &s_rt[ax];   /* 쓰기는 axis_set_speed 만 한다 */

    if (remaining <= 0) {
        /* 도착했거나 애초에 정지 중이다. 시작 속도로 되감아 둔다 — 안 하면
         * 다음 이동이 직전 이동의 순항 속도로 급출발한다(램프를 넣은 이유가
         * 통째로 사라진다). */
        axis_set_speed(ax, MOTOR_V_START_Q8);
    } else {
        const uint32_t cruise_q8 = MOTOR_V_Q8(cfg->cruise_pps);
        const uint32_t v_pps     = rt->v_q8 / 256u;
        uint32_t v_q8  = rt->v_q8;
        uint32_t dv_q8 = (cfg->accel_pps2 * rt->c_us) / MOTOR_DV_DIV;

        /* 가속도가 아주 작고 간격이 짧으면 절삭으로 0 이 된다. 그대로 두면
         * 램프가 얼어붙으므로 최소 1 (=1/256 pps) 은 움직이게 한다. */
        if (dv_q8 == 0u) {
            dv_q8 = 1u;
        }

        if ((uint32_t)remaining <= axis_decel_pulses(v_pps, cfg->accel_pps2)) {
            /* 감속 구간 — 남은 펄스로 시작 속도까지 못 내려오는 시점이다 */
            v_q8 = ((v_q8 - MOTOR_V_START_Q8) > dv_q8) ? (v_q8 - dv_q8)
                                                       : MOTOR_V_START_Q8;
        } else if (v_q8 < cruise_q8) {
            v_q8 += dv_q8;
            if (v_q8 > cruise_q8) {
                v_q8 = cruise_q8;
            }
        } else {
            /* 순항 중 — 손댈 것이 없다(ARR 도 다시 안 쓴다) */
        }
        axis_set_speed(ax, v_q8);
    }
}

/* ---------------------------------------------------------------------------
 *  수명주기
 * ------------------------------------------------------------------------- */
/* 스텝 타이머의 시간축을 이 계층이 직접 잡는다.
 *
 * 핵심: CubeMX 가 MX_TIMx_Init 에 넣어 둔 Prescaler/Period 를 **쓰지 않는다.**
 *   램프 계산이 "1틱 = 1us" 를 전제로 하는데, .ioc 를 다시 생성하면서 그 값이
 *   바뀌면 축 속도가 통째로 배수만큼 어긋난다. 생성 코드가 어떤 값을 들고
 *   있든 결과가 같도록 여기서 확정한다.
 *
 *   TIM1(APB2)/TIM2(APB1) 모두 타이머 클럭은 SystemCoreClock 과 같다 —
 *   APB 프리스케일러가 1 이면 그대로고, 2 이상이면 타이머 쪽에서 2배로
 *   보상되어 84MHz 로 같아진다(현 클럭트리: APB1=/2, APB2=/1).
 *
 * 주의: HAL_TIM_Base_Start_IT() 보다 먼저 불려야 한다(main.c 는 그렇게 부른다). */
static void axis_timer_init(motor_axis_t ax)
{
    TIM_TypeDef *tim = s_cfg[ax].tim;

    tim->PSC = (SystemCoreClock / MOTOR_TIM_TICK_HZ) - 1u;

    /* 0 으로 두면 axis_set_speed 가 "값이 같다" 로 보고 건너뛰지 않는다 */
    s_rt[ax].v_q8 = 0u;
    axis_set_speed(ax, MOTOR_V_START_Q8);

    /* PSC 는 섀도 레지스터라 업데이트 이벤트가 있어야 실려온다. 강제로 한 번
     * 일으키고, 그때 선 업데이트 플래그는 지운다 — 안 지우면 타이머를 시작하는
     * 순간 밀린 인터럽트가 한 번 튄다. */
    /* 플래그는 0 을 써서 지운다(1 은 무시된다). 합성식(비트마스크 매크로)을
     * 바로 부정하지 않고 변수로 먼저 받는다 — MISRA 10.8. */
    const uint32_t uif = TIM_SR_UIF;

    tim->EGR = TIM_EGR_UG;
    tim->SR  = ~uif;
}

void motor_init(void)
{
    for (motor_axis_t ax = 0; ax < MOTOR_AXIS_COUNT; ax++) {
        s_rt[ax].pulse        = 0;
        s_rt[ax].target       = 0;
        s_rt[ax].forward_prev = false;
        HAL_GPIO_WritePin(s_cfg[ax].step_port, s_cfg[ax].step_pin, GPIO_PIN_RESET);
        axis_timer_init(ax);
    }
    motor_disarm();   /* 부팅 시 전류 차단 상태로 시작 */
}

void motor_enable(void)
{
    /* EN 은 active-low */
    HAL_GPIO_WritePin(s_cfg[MOTOR_AXIS_PAN].en_port,
                      s_cfg[MOTOR_AXIS_PAN].en_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(s_cfg[MOTOR_AXIS_TILT].en_port,
                      s_cfg[MOTOR_AXIS_TILT].en_pin, GPIO_PIN_RESET);
}

void motor_disarm(void)
{
    /* 목표를 현재 위치로 당겨 ISR 이 더 이상 펄스를 내지 않게 한 뒤 전류 차단.
     * 순서가 중요하다 — 전류부터 끊고 목표를 두면, 다시 enable 되는 순간
     * 축이 옛 목표를 향해 갑자기 달려나간다. */
    for (motor_axis_t ax = 0; ax < MOTOR_AXIS_COUNT; ax++) {
        s_rt[ax].target = s_rt[ax].pulse;
    }
    HAL_GPIO_WritePin(s_cfg[MOTOR_AXIS_PAN].en_port,
                      s_cfg[MOTOR_AXIS_PAN].en_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(s_cfg[MOTOR_AXIS_TILT].en_port,
                      s_cfg[MOTOR_AXIS_TILT].en_pin, GPIO_PIN_SET);
}

/* ---------------------------------------------------------------------------
 *  위치 제어 / 관측
 * ------------------------------------------------------------------------- */
void motor_set_target(motor_axis_t ax, int32_t pulse)
{
    if (ax < MOTOR_AXIS_COUNT) {
        s_rt[ax].target = pulse;
    }
}

int32_t motor_get_target(motor_axis_t ax)
{
    return (ax < MOTOR_AXIS_COUNT) ? s_rt[ax].target : 0;
}

int32_t motor_get_pulse(motor_axis_t ax)
{
    return (ax < MOTOR_AXIS_COUNT) ? s_rt[ax].pulse : 0;
}

int16_t motor_get_ddeg(motor_axis_t ax)
{
    return (int16_t)motor_pulse_to_ddeg(motor_get_pulse(ax));
}

bool motor_is_idle(motor_axis_t ax)
{
    return (ax >= MOTOR_AXIS_COUNT) || (s_rt[ax].pulse == s_rt[ax].target);
}

void motor_sync_pulse(motor_axis_t ax, int32_t pulse)
{
    if (ax < MOTOR_AXIS_COUNT) {
        /* pulse 는 평소 ISR 이 소유하는 변수다. 여기서만 메인루프가 덮어쓰므로
         * 두 워드를 한 덩어리로 바꾸도록 인터럽트를 잠깐 막는다. 수 사이클이다. */
        __disable_irq();
        s_rt[ax].pulse  = pulse;
        s_rt[ax].target = pulse;
        __enable_irq();
    }
}

/* ---------------------------------------------------------------------------
 *  엔코더 (블로킹 — 메인루프 전용)
 * ------------------------------------------------------------------------- */
/* 축 → I2C 핸들. 공개해 두는 이유는 버스 진단(App/hallEffectSensor 의 벤치)이
 * 판독이 아니라 버스 자체(주소 스캔·물림 복구)를 다뤄야 해서다. 여기서만
 * 핸들을 알게 해 두면 extern 선언이 파일마다 흩어지지 않는다(MISRA 8.5). */
I2C_HandleTypeDef *motor_axis_i2c(motor_axis_t ax)
{
    return (ax == MOTOR_AXIS_PAN) ? &hi2c3 : &hi2c1;
}

/* 엔코더 판독 1회. 실패하면 페리페럴을 되살리고 다시 시도한다.
 *
 * 핵심: 재시도가 **여기** 있는 이유 (2026-08-12):
 *   예전에는 재시도가 motor_read_encoder_pulse 에만 있었고, 정작 홈
 *   (scan_do_homing)은 재시도 없는 이 함수를 썼다. 그래서 I2C 가 **한 번**
 *   튀면 그대로 ERR 로 확정돼 스캔이 시작조차 못 했다. 두 경로가 같은
 *   신뢰도를 갖도록 재시도를 아래층으로 내렸다.
 *
 * 핵심: 재시도 사이에 Encoder_BusRecover 를 부르는 이유:
 *   HAL 이 타임아웃/NACK 뒤 상태를 래치해서, 되살리지 않으면 이어지는
 *   시도가 전부 즉시 HAL_BUSY 로 튕긴다 — 재시도가 무의미해진다.
 *   실기 증상: DISARM 뒤 홈을 걸면 안 읽히고 STM32 를 리셋해야 돌아왔다.
 *   NRST(전원 유지) 만으로 복구된 것이 "슬레이브가 아니라 STM32 쪽 상태"
 *   라는 근거다.
 *
 * 주의: 몇 번 만에 성공했는지 세어 둔다(motor_encoder_retry_count). 재시도가
 *   조용히 성공하면 배선이 나빠지는 것을 아무도 모른 채 지나가기 때문이다 —
 *   복구가 문제를 가리는 것을 막으려면 횟수가 보여야 한다. */
static uint32_t s_enc_retry_total[MOTOR_AXIS_COUNT];

HAL_StatusTypeDef motor_read_encoder(motor_axis_t ax, Encoder_t *out)
{
    HAL_StatusTypeDef st = HAL_ERROR;

    if ((ax < MOTOR_AXIS_COUNT) && (out != NULL)) {
        I2C_HandleTypeDef *hi2c = motor_axis_i2c(ax);
        uint32_t attempt;

        for (attempt = 0u;
             (attempt < MOTOR_ENC_MAX_RETRY) && (st != HAL_OK);
             attempt++) {
            if (attempt > 0u) {
                /* 직전 시도가 실패했다 — 페리페럴을 되살리고 잠깐 쉰다.
                 * 되살리지 않으면 아래 Encoder_Read 가 즉시 HAL_BUSY 다. */
                (void)Encoder_BusRecover(hi2c);
                HAL_Delay(MOTOR_ENC_RETRY_DELAY_MS);
                s_enc_retry_total[ax]++;
            }
            st = Encoder_Read(hi2c, out);
        }
    }
    return st;
}

uint32_t motor_encoder_retry_count(motor_axis_t ax)
{
    return (ax < MOTOR_AXIS_COUNT) ? s_enc_retry_total[ax] : 0u;
}

int32_t motor_encoder_deg_to_pulse(motor_axis_t ax, float deg)
{
    const int8_t sign = (ax < MOTOR_AXIS_COUNT) ? s_cfg[ax].enc_sign : (int8_t)1;
    float rel = deg - ((ax < MOTOR_AXIS_COUNT)
                       ? s_cfg[ax].zero_offset_deg : 0.0f);

    /* 주의: 엔코더는 0~360 순환이라 뺄셈만 하면 한 바퀴 경계에서 터진다.
     *   예) 틸트 영점이 313.5도일 때 기구각 +90도의 실측은 403.5 가 아니라
     *       43.5 로 읽힌다. 그대로 빼면 -270 이 나와 ERR_STALL 이나 엉뚱한
     *       재영점으로 이어진다. -180..+180 으로 접어 가장 가까운 해를 쓴다.
     *   양축 가동범위가 180도 이내라 이 접기로 모호함이 없다. */
    if (rel > 180.0f) {
        rel -= 360.0f;
    } else if (rel < -180.0f) {
        rel += 360.0f;
    } else {
        /* 범위 안 */
    }

    /* 엔코더 방향 반영 (위 enc_sign 주석 참조) */
    rel *= (float)sign;

    /* 주의: 버림이 아니라 반올림. (int32_t) 캐스트는 0 방향 절삭이라 최대 1펄스
     *   (0.1125도) 오차에 **한쪽으로 치우친 편향**까지 생긴다. 반올림하면
     *   최대 0.5펄스(0.056도)로 절반이 되고 편향도 사라진다 — 홈 정확도가
     *   그만큼 좋아진다. ddeg<->pulse 변환은 이미 반올림을 쓰고 있었는데
     *   (motor.h §17-7) 엔코더 변환만 빠져 있었다. */
    const float q = rel / MOTOR_DEG_PER_PULSE;
    return (int32_t)((q >= 0.0f) ? (q + 0.5f) : (q - 0.5f));
}

HAL_StatusTypeDef motor_read_encoder_pulse(motor_axis_t ax, int32_t *out_pulse)
{
    Encoder_t enc;
    HAL_StatusTypeDef st = HAL_ERROR;

    if ((ax < MOTOR_AXIS_COUNT) && (out_pulse != NULL)) {
        /* 재시도·버스 복구는 motor_read_encoder 가 한다(그쪽 주석 참조).
         * 예전에는 이 루프가 재시도를 들고 있었는데, 그러면 같은 함수를
         * 쓰는 홈 경로는 재시도 없이 한 방에 실패했다.
         *
         * 주의: 실패를 조용히 넘기면 호출자가 위치를 0 으로 두게 되고, 목표(0)와
         *   우연히 일치해 "축이 실제로는 안 움직였는데 홈 완료" 가 된다.
         *   그래서 실패는 반드시 상태로 돌려준다. */
        st = motor_read_encoder(ax, &enc);
        if (st == HAL_OK) {
            *out_pulse = motor_encoder_deg_to_pulse(ax, enc.degree);
        }
    }
    return st;
}

/* ---------------------------------------------------------------------------
 *  타이머 ISR — 호출 1회당 최대 1펄스
 *
 *  여기서 하지 않는 것: 분기하는 상태머신, printf, I2C, HAL_Delay.
 *  전부 App/scan 이 메인루프에서 한다.
 *
 *  펄스를 낸 뒤 램프가 다음 호출까지의 간격(ARR)을 정한다. 나눗셈 두 번
 *  (UDIV 는 M4 에서 2~12사이클) 이 전부라 STEP 펄스 폭 스핀보다 훨씬 짧다.
 * ------------------------------------------------------------------------- */
static inline void axis_step(motor_axis_t ax)
{
    const struct axis_cfg *cfg = &s_cfg[ax];
    struct axis_rt *rt = &s_rt[ax];

    const int32_t cur = rt->pulse;
    const int32_t tgt = rt->target;

    /* 도착했으면 펄스 없음 */
    if (cur != tgt) {
        const bool forward = (cur < tgt);
        int32_t next;
        int32_t remaining;

        /* 이동 중에 목표가 반대쪽으로 바뀌었다면 속도를 되감고 반전한다.
         * 순항 속도 그대로 방향만 뒤집으면 그 자리에서 탈조한다. 감속은
         * 로터가 못 따라올 수 없으므로(느려지는 건 언제나 안전하다) 이번
         * 펄스부터 바로 적용해도 된다. */
        if (forward != rt->forward_prev) {
            axis_set_speed(ax, MOTOR_V_START_Q8);
            rt->forward_prev = forward;
        }

        HAL_GPIO_WritePin(cfg->dir_port, cfg->dir_pin,
                          forward ? cfg->dir_forward
                                  : ((cfg->dir_forward == GPIO_PIN_SET) ? GPIO_PIN_RESET
                                                                        : GPIO_PIN_SET));

        /* DIR 셋업(650ns) 확보 후 STEP 상승 */
        for (volatile uint32_t i = 0u; i < MOTOR_DIR_SETUP_SPIN; i++) { }

        HAL_GPIO_WritePin(cfg->step_port, cfg->step_pin, GPIO_PIN_SET);
        for (volatile uint32_t i = 0u; i < MOTOR_STEP_PULSE_SPIN; i++) { }
        HAL_GPIO_WritePin(cfg->step_port, cfg->step_pin, GPIO_PIN_RESET);

        next = forward ? (cur + 1) : (cur - 1);
        rt->pulse = next;

        remaining = (tgt > next) ? (tgt - next) : (next - tgt);
        axis_ramp(ax, remaining);
    } else {
        /* 정지 중 — 다음 이동이 시작 속도로 출발하도록 유지한다.
         * (motor_sync_pulse 나 즉시 정지로 순항 중에 멈춰선 경우가 여기다) */
        axis_ramp(ax, 0);
    }
}

void motor_pan_isr(void)
{
    axis_step(MOTOR_AXIS_PAN);
}

void motor_tilt_isr(void)
{
    axis_step(MOTOR_AXIS_TILT);
}
