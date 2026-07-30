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
 *  ⚠️ 실측으로 정한 값이다. 원 구현 주석 기준:
 *     홈 수행 시 소프트웨어는 0 으로 수렴했다고 판단했는데 엔코더 실측은
 *     반대로 갔다 — 즉 초기 배치가 뒤집혀 있어 극성을 반전한 결과다.
 *     배선을 바꾸지 않는 한 이 값을 임의로 뒤집지 말 것.
 * ------------------------------------------------------------------------- */
struct axis_cfg {
    GPIO_TypeDef *step_port;  uint16_t step_pin;
    GPIO_TypeDef *dir_port;   uint16_t dir_pin;
    GPIO_TypeDef *en_port;    uint16_t en_pin;
    GPIO_PinState dir_forward;
    float         zero_offset_deg;
};

static const struct axis_cfg s_cfg[MOTOR_AXIS_COUNT] = {
    [MOTOR_AXIS_PAN] = {
        .step_port = PAN_STEP_GPIO_Port, .step_pin = PAN_STEP_Pin,
        .dir_port  = PAN_DIR_GPIO_Port,  .dir_pin  = PAN_DIR_Pin,
        .en_port   = PAN_EN_GPIO_Port,   .en_pin   = PAN_EN_Pin,
        .dir_forward = GPIO_PIN_RESET,
        .zero_offset_deg = MOTOR_PAN_ZERO_OFFSET_DEG,
    },
    [MOTOR_AXIS_TILT] = {
        .step_port = TILT_STEP_GPIO_Port, .step_pin = TILT_STEP_Pin,
        .dir_port  = TILT_DIR_GPIO_Port,  .dir_pin  = TILT_DIR_Pin,
        .en_port   = TILT_EN_GPIO_Port,   .en_pin   = TILT_EN_Pin,
        .dir_forward = GPIO_PIN_SET,
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
};

static struct axis_rt s_rt[MOTOR_AXIS_COUNT];

/* ---------------------------------------------------------------------------
 *  수명주기
 * ------------------------------------------------------------------------- */
void motor_init(void)
{
    for (motor_axis_t ax = 0; ax < MOTOR_AXIS_COUNT; ax++) {
        s_rt[ax].pulse  = 0;
        s_rt[ax].target = 0;
        HAL_GPIO_WritePin(s_cfg[ax].step_port, s_cfg[ax].step_pin, GPIO_PIN_RESET);
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
static I2C_HandleTypeDef *axis_i2c(motor_axis_t ax)
{
    return (ax == MOTOR_AXIS_PAN) ? &hi2c3 : &hi2c1;
}

HAL_StatusTypeDef motor_read_encoder(motor_axis_t ax, Encoder_t *out)
{
    HAL_StatusTypeDef st = HAL_ERROR;

    if ((ax < MOTOR_AXIS_COUNT) && (out != NULL)) {
        st = Encoder_Read(axis_i2c(ax), out);
    }
    return st;
}

int32_t motor_encoder_deg_to_pulse(motor_axis_t ax, float deg)
{
    const float rel = deg - ((ax < MOTOR_AXIS_COUNT)
                             ? s_cfg[ax].zero_offset_deg : 0.0f);
    return (int32_t)(rel / MOTOR_DEG_PER_PULSE);
}

HAL_StatusTypeDef motor_read_encoder_pulse(motor_axis_t ax, int32_t *out_pulse)
{
    Encoder_t enc;
    HAL_StatusTypeDef st = HAL_ERROR;

    if ((ax < MOTOR_AXIS_COUNT) && (out_pulse != NULL)) {
        /* 부팅 직후 첫 판독이 NACK 나는 경우가 있어 재시도한다.
         * ⚠️ 실패를 조용히 넘기면 호출자가 위치를 0 으로 두게 되고, 목표(0)와
         *   우연히 일치해 "축이 실제로는 안 움직였는데 홈 완료" 가 된다.
         *   그래서 실패는 반드시 상태로 돌려준다. */
        for (uint32_t retry = 0u;
             (retry < MOTOR_ENC_MAX_RETRY) && (st != HAL_OK);
             retry++) {
            st = motor_read_encoder(ax, &enc);
            if (st == HAL_OK) {
                *out_pulse = motor_encoder_deg_to_pulse(ax, enc.degree);
            } else {
                HAL_Delay(MOTOR_ENC_RETRY_DELAY_MS);
            }
        }
    }
    return st;
}

/* ---------------------------------------------------------------------------
 *  타이머 ISR — 호출 1회당 최대 1펄스
 *
 *  여기서 하지 않는 것: 분기하는 상태머신, printf, I2C, HAL_Delay.
 *  전부 App/scan 이 메인루프에서 한다.
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

        HAL_GPIO_WritePin(cfg->dir_port, cfg->dir_pin,
                          forward ? cfg->dir_forward
                                  : ((cfg->dir_forward == GPIO_PIN_SET) ? GPIO_PIN_RESET
                                                                        : GPIO_PIN_SET));

        /* DIR 셋업(650ns) 확보 후 STEP 상승 */
        for (volatile uint32_t i = 0u; i < MOTOR_DIR_SETUP_SPIN; i++) { }

        HAL_GPIO_WritePin(cfg->step_port, cfg->step_pin, GPIO_PIN_SET);
        for (volatile uint32_t i = 0u; i < MOTOR_STEP_PULSE_SPIN; i++) { }
        HAL_GPIO_WritePin(cfg->step_port, cfg->step_pin, GPIO_PIN_RESET);

        rt->pulse = forward ? (cur + 1) : (cur - 1);
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
