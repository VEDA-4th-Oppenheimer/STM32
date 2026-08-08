/* ============================================================================
 *  scan.c  --  2축 스캔 시퀀서 구현
 * ----------------------------------------------------------------------------
 *  담당: 이현우
 *  계약과 배경은 scan.h 상단 참조.
 * ==========================================================================*/
#include "scan.h"
#include "motor.h"
#include "uart_rpi.h"
#include <stddef.h>

/* ---------------------------------------------------------------------------
 *  시퀀서 상태
 * ------------------------------------------------------------------------- */
static struct {
    scan_state_t state;
    bool         homed;

    /* 요청 (기구각, 0.1도) */
    int16_t  pan_start_ddeg;
    int16_t  pan_end_ddeg;
    int16_t  tilt_start_ddeg;
    int16_t  tilt_end_ddeg;
    uint16_t step_ddeg;

    /* 진행 */
    uint32_t line;          /* 현재 줄 (0 .. n_lines-1)            */
    uint32_t n_lines;
    bool     tilt_to_end;   /* 이번 줄이 start->end 방향인가       */

    /* 홈 자세 보정 재시도 횟수 (수렴 못 하면 ERR_STALL) */
    uint8_t  home_retry;

    /* 홈 판독 결과. CMD_HOMED 를 홈 자세 이동이 **끝난 뒤에** 보내야 해서
     * 상태를 넘어 들고 가야 한다 (아래 SC_HOME_POSE 주석 참조). */
    struct proto_homed home;

    /* 정착 대기 마감 시각(HAL tick).
     * 축이 움직이는 동안 계속 앞으로 밀리므로, 멈춘 뒤 "도착 시각 + 대기"가
     * 자연히 마감이 된다. 상태를 새로 만들지 않아도 되는 이유다. */
    uint32_t settle_until_ms;

    /* 라이다 각도 래치 */
    volatile int16_t latch_pan_ddeg;
    volatile int16_t latch_tilt_ddeg;
} s;

/* 마감 지났나.
 *
 * HAL_GetTick 은 49.7일에 한 번 랩어라운드하므로 `now >= deadline` 같은 대소
 * 비교를 쓰면 랩 순간에 판정이 뒤집힌다. 부호 없는 뺄셈은 랩을 넘어도 올바른
 * 경과량을 주므로, 그 차이를 부호 있는 형으로 보고 0 이상인지만 본다.
 *
 * 합성식을 바로 캐스트하지 않고 뺄셈 결과를 변수로 먼저 받는다 (MISRA 10.8). */
/* cppcheck-suppress knownConditionTrueFalse ; HAL_GetTick 을 모델링 못 해
 * 경과량을 상수로 접는 오탐. 실행 시 두 값 모두 변한다. */
static bool scan_settled(void)
{
    const uint32_t elapsed = HAL_GetTick() - s.settle_until_ms;
    return ((int32_t)elapsed >= 0);
}

/* ---------------------------------------------------------------------------
 *  보조
 * ------------------------------------------------------------------------- */
static void scan_report_err(uint8_t code)
{
    struct proto_err e;
    e.code = code;
    uart_rpi_send_frame((uint8_t)CMD_ERROR, &e, (uint8_t)sizeof(e));
}

/* 이번 줄의 팬 목표(펄스).
 *
 * ⚠️ 반드시 **절대각에서** 계산한다. 1도 스텝을 펄스로 굳혀 매 줄 더하면
 *   1 펄스가 1.125 ddeg 라 나머지가 누적된다(180줄에서 2도 이상 어긋남).
 *   여기서 start + line*step 을 매번 통째로 환산하면 오차가 안 쌓인다. */
static int32_t scan_pan_target_pulse(uint32_t line)
{
    const int32_t ddeg = (int32_t)s.pan_start_ddeg
                       + ((int32_t)line * (int32_t)s.step_ddeg);
    return motor_ddeg_to_pulse(ddeg);
}

/* 이번 줄 틸트 목표(펄스). serpentine 이라 줄마다 방향이 뒤집힌다. */
static int32_t scan_tilt_target_pulse(void)
{
    return motor_ddeg_to_pulse(s.tilt_to_end ? (int32_t)s.tilt_end_ddeg
                                             : (int32_t)s.tilt_start_ddeg);
}

static int32_t scan_abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* 엔코더 실측과 스텝카운트를 대조한다. 반환은 오차(ddeg, 부호),
 * 판독 실패면 *ok=false. 메인루프 전용 — 블로킹 I2C 를 탄다.
 *
 * ★ 홈 자세 검증에서만 쓴다. 스윕 중에는 호출하지 않는다 — 이유는 scan.h
 *   SCAN_HOME_TOL_DDEG 주석 참조. */
static int32_t scan_encoder_error_ddeg(motor_axis_t ax, bool *ok,
                                       int32_t *out_enc_pulse)
{
    int32_t err = 0;

#if SCAN_NO_ENCODER
    /* 브링업 모드: 판독을 아예 시도하지 않는다. 단순히 실패시키는 것과
     * 다르다 — 실패하면 motor_read_encoder_pulse 가 5회 x (I2C 타임아웃
     * 10ms + 대기 10ms) = 약 100ms 를 잡아먹는다. 호출 자체를 건너뛴다. */
    (void)ax;
    *ok = false;
    *out_enc_pulse = 0;
#else
    int32_t enc_pulse = 0;

    *ok = (motor_read_encoder_pulse(ax, &enc_pulse) == HAL_OK);
    if (*ok) {
        err = motor_pulse_to_ddeg(enc_pulse - motor_get_pulse(ax));
    }
    /* 판독한 펄스를 그대로 넘긴다. 보정할 때 다시 읽으면 그 사이 잡음으로
     * 값이 달라져, 방금 판정한 오차와 다른 값을 기준으로 삼게 된다. */
    *out_enc_pulse = enc_pulse;
#endif
    return err;
}

/* 축 하나를 대조하고, 허용 오차를 벗어났으면 **보정 이동을 건다**.
 *   반환 true  = 오차 안 (또는 판독 불가라 판정 보류)
 *        false = 보정 이동을 걸었음 → 호출자가 다시 기다렸다 재검사
 *
 * 보정 방식은 "실측을 진실로 삼고 다시 0 을 목표로 준다" 이다. 남은 오차만큼
 * 상대 이동을 주는 것보다 안전한데, 스텝카운트가 실측과 어긋난 채로 남아 있으면
 * 이후 모든 목표가 그 오차를 안고 가기 때문이다. 여기서 카운트를 진실에
 * 맞춰 두면 그 다음부터는 절대각 계산이 다시 유효해진다. */
static bool scan_home_axis_settled(motor_axis_t ax)
{
    bool ok;
    int32_t enc_pulse = 0;
    const int32_t err = scan_encoder_error_ddeg(ax, &ok, &enc_pulse);
    bool done = true;

    if (ok && (scan_abs32(err) > SCAN_HOME_TOL_DDEG)) {
        motor_sync_pulse(ax, enc_pulse);   /* 실측이 진실 */
        motor_set_target(ax, 0);           /* 다시 0 으로 */
        done = false;
    }
    return done;
}

/* ---------------------------------------------------------------------------
 *  수명주기
 * ------------------------------------------------------------------------- */
void scan_init(void)
{
    s.state           = SC_IDLE;
    s.homed           = false;
    s.line            = 0u;
    s.n_lines         = 0u;
    s.tilt_to_end     = true;
    s.settle_until_ms = 0u;
    s.home_retry      = 0u;
    s.latch_pan_ddeg  = 0;
    s.latch_tilt_ddeg = 0;
}

void scan_home(void)
{
    /* 실제 판독은 scan_process 에서 한다 — 여기서 하면 uart_rpi 디스패처
     * 안에서 수십 ms 를 블로킹하게 되고, 그 사이 들어온 프레임이 밀린다.
     *
     * ⚠️ 이미 홈 절차 중이면 무시한다. 데몬은 CMD_HOMED 가 올 때까지
     *   HOME_RETRY_MS(500ms) 마다 CMD_HOME 을 다시 보내는데, 홈 자세 이동은
     *   그보다 오래 걸린다(최대 180도 = 8초). 재시도를 그대로 받으면 상태가
     *   매번 SC_HOMING 으로 되돌아가 정착 대기가 처음부터 다시 시작되고,
     *   결국 CMD_HOMED 를 영영 못 보낸다. */
    if (s.state == SC_IDLE) {
        s.state = SC_HOMING;
    }
}

void scan_start(const struct proto_scan_start *ss)
{
    if (ss == NULL) {
        /* 방어 — 디스패처가 크기를 검사하므로 정상 경로에서는 오지 않는다 */
    } else if (!s.homed) {
        scan_report_err((uint8_t)ERR_NOT_HOMED);
    } else if (ss->step_ddeg == 0u) {
        scan_report_err((uint8_t)ERR_OUT_OF_RANGE);
    } else {
        int32_t span;

        s.pan_start_ddeg  = ss->pan_start_ddeg;
        s.pan_end_ddeg    = ss->pan_end_ddeg;
        s.tilt_start_ddeg = ss->tilt_start_ddeg;
        s.tilt_end_ddeg   = ss->tilt_end_ddeg;
        s.step_ddeg       = ss->step_ddeg;

        /* 줄 수. 팬이 한 바퀴를 넘어 감기지 않도록 랩어라운드로 계산한다. */
        span = (int32_t)s.pan_end_ddeg - (int32_t)s.pan_start_ddeg;
        if (span < 0) {
            span += 3600;
        }
        /* 합성식을 바로 캐스트하지 않고 나눗셈 결과를 먼저 받는다 (MISRA 10.8) */
        span = span / (int32_t)s.step_ddeg;
        s.n_lines = (uint32_t)span + 1u;

        s.line        = 0u;
        s.tilt_to_end = true;

        motor_enable();
        motor_set_target(MOTOR_AXIS_PAN,  scan_pan_target_pulse(0u));
        motor_set_target(MOTOR_AXIS_TILT,
                         motor_ddeg_to_pulse((int32_t)s.tilt_start_ddeg));

        /* ⚠️ 여기서 반드시 마감을 새로 잡는다. 축이 이미 시작점에 있으면
         *   SC_MOVE_START 가 첫 바퀴에 곧장 idle 로 판정하는데, 그때 이전
         *   스캔의 낡은 마감이 남아 있으면 이미 지난 시각이라 대기가 통째로
         *   건너뛰어진다. */
        s.settle_until_ms = HAL_GetTick() + SCAN_START_SETTLE_MS;
        s.state = SC_MOVE_START;
    }
}

void scan_stop(void)
{
    if (s.state != SC_IDLE) {
        /* 지금 위치에서 멈추고 완료 통지까지 보낸다. 데몬은 SCAN_DONE 의
         * point_count 로 실제 받은 점 수를 대조하므로, 중단이어도 통지가
         * 있어야 "몇 점에서 끊겼는지" 를 알 수 있다. */
        motor_set_target(MOTOR_AXIS_PAN,  motor_get_pulse(MOTOR_AXIS_PAN));
        motor_set_target(MOTOR_AXIS_TILT, motor_get_pulse(MOTOR_AXIS_TILT));
        s.state = SC_DONE;
    }
}

bool scan_is_busy(void)
{
    return (s.state != SC_IDLE);
}

scan_state_t scan_get_state(void)
{
    return s.state;
}

/* ---------------------------------------------------------------------------
 *  상태별 처리
 * ------------------------------------------------------------------------- */
static void scan_do_homing(void)
{
#if SCAN_NO_ENCODER
    /* 브링업 모드: 판독 없이 "지금 있는 자리 = 기구각 0" 으로 선언한다.
     * 즉 사용자가 틸트를 바닥(nadir), 팬을 기준 방위에 물리적으로 맞춰 둔
     * 상태여야 한다. 맞추지 않으면 산출물이 그 오차만큼 통째로 돌아간다.
     *
     * 엔코더 raw 는 0xFFFF 로 보낸다 — 14비트가 낼 수 없는 값이라 산출물만
     * 보고도 "엔코더 없이 찍은 스캔" 임을 구분할 수 있다. 0 을 보내면 정상
     * 판독값과 섞여 나중에 이 데이터를 신뢰해버릴 위험이 있다. */
    struct proto_homed h;

    motor_sync_pulse(MOTOR_AXIS_PAN,  0);
    motor_sync_pulse(MOTOR_AXIS_TILT, 0);
    s.homed = true;

    /* 0xFFFF = 14비트 엔코더(최대 16383)가 낼 수 없는 값 → "미사용" 표식 */
    h.pan_encoder_raw  = 0xFFFFu;
    h.tilt_encoder_raw = 0xFFFFu;
    h.pan_ddeg  = 0;
    h.tilt_ddeg = 0;
    uart_rpi_send_frame((uint8_t)CMD_HOMED, &h, (uint8_t)sizeof(h));
    s.state = SC_IDLE;   /* 이 모드는 "지금 자리 = 0" 이라 이동할 것이 없다 */
#else
    Encoder_t pan_enc;
    Encoder_t tilt_enc;
    bool      ok;

    /* 절대 엔코더라 구동이 필요 없다 — 읽어서 현재 위치를 확정하면 끝.
     * (리밋스위치 방식이라면 여기서 "리밋을 찾을 때까지 회전" 이 필요했다) */
    ok = (motor_read_encoder(MOTOR_AXIS_PAN,  &pan_enc)  == HAL_OK)
      && (motor_read_encoder(MOTOR_AXIS_TILT, &tilt_enc) == HAL_OK);

    if (ok) {
        const int32_t pan_pulse =
            motor_encoder_deg_to_pulse(MOTOR_AXIS_PAN,  pan_enc.degree);
        const int32_t tilt_pulse =
            motor_encoder_deg_to_pulse(MOTOR_AXIS_TILT, tilt_enc.degree);

        motor_sync_pulse(MOTOR_AXIS_PAN,  pan_pulse);
        motor_sync_pulse(MOTOR_AXIS_TILT, tilt_pulse);
        s.homed = true;

        /* 엔코더 raw 를 함께 올린다. 영점 상수가 나중에 틀렸다고 밝혀져도
         * raw 로부터 각도를 재계산할 수 있어야 이미 찍어둔 스캔을 안 버린다. */
        s.home.pan_encoder_raw  = pan_enc.raw_angle;
        s.home.tilt_encoder_raw = tilt_enc.raw_angle;
        s.home.pan_ddeg  = (int16_t)motor_pulse_to_ddeg(pan_pulse);
        s.home.tilt_ddeg = (int16_t)motor_pulse_to_ddeg(tilt_pulse);

        /* ★ 홈 자세로 이동한다 — 양축 기구각 0 (틸트 0 = nadir = **수직**).
         *
         * 판독만으로도 좌표계는 확립된다(절대 엔코더라 구동이 필요 없다).
         * 그런데도 굳이 0 으로 보내는 이유는 **영점 상수를 눈으로 검증**하기
         * 위해서다. 리밋스위치가 없어서 MOTOR_*_ZERO_OFFSET_DEG 가 맞는지
         * 확인할 물리적 수단이 이것뿐이다 — 홈이 끝났는데 틸트가 수직으로
         * 서지 않으면 그 상수가 틀린 것이고, 그 상태로 찍은 스캔은 전부
         * 그 오차만큼 통째로 돌아간다.
         *
         * CMD_HOMED 는 여기서 보내지 않고 **이동이 끝난 뒤**(SC_HOME_POSE)
         * 보낸다. 데몬이 그 프레임을 받아야 SCAN_START 를 내므로, 이렇게
         * 하면 별도 동기 없이 "자세 잡을 때까지 기다림"이 성립한다. */
        motor_enable();
        motor_set_target(MOTOR_AXIS_PAN,  0);
        motor_set_target(MOTOR_AXIS_TILT, 0);
        s.home_retry      = 0u;
        s.settle_until_ms = HAL_GetTick() + SCAN_HOME_SETTLE_MS;
        s.state = SC_HOME_POSE;
    } else {
        /* 판독 실패를 조용히 넘기면 위치가 0 에 머물고 목표(0)와 우연히
         * 일치해 "축이 안 움직였는데 홈 완료" 가 된다. 반드시 알린다.
         *
         * 코드는 ERR_NOT_HOMED 를 쓴다. 엔코더 I2C 전용 코드가 프로토콜에
         * 없기도 하고, 홈이 실패한 뒤의 상태가 정확히 "홈 안 된 상태" 라
         * 의미가 맞는다. 이 상태에서 SCAN_START 가 오면 같은 코드로 거절된다. */
        s.homed = false;
        scan_report_err((uint8_t)ERR_NOT_HOMED);
        s.state = SC_IDLE;
    }
#endif /* SCAN_NO_ENCODER */
}

/* 홈 자세(양축 0) 도달 후. 여기서 CMD_HOMED 를 보낸다.
 *
 * ★ 엔코더를 마지막으로 한 번 더 읽어 **실제로 0 에 왔는지** 확인한다.
 *   여기가 스캔 전체에서 엔코더를 쓰는 마지막 지점이다. 스윕 중에는 안 읽으므로
 *   좌표계가 옳다는 근거는 오직 이 검증뿐이다.
 *
 *   무엇을 잡나: **DIR 극성 반전**(오차가 이동량의 2배로 벌어진다)과
 *   **축 걸림/탈조**(목표에 못 온다). 통과 못 하면 스캔을 시작하지 않는다 —
 *   여기서 막지 않으면 27분을 돌고 나서 통째로 틀린 산출물을 얻는다.
 *
 *   ⚠️ **영점 상수(MOTOR_*_ZERO_OFFSET_DEG)가 틀린 건 여기서 못 잡는다.**
 *     홈에서 위치를 세울 때와 여기서 대조할 때 같은 offset 을 쓰므로 수식에서
 *     통째로 상쇄된다:
 *         홈   P    = (enc0 - offset)/k,  스텝카운트 = P
 *         이동 0 으로 → 축이 -P*k 만큼 움직임 → enc1 = enc0 - P*k
 *         검증 (enc1 - offset)/k = P - P = 0,  err = 0   ← offset 이 뭐든 통과
 *
 *     즉 offset 이 90도 틀려 있어도 이 검사는 조용히 통과한다. 그걸 잡는
 *     유일한 수단은 **SCAN_HOME_SETTLE_MS 동안 사람이 자세를 보는 것**이다.
 *     틸트가 수직(nadir)으로 서 있지 않으면 offset 이 틀린 것이다. */
static void scan_do_home_pose(void)
{
    /* 양축을 각각 대조한다. || 로 단락시키지 않고 둘 다 호출하는 이유는,
     * 한쪽이 어긋났다고 다른 쪽 보정을 건너뛰면 축마다 번갈아 한 번씩만
     * 움직이게 되어 수렴이 느려지기 때문이다. */
    const bool tilt_ok = scan_home_axis_settled(MOTOR_AXIS_TILT);
    const bool pan_ok  = scan_home_axis_settled(MOTOR_AXIS_PAN);

    if (tilt_ok && pan_ok) {
        s.home_retry = 0u;
        uart_rpi_send_frame((uint8_t)CMD_HOMED, &s.home,
                            (uint8_t)sizeof(s.home));
        s.state = SC_IDLE;
    } else if (s.home_retry < SCAN_HOME_MAX_RETRY) {
        /* 보정 이동이 걸려 있다. 상태를 유지하면 다음 바퀴에 !idle 로 보여
         * 정착 대기가 다시 잡히고, 멈춘 뒤 재검사로 돌아온다. */
        s.home_retry++;
    } else {
        /* 정해진 횟수 안에 못 잡았다 = 보정으로 될 문제가 아니다.
         * 축이 걸렸거나, DIR 극성이 뒤집혀 보정이 오히려 멀어지게 하거나,
         * 토크가 부족해 목표에 못 간다. 계속 시도해봐야 같은 자리다.
         *
         * 홈을 무효로 되돌리고 알린다. 통지만 안 하면 데몬이 HOME_TIMEOUT
         * 까지 기다리다 취소하는데, 그러면 원인이 "무응답"으로 보여 오해를
         * 부른다. */
        s.home_retry = 0u;
        s.homed      = false;
        scan_report_err((uint8_t)ERR_STALL);
        s.state = SC_IDLE;
    }
}

static void scan_do_line_end(void)
{
    /* 여기서 엔코더를 읽지 않는다.
     *
     * 예전에는 틸트를 줄 끝마다 엔코더로 재영점했다. 그런데 급정지 링잉과
     * 판독 잡음이 임계를 넘겨 헛 ERR_STALL 이 줄마다 났고, 스캔이 첫 줄에서
     * 죽어 산출물이 아예 안 나왔다. 판독 시각과 라이다 샘플 시각이 다르다는
     * 원래의 문제도 그대로다.
     *
     * 대신 각도는 **절대각에서 매번 새로 환산**하므로(scan_pan_target_pulse /
     * scan_tilt_target_pulse) 재영점 없이도 절삭 오차는 누적되지 않는다.
     * 남는 위험은 순수한 기계적 탈조뿐이고, 그건 스캔이 끝난 뒤 파킹 자세를
     * 눈으로 확인해 가린다(scan.h SCAN_HOME_TOL_DDEG 주석). */
    s.line++;
    if (s.line >= s.n_lines) {
        s.state = SC_DONE;
    } else {
        motor_set_target(MOTOR_AXIS_PAN, scan_pan_target_pulse(s.line));
        s.state = SC_PAN_STEP;
    }
}

static void scan_do_pan_step_done(void)
{
    /* 팬도 엔코더를 읽지 않는다. 원래도 **보정은 안 하고 세기만** 했는데
     * (정지 상태 1도 이동은 탈조 위험이 사실상 없고, 폐루프 보정을 넣으면
     *  엔코더 잡음이 불필요한 보정 이동을 만들어 새 실패 모드가 된다),
     * 그 세는 값조차 상행 경로가 없어 아무도 못 보는 상태였다. */
    s.tilt_to_end = !s.tilt_to_end;          /* serpentine 반전 */
    motor_set_target(MOTOR_AXIS_TILT, scan_tilt_target_pulse());
    s.state = SC_SWEEP;
}

static void scan_do_done(void)
{
    uart_rpi_send_scan_done();

    /* 양축을 **기구각 0** 으로 되돌린다(팬 0 / 틸트 0=nadir).
     *
     * 펄스 0 이 곧 기구각 0 이다 — 홈에서 motor_sync_pulse 로 엔코더 실측을
     * 펄스로 환산해 심었기 때문에 그 이후로 둘은 같은 원점을 가리킨다.
     *
     * 예전에는 팬만, 그것도 **그 스캔의 시작각**으로 되돌렸다. 그러면 파킹
     * 자세가 스캔 파라미터마다 달라져서
     *   ① 킷을 떼거나 보관할 때 자세가 매번 다르고
     *   ② 틸트는 마지막 serpentine 줄이 끝난 쪽(±90)에 그대로 남아 있고
     *   ③ 무엇보다 **눈으로 하는 탈조 확인**이 안 된다.
     * ③ 이 실질적인 이유다. 팬은 리밋이 없어 개루프 드리프트를 잡을 수단이
     * 없는데(§17-4), 매번 같은 자세로 서면 스캔이 끝났을 때 육안으로 어긋남을
     * 알아챌 수 있다. 파킹 자세가 스캔마다 달라지면 그 기준선이 사라진다.
     *
     * ⚠️ 홈이 없으면 펄스 0 이 무엇을 뜻하는지 모르므로 움직이지 않는다.
     *   (엔코더가 없는 브링업 빌드에서는 "지금 자리 = 0" 이라 제자리다)
     *
     * 기다리지 않고 바로 IDLE 로 간다 — 통지를 먼저 보내야 데몬이 산출물을
     * 마감할 수 있고, 파킹은 모터 계층이 알아서 굴러간다. 파킹 도중 새 스캔이
     * 들어와도 SC_MOVE_START 가 목표를 덮어쓰고 idle 을 기다리므로 안전하다. */
    if (s.homed) {
        motor_set_target(MOTOR_AXIS_PAN,  0);
        motor_set_target(MOTOR_AXIS_TILT, 0);
    }
    s.state = SC_IDLE;
}

void scan_process(void)
{
    switch (s.state) {
    case SC_HOMING:
        scan_do_homing();
        break;

    case SC_HOME_POSE:
        /* 양축이 0 에 도달하고 링잉이 잦아들 때까지 기다린 뒤 검증한다. */
        if (!motor_is_idle(MOTOR_AXIS_PAN) || !motor_is_idle(MOTOR_AXIS_TILT)) {
            s.settle_until_ms = HAL_GetTick() + SCAN_HOME_SETTLE_MS;
        } else if (scan_settled()) {
            scan_do_home_pose();
        } else {
            /* 정착 대기 중 */
        }
        break;

    case SC_MOVE_START:
        /* 시작점 도착 후 SCAN_START_SETTLE_MS 를 기다린다. 이동 중에는 마감을
         * 계속 밀어 두므로, 멈춘 순간이 곧 "도착 시각"이 된다. */
        if (!motor_is_idle(MOTOR_AXIS_PAN) || !motor_is_idle(MOTOR_AXIS_TILT)) {
            s.settle_until_ms = HAL_GetTick() + SCAN_START_SETTLE_MS;
        } else if (scan_settled()) {
            s.tilt_to_end = true;
            motor_set_target(MOTOR_AXIS_TILT, scan_tilt_target_pulse());
            s.state = SC_SWEEP;
        } else {
            /* 정착 대기 중 */
        }
        break;

    case SC_SWEEP:
        if (!motor_is_idle(MOTOR_AXIS_TILT)) {
            s.settle_until_ms = HAL_GetTick() + SCAN_LINE_SETTLE_MS;
        } else if (scan_settled()) {
            s.state = SC_LINE_END;
        } else {
            /* 링잉이 잦아들기를 기다린다 — 아래 SCAN_LINE_SETTLE_MS 주석 참조 */
        }
        break;

    case SC_LINE_END:
        scan_do_line_end();
        break;

    case SC_PAN_STEP:
        if (!motor_is_idle(MOTOR_AXIS_PAN)) {
            s.settle_until_ms = HAL_GetTick() + SCAN_LINE_SETTLE_MS;
        } else if (scan_settled()) {
            scan_do_pan_step_done();
        } else {
            /* 정착 대기 중 */
        }
        break;

    case SC_DONE:
        scan_do_done();
        break;

    case SC_IDLE:
    default:
        break;
    }
}

/* ---------------------------------------------------------------------------
 *  라이다 연동
 * ------------------------------------------------------------------------- */
void scan_latch_angles(int16_t *out_pan_ddeg, int16_t *out_tilt_ddeg)
{
    /* ISR 에서 호출된다. 모터 카운터를 읽기만 하고(32비트 정렬 워드라 원자적)
     * 상태를 바꾸지 않으므로 안전하다. */
    if (out_pan_ddeg != NULL) {
        *out_pan_ddeg = motor_get_ddeg(MOTOR_AXIS_PAN);
    }
    if (out_tilt_ddeg != NULL) {
        *out_tilt_ddeg = motor_get_ddeg(MOTOR_AXIS_TILT);
    }
}

void scan_submit_sample(int16_t pan_ddeg, int16_t tilt_ddeg,
                        uint16_t d_mm, uint16_t signal_strength,
                        uint32_t device_time_ms,
                        uint8_t dis_status, uint8_t range_precision)
{
    /* 스윕 구간의 점만 올린다. 줄 끝 정지·팬 이동 중의 점은 격자에 넣을
     * 각도가 아니다(같은 각도에 몰려 중복만 만든다). */
    if (s.state == SC_SWEEP) {
        uart_rpi_send_scan_point(pan_ddeg, tilt_ddeg, d_mm, signal_strength,
                                 device_time_ms, dis_status, range_precision);
    }
}
