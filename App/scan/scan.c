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

    /* 진단 (누적) */
    uint32_t tilt_resync;   /* 틸트 줄 끝 재영점 횟수              */
    uint32_t pan_deviate;   /* 팬 대조 이탈 횟수 (보정은 안 함)    */

    /* 라이다 각도 래치 */
    volatile int16_t latch_pan_ddeg;
    volatile int16_t latch_tilt_ddeg;
} s;

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

/* 엔코더 실측과 스텝카운트를 대조한다.
 *   반환: 오차(ddeg, 부호). 판독 실패면 *ok=false.
 * 메인루프 전용 — 블로킹 I2C 를 탄다. */
static int32_t scan_encoder_error_ddeg(motor_axis_t ax, bool *ok)
{
    int32_t err = 0;

#if SCAN_NO_ENCODER
    /* 브링업 모드: 판독을 아예 시도하지 않는다.
     * 단순히 실패시키는 것과 다르다 — 실패하면 motor_read_encoder_pulse 가
     * 5회 × (I2C 타임아웃 10ms + 대기 10ms) = 약 100ms 를 잡아먹고, 그게
     * 줄마다 두 축이면 스캔 전체에 36초가 헛돈다. 호출 자체를 건너뛴다. */
    (void)ax;
    *ok = false;               /* 호출자가 대조·재영점을 건너뛴다 */
#else
    int32_t enc_pulse = 0;

    *ok = (motor_read_encoder_pulse(ax, &enc_pulse) == HAL_OK);
    if (*ok) {
        err = motor_pulse_to_ddeg(enc_pulse - motor_get_pulse(ax));
    }
#endif
    return err;
}

static int32_t scan_abs32(int32_t v)
{
    return (v < 0) ? -v : v;
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
    s.tilt_resync     = 0u;
    s.pan_deviate     = 0u;
    s.latch_pan_ddeg  = 0;
    s.latch_tilt_ddeg = 0;
}

void scan_home(void)
{
    /* 실제 판독은 scan_process 에서 한다 — 여기서 하면 uart_rpi 디스패처
     * 안에서 수십 ms 를 블로킹하게 되고, 그 사이 들어온 프레임이 밀린다. */
    s.state = SC_HOMING;
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
        s.tilt_resync = 0u;
        s.pan_deviate = 0u;

        motor_enable();
        motor_set_target(MOTOR_AXIS_PAN,  scan_pan_target_pulse(0u));
        motor_set_target(MOTOR_AXIS_TILT,
                         motor_ddeg_to_pulse((int32_t)s.tilt_start_ddeg));
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
        {
            struct proto_homed h;
            h.pan_encoder_raw  = pan_enc.raw_angle;
            h.tilt_encoder_raw = tilt_enc.raw_angle;
            h.pan_ddeg  = (int16_t)motor_pulse_to_ddeg(pan_pulse);
            h.tilt_ddeg = (int16_t)motor_pulse_to_ddeg(tilt_pulse);
            uart_rpi_send_frame((uint8_t)CMD_HOMED, &h, (uint8_t)sizeof(h));
        }
    } else {
        /* 판독 실패를 조용히 넘기면 위치가 0 에 머물고 목표(0)와 우연히
         * 일치해 "축이 안 움직였는데 홈 완료" 가 된다. 반드시 알린다.
         *
         * 코드는 ERR_NOT_HOMED 를 쓴다. 엔코더 I2C 전용 코드가 프로토콜에
         * 없기도 하고, 홈이 실패한 뒤의 상태가 정확히 "홈 안 된 상태" 라
         * 의미가 맞는다. 이 상태에서 SCAN_START 가 오면 같은 코드로 거절된다. */
        s.homed = false;
        scan_report_err((uint8_t)ERR_NOT_HOMED);
    }
#endif /* SCAN_NO_ENCODER */
    s.state = SC_IDLE;
}

static void scan_do_line_end(void)
{
    bool ok;
    const int32_t err = scan_encoder_error_ddeg(MOTOR_AXIS_TILT, &ok);

    /* 틸트는 줄 끝마다 방향을 뒤집는다 — 1축 브링업에서 스캔당 1.25도씩
     * 밀렸던 자리가 정확히 여기다. 엔코더로 재영점해 누적을 끊는다. */
    if (ok && (scan_abs32(err) > SCAN_STALL_DDEG)) {
        scan_report_err((uint8_t)ERR_STALL);
        s.state = SC_DONE;
    } else {
        if (ok && (scan_abs32(err) > SCAN_RESYNC_DDEG)) {
            int32_t enc_pulse = 0;
            if (motor_read_encoder_pulse(MOTOR_AXIS_TILT, &enc_pulse) == HAL_OK) {
                motor_sync_pulse(MOTOR_AXIS_TILT, enc_pulse);
                s.tilt_resync++;
            }
        }

        s.line++;
        if (s.line >= s.n_lines) {
            s.state = SC_DONE;
        } else {
            motor_set_target(MOTOR_AXIS_PAN, scan_pan_target_pulse(s.line));
            s.state = SC_PAN_STEP;
        }
    }
}

static void scan_do_pan_step_done(void)
{
    bool ok;
    const int32_t err = scan_encoder_error_ddeg(MOTOR_AXIS_PAN, &ok);

    /* 팬은 **보정하지 않고 검증만** 한다.
     * 정지 상태에서 1도(=9스텝)를 저속으로 가는 거라 탈조 위험이 사실상
     * 없고, 폐루프 보정을 넣으면 엔코더 잡음으로 인한 불필요한 보정 이동이
     * 오히려 새 실패 모드가 된다. 다만 리밋스위치가 없어 한 번 미끄러지면
     * 남은 스캔 내내 어긋나는데 이를 잡을 다른 수단이 없으므로, 정지 중이라
     * 공짜인 판독만 남겨 이탈을 셈한다. */
    if (ok && (scan_abs32(err) > SCAN_STALL_DDEG)) {
        scan_report_err((uint8_t)ERR_STALL);
        s.state = SC_DONE;
    } else {
        if (ok && (scan_abs32(err) > SCAN_RESYNC_DDEG)) {
            s.pan_deviate++;
        }
        s.tilt_to_end = !s.tilt_to_end;          /* serpentine 반전 */
        motor_set_target(MOTOR_AXIS_TILT, scan_tilt_target_pulse());
        s.state = SC_SWEEP;
    }
}

static void scan_do_done(void)
{
    uart_rpi_send_scan_done();

    /* 다음 스캔을 바로 걸 수 있도록 팬만 시작점으로 되돌린다.
     * 모터 계층이 알아서 굴러가므로 여기서 기다리지 않는다 — 통지를
     * 먼저 보내야 데몬이 산출물을 마감할 수 있다. */
    if (s.homed) {
        motor_set_target(MOTOR_AXIS_PAN, scan_pan_target_pulse(0u));
    }
    s.state = SC_IDLE;
}

void scan_process(void)
{
    switch (s.state) {
    case SC_HOMING:
        scan_do_homing();
        break;

    case SC_MOVE_START:
        if (motor_is_idle(MOTOR_AXIS_PAN) && motor_is_idle(MOTOR_AXIS_TILT)) {
            s.tilt_to_end = true;
            motor_set_target(MOTOR_AXIS_TILT, scan_tilt_target_pulse());
            s.state = SC_SWEEP;
        }
        break;

    case SC_SWEEP:
        if (motor_is_idle(MOTOR_AXIS_TILT)) {
            s.state = SC_LINE_END;
        }
        break;

    case SC_LINE_END:
        scan_do_line_end();
        break;

    case SC_PAN_STEP:
        if (motor_is_idle(MOTOR_AXIS_PAN)) {
            scan_do_pan_step_done();
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
