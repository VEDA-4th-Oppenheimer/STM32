/* ============================================================================
 *  scan.c  --  2축 스캔 시퀀서 구현
 * ----------------------------------------------------------------------------
 *  담당: 강유근 (원 구현) / 이현우 (계층 분리 리팩터링)
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

    /* 진행 중이라 거절한 SCAN_START 수 (진단 — 프로토콜에 ERR_BUSY 가 없다) */
    uint32_t reject_busy;

    /* 홈 판독 결과. CMD_HOMED 를 홈 자세 이동이 **끝난 뒤에** 보내야 해서
     * 상태를 넘어 들고 가야 한다 (아래 SC_HOME_POSE 주석 참조). */
    struct proto_homed home;

    /* 정착 대기 마감 시각(HAL tick).
     * 축이 움직이는 동안 계속 앞으로 밀리므로, 멈춘 뒤 "도착 시각 + 대기"가
     * 자연히 마감이 된다. 상태를 새로 만들지 않아도 되는 이유다. */
    uint32_t settle_until_ms;

    /* 파킹 포기 시각(HAL tick). settle_until_ms 와 달리 **밀리지 않는다** —
     * 축이 계속 움직이는 한 정착 마감은 앞으로 밀리므로, 도착을 못 하는
     * 상황에서 전류가 무한정 흐르는 걸 막으려면 별도의 절대 마감이 필요하다. */
    uint32_t park_deadline_ms;

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
/* v6: 코드는 "무엇이", axis 는 "어디서" 를 말한다. 축과 무관한 오류는
 * ERR_AXIS_NONE 을 준다. */
static void scan_report_err(uint8_t code, uint8_t axis)
{
    struct proto_err e;
    e.code = code;
    e.axis = axis;
    (void)uart_rpi_send_frame((uint8_t)CMD_ERROR, &e, (uint8_t)sizeof(e));
}

/* 이번 줄의 팬 목표(펄스).
 *
 * 주의: 반드시 **절대각에서** 계산한다. 1도 스텝을 펄스로 굳혀 매 줄 더하면
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
 * 핵심: 홈 자세 검증에서만 쓴다. 스윕 중에는 호출하지 않는다 — 이유는 scan.h
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

/* 축 하나를 대조하고, 필요하면 보정 이동을 건다.
 *   반환 true  = 세부조정 목표(FINE) 안에 들었다 → 이 축은 완료
 *        false = 아직 멀다 → 보정을 걸었으니 호출자가 다시 기다렸다 재검사
 *
 * 핵심: 두 경우 모두 스텝카운트를 실측에 맞춘다(motor_sync_pulse). 축이 물리적으로
 *   0 에 정확히 서는 것보다 **카운트가 진실을 담는 것**이 중요하기 때문이다.
 *   예전에는 허용 오차 안이면 아무것도 안 했는데, 그러면 엔코더가 "너 3.9도
 *   지점에 있어" 라고 말하는 걸 무시하고 "여기가 0" 으로 확정해 버려서
 *   홈 정확도가 임계값만큼 벌어졌다.
 *
 * 보정 방식은 "실측을 진실로 삼고 절대 목표 0 을 다시 준다" 이다. 남은 오차만큼
 * 상대 이동을 주는 것보다 안전한데, 스텝카운트가 실측과 어긋난 채 남으면 이후
 * 모든 절대각 계산이 그 오차를 안고 가기 때문이다. */
/* 홈 자세 재검증 결과.
 *
 * 주의: 예전에는 bool 하나였고 **판독 실패를 SETTLED 로 접었다.** 그래서
 *   한 축의 I2C 가 죽어도 다른 축만 수렴하면 `true && true` 가 되어 검증
 *   없이 CMD_HOMED 가 나갔다. 축마다 I2C 버스가 분리돼 있어(Pan=I2C3 /
 *   Tilt=I2C1) **한쪽만 죽는 것이 오히려 흔한 시나리오**다 — 실제로 겪은
 *   고장이 전부 그랬다(§18-1 틸트 SDA 단락, §18-4 팬 I2C 미복구).
 *
 *   무한 재시도를 안 한다는 원래 판단은 유지한다(축당 100ms 씩 태우며 영영
 *   안 끝난다). 대신 재시도를 다 쓰면 ERR_NOT_HOMED 로 정직하게 끝낸다. */
typedef enum {
    HOME_SETTLED = 0,   /* 판독 성공 + 허용 오차 안         */
    HOME_ADJUSTING,     /* 판독 성공 + 아직 멀다 (보정 중)  */
    HOME_UNREADABLE     /* 판독 실패 — 자세를 확인하지 못함 */
} home_check_t;

static home_check_t scan_home_axis_settled(motor_axis_t ax)
{
    bool ok;
    int32_t enc_pulse = 0;
    const int32_t err = scan_encoder_error_ddeg(ax, &ok, &enc_pulse);
    home_check_t r = HOME_UNREADABLE;

    if (ok) {
        /* 실측을 카운트에 반영 — 수렴했든 아니든 항상. */
        motor_sync_pulse(ax, enc_pulse);

        if (scan_abs32(err) > SCAN_HOME_FINE_DDEG) {
            motor_set_target(ax, 0);   /* 아직 머니 한 번 더 당긴다 */
            r = HOME_ADJUSTING;
        } else {
            r = HOME_SETTLED;
        }
    }
    return r;
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
    s.park_deadline_ms = 0u;
    s.home_retry      = 0u;
    s.latch_pan_ddeg  = 0;
    s.latch_tilt_ddeg = 0;
}

void scan_home(void)
{
    /* 실제 판독은 scan_process 에서 한다 — 여기서 하면 uart_rpi 디스패처
     * 안에서 수십 ms 를 블로킹하게 되고, 그 사이 들어온 프레임이 밀린다.
     *
     * 주의: 이미 홈 절차 중이면 무시한다. 데몬은 CMD_HOMED 가 올 때까지
     *   HOME_RETRY_MS(500ms) 마다 CMD_HOME 을 다시 보내는데, 홈 자세 이동은
     *   그보다 오래 걸린다(최대 180도 = 8초). 재시도를 그대로 받으면 상태가
     *   매번 SC_HOMING 으로 되돌아가 정착 대기가 처음부터 다시 시작되고,
     *   결국 CMD_HOMED 를 영영 못 보낸다. */
    if (s.state == SC_IDLE) {
        /* 홈을 다시 잡는 동안은 "홈이 서 있지 않다" 가 맞다.
         *
         * 주의: 이걸 안 내리면 CMD_STATUS(1초 주기)가 이전 홈의 s.homed=true 를
         *   계속 실어 보낸다. 데몬은 TURRET_HOME ioctl 에서 드라이버 캐시의
         *   STF_HOMED 를 내리고 "이후 homed==1 은 반드시 이번 HOME 에 대한
         *   응답" 이라는 불변식에 기대는데, STATUS 가 그 플래그를 되살려
         *   **홈이 끝나기도 전에 SCAN_START 를 보내게 된다.** 그러면 STM 이
         *   아직 SC_HOMING 이라 ERR_BUSY 로 거절하고 스캔이 통째로 실패한다
         *   (실기에서 그렇게 났다 — proto v6 로 STATUS 를 실제로 보내기
         *   시작하면서 드러난 상호작용이다). */
        s.homed = false;
        s.state = SC_HOMING;
    }
}

/* 각도 요청이 프로토콜 범위 안인가.
 *
 * 주의: 데몬과 드라이버가 각각 같은 검사를 하지만 여기서도 본다. 세 곳 중
 *   어느 하나가 뚫리면 그 결과가 곧바로 모터 목표가 되기 때문이다. 특히
 *   pan 범위를 벗어나면 아래 span 계산의 +3600 보정으로도 음수가 남아
 *   uint32_t 로 접히면서 n_lines 가 수십억이 된다. */
static bool scan_request_in_range(const struct proto_scan_start *ss)
{
    return (ss->pan_start_ddeg  >= PAN_MIN)  && (ss->pan_start_ddeg  <= PAN_MAX)
        && (ss->pan_end_ddeg    >= PAN_MIN)  && (ss->pan_end_ddeg    <= PAN_MAX)
        && (ss->tilt_start_ddeg >= TILT_MIN) && (ss->tilt_start_ddeg <= TILT_MAX)
        && (ss->tilt_end_ddeg   >= TILT_MIN) && (ss->tilt_end_ddeg   <= TILT_MAX)
        && (ss->step_ddeg > 0u) && (ss->step_ddeg <= 3600u);
}

void scan_start(const struct proto_scan_start *ss)
{
    if (ss == NULL) {
        /* 방어 — 디스패처가 크기를 검사하므로 정상 경로에서는 오지 않는다 */
    } else if (s.state != SC_IDLE) {
        /* 이미 스캔·홈·파킹 중이다. 요청을 **버린다**(덮어쓰지 않는다).
         *
         * 예전에는 상태를 안 보고 파라미터를 통째로 갈아끼웠다. 진행 중인
         * 스윕이 그 자리에서 새 범위로 바뀌는데, 산출물은 이미 옛 격자로
         * 절반쯤 채워져 있어 두 스캔이 한 파일에 섞인다.
         *
         * 주의: 오류를 못 올린다 — 프로토콜에 "지금은 못 받는다" 를 뜻하는
         *   코드가 없다. 있는 코드를 빌려 쓰면 나중에 그 코드가 진짜로 났을
         *   때 오독하게 되므로 v5 까지는 카운터로만 남겼다. v6 에서 ERR_BUSY 가
         *   생겨 이제 사유를 그대로 올린다. 카운터는 누적 진단으로 유지한다
         *   (CMD_STATUS 에 실린다). */
        s.reject_busy++;
        scan_report_err((uint8_t)ERR_BUSY, (uint8_t)ERR_AXIS_NONE);
    } else if (!s.homed) {
        scan_report_err((uint8_t)ERR_NOT_HOMED, (uint8_t)ERR_AXIS_NONE);
    } else if (!scan_request_in_range(ss)) {
        scan_report_err((uint8_t)ERR_OUT_OF_RANGE, (uint8_t)ERR_AXIS_NONE);
    } else {
        int32_t span;

        /* 카운터는 **요청이 받아들여진 뒤에** 민다. 디스패처에서 먼저 밀면
         * 거절된 요청 하나가 진행 중인 스캔의 점 수를 0 으로 지워, SCAN_DONE
         * 의 point_count 가 실제보다 작게 나간다. */
        uart_rpi_reset_scan_count();

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

        /* 주의: 여기서 반드시 마감을 새로 잡는다. 축이 이미 시작점에 있으면
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
         * 있어야 "몇 점에서 끊겼는지" 를 알 수 있다.
         *
         * SC_DONE 으로 보내는 것은 파킹까지 거치기 위해서다 — 축이 스윕
         * 도중 아무 데나 서 있으면 육안 탈조 확인의 기준선이 사라진다. */
        motor_set_target(MOTOR_AXIS_PAN,  motor_get_pulse(MOTOR_AXIS_PAN));
        motor_set_target(MOTOR_AXIS_TILT, motor_get_pulse(MOTOR_AXIS_TILT));
        s.state = SC_DONE;
    }
}

/* 비상정지용. 통지도 파킹도 없이 그 자리에서 시퀀스를 버린다.
 *
 * 주의: CMD_DISARM 이 scan_stop() 을 부르면 안 된다. 그러면 SC_DONE 으로
 *   가는데, 호출자가 곧이어 motor_disarm() 으로 전류를 끊은 **다음 루프에서**
 *   scan_do_done() 이 돌면서 두 가지를 한다:
 *     ① 가짜 SCAN_DONE 송신 — 스캔이 끝난 적이 없는데 완료 통지가 나간다
 *     ② 파킹 목표를 0 으로 재설정 — 전류가 없으니 축은 안 움직이는데
 *        스텝카운트만 0 까지 흘러가 좌표계가 조용히 틀어진다
 *   ②는 motor.c 의 s_armed 로도 막지만, 애초에 이 경로를 타지 않는 것이 맞다.
 *
 * homed 는 그대로 둔다. 데몬이 스캔마다 홈을 다시 잡으므로(§17-10) 여기서
 * 내려도 실익이 없고, 내리면 cmd/home 없이 재개하는 흐름이 막힌다. */
void scan_abort(void)
{
    s.state = SC_IDLE;
}

bool scan_is_busy(void)
{
    return (s.state != SC_IDLE);
}

bool scan_is_homed(void)
{
    /* 홈 **절차가 끝났을 때만** 참이다. 엔코더 판독만 되고 자세 이동(SC_HOME_POSE)
     * 이 남아 있으면 거짓이다.
     *
     * 주의: 데몬은 이 플래그(STF_HOMED)로 "스캔을 시작해도 되나" 를 판단한다.
     *   s.homed 는 SC_HOMING 에서 판독이 성공한 순간 참이 되는데, 그걸 그대로
     *   내보내면 자세 이동이 몇 초 남았는데도 데몬이 SCAN_START 를 보내고
     *   STM 은 ERR_BUSY 로 거절한다(실기 발생). CMD_HOMED 를 보내는 시점과
     *   이 플래그가 서는 시점이 같아야 한다. */
    return s.homed && (s.state != SC_HOMING) && (s.state != SC_HOME_POSE);
}

uint32_t scan_reject_busy_count(void)
{
    return s.reject_busy;
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
    (void)uart_rpi_send_frame((uint8_t)CMD_HOMED, &h, (uint8_t)sizeof(h));
    s.state = SC_IDLE;   /* 이 모드는 "지금 자리 = 0" 이라 이동할 것이 없다 */
#else
    Encoder_t pan_enc;
    Encoder_t tilt_enc;

    /* 절대 엔코더라 구동이 필요 없다 — 읽어서 현재 위치를 확정하면 끝.
     * (리밋스위치 방식이라면 여기서 "리밋을 찾을 때까지 회전" 이 필요했다)
     *
     * 주의: 예전엔 두 판독을 && 로 한 줄에 묶었는데, 그러면 팬이 실패한 순간
     *   단락되어 **틸트를 아예 읽지 않는다**. 판정에는 문제가 없지만 어느 축이
     *   죽었는지 알 길이 없어져서, 실기에서 last_err=3 만 보고 축을 못 가렸다.
     *   둘 다 읽어 각각 남긴다 — I2C 판독은 실패해도 10ms 라 비용도 무시할
     *   수준이다. */
    const bool pan_ok  = (motor_read_encoder(MOTOR_AXIS_PAN,  &pan_enc)  == HAL_OK);
    const bool tilt_ok = (motor_read_encoder(MOTOR_AXIS_TILT, &tilt_enc) == HAL_OK);
    const bool ok      = pan_ok && tilt_ok;

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

        /* 핵심: 홈 자세로 이동한다 — 양축 기구각 0 (틸트 0 = nadir = **수직**).
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
        /* v6: 축을 프로토콜이 나른다. 예전에는 축 필드가 없어 다른 오류코드를
         * 빌려 표시했는데(SCAN_HOME_AXIS_PROBE: 4=팬 / 6=틸트), 빌린 코드는
         * 그 코드가 진짜로 났을 때 오독을 부른다. 이제 코드는 ERR_ENCODER 로
         * 고정하고 어느 축인지는 axis 가 말한다. */
        const uint8_t axis = (uint8_t)((pan_ok  ? 0u : (uint8_t)ERR_AXIS_PAN)
                                     | (tilt_ok ? 0u : (uint8_t)ERR_AXIS_TILT));

        scan_report_err((uint8_t)ERR_ENCODER, axis);
        s.state = SC_IDLE;
    }
#endif /* SCAN_NO_ENCODER */
}

/* 홈 자세(양축 0) 도달 후. 여기서 CMD_HOMED 를 보낸다.
 *
 * 핵심: 엔코더를 마지막으로 한 번 더 읽어 **실제로 0 에 왔는지** 확인한다.
 *   여기가 스캔 전체에서 엔코더를 쓰는 마지막 지점이다. 스윕 중에는 안 읽으므로
 *   좌표계가 옳다는 근거는 오직 이 검증뿐이다.
 *
 *   무엇을 잡나: **DIR 극성 반전**(오차가 이동량의 2배로 벌어진다)과
 *   **축 걸림/탈조**(목표에 못 온다). 통과 못 하면 스캔을 시작하지 않는다 —
 *   여기서 막지 않으면 수백 줄 스캔을 돌고 나서 통째로 틀린 산출물을 얻는다.
 *
 *   주의: **영점 상수(MOTOR_*_ZERO_OFFSET_DEG)가 틀린 건 여기서 못 잡는다.**
 *     홈에서 위치를 세울 때와 여기서 대조할 때 같은 offset 을 쓰므로 수식에서
 *     통째로 상쇄된다:
 *         홈   P    = (enc0 - offset)/k,  스텝카운트 = P
 *         이동 0 으로 → 축이 -P*k 만큼 움직임 → enc1 = enc0 - P*k
 *         검증 (enc1 - offset)/k = P - P = 0,  err = 0   ← offset 이 뭐든 통과
 *
 *     즉 offset 이 90도 틀려 있어도 이 검사는 조용히 통과한다. 그걸 잡는
 *     유일한 수단은 **SCAN_HOME_SETTLE_MS 동안 사람이 자세를 보는 것**이다.
 *     틸트가 수직(nadir)으로 서 있지 않으면 offset 이 틀린 것이다. */
/* 홈 확립 완료를 알린다. CMD_HOMED 의 payload 는 **최초 판독값**이다 —
 * 이후 보정 이동으로 축이 조금 움직였어도, provenance 로 남겨야 하는 것은
 * "홈을 세울 때 엔코더가 무엇을 읽었나" 이기 때문이다. */
static void scan_home_finish(void)
{
    s.home_retry = 0u;
    (void)uart_rpi_send_frame((uint8_t)CMD_HOMED, &s.home,
                        (uint8_t)sizeof(s.home));
    s.state = SC_IDLE;
}

static void scan_do_home_pose(void)
{
    /* 양축을 각각 대조한다. || 로 단락시키지 않고 둘 다 호출하는 이유는,
     * 한쪽이 어긋났다고 다른 쪽 보정을 건너뛰면 축마다 번갈아 한 번씩만
     * 움직이게 되어 수렴이 느려지기 때문이다. */
    const home_check_t tilt_r = scan_home_axis_settled(MOTOR_AXIS_TILT);
    const home_check_t pan_r  = scan_home_axis_settled(MOTOR_AXIS_PAN);
    const bool unreadable = (tilt_r == HOME_UNREADABLE)
                         || (pan_r  == HOME_UNREADABLE);

    if (!unreadable && (tilt_r == HOME_SETTLED) && (pan_r == HOME_SETTLED)) {
        scan_home_finish();                 /* FINE 안에 수렴 */
    } else if (s.home_retry < SCAN_HOME_MAX_RETRY) {
        /* 보정 이동이 걸려 있다. 상태를 유지하면 다음 바퀴에 !idle 로 보여
         * 정착 대기가 다시 잡히고, 멈춘 뒤 재검사로 돌아온다. */
        s.home_retry++;
    } else {
        /* 재시도를 다 썼다. 여기서 갈린다 —
         *   TOL 안 : 세부조정만 못 끝냈을 뿐 카운트는 실측에 맞춰져 있다.
         *            물리적으로 0 에 못 선 것뿐이라 **정상 진행**한다.
         *   TOL 밖 : 보정으로 될 문제가 아니다. DIR 극성 반전(보정할수록
         *            멀어짐)·축 걸림·토크 부족·영점 상수 오류. */
        bool ok_t;
        bool ok_p;
        int32_t dummy = 0;
        const int32_t et = scan_encoder_error_ddeg(MOTOR_AXIS_TILT, &ok_t, &dummy);
        const int32_t ep = scan_encoder_error_ddeg(MOTOR_AXIS_PAN,  &ok_p, &dummy);
        const bool too_far = (ok_t && (scan_abs32(et) > SCAN_HOME_TOL_DDEG))
                          || (ok_p && (scan_abs32(ep) > SCAN_HOME_TOL_DDEG));

        if (!ok_t || !ok_p) {
            /* 재시도를 다 쓰도록 자세를 확인하지 못했다. TOL 판정에서 그 축을
             * 빼고 "괜찮다" 로 넘기면 검증 안 된 원점으로 스캔이 돌아간다.
             * v6: 판독 실패이므로 ERR_ENCODER 이고, 어느 축인지는 axis 가
             * 말한다(예전에는 ERR_NOT_HOMED 로 뭉뚱그렸다). */
            /* 축 값이 비트 플래그라 OR 로 합쳐진다(protocol.h 주석 참조).
             * 삼항 중첩보다 짧고, "둘 다 실패" 를 따로 안 써도 된다. */
            const uint8_t ax = (uint8_t)((ok_p ? 0u : (uint8_t)ERR_AXIS_PAN)
                                       | (ok_t ? 0u : (uint8_t)ERR_AXIS_TILT));
            s.home_retry = 0u;
            s.homed      = false;
            scan_report_err((uint8_t)ERR_ENCODER, ax);
            s.state = SC_IDLE;
        } else if (too_far) {
            /* 통지만 안 하면 데몬이 HOME_TIMEOUT 까지 기다리다 취소하는데,
             * 그러면 원인이 "무응답" 으로 보여 오해를 부른다. 명시적으로 알린다.
             *
             * 축 값은 비트 플래그(PAN=1, TILT=2)이므로 OR 로 합쳐 전송한다.
             * 삼항 연산자 단일 선택을 쓰면 양축 동시 탈조 시 Tilt 에러 비트가
             * 누락되므로, 각각 판정해 합쳐 ERR_AXIS_BOTH(3)를 온전히 보존한다. */
            const uint8_t ax = (uint8_t)(((scan_abs32(ep) > SCAN_HOME_TOL_DDEG) ? (uint8_t)ERR_AXIS_PAN  : 0u)
                                       | ((scan_abs32(et) > SCAN_HOME_TOL_DDEG) ? (uint8_t)ERR_AXIS_TILT : 0u));
            s.home_retry = 0u;
            s.homed      = false;
            scan_report_err((uint8_t)ERR_STALL, ax);
            s.state = SC_IDLE;
        } else {
            scan_home_finish();
        }
    }
}

static void scan_do_line_end(void)
{
#if !SCAN_NO_ENCODER
    bool ok = false;
    int32_t enc_pulse = 0;
    const int32_t err = scan_encoder_error_ddeg(MOTOR_AXIS_TILT, &ok, &enc_pulse);

    if (!ok) {
        /* I2C 판독 실패 시 비상정지 및 에러 통지 */
        motor_disarm();
        s.homed = false;
        scan_report_err((uint8_t)ERR_ENCODER, (uint8_t)ERR_AXIS_TILT);
        s.state = SC_IDLE;
        return;
    }

    if (scan_abs32(err) > SCAN_STALL_TILT_DDEG) {
        /* [탈조 확정] 모터 차단 및 에러 전송 (RPi 사양: code=5, axis=2) */
        motor_disarm();
        s.homed = false;
        scan_report_err((uint8_t)ERR_STALL, (uint8_t)ERR_AXIS_TILT);
        s.state = SC_IDLE;
        return;
    }

    /* [정상] 탈조 없음 확인 완료.
     * 주의: 여기서 motor_sync_pulse(재영점)를 하지 않는다!
     * 엔코더 지터(0.42도)가 스텝 카운터로 주입되면 매 스윕마다 랜덤 오프셋이 생겨
     * 3D 포인트 클라우드에 줄무늬(Striping)와 표면 노이즈(+27%)가 발생한다.
     * 스텝모터의 매끄러운 기구 궤적을 보존하고 순수 탈조 감시(Monitor-Only)만 수행한다. */
#endif

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
#if !SCAN_NO_ENCODER
    bool ok = false;
    int32_t enc_pulse = 0;
    const int32_t err = scan_encoder_error_ddeg(MOTOR_AXIS_PAN, &ok, &enc_pulse);

    if (!ok) {
        /* I2C 판독 실패 시 비상정지 및 에러 통지 */
        motor_disarm();
        s.homed = false;
        scan_report_err((uint8_t)ERR_ENCODER, (uint8_t)ERR_AXIS_PAN);
        s.state = SC_IDLE;
        return;
    }

    if (scan_abs32(err) > SCAN_STALL_PAN_DDEG) {
        /* [탈조 확정] 모터 차단 및 에러 전송 (RPi 사양: code=5, axis=1) */
        motor_disarm();
        s.homed = false;
        scan_report_err((uint8_t)ERR_STALL, (uint8_t)ERR_AXIS_PAN);
        s.state = SC_IDLE;
        return;
    }
#endif

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
     * 주의: 홈이 없으면 펄스 0 이 무엇을 뜻하는지 모르므로 움직이지 않는다.
     *   (엔코더가 없는 브링업 빌드에서는 "지금 자리 = 0" 이라 제자리다)
     *
     * 통지를 **먼저** 보낸다 — 데몬이 그걸 받아야 산출물을 마감한다. 파킹을
     * 기다렸다가 보내면 데몬이 그 시간만큼 더 ST_SCANNING 에 머문다.
     *
     * 주의: 예전에는 여기서 곧장 SC_IDLE 로 갔다. 그러면 파킹은 모터 계층이
     *   알아서 끝내지만 **코일 전류가 영원히 켜진 채 남는다**(scan.h 의
     *   SCAN_PARK_* 주석 참조 — 모터가 뜨거워진 원인이 이것이다).
     *   도착을 확인하고 끊어야 해서 SC_PARK 로 넘긴다.
     *
     * 파킹 도중 새 스캔이 들어와도 안전하다 — SC_MOVE_START 가 목표를
     * 덮어쓰고 idle 을 기다리며, scan_start 가 motor_enable() 을 다시 부른다. */
    if (s.homed) {
        motor_set_target(MOTOR_AXIS_PAN,  0);
        motor_set_target(MOTOR_AXIS_TILT, 0);
    }
    s.park_deadline_ms = HAL_GetTick() + SCAN_PARK_TIMEOUT_MS;
    s.settle_until_ms  = HAL_GetTick() + SCAN_PARK_SETTLE_MS;
    s.state = SC_PARK;
}

/* 파킹 도착 대기 → 전류 차단. 스캔 시퀀스의 마지막이다. */
static void scan_do_park(void)
{
    /* 이동 중에는 마감을 계속 밀어 둔다 — 멈춘 순간이 곧 도착 시각이 된다.
     * 정착을 짧게(500ms) 기다리는 이유는 급정지 링잉이 잦아들기 전에 전류를
     * 끊으면 그 진동이 그대로 위치 오차로 굳기 때문이다. */
    if (!motor_is_idle(MOTOR_AXIS_PAN) || !motor_is_idle(MOTOR_AXIS_TILT)) {
        s.settle_until_ms = HAL_GetTick() + SCAN_PARK_SETTLE_MS;
    }

    /* 타임아웃은 정착 조건을 **무시하고** 끊는다. 도착을 영영 못 하는 상황
     * (드라이버 불량·축 걸림)에서 전류가 계속 흐르는 것이 바로 여기서
     * 막으려던 그 상태이므로, 못 왔다고 켜둔 채 기다리면 안 된다. */
    /* 뺄셈 결과를 변수로 먼저 받는다 — 합성식을 바로 캐스트하면 MISRA 10.8.
     * 랩어라운드 안전한 비교 방식은 scan_settled() 주석 참조. */
    const uint32_t park_elapsed = HAL_GetTick() - s.park_deadline_ms;
    const bool timed_out = ((int32_t)park_elapsed >= 0);

    if (scan_settled() || timed_out) {
        motor_disarm();
        s.state = SC_IDLE;
    }
}

void scan_process(void)
{
    switch (s.state) {
    case SC_HOMING:
        scan_do_homing();
        break;

    case SC_HOME_POSE:
        /* 양축이 0 에 도달하고 링잉이 잦아들 때까지 기다린 뒤 검증한다.
         *
         * 정착 시간이 두 가지인 이유: 첫 이동은 최대 180도라 링잉이 크고
         * 사람이 자세를 확인할 창도 겸하므로 길게(3초), 이후 세부조정은
         * 몇 펄스짜리 이동이라 짧게(300ms) 잡는다. 안 그러면 10회 반복에
         * 30초가 날아간다. */
        if (!motor_is_idle(MOTOR_AXIS_PAN) || !motor_is_idle(MOTOR_AXIS_TILT)) {
            s.settle_until_ms = HAL_GetTick()
                              + ((s.home_retry == 0u) ? SCAN_HOME_SETTLE_MS
                                                      : SCAN_HOME_FINE_SETTLE_MS);
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

    case SC_PARK:
        scan_do_park();
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
