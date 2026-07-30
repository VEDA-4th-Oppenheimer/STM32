/* ============================================================================
 *  scan.c  --  스캔 시퀀스 구현
 *  담당: 이현우
 *
 *  [각도 산출 방식 — 실제 스텝 카운트]
 *    스윕 타이머(TIM1 또는 TIM3, motor.h 의 MOTOR_SWEEP_ON_TILT 선택)의
 *    업데이트 인터럽트가 STEP 펄스 1개마다 카운터를 올린다.
 *    scan 은 motor_get_pan_ddeg() / motor_get_tilt_ddeg() 만 호출하므로,
 *    강유근이 폐루프·엔코더·리밋스위치 홈을 넣어도 이 파일은 수정 불필요.
 *
 *    스윕 종료 판정은 "목표 스텝 수 도달"이 아니라 시간으로 한다.
 *    pps 는 scan_start() 가 motor_get_pan_pps() 로 타이머 레지스터에서
 *    실측하므로(상수 아님) PWM 하드웨어와 정확히 등가다.
 *
 *  [ISR / 메인루프 분리]
 *    scan_on_lidar_sample() 은 라이다 RX 완료 ISR 에서 불린다.
 *    거기서 UART 송신(HAL_UART_Transmit, ~0.95ms 블로킹)을 하면 라이다 바이트를
 *    놓치므로, ISR 에서는 링버퍼 적재만 하고 상행은 scan_tick() 에서 한다.
 * ==========================================================================*/
#include "scan.h"
#include "motor.h"
#include "uart_rpi.h"
#include "main.h"          /* HAL_GetTick */
#include <string.h>
#include <stdbool.h>

/* ---- 하드웨어 상수 -------------------------------------------------------- */
/* 스텝모터 17HS4401: 1.8도/full step = 200 step/회전 */
#define FULL_STEPS_PER_REV   200u
/* DRV8825 마이크로스텝 (MS1/MS2/MS3 물리 설정과 반드시 일치시킬 것)
 *   1/16 → 3200 step/회전, 1스텝 = 0.1125도
 *   1/32 → 6400 step/회전, 1스텝 = 0.05625도   ← 실물이 1/32 면 32 로 변경 */
#define MICROSTEP            16u
#define STEPS_PER_REV        (FULL_STEPS_PER_REV * MICROSTEP)   /* 3200 */

/* STEP PWM 주파수(pps)는 상수로 두지 않고 motor_get_pan_pps() 로 실측한다.
 *
 * ⚠️ 과거에 1000 으로 박아뒀다가 CubeMX 에서 ARR 이 999→9999 로 바뀌면서
 *   실제 100pps 가 됐는데, 빌드는 통과하고 스윕만 1/10 만에 끝나 스캔이
 *   36도에서 멈추는 버그가 있었다. 레지스터에서 읽으면 재발하지 않는다.
 *
 * 각속도 참고(1/16 마이크로스텝, 1스텝 = 0.1125도):
 *    100 pps →  11.25 도/s (한 바퀴 32초)
 *   1000 pps → 112.50 도/s (한 바퀴 3.2초)
 * 스캔 목표 100도/s 를 맞추려면 약 889pps (ARR ≈ 1124). */
static uint32_t s_pan_pps = 1u;      /* scan_start() 에서 실측값으로 갱신 */

/* 래치 링버퍼 (ISR → 메인루프).
 * 16칸은 100Hz 기준 160ms분이라, 메인루프가 잠깐만 밀려도 넘쳤다(실측: 점 유실).
 * 64칸 = 640ms 여유. v5 로 엔트리가 18B 가 되어 RAM 64*20=1.3KB (여유 있음). */
#define SCAN_FIFO_LEN        64u

/* ---- 내부 상태 ------------------------------------------------------------ */
/* ISR -> 메인루프 전달 단위. protocol.h v5 의 proto_scan_point 와 1:1 대응. */
typedef struct {
    int16_t  pan_ddeg;
    int16_t  tilt_ddeg;
    uint16_t d_mm;
    uint16_t signal_strength;
    uint32_t device_time_ms;
    uint32_t stm_ts_ms;
    uint8_t  dis_status;
    uint8_t  range_precision;
} scan_sample_t;

static volatile scan_sample_t s_fifo[SCAN_FIFO_LEN];
static volatile uint8_t  s_fifo_head = 0u;      /* ISR 이 씀   */
static volatile uint8_t  s_fifo_tail = 0u;      /* 메인이 씀   */
static volatile uint32_t s_dropped   = 0u;      /* FIFO 오버런 카운터 */
static volatile uint32_t s_sent      = 0u;      /* 상행 완료 점 수     */
/* 스캔이 방금 끝났음(1회성 플래그). main.c 가 최종 통계를 출력하고 소비한다. */
static volatile uint8_t  s_just_finished = 0u;

/* ---------------------------------------------------------------------------
 *  스캔 상태
 *
 *  SC_SWEEP  : 측정 중 — 라이다 점을 래치한다
 *  SC_SETTLE : 방향 전환 전 로터 안정 대기 — 래치하지 않는다
 *  SC_REWIND : 되감기 중 — **점을 래치하지 않는다**
 *
 *  ⚠️ SC_SETTLE 이 필요한 이유(실측): 스윕 직후 곧바로 역방향으로 급기동하면
 *    로터가 아직 진동 중이라 CCW 초반에 스텝을 잃는다. 카운터는 명령한 펄스만
 *    세므로 "갔다"고 믿어 되감기가 일찍 끝나고, **스캔마다 +1.25도(11스텝)씩
 *    시작 각도가 밀렸다**(5회 연속 스캔 상호상관 측정: 0/1.50/2.75/4.00/4.75도).
 *    한 방향 연속 스윕은 탈조가 0이었으므로(358.9도) 원인은 방향 전환뿐이다.
 *
 *  ⚠️ 되감기가 필요한 이유: 라이다 케이블이 회전축에 감겨서 한 방향으로
 *    연속 회전을 못 한다(슬립링 없음). 스윕이 끝나면 같은 스텝만큼 역회전해
 *    케이블을 풀고 시작 각도로 복귀한다.
 *    부수 효과로 **책 층마다 같은 각도에서 시작**하게 되어 z층 정렬도 맞는다.
 * ------------------------------------------------------------------------- */
typedef enum {
    SC_IDLE   = 0,
    SC_SWEEP  = 1,
    SC_SETTLE = 2,
    SC_REWIND = 3
} sc_state_t;

/* 방향 전환 전 로터 안정 대기(ms).
 * 가감속 램프(강유근 TODO)가 들어오면 더 줄이거나 제거할 수 있다. */
#define SCAN_SETTLE_MS   150u

static volatile sc_state_t s_state = SC_IDLE;
/* 스캔 전체(모든 줄) 동안 팬이 이동한 총 스텝 — 되감기 양. */
static int32_t  s_scan_total_steps  = 0;
static uint32_t s_sweep_start_tick   = 0u;      /* 현재 줄 스윕 시작 시각 */
static uint32_t s_sweep_span_ms      = 0u;      /* 현재 줄 스윕 소요 시간 */

/* 요청 파라미터 (스캔 중 불변) */
static int16_t  s_pan_start_ddeg = 0;
static int16_t  s_tilt_cur_ddeg  = 0;           /* 현재 줄의 틸트각 */
static int16_t  s_tilt_end_ddeg  = 0;
static uint16_t s_step_ddeg      = 10u;

/* ---- 내부 헬퍼 ------------------------------------------------------------ */

/* 팬 스윕 각도폭(0.1도)을 소요 시간(ms)으로 환산.
 *
 * ⚠️ 오버플로우 주의: (span * STEPS_PER_REV * 1000) 을 한 번에 곱하면
 *   3599 * 3200 * 1000 = 115억 > uint32 최대(43억) 로 넘친다.
 *   단계를 나눠 중간값을 작게 유지한다. */
static uint32_t scan_span_to_ms(uint32_t span_ddeg)
{
    /* ① 필요한 스텝 수 : 3599 * 3200 = 1151만 (uint32 안전) */
    const uint32_t steps = (span_ddeg * STEPS_PER_REV) / 3600u;
    /* ② 소요 시간(ms) : 3199 * 1000 = 320만 (uint32 안전) */
    return (s_pan_pps != 0u) ? ((steps * 1000u) / s_pan_pps) : 0u;
}

/* 한 줄(팬 스윕) 시작 */
static void scan_begin_line(void)
{
    /* 이전 줄에서 이동한 스텝을 되감기 총량에 누적한 뒤 카운터를 0 으로. */
    s_scan_total_steps += motor_get_pan_steps();

    s_sweep_start_tick = HAL_GetTick();
    motor_pan_reset_steps();      /* 이 줄의 시작을 각도 0 기준으로 */
    motor_pan_sweep_start();
}

/* ---- 공개 API ------------------------------------------------------------- */

void scan_init(void)
{
    s_fifo_head = 0u;
    s_fifo_tail = 0u;
    s_dropped   = 0u;
    s_state     = SC_IDLE;
}

void scan_start(const struct proto_scan_start *ss)
{
    /* 중복 시작 방지 (MISRA 15.5: 단일 exit) */
    if ((ss != NULL) && (s_state == SC_IDLE)) {
        int32_t span;

        /* 스윕 시간 계산 전에 실제 STEP 주파수를 타이머에서 읽어온다.
         * (CubeMX 에서 ARR 이 바뀌어도 스캔 범위가 틀어지지 않도록) */
        s_pan_pps = motor_get_pan_pps();

        s_pan_start_ddeg = ss->pan_start_ddeg;
        s_tilt_cur_ddeg  = ss->tilt_start_ddeg;
        s_tilt_end_ddeg  = ss->tilt_end_ddeg;
        s_step_ddeg      = (ss->step_ddeg > 0u) ? ss->step_ddeg : 10u;

        /* 팬 스윕 폭: end - start (0 이하면 한 바퀴로 간주) */
        span = (int32_t)ss->pan_end_ddeg - (int32_t)ss->pan_start_ddeg;
        if (span <= 0) {
            span += 3600;             /* 0→0 또는 역순이면 360도 한 바퀴 */
        }
        s_sweep_span_ms = scan_span_to_ms((uint32_t)span);

        s_fifo_head = 0u;
        s_fifo_tail = 0u;
        s_dropped   = 0u;
        s_sent      = 0u;
        s_scan_total_steps = 0;
        s_state     = SC_SWEEP;

        scan_begin_line();
    }
}

void scan_stop(void)
{
    s_state = SC_IDLE;
    s_scan_total_steps = 0;
    motor_pan_stop();
}

void scan_get_stats(uint32_t *sent, uint32_t *dropped)
{
    if (sent != NULL)    { *sent    = s_sent; }
    if (dropped != NULL) { *dropped = s_dropped; }
}

uint8_t scan_is_busy(void)
{
    return (s_state != SC_IDLE) ? 1u : 0u;
}

uint8_t scan_take_finished(void)
{
    const uint8_t f = s_just_finished;
    s_just_finished = 0u;      /* 1회성 소비 */
    return f;
}

/* ISR 문맥: 각도 래치 + 링버퍼 적재만. 절대 블로킹 금지. */
void scan_on_lidar_sample(const lidar_sample_t *smp)
{
    if ((smp != NULL) && (s_state == SC_SWEEP)) {   /* 되감기 중 래치 안 함 */
        const uint8_t head = s_fifo_head;
        const uint8_t next = (uint8_t)((head + 1u) % SCAN_FIFO_LEN);

        if (next == s_fifo_tail) {
            s_dropped++;              /* 가득 참 — 메인루프가 밀림 */
        } else {
            /* 실제 STEP 펄스 카운트 기반 각도 (스윕 타이머 ISR 이 센 값).
             * 스윕 시작각 + 이동각으로 절대각 산출. */
            int32_t pan = (int32_t)s_pan_start_ddeg
                        + (int32_t)motor_get_pan_ddeg();
            pan %= 3600;

            s_fifo[head].pan_ddeg        = (int16_t)pan;
            s_fifo[head].tilt_ddeg       = s_tilt_cur_ddeg;  /* 1축이면 고정 */
            s_fifo[head].d_mm            = (uint16_t)smp->raw_mm;
            s_fifo[head].signal_strength = smp->signal_strength;
            s_fifo[head].device_time_ms  = smp->device_time_ms;
            s_fifo[head].stm_ts_ms       = HAL_GetTick();   /* 래치 시각 */
            s_fifo[head].dis_status      = smp->dis_status;
            s_fifo[head].range_precision = smp->range_precision;
            s_fifo_head = next;
        }
    }
}

void scan_tick(void)
{
    /* 되감기 타이밍 — 이 함수 안에서만 쓰이므로 블록 스코프(MISRA 8.9) */
    static uint32_t s_rewind_start_tick = 0u;
    static uint32_t s_rewind_limit_ms   = 0u;   /* 안전망: 초과 시 강제 종료 */
    static uint32_t s_settle_start_tick = 0u;   /* 방향 전환 전 안정 대기 */

    /* ① 래치된 점 상행 (UART 블로킹은 여기서만) */
    while (s_fifo_tail != s_fifo_head) {
        const uint8_t tail = s_fifo_tail;
        const scan_sample_t s = {
            .pan_ddeg        = s_fifo[tail].pan_ddeg,
            .tilt_ddeg       = s_fifo[tail].tilt_ddeg,
            .d_mm            = s_fifo[tail].d_mm,
            .signal_strength = s_fifo[tail].signal_strength,
            .device_time_ms  = s_fifo[tail].device_time_ms,
            .stm_ts_ms       = s_fifo[tail].stm_ts_ms,
            .dis_status      = s_fifo[tail].dis_status,
            .range_precision = s_fifo[tail].range_precision,
        };
        s_fifo_tail = (uint8_t)((tail + 1u) % SCAN_FIFO_LEN);
        {
            struct proto_scan_point pt;
            pt.pan_ddeg        = s.pan_ddeg;
            pt.tilt_ddeg       = s.tilt_ddeg;
            pt.d_mm            = s.d_mm;
            pt.signal_strength = s.signal_strength;
            pt.device_time_ms  = s.device_time_ms;
            pt.stm_ts_ms       = s.stm_ts_ms;
            pt.dis_status      = s.dis_status;
            pt.range_precision = s.range_precision;
            uart_rpi_send_scan_point(&pt);
        }
        s_sent++;
    }

    /* ② 스윕 완료 판정 (MISRA 15.5: 단일 exit) */
    if ((s_state == SC_SWEEP) &&
        ((HAL_GetTick() - s_sweep_start_tick) >= s_sweep_span_ms)) {

        /* 다음 줄이 있는가? (1축이면 tilt_cur == tilt_end 라 즉시 종료) */
        const int32_t remain = (int32_t)s_tilt_end_ddeg - (int32_t)s_tilt_cur_ddeg;
        const int32_t step   = (int32_t)s_step_ddeg;

        if ((remain >= step) || (remain <= -step)) {
            /* ── 2축: 틸트 한 칸 이동 후 다음 줄 ──
             * ⚠️ motor_tilt_step() 은 현재 스텁(빈 함수). 강유근이 채우면
             *    이 파일 수정 없이 2축 스캔이 된다.
             * ⚠️ 여러 줄을 같은 방향으로 돌면 케이블이 줄 수만큼 감긴다.
             *    2축 확장 시에는 줄마다 방향을 뒤집는 serpentine 이 정석. */
            s_tilt_cur_ddeg = (int16_t)(s_tilt_cur_ddeg +
                                        ((remain > 0) ? step : -step));
            motor_tilt_step(s_tilt_cur_ddeg);
            scan_begin_line();
        } else {
            /* ── 마지막 줄 → 측정 종료, 되감기 시작 ── */
            motor_pan_stop();
            s_scan_total_steps += motor_get_pan_steps();   /* 마지막 줄분 누적 */

            uart_rpi_send_scan_done();      /* 데이터는 여기서 끝 (RPi 는 즉시 마감) */

            if (s_scan_total_steps > 0) {
                /* 로터 링잉이 잦아들 때까지 대기 후 역방향 (SC_SETTLE).
                 * 곧바로 뒤집으면 CCW 초반에 스텝을 잃어 되감기가 짧아진다. */
                s_settle_start_tick = HAL_GetTick();
                s_state = SC_SETTLE;
            } else {
                s_state = SC_IDLE;
                s_just_finished = 1u;
            }
        }
    }
    /* ③ 안정 대기 완료 → 역방향 기동 */
    else if ((s_state == SC_SETTLE) &&
             ((HAL_GetTick() - s_settle_start_tick) >= SCAN_SETTLE_MS)) {
        /* 안전망: 되감기가 스윕보다 오래 걸릴 수 없다(같은 속도). */
        s_rewind_limit_ms   = (s_sweep_span_ms * 2u) + 1000u;
        s_rewind_start_tick = HAL_GetTick();
        motor_pan_reset_steps();                   /* 0 에서 음수로 내려감 */
        motor_pan_sweep_start_dir(MOTOR_DIR_CCW);
        s_state = SC_REWIND;
    }
    /* ④ 되감기 완료 판정 — 시작 스텝만큼 역회전했거나 시간 초과 */
    else if (s_state == SC_REWIND) {
        const bool done    = (motor_get_pan_steps() <= -s_scan_total_steps);
        const bool timeout = ((HAL_GetTick() - s_rewind_start_tick) >= s_rewind_limit_ms);

        if (done || timeout) {
            motor_pan_stop();
            motor_pan_reset_steps();        /* 시작 각도 = 0 재정의 */
            s_scan_total_steps = 0;
            s_state = SC_IDLE;
            s_just_finished = 1u;           /* main.c 가 최종 통계를 1회 출력 */
        }
    } else {
        /* 대기 상태 — 할 일 없음 */
    }
}
