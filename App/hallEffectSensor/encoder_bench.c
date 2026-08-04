/* ============================================================================
 *  encoder_bench.c  --  MT6701 엔코더 벤치 판독 구현 (임시 브링업 도구)
 *  담당: 이현우.  목적·사용법은 encoder_bench.h 상단 참조.
 * ==========================================================================*/
#include "encoder_bench.h"

#if ENCODER_BENCH_TEST

#include "motor.h"
#include "hallEffectSensor.h"
#include <stdbool.h>
#include <math.h>

/* MISRA-C:2012 Rule 21.6 (표준 I/O 금지) deviation.
 *   이 파일은 벤치 브링업 전용이고, ENCODER_BENCH_TEST=0 이면 통째로
 *   컴파일아웃되어 릴리즈 빌드에는 stdio 가 아예 남지 않는다.
 *   App/uart_rpi 의 DBG() 와 동일한 처리 방식이다. */
/* cppcheck-suppress misra-c2012-21.6 ; 브링업 빌드 한정 stdio deviation */
#include <stdio.h>
/* cppcheck-suppress misra-c2012-21.6 ; 브링업 빌드 한정 printf deviation */
#define BP(...)   ((void)printf(__VA_ARGS__))

/* 출력 주기(ms). 사람이 읽는 용도라 10Hz 면 충분하다. */
#define ENCODER_BENCH_PERIOD_MS   100u

/* 축별 누적 상태. 손으로 돌렸을 때 "얼마나" 움직였는지 보려면 기준이 필요하다. */
struct bench_axis {
    bool     have_ref;      /* 첫 성공 판독을 기준으로 잡았나 */
    float    ref_deg;
    uint32_t err_run;       /* 연속 실패 횟수 */
    uint32_t ok_count;
};

static struct bench_axis s_ax[MOTOR_AXIS_COUNT];

/* -180..+180 로 감싼 상대각. 0 을 넘나들 때 359도 점프로 보이지 않게 한다.
 * ⚠️ 루프로 감으면 MISRA 14.1(부동소수 루프 카운터)·17.8(인자 수정)에 걸리고,
 *   입력이 비정상적으로 클 때 반복 횟수도 예측이 안 된다. fmodf 로 한 번에. */
static float wrap180(float d)
{
    float r = fmodf(d + 180.0f, 360.0f);

    if (r < 0.0f) {
        r += 360.0f;
    }
    return r - 180.0f;
}

static void bench_banner(void)
{
    BP("\r\n");
    BP("=== MT6701 엔코더 벤치 판독 (ENCODER_BENCH_TEST=1) ===\r\n");
    BP("  Pan  = I2C3 (PA8 SCL / PC9 SDA)\r\n");
    BP("  Tilt = I2C1 (PB8 SCL / PB9 SDA)\r\n");
    BP("\r\n");
    BP("  [사용법]\r\n");
    BP("   1) 축을 손으로 돌려 숫자가 따라 움직이면 배선 정상.\r\n");
    BP("   2) 축을 기준 자세에 맞춘다 (팬=기준 방위 / 틸트=바닥 nadir).\r\n");
    BP("   3) 그때의 OFFSET 값을 motor.h 의\r\n");
    BP("      MOTOR_PAN_ZERO_OFFSET_DEG / MOTOR_TILT_ZERO_OFFSET_DEG 에 적는다.\r\n");
    BP("   4) ENCODER_BENCH_TEST 와 SCAN_NO_ENCODER 를 0 으로 되돌리고 재빌드.\r\n");
    BP("\r\n");
    BP("  raw   = 14비트 원본(0~16383, 1 count = 0.022도)\r\n");
    BP("  Zref  = 벤치 시작 시점 대비 이동량 (기구 가동범위 확인용)\r\n");
    BP("  mech  = 현재 영점 상수를 적용한 기구각 (이게 0 이면 영점이 맞은 것)\r\n");
    BP("--------------------------------------------------------------\r\n");
}

/* 한 축 출력. 실패는 조용히 넘기지 않는다 — 배선 문제가 이 도구의 1순위
 * 용도인데 실패를 안 찍으면 "값이 안 변한다" 로만 보여 원인을 못 찾는다. */
static void bench_axis_print(motor_axis_t ax, const char *name)
{
    Encoder_t enc;
    struct bench_axis *st = &s_ax[ax];

    if (motor_read_encoder(ax, &enc) != HAL_OK) {
        st->err_run++;
        BP("%-4s I2C 실패 (연속 %lu회)                          ",
           name, (unsigned long)st->err_run);
    } else {
        st->err_run = 0u;
        st->ok_count++;
        if (!st->have_ref) {
            st->ref_deg  = enc.degree;
            st->have_ref = true;
        }

        /* 영점 상수를 적용하면 기구각이 몇 도가 되는지. 축을 기준 자세에
         * 두었을 때 이 값이 0 에 가까우면 상수가 맞은 것이다. */
        {
            const int32_t mech_pulse = motor_encoder_deg_to_pulse(ax, enc.degree);
            const int32_t mech_ddeg  = motor_pulse_to_ddeg(mech_pulse);

            BP("%-4s raw=%5u %7.2fdeg  Zref=%+7.2f  mech=%+6.1f  ",
               name, (unsigned)enc.raw_angle, (double)enc.degree,
               (double)wrap180(enc.degree - st->ref_deg),
               (double)mech_ddeg / 10.0);
        }
    }
}

void encoder_bench_run(void)
{
    /* 이 둘은 이 함수에서만 쓰므로 블록 스코프에 둔다 (MISRA 8.9). */
    static bool     s_banner_done = false;
    static uint32_t s_last_ms     = 0u;

    const uint32_t now = HAL_GetTick();

    if (!s_banner_done) {
        bench_banner();
        s_banner_done = true;
        s_last_ms     = now;
    }

    /* ⚠️ 부호 없는 뺄셈이라 tick 랩어라운드(49.7일)에도 경과시간이 옳게
     *   나온다. 부호 캐스팅(MISRA 10.8)이 필요 없는 형태다. */
    if ((now - s_last_ms) >= ENCODER_BENCH_PERIOD_MS) {
        s_last_ms = now;

        BP("[ENC] ");
        bench_axis_print(MOTOR_AXIS_PAN,  "PAN");
        BP("| ");
        bench_axis_print(MOTOR_AXIS_TILT, "TILT");

        /* 기준 자세에 맞췄을 때 그대로 옮겨 적을 값. degree 를 그대로 쓰는
         * 이유는 규약이 "기구각 = 실측각 − offset" 이라, 기준 자세에서는
         * 실측각 == offset 이기 때문이다. */
        if (s_ax[MOTOR_AXIS_PAN].have_ref && s_ax[MOTOR_AXIS_TILT].have_ref) {
            BP("\r\n      → OFFSET 후보  PAN %.2ff / TILT %.2ff",
               (double)s_ax[MOTOR_AXIS_PAN].ref_deg,
               (double)s_ax[MOTOR_AXIS_TILT].ref_deg);
        }
        BP("\r\n");
    }
}

#else  /* ENCODER_BENCH_TEST == 0 */

void encoder_bench_run(void)
{
    /* 꺼짐 — 호출부를 지우지 않아도 되도록 빈 함수를 남긴다. */
}

#endif /* ENCODER_BENCH_TEST */
