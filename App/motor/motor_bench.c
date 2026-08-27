/* ============================================================================
 *  motor_bench.c  --  모터 축 단독 왕복 테스트 구현 (임시 브링업 도구)
 *  담당: 이현우.  목적·판별법은 motor_bench.h 상단 참조.
 * ==========================================================================*/
#include "motor_bench.h"

#if MOTOR_BENCH_TEST

#include "main.h"
#include "motor.h"
#include "bench_common.h"
#include <stdbool.h>

/* MISRA-C:2012 Rule 21.6 (표준 I/O 금지) deviation — 브링업 빌드 한정.
 * MOTOR_BENCH_TEST=0 이면 파일이 통째로 컴파일아웃되어 릴리즈엔 안 남는다. */
/* cppcheck-suppress misra-c2012-21.6 ; 브링업 빌드 한정 stdio deviation */
#include <stdio.h>
/* cppcheck-suppress misra-c2012-21.6 ; 브링업 빌드 한정 printf deviation */
#define MB(...)   ((void)printf(__VA_ARGS__))


#define MB_PRINT_MS   500u

static const char *pin_str(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? "H(3.3V)" : "L(0V)";
}

/* MOTOR_BENCH_AXIS 로 고른 축만 구동 대상이다. */
static bool axis_driven(motor_axis_t ax)
{
#if   MOTOR_BENCH_AXIS == 1
    return (ax == MOTOR_AXIS_PAN);
#elif MOTOR_BENCH_AXIS == 2
    return (ax == MOTOR_AXIS_TILT);
#else
    (void)ax;
    return true;
#endif
}

static const char *axis_mode_str(void)
{
#if   MOTOR_BENCH_AXIS == 1
    return "팬만";
#elif MOTOR_BENCH_AXIS == 2
    return "틸트만";
#else
    return "두 축 동시";
#endif
}

static void mb_banner(void)
{
    MB("\r\n");
    MB("=== 모터 왕복 테스트 (MOTOR_BENCH_TEST=1) ===\r\n");
    MB("  스캔·라이다·데몬을 우회하고 축에 직접 목표를 준다.\r\n");
    MB("  ** 구동 축: %s **   (MOTOR_BENCH_AXIS  0=둘다 1=팬만 2=틸트만)\r\n",
       axis_mode_str());
    MB("  쉬는 축도 EN 은 LOW 를 유지한다 — 손으로 만져 전류 유무를 확인할 것.\r\n");
    MB("  진폭 +-%d도 / %lums 마다 방향 반전\r\n",
       MOTOR_BENCH_SWING_DEG, (unsigned long)MOTOR_BENCH_FLIP_MS);
    MB("\r\n");
    MB("  [판별법]\r\n");
    MB("   pulse 가 오른다 + 축이 안 움직인다\r\n");
    MB("       -> 펌웨어는 결백. STEP 핀은 실제로 토글되고 있다.\r\n");
    MB("          VMOT / 드라이버 모듈 / Vref / nSLEEP·nRESET / 제어선을 볼 것.\r\n");
    MB("   pulse 가 안 오른다\r\n");
    MB("       -> 타이머·ISR·목표설정 문제. 펌웨어를 봐야 한다.\r\n");
    MB("\r\n");
    MB("  한 축만 돌리면 소리·진동으로 어느 쪽인지 확실히 구분된다.\r\n");
    MB("  쉬는 축은 d 가 계속 0 이어야 정상이다.\r\n");
    MB("\r\n");
    MB("  EN 은 active-low — 구동 중엔 **L(0V)** 이어야 한다.\r\n");
    MB("  H 로 읽히면 enable 이 안 걸린 것(배선이 옛 핀맵일 가능성).\r\n");
    MB("  핀: PAN STEP=PB13 DIR=PB15 EN=PB1 / TILT STEP=PA6 DIR=PA7 EN=PB12\r\n");
    MB("--------------------------------------------------------------\r\n");
}

void motor_bench_run(void)
{
    static bool     s_init_done = false;
    static bool     s_positive  = true;
    static uint32_t s_last_flip = 0u;
    static uint32_t s_last_prn  = 0u;

    const uint32_t now = HAL_GetTick();

    if (!s_init_done) {
        mb_banner();
        motor_enable();            /* 전류 투입 — 이게 없으면 축이 헐렁하다 */
        s_init_done = true;
        s_last_flip = now;
        s_last_prn  = now;
    }

    /* 방향 반전. 절대각으로 목표를 계산한다 — 증분으로 누적하면 정수 절삭이
     * 쌓여 왕복 중심이 서서히 밀린다(§17-7 에서 실제로 겪은 함정). */
    if ((now - s_last_flip) >= MOTOR_BENCH_FLIP_MS) {
        const int32_t ddeg = (int32_t)MOTOR_BENCH_SWING_DEG * 10;
        const int32_t tgt  = motor_ddeg_to_pulse(s_positive ? ddeg : -ddeg);

        /* 쉬는 축은 목표를 **현재 위치**로 둔다. 0 으로 두면 그쪽으로 달려가
         * 버리고, 아예 안 건드리면 이전 목표가 남아 계속 움직인다. */
        for (motor_axis_t ax = 0; ax < MOTOR_AXIS_COUNT; ax++) {
            motor_set_target(ax, axis_driven(ax) ? tgt : motor_get_pulse(ax));
        }
        s_positive  = !s_positive;
        s_last_flip = now;
        MB("  -- 목표 반전 -> %+ld.0도 (pulse %+ld)  [%s]\r\n",
           (long)(s_positive ? -ddeg : ddeg) / 10, (long)tgt, axis_mode_str());
    }

    if ((now - s_last_prn) >= MB_PRINT_MS) {
        /* 직전 표본. static 이라 블록 안에 둬도 호출 간에 값이 유지된다
         * (MISRA 8.9 — 쓰이는 곳에 가장 가깝게). */
        static int32_t s_pan_prev  = 0;
        static int32_t s_tilt_prev = 0;

        const int32_t pan  = motor_get_pulse(MOTOR_AXIS_PAN);
        const int32_t tilt = motor_get_pulse(MOTOR_AXIS_TILT);

        /* 복합식을 그대로 넓은 형으로 캐스팅하면 MISRA 10.8 이라 중간 변수를 쓴다. */
        const int32_t d_pan  = pan  - s_pan_prev;
        const int32_t d_tilt = tilt - s_tilt_prev;

        s_last_prn = now;

        /* delta 가 0 이면 그 축은 펄스가 안 나가고 있다는 뜻이다. */
        MB("[MOT] PAN  pulse=%+6ld (d%+4ld) EN=%-7s | "
           "TILT pulse=%+6ld (d%+4ld) EN=%-7s\r\n",
           (long)pan,  (long)d_pan,
           pin_str(PAN_EN_GPIO_Port,  PAN_EN_Pin),
           (long)tilt, (long)d_tilt,
           pin_str(TILT_EN_GPIO_Port, TILT_EN_Pin));

        s_pan_prev  = pan;
        s_tilt_prev = tilt;
    }

    BENCH_SERVICE();
}

#else  /* MOTOR_BENCH_TEST == 0 */

void motor_bench_run(void)
{
    /* 꺼짐 — 호출부를 지우지 않아도 되도록 빈 함수를 남긴다. */
}

#endif /* MOTOR_BENCH_TEST */
