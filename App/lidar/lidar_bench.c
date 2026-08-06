/* ============================================================================
 *  lidar_bench.c  --  TOFSense-F2P 단독 수신 테스트 구현 (임시 브링업 도구)
 *  담당: 이현우.  목적·판별법은 lidar_bench.h 상단 참조.
 * ==========================================================================*/
#include "lidar_bench.h"

#if LIDAR_BENCH_TEST

#include "lidar.h"
#include "bench_common.h"
#include <stdbool.h>

/* MISRA-C:2012 Rule 21.6 (표준 I/O 금지) deviation — 브링업 빌드 한정.
 * LIDAR_BENCH_TEST=0 이면 파일이 통째로 컴파일아웃되어 릴리즈엔 안 남는다. */
/* cppcheck-suppress misra-c2012-21.6 ; 브링업 빌드 한정 stdio deviation */
#include <stdio.h>
/* cppcheck-suppress misra-c2012-21.6 ; 브링업 빌드 한정 printf deviation */
#define LB(...)   ((void)printf(__VA_ARGS__))


static void lb_banner(void)
{
    LB("\r\n");
    LB("=== 라이다 수신 테스트 (LIDAR_BENCH_TEST=1) ===\r\n");
    LB("  TOFSense-F2P / USART6 (PC6=TX, PC7=RX)\r\n");
    LB("  라이다의 TX 가 STM32 PC7 로 들어와야 한다.\r\n");
    LB("\r\n");
    LB("  [판별법]\r\n");
    LB("   bytes=0                 -> 물리 배선. TX/RX 뒤바뀜·GND·라이다 전원\r\n");
    LB("   bytes>0 이고 frames=0   -> 보레이트 불일치 또는 출력모드가 NLink 아님\r\n");
    LB("   csum_err 가 같이 증가   -> 신호 지저분. 선 길이·간섭·접촉 불량\r\n");
    LB("   drops 가 증가           -> 메인루프 지연으로 큐(8칸=80ms) 넘침\r\n");
    LB("   frames 증가, drops=0    -> 라이다 정상\r\n");
    LB("\r\n");
    LB("  rate 가 50Hz 근처면 아직 100Hz 설정 전이다(F2P 기본 50Hz).\r\n");
    LB("  스캔 샘플 간격 = 틸트속도 / rate 이므로 반드시 100 이어야 한다.\r\n");
    LB("  status 는 1=valid. 손을 앞뒤로 움직여 d_mm 이 따라오면 정상.\r\n");
    LB("--------------------------------------------------------------\r\n");
}

void lidar_bench_run(void)
{
    static bool     s_init_done = false;
    static uint32_t s_last_ms   = 0u;

    const uint32_t now = HAL_GetTick();

    if (!s_init_done) {
        lb_banner();
        s_init_done = true;
        s_last_ms   = now;
    }

    if ((now - s_last_ms) >= LIDAR_BENCH_PERIOD_MS) {
        /* 직전 표본 — static 이라 블록 안에 둬도 호출 간에 값이 유지된다. */
        static uint32_t s_prev_bytes  = 0u;
        static uint32_t s_prev_frames = 0u;
        static uint32_t s_prev_csum   = 0u;
        static uint32_t s_prev_drops  = 0u;

        const uint32_t bytes  = lidar_get_rx_bytes();
        const uint32_t frames = lidar_get_frame_count();
        const uint32_t csum   = lidar_get_csum_errors();
        const uint32_t drops  = lidar_get_queue_drops();

        /* 실제 경과로 나눠야 정확하다 — 메인루프가 밀리면 주기가 늘어난다. */
        const uint32_t elapsed = now - s_last_ms;
        const uint32_t d_bytes  = bytes  - s_prev_bytes;
        const uint32_t d_frames = frames - s_prev_frames;
        const uint32_t d_csum   = csum   - s_prev_csum;
        const uint32_t d_drops  = drops  - s_prev_drops;
        const uint32_t rate     = (elapsed > 0u) ? ((d_frames * 1000u) / elapsed)
                                                 : 0u;

        lidar_frame_t f;

        s_last_ms = now;

        LB("[LID] bytes=%lu(+%lu) frames=%lu(+%lu) rate=%luHz "
           "csum_err=%lu(+%lu) drops=%lu(+%lu)\r\n",
           (unsigned long)bytes,  (unsigned long)d_bytes,
           (unsigned long)frames, (unsigned long)d_frames,
           (unsigned long)rate,
           (unsigned long)csum,   (unsigned long)d_csum,
           (unsigned long)drops,  (unsigned long)d_drops);

        if (lidar_get_last_frame(&f)) {
            LB("      d=%lumm  strength=%u  status=%u%s  precision=0x%02X"
               "  dev_time=%lums\r\n",
               (unsigned long)f.d_mm, (unsigned)f.signal_strength,
               (unsigned)f.dis_status,
               (f.dis_status == 1u) ? "(valid)" : "(INVALID)",
               (unsigned)f.range_precision,
               (unsigned long)f.device_time_ms);
        } else {
            LB("      (아직 유효 프레임 없음)\r\n");
        }

        s_prev_bytes  = bytes;
        s_prev_frames = frames;
        s_prev_csum   = csum;
        s_prev_drops  = drops;
    }

    BENCH_SERVICE();
}

#else  /* LIDAR_BENCH_TEST == 0 */

void lidar_bench_run(void)
{
    /* 꺼짐 — 호출부를 지우지 않아도 되도록 빈 함수를 남긴다. */
}

#endif /* LIDAR_BENCH_TEST */
