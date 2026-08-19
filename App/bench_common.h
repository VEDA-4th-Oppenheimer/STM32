/* ============================================================================
 *  bench_common.h  --  브링업 테스트 도구 공용 정의
 * ----------------------------------------------------------------------------
 *  담당: 이현우
 *
 *  encoder_bench / motor_bench / lidar_bench 가 공유한다. 각자 선언하면
 *  MISRA 8.5(외부 객체는 한 파일에서만 선언) 에 걸리고, 무엇보다 아래
 *  BENCH_SERVICE() 를 빠뜨리는 사고가 반복된다.
 *
 *  주의: 왜 이 매크로가 필요한가 — 실제로 겪은 일이다.
 *    엔코더 벤치가 메인루프를 수 초씩 붙잡는 동안 워치독은 챙겼는데
 *    **uart_rpi_process() 를 빠뜨렸다**. PONG 은 ISR 이 아니라 그 함수가
 *    보내기 때문에, RPi 데몬이 300ms 무응답으로 link_dead → DISARM 을 띄웠고
 *    원인을 찾는 데 한참 걸렸다. 블로킹 구간이 있는 브링업 도구는 반드시
 *    이 둘을 함께 돌려야 한다.
 *
 *  주의: 이 도구들은 전부 기구 브링업이 끝나면 지운다. 그때 이 헤더도 같이.
 * ==========================================================================*/
#ifndef BENCH_COMMON_H
#define BENCH_COMMON_H

#include "stm32f4xx_hal.h"
#include "uart_rpi.h"

/* main.c 가 소유한다(CubeMX 생성). 브링업 도구가 워치독을 직접 먹이려면 필요. */
extern IWDG_HandleTypeDef hiwdg;

/* 워치독 먹이기 + RPi 링크 유지. 블로킹 루프 안에서 주기적으로 부를 것. */
#define BENCH_SERVICE()                     \
    do {                                    \
        (void)HAL_IWDG_Refresh(&hiwdg);     \
        uart_rpi_process();                 \
    } while (0)

#endif /* BENCH_COMMON_H */
