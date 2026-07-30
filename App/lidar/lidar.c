/* ============================================================================
 *  lidar.c  --  TOFSense-F2P UART 수신 드라이버 구현
 *  담당: 송영빈 (원 구현) / 이현우 (각도 래치 + 샘플 큐 확장)
 *  계약과 데이터 흐름은 lidar.h 상단 참조.
 * ==========================================================================*/
#include "lidar.h"
#include "lidar_parser.h"
#include "scan.h"
#include <stddef.h>

/* 패킷 수신 진행 상태 */
typedef enum {
    STATE_WAIT_HEADER = 0,
    STATE_WAIT_FUNC_MARK,
    STATE_COLLECT_BODY
} lidar_rx_state_t;

/* 프레임 1개 + 완성 순간의 각도. ISR 이 채우고 메인루프가 소비한다. */
typedef struct {
    lidar_frame_t f;
    int16_t       pan_ddeg;
    int16_t       tilt_ddeg;
} lidar_sample_t;

/* ===== 내부 정적 변수 ===== */
static UART_HandleTypeDef *g_huart = NULL;

/* cppcheck-suppress misra-c2012-8.9 ; HAL_UART_Receive_IT 이 주소를 물고 있어 파일 스코프 필요 */
static uint8_t           g_rx_byte = 0U;
static uint8_t           g_idx     = 0U;
static volatile uint16_t g_raw_dist_mm = 0U;

/* SPSC 링버퍼: 생산자 = RX ISR, 소비자 = 메인루프 lidar_process().
 * head/tail 이 각각 한쪽에서만 증가하고 8비트 단일 워드 접근이라 락이 없어도
 * 안전하다. 깊이가 2의 거듭제곱이므로 마스킹으로 감싼다. */
static lidar_sample_t     g_q[LIDAR_SAMPLE_QUEUE_LEN];
static volatile uint8_t   g_q_head = 0U;   /* ISR 이 증가   */
static volatile uint8_t   g_q_tail = 0U;   /* 메인루프 증가 */

/* 진단 카운터 */
static volatile uint32_t g_frames     = 0U;
static volatile uint32_t g_csum_err   = 0U;
static volatile uint32_t g_queue_drop = 0U;

/* ===== 내부 함수 선언 ===== */
static void lidar_reset_rx(void);
static void lidar_rearm_it(void);
static void lidar_push_sample(const lidar_frame_t *f);

/* ---------------------------------------------------------------------------
 *  초기화
 * ------------------------------------------------------------------------- */
void lidar_init(UART_HandleTypeDef *huart)
{
    if (huart != NULL)
    {
        g_huart  = huart;
        g_q_head = 0U;
        g_q_tail = 0U;
        lidar_reset_rx();
        lidar_rearm_it();
    }
}

uint16_t lidar_get_distance_mm(void)
{
    return g_raw_dist_mm;
}

uint32_t lidar_get_frame_count(void) { return g_frames; }
uint32_t lidar_get_csum_errors(void) { return g_csum_err; }
uint32_t lidar_get_queue_drops(void) { return g_queue_drop; }

/* ---------------------------------------------------------------------------
 *  UART 수신 콜백 (1바이트마다) — ISR 컨텍스트
 * ------------------------------------------------------------------------- */
/* cppcheck-suppress constParameterPointer ; HAL 콜백 ABI 고정 */
void lidar_on_rx_cplt(UART_HandleTypeDef *huart)
{
    if ((g_huart != NULL) && (huart != NULL))
    {
        if (huart->Instance == g_huart->Instance)
        {
            static uint8_t g_buf[LIDAR_PACKET_SIZE];
            const lidar_rx_state_t state = (g_idx == 0U) ? STATE_WAIT_HEADER
                                          : ((g_idx == 1U) ? STATE_WAIT_FUNC_MARK
                                          : STATE_COLLECT_BODY);

            switch (state)
            {
                case STATE_WAIT_HEADER:
                    if (lidar_parser_is_header(g_rx_byte) == true)
                    {
                        g_buf[g_idx] = g_rx_byte;
                        g_idx++;
                    }
                    break;

                case STATE_WAIT_FUNC_MARK:
                    if (lidar_parser_is_func_mark(g_rx_byte) == true)
                    {
                        g_buf[g_idx] = g_rx_byte;
                        g_idx++;
                    }
                    else
                    {
                        lidar_reset_rx();   /* 헤더 오탐 → 재동기화 */
                    }
                    break;

                case STATE_COLLECT_BODY:
                    g_buf[g_idx] = g_rx_byte;
                    g_idx++;

                    if ((uint32_t)g_idx >= (uint32_t)LIDAR_PACKET_SIZE)
                    {
                        lidar_frame_t f;
                        if (lidar_parser_parse(g_buf, &f) != false)
                        {
                            g_frames++;
                            g_raw_dist_mm = (uint16_t)f.d_mm;
                            /* ★ 각도를 여기서 잡는다. 이 시점이 프레임이
                             *   완성된 순간이라 거리와 시간축이 맞는다. */
                            lidar_push_sample(&f);
                        }
                        else
                        {
                            g_csum_err++;
                        }
                        g_idx = 0U;
                    }
                    break;

                default:
                    lidar_reset_rx();
                    break;
            }

            /* 오버플로우 가드 */
            if ((size_t)g_idx >= sizeof(g_buf))
            {
                g_idx = 0U;
            }

            lidar_rearm_it();
        }
    }
}

void lidar_on_error(UART_HandleTypeDef *huart)
{
    if ((g_huart != NULL) && (huart != NULL))
    {
        if (huart->Instance == g_huart->Instance)
        {
            /* cppcheck-suppress misra-c2012-14.4 ; ST HAL 매크로(do{...}while(0U)) 관용구 deviation */
            __HAL_UART_CLEAR_PEFLAG(huart);
            lidar_reset_rx();
            lidar_rearm_it();
        }
    }
}

/* ---------------------------------------------------------------------------
 *  메인루프 소비
 * ------------------------------------------------------------------------- */
void lidar_process(void)
{
    while (g_q_tail != g_q_head)
    {
        const lidar_sample_t *s = &g_q[g_q_tail];

        /* 거리는 24비트라 uint32 로 받았지만 프로토콜은 uint16(mm) 이다.
         * F2P 사양 상한이 25m 이므로 정상 값은 항상 들어간다. 범위를 넘는
         * 값은 센서 오류이므로 잘라 담되 dis_status 를 그대로 올려 데몬이
         * 판정하게 한다 — 여기서 버리지 않는다. */
        const uint16_t d_mm = (s->f.d_mm > 0xFFFFU) ? 0xFFFFU
                                                    : (uint16_t)s->f.d_mm;

        scan_submit_sample(s->pan_ddeg, s->tilt_ddeg, d_mm,
                           s->f.signal_strength, s->f.device_time_ms,
                           s->f.dis_status, s->f.range_precision);

        g_q_tail = (uint8_t)((g_q_tail + 1U) & (LIDAR_SAMPLE_QUEUE_LEN - 1U));
    }
}

/* ---------------------------------------------------------------------------
 *  내부
 * ------------------------------------------------------------------------- */
static void lidar_push_sample(const lidar_frame_t *f)
{
    const uint8_t next = (uint8_t)((g_q_head + 1U) & (LIDAR_SAMPLE_QUEUE_LEN - 1U));

    if (next == g_q_tail)
    {
        /* 큐가 꽉 찼다 = 메인루프가 80ms 이상 밀렸다는 뜻.
         * 가장 오래된 것을 밀어내지 않고 새 것을 버린다 — 이미 들어간
         * 샘플들은 각도가 짝지어져 있고, 순서를 흐트러뜨리면 데몬이 격자에
         * 넣을 때 더 헷갈린다. */
        g_queue_drop++;
    }
    else
    {
        g_q[g_q_head].f = *f;
        scan_latch_angles(&g_q[g_q_head].pan_ddeg, &g_q[g_q_head].tilt_ddeg);
        g_q_head = next;
    }
}

static void lidar_reset_rx(void)
{
    g_idx = 0U;
}

static void lidar_rearm_it(void)
{
    if (g_huart != NULL)
    {
        (void)HAL_UART_Receive_IT(g_huart, &g_rx_byte, 1U);
    }
}
