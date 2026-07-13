/* ============================================================================
 *  uart_rpi.c  --  RPi <-> STM32 UART 포트 제어 및 프로토콜 디스패처
 *  담당: 이현우
 *  (main.c 에서 추출. 동작은 검증본과 동일 — printf 디버그 유지)
 * ==========================================================================*/
#include "uart_rpi.h"
#include <stdio.h>
#include <string.h>

/* ---- 내부 상태 ---- */
static UART_HandleTypeDef *s_huart;        /* RPi 링크 UART (USART1)      */
static volatile uint8_t    s_rx;           /* 1바이트 수신 버퍼 (IT)       */
static volatile uint8_t    s_rb[256];      /* 수신 링버퍼                  */
static volatile uint16_t   s_rb_head = 0;
static volatile uint16_t   s_rb_tail = 0;

/* protocol.h 프레임 빌드 → USART1 TX 전송 (상행) */
void uart_rpi_send_frame(uint8_t cmd, const void *payload, uint8_t payload_len)
{
    uint8_t  frame[PROTO_MAX_FRAME];
    uint16_t crc;
    uint8_t  total;

    if (payload_len > PROTO_MAX_PAYLOAD) return;   /* CWE-120 경계검사 */

    frame[0] = PROTO_SOF;
    frame[1] = cmd;
    frame[2] = payload_len;

    if (payload_len > 0 && payload != NULL)
        memcpy(&frame[PROTO_HEADER_LEN], payload, payload_len);

    total = PROTO_HEADER_LEN + payload_len;
    crc = proto_crc16(frame, total);
    frame[total]     = (uint8_t)(crc & 0xFF);          /* little-endian */
    frame[total + 1] = (uint8_t)((crc >> 8) & 0xFF);
    total += PROTO_CRC_LEN;

    HAL_UART_Transmit(s_huart, frame, total, 100);
    printf("[TX] cmd=0x%02X len=%u\r\n", cmd, payload_len);
}

/* 바이트 스트림 → 프레임 파싱 + CMD 디스패치 (상태머신) */
static void proto_feed(uint8_t b)
{
    static uint8_t buf[PROTO_MAX_FRAME];
    static uint8_t idx  = 0;
    static uint8_t need = 0;

    printf("[feed] byte=0x%02X idx=%u\r\n", b, idx);   /* ① 모든 수신 바이트 */

    if (idx == 0) {
        if (b != PROTO_SOF) {
            printf("  skip (not SOF)\r\n");             /* ② SOF 아님 → 버림 */
            return;
        }
        buf[idx++] = b;
        need = 0;
        printf("  SOF ok\r\n");                         /* ③ 프레임 시작 */
        return;
    }

    buf[idx++] = b;

    if (idx == PROTO_HEADER_LEN) {
        uint8_t len = buf[2];
        if (len > PROTO_MAX_PAYLOAD) {
            printf("ERR bad len=%u\r\n", len);
            idx = 0; return;
        }
        need = PROTO_HEADER_LEN + len + PROTO_CRC_LEN;
        printf("  header: cmd=0x%02X len=%u need=%u\r\n", buf[1], len, need);  /* ④ */
    }

    if (need && idx >= need) {
        uint8_t  len    = buf[2];
        uint16_t rx_crc = buf[need-2] | (buf[need-1] << 8);
        uint16_t calc   = proto_crc16(buf, PROTO_HEADER_LEN + len);

        if (rx_crc == calc) {
            uint8_t cmd = buf[1];
            printf("  CRC OK, cmd=0x%02X\r\n", cmd);     /* ⑤ 검증 성공 */

            switch (cmd) {
            case CMD_SET_TARGET:
                if (len == sizeof(struct proto_target)) {
                    struct proto_target t;
                    memcpy(&t, &buf[PROTO_HEADER_LEN], sizeof(t));
                    printf("  SET_TARGET theta=%d phi=%d\r\n", t.theta_ddeg, t.phi_ddeg);
                }
                break;

            case CMD_HOME:
                printf("  HOME (모터부 담당) -> HOMED 회신\r\n");
                uart_rpi_send_frame(CMD_HOMED, NULL, 0);   /* 상행 경로 검증용 */
                break;

            case CMD_SET_MODE:
                if (len == sizeof(struct proto_mode)) {
                    struct proto_mode m;
                    memcpy(&m, &buf[PROTO_HEADER_LEN], sizeof(m));
                    printf("  SET_MODE mode=%u\r\n", m.mode);
                }
                break;

            case CMD_DISARM:
                printf("  DISARM (스텝 2축 disable)\r\n");
                break;

            case CMD_PING:
                uart_rpi_send_frame(CMD_PONG, NULL, 0);    /* heartbeat 응답 */
                printf("  PING -> PONG\r\n");
                break;

            case CMD_QUERY_DIST: {
                /* 라이다 미연결: 더미 거리로 상행 경로 검증 */
                struct proto_distance d = { .distance_mm = 2221 };
                uart_rpi_send_frame(CMD_DISTANCE, &d, sizeof(d));
                printf("  QUERY_DIST -> DISTANCE %u mm (dummy)\r\n", d.distance_mm);
                break;
            }

            default:
                printf("  (unhandled cmd 0x%02X)\r\n", cmd);
                break;
            }
        } else {
            printf("CRC FAIL rx=%04X calc=%04X\r\n", rx_crc, calc);
        }
        idx = 0; need = 0;
    }
}

/* ---- 공개 훅 ---- */

void uart_rpi_init(UART_HandleTypeDef *huart)
{
    s_huart   = huart;
    s_rb_head = 0;
    s_rb_tail = 0;
    HAL_UART_Receive_IT(s_huart, (uint8_t *)&s_rx, 1);   /* 수신 시작 */
}

void uart_rpi_on_rx_cplt(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        s_rb[s_rb_head++] = s_rx;                        /* 링버퍼 적재만 */
        s_rb_head &= 0xFF;                                /* 256 wrap */
        HAL_UART_Receive_IT(huart, (uint8_t *)&s_rx, 1);
    }
}

void uart_rpi_on_error(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(huart, (uint8_t *)&s_rx, 1);
    }
}

void uart_rpi_process(void)
{
    while (s_rb_tail != s_rb_head) {                     /* 버퍼에 데이터 있으면 */
        uint8_t b = s_rb[s_rb_tail++];
        s_rb_tail &= 0xFF;
        proto_feed(b);                                   /* 파싱 (느긋해도 안전) */
    }
}
